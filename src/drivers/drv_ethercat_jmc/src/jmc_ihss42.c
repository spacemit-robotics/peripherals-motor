/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file jmc_ihss42.c
 * @brief JMC IHSS42-EC 步进电机驱动实现
 */

#include "jmc_ihss42.h"

#include <stdio.h>
#include <string.h>

/****************************************************************************/
/* PDO 映射配置 */
/****************************************************************************/

static ec_pdo_entry_info_t jmc_rxpdo_entries[] = {
    {0x607A, 0x00, 32},  // 01h target_position       4 bytes
    {0x6081, 0x00, 32},  // 02h profile_velocity      4 bytes
    {0x60FF, 0x00, 32},  // 03h target_velocity       4 bytes
    {0x6083, 0x00, 32},  // 04h profile_acceleration  4 bytes
    {0x6084, 0x00, 32},  // 05h profile_deceleration  4 bytes
    {0x6040, 0x00, 16},  // 06h controlword           2 bytes
    {0x6060, 0x00, 8},   // 07h op_mode               1 byte
    {0x0000, 0x00, 8},   // 08h padding               1 byte
};

static ec_pdo_entry_info_t jmc_pt_rxpdo_entries[] = {
    {0x6080, 0x00, 32},  // 01h max_speed             4 bytes
    {0x6087, 0x00, 32},  // 02h torque_slope          4 bytes
    {0x6040, 0x00, 16},  // 03h controlword           2 bytes
    {0x6071, 0x00, 16},  // 04h target_torque         2 bytes
    {0x6060, 0x00, 8},   // 05h op_mode               1 byte
    {0x0000, 0x00, 8},   // 06h padding               1 byte
    {0x0000, 0x00, 16},  // 07h padding               2 bytes
};
static ec_pdo_entry_info_t jmc_pt_txpdo_entries[] = {
    {0x6064, 0x00, 32},  // 01h actual_position       4 bytes
    {0x606C, 0x00, 32},  // 02h actual_velocity       4 bytes
    {0x603f, 0x00, 16},  // 03h err_code              2 bytes
    {0x6041, 0x00, 16},  // 04h statusword            2 bytes
    {0x6077, 0x00, 16},  // 05h actual_torque         2 bytes
    {0x6061, 0x00, 8},   // 06h op_mode_display       1 byte
    {0x0000, 0x00, 8},   // 07h padding               1 byte
};

static ec_pdo_info_t jmc_pt_pdos[] = {{0x1602, 7, jmc_pt_rxpdo_entries}, {0x1A02, 7, jmc_pt_txpdo_entries}};
static ec_sync_info_t jmc_pt_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE}, {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, &jmc_pt_pdos[0], EC_WD_ENABLE}, {3, EC_DIR_INPUT, 1, &jmc_pt_pdos[1], EC_WD_DISABLE},
    {0xff, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE}  // 哨兵
};

static ec_pdo_entry_info_t jmc_cst_rxpdo_entries[] = {
    {0x6080, 0x00, 32},  // 01h max_speed             4 bytes
    {0x6040, 0x00, 16},  // 02h controlword           2 bytes
    {0x6071, 0x00, 16},  // 03h target_torque         2 bytes
    {0x6060, 0x00, 8},   // 04h op_mode               1 byte
    {0x0000, 0x00, 8},   // 05h padding               1 byte
    {0x0000, 0x00, 16},  // 06h padding               2 bytes
};
static ec_pdo_entry_info_t jmc_cst_txpdo_entries[] = {
    {0x6064, 0x00, 32},  // 01h actual_position       4 bytes
    {0x606C, 0x00, 32},  // 02h actual_velocity       4 bytes
    {0x60FD, 0x00, 32},  // 03h digital_inputs        4 bytes
    {0x6041, 0x00, 16},  // 04h statusword            2 bytes
    {0x6077, 0x00, 16},  // 05h actual_torque         2 bytes
    {0x6061, 0x00, 8},   // 06h op_mode_display       1 byte
    {0x0000, 0x00, 8},   // 07h padding               1 byte
    {0x0000, 0x00, 16},  // 08h padding               2 bytes
};

static ec_pdo_info_t jmc_cst_pdos[] = {{0x1602, 6, jmc_cst_rxpdo_entries}, {0x1A02, 8, jmc_cst_txpdo_entries}};
static ec_sync_info_t jmc_cst_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE}, {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, &jmc_cst_pdos[0], EC_WD_ENABLE}, {3, EC_DIR_INPUT, 1, &jmc_cst_pdos[1], EC_WD_DISABLE},
    {0xff, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE}  // 哨兵
};

// RxPDO 0x1600 - PP 模式 (仅供参考，不通过 XML 映射到周期报文)
static __attribute__((unused)) ec_pdo_entry_info_t jmc_p_rxpdo_entries[] = {
    {0x6040, 0x00, 16},  // 01h controlword           2 bytes
    {0x6060, 0x00, 8},   // 02h op_mode               1 byte
    {0x607A, 0x00, 32},  // 03h target_position       4 bytes
    {0x6081, 0x00, 32},  // 04h profile_velocity      4 bytes
    {0x6083, 0x00, 32},  // 05h profile_acceleration  4 bytes
    {0x6084, 0x00, 32},  // 06h profile_deceleration  4 bytes
};

// TxPDO 0x1A00 - PP/CSP模式建议映射 (表164)
static ec_pdo_entry_info_t jmc_txpdo_entries[] = {
    {0x6064, 0x00, 32},  // 01h actual_position       4 bytes
    {0x606C, 0x00, 32},  // 02h actual_velocity       4 bytes
    {0x6041, 0x00, 16},  // 03h statusword            2 bytes
    {0x603F, 0x00, 16},  // 04h error_code            2 bytes
    {0x6061, 0x00, 8},   // 05h op_mode_display       1 byte
    {0x0000, 0x00, 8},   // 06h padding               1 byte
};

// PDO 配置
static ec_pdo_info_t jmc_pdos[] = {
    {0x1600, 8, jmc_rxpdo_entries},  // RxPDO: 8 个条目 (包含 padding)
    {0x1A00, 5, jmc_txpdo_entries},  // TxPDO: 5 个条目
};

// Sync Manager 配置
static ec_sync_info_t jmc_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE}, {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, &jmc_pdos[0], EC_WD_ENABLE}, {3, EC_DIR_INPUT, 1, &jmc_pdos[1], EC_WD_DISABLE},
    {0xff, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE}  // 哨兵
};

/****************************************************************************/
/* SDO 状态定义 */
/****************************************************************************/

typedef struct {
    ec_sdo_request_t* sdo_op_mode;
    ec_sdo_request_t* sdo_target_pos;
    ec_sdo_request_t* sdo_profile_vel;
    ec_sdo_request_t* sdo_profile_acc;
    ec_sdo_request_t* sdo_profile_dec;
    ec_sdo_request_t* sdo_homing_method;
    ec_sdo_request_t* sdo_target_velocity;
    int sdo_state;  // 0: 初始化, 1: 配置OP_MODE和参数, 2: 等待配置完成, 3: 完成
} jmc_sdo_ctx_t;

static jmc_sdo_ctx_t sdo_ctx[EC_MAX_MOTORS];

/****************************************************************************/
/* 常量定义 */
/****************************************************************************/

// 最大转矩限制
#define EC_MAX_TORQUE_LIMIT 1000

/****************************************************************************/
/* 内部函数 */
/****************************************************************************/

static int jmc_config_slave(ec_master_ctx_t* ctx, int motor_idx) {
    if (!ctx || motor_idx < 0 || motor_idx >= EC_MAX_MOTORS)
        return -1;

    ec_motor_t* motor = &ctx->motors[motor_idx];
    if (!motor->sc) {
        fprintf(stderr, "错误: M%d 从站配置为空\n", motor_idx);
        return -1;
    }

    // 为 PP 模式分配 SDO 请求 (仅用于 op_mode SDO 配置，其他参数通过 PDO)
    sdo_ctx[motor_idx].sdo_op_mode = ecrt_slave_config_create_sdo_request(motor->sc, 0x6060, 0x00, 1);
    // 设置从站 PREOP 阶段的默认模式为 CSP (8)
    ecrt_slave_config_sdo8(motor->sc, 0x6060, 0x00, 8);
    printf("M%d: 已配置 SDO 0x6060 (Modes of Operation) = 8 (CSP)\n", motor_idx);

    // 为 HM 模式分配额外的 SDO 请求 (用于那些放不进 PDO 的参数)
    sdo_ctx[motor_idx].sdo_target_pos =
        ecrt_slave_config_create_sdo_request(motor->sc, 0x609A, 0x00, 4);  // homing_accel
    sdo_ctx[motor_idx].sdo_profile_vel =
        ecrt_slave_config_create_sdo_request(motor->sc, 0x6099, 0x01, 4);  // speed_switch
    sdo_ctx[motor_idx].sdo_profile_acc =
        ecrt_slave_config_create_sdo_request(motor->sc, 0x6099, 0x02, 4);  // speed_zero
    sdo_ctx[motor_idx].sdo_profile_dec =
        ecrt_slave_config_create_sdo_request(motor->sc, 0x607C, 0x00, 4);  // homing_offset
    sdo_ctx[motor_idx].sdo_homing_method =
        ecrt_slave_config_create_sdo_request(motor->sc, 0x6098, 0x00, 1);  // homing_method
    sdo_ctx[motor_idx].sdo_target_velocity =
        ecrt_slave_config_create_sdo_request(motor->sc, 0x60FF, 0x00, 4);  // target_velocity
    sdo_ctx[motor_idx].sdo_state = 0;

    if (!sdo_ctx[motor_idx].sdo_op_mode) {
        fprintf(stderr, "错误: M%d 创建 SDO 请求失败\n", motor_idx);
        return -1;
    }

    // 设置超时时间
    ecrt_sdo_request_timeout(sdo_ctx[motor_idx].sdo_op_mode, 500);
    ecrt_sdo_request_timeout(sdo_ctx[motor_idx].sdo_target_velocity, 500);

    // 如果是 PT 模式，重新配置 PDO
    if (motor->mode == EC_MODE_PT) {
        printf("M%d: 正在配置 PT 模式 SM/PDO (Indices: 0, 1, 2:%04X, 3:%04X)\n", motor_idx, jmc_pt_pdos[0].index,
            jmc_pt_pdos[1].index);
        if (ecrt_slave_config_pdos(motor->sc, EC_END, jmc_pt_syncs)) {
            fprintf(stderr, "错误: M%d PT模式 PDO 重新配置失败 (Invalid argument)\n", motor_idx);
        } else {
            printf("M%d: PT模式 PDO 已通过回调重新配置\n", motor_idx);
        }

        // 在 PT 模式下，显式写入最大转矩 0x6072，单位为千分之额定转矩，1000 即 100%
        ecrt_slave_config_sdo16(motor->sc, 0x6072, 0x00, 1000);
    } else if (motor->mode == EC_MODE_CST) {
        printf("M%d: 正在配置 CST 模式 SM/PDO (Indices: 0, 1, 2:%04X, 3:%04X)\n", motor_idx, jmc_cst_pdos[0].index,
            jmc_cst_pdos[1].index);
        if (ecrt_slave_config_pdos(motor->sc, EC_END, jmc_cst_syncs)) {
            fprintf(stderr, "错误: M%d CST模式 PDO 重新配置失败 (Invalid argument)\n", motor_idx);
        } else {
            printf("M%d: CST模式 PDO 已通过回调重新配置\n", motor_idx);
        }
    }

    printf("M%d: PP/HM模式 SDO 配置已创建\n", motor_idx);

    return 0;
}

static int jmc_register_pdo_internal(ec_master_ctx_t* ctx, int motor_idx) {
    (void)ctx;
    (void)motor_idx;
    // 此函数由外部调用 jmc_ihss42_register_pdo 实现
    return 0;
}

// 从站驱动定义
static const ec_slave_driver_t jmc_ihss42_driver = {
    .name = "JMC IHSS42-EC",
    .vendor_id = JMC_VENDOR_ID,
    .product_code = JMC_IHSS42_PRODUCT_CODE,
    .sync_info = jmc_syncs,
    .pdo_entry_count = 14,  // 8 个 RxPDO + 6 个 TxPDO
    .config_slave = jmc_config_slave,
    .register_pdo = jmc_register_pdo_internal,
};

/****************************************************************************/
/* 公共接口 */
/****************************************************************************/

const ec_slave_driver_t* jmc_ihss42_get_driver(void) { return &jmc_ihss42_driver; }

int jmc_ihss42_configure(ec_master_ctx_t* ctx, int motor_idx, uint16_t alias, uint16_t position, ec_app_mode_t mode) {
    if (!ctx)
        return -1;

    // 设置模式
    if (motor_idx >= 0 && motor_idx < EC_MAX_MOTORS) {
        ctx->motors[motor_idx].mode = mode;
    }

    return ec_slave_configure(ctx, motor_idx, alias, position, &jmc_ihss42_driver);
}

int jmc_ihss42_register_pdo(ec_master_ctx_t* ctx, int motor_idx) {
    if (!ctx || !ctx->domain || motor_idx < 0 || motor_idx >= ctx->motor_count)
        return -1;

    ec_motor_t* motor = &ctx->motors[motor_idx];

    // 调试：打印从站信息
    printf("M%d: 准备注册 PDO (alias=%u, pos=%u, vendor=0x%08X, product=0x%08X)\n", motor_idx, motor->alias,
        motor->position, motor->vendor_id, motor->product_code);

    motor->off.ctrl_word = 0;
    motor->off.op_mode = 0;
    motor->off.target_position = 0;
    motor->off.profile_velocity = 0;
    motor->off.target_velocity = 0;
    motor->off.profile_accel = 0;
    motor->off.profile_decel = 0;
    motor->off.homing_method = 0;
    motor->off.homing_accel = 0;
    motor->off.homing_speed_switch = 0;
    motor->off.homing_speed_zero = 0;
    motor->off.homing_offset = 0;
    motor->off.target_torque = 0;
    motor->off.torque_slope = 0;
    motor->off.max_speed = 0;
    motor->off.status_word = 0;
    motor->off.actual_torque = 0;
    motor->off.op_mode_display = 0;
    motor->off.actual_position = 0;
    motor->off.actual_velocity = 0;
    motor->off.digital_inputs = 0;

    ec_pdo_entry_reg_t regs[] = {  // RxPDO
        {motor->alias, motor->position, JMC_VENDOR_ID, JMC_IHSS42_PRODUCT_CODE, 0x607A, 0x00,
            &motor->off.target_position, NULL},
        {motor->alias, motor->position, JMC_VENDOR_ID, JMC_IHSS42_PRODUCT_CODE, 0x6081, 0x00,
            &motor->off.profile_velocity, NULL},
        {motor->alias, motor->position, JMC_VENDOR_ID, JMC_IHSS42_PRODUCT_CODE, 0x60FF, 0x00,
            &motor->off.target_velocity, NULL},
        {motor->alias, motor->position, JMC_VENDOR_ID, JMC_IHSS42_PRODUCT_CODE, 0x6083, 0x00, &motor->off.profile_accel,
            NULL},
        {motor->alias, motor->position, JMC_VENDOR_ID, JMC_IHSS42_PRODUCT_CODE, 0x6084, 0x00, &motor->off.profile_decel,
            NULL},
        {motor->alias, motor->position, JMC_VENDOR_ID, JMC_IHSS42_PRODUCT_CODE, 0x6040, 0x00, &motor->off.ctrl_word,
            NULL},
        {motor->alias, motor->position, JMC_VENDOR_ID, JMC_IHSS42_PRODUCT_CODE, 0x6060, 0x00, &motor->off.op_mode,
            NULL},
        // TxPDO
        {motor->alias, motor->position, JMC_VENDOR_ID, JMC_IHSS42_PRODUCT_CODE, 0x6064, 0x00,
            &motor->off.actual_position, NULL},
        {motor->alias, motor->position, JMC_VENDOR_ID, JMC_IHSS42_PRODUCT_CODE, 0x606C, 0x00,
            &motor->off.actual_velocity, NULL},
        {motor->alias, motor->position, JMC_VENDOR_ID, JMC_IHSS42_PRODUCT_CODE, 0x6041, 0x00, &motor->off.status_word,
            NULL},
        {motor->alias, motor->position, JMC_VENDOR_ID, JMC_IHSS42_PRODUCT_CODE, 0x603F, 0x00, &motor->off.error_code,
            NULL},
        {motor->alias, motor->position, JMC_VENDOR_ID, JMC_IHSS42_PRODUCT_CODE, 0x6061, 0x00,
            &motor->off.op_mode_display, NULL},
        {}};

    ec_pdo_entry_reg_t pt_regs[] = {  // RxPDO PT
        {motor->alias, motor->position, JMC_VENDOR_ID, JMC_IHSS42_PRODUCT_CODE, 0x6080, 0x00, &motor->off.max_speed,
            NULL},
        {motor->alias, motor->position, JMC_VENDOR_ID, JMC_IHSS42_PRODUCT_CODE, 0x6087, 0x00, &motor->off.torque_slope,
            NULL},
        {motor->alias, motor->position, JMC_VENDOR_ID, JMC_IHSS42_PRODUCT_CODE, 0x6040, 0x00, &motor->off.ctrl_word,
            NULL},
        {motor->alias, motor->position, JMC_VENDOR_ID, JMC_IHSS42_PRODUCT_CODE, 0x6071, 0x00, &motor->off.target_torque,
            NULL},
        {motor->alias, motor->position, JMC_VENDOR_ID, JMC_IHSS42_PRODUCT_CODE, 0x6060, 0x00, &motor->off.op_mode,
            NULL},
        // TxPDO PT
        {motor->alias, motor->position, JMC_VENDOR_ID, JMC_IHSS42_PRODUCT_CODE, 0x6064, 0x00,
            &motor->off.actual_position, NULL},
        {motor->alias, motor->position, JMC_VENDOR_ID, JMC_IHSS42_PRODUCT_CODE, 0x606C, 0x00,
            &motor->off.actual_velocity, NULL},
        {motor->alias, motor->position, JMC_VENDOR_ID, JMC_IHSS42_PRODUCT_CODE, 0x603f, 0x00, &motor->off.error_code,
            NULL},
        {motor->alias, motor->position, JMC_VENDOR_ID, JMC_IHSS42_PRODUCT_CODE, 0x6041, 0x00, &motor->off.status_word,
            NULL},
        {motor->alias, motor->position, JMC_VENDOR_ID, JMC_IHSS42_PRODUCT_CODE, 0x6077, 0x00, &motor->off.actual_torque,
            NULL},
        {motor->alias, motor->position, JMC_VENDOR_ID, JMC_IHSS42_PRODUCT_CODE, 0x6061, 0x00,
            &motor->off.op_mode_display, NULL},
        {}};

    ec_pdo_entry_reg_t cst_regs[] = {  // RxPDO CST
        {motor->alias, motor->position, JMC_VENDOR_ID, JMC_IHSS42_PRODUCT_CODE, 0x6080, 0x00, &motor->off.max_speed,
            NULL},
        {motor->alias, motor->position, JMC_VENDOR_ID, JMC_IHSS42_PRODUCT_CODE, 0x6040, 0x00, &motor->off.ctrl_word,
            NULL},
        {motor->alias, motor->position, JMC_VENDOR_ID, JMC_IHSS42_PRODUCT_CODE, 0x6071, 0x00, &motor->off.target_torque,
            NULL},
        {motor->alias, motor->position, JMC_VENDOR_ID, JMC_IHSS42_PRODUCT_CODE, 0x6060, 0x00, &motor->off.op_mode,
            NULL},
        // TxPDO CST
        {motor->alias, motor->position, JMC_VENDOR_ID, JMC_IHSS42_PRODUCT_CODE, 0x6064, 0x00,
            &motor->off.actual_position, NULL},
        {motor->alias, motor->position, JMC_VENDOR_ID, JMC_IHSS42_PRODUCT_CODE, 0x606C, 0x00,
            &motor->off.actual_velocity, NULL},
        {motor->alias, motor->position, JMC_VENDOR_ID, JMC_IHSS42_PRODUCT_CODE, 0x60FD, 0x00,
            &motor->off.digital_inputs, NULL},
        {motor->alias, motor->position, JMC_VENDOR_ID, JMC_IHSS42_PRODUCT_CODE, 0x6041, 0x00, &motor->off.status_word,
            NULL},
        {motor->alias, motor->position, JMC_VENDOR_ID, JMC_IHSS42_PRODUCT_CODE, 0x6077, 0x00, &motor->off.actual_torque,
            NULL},
        {motor->alias, motor->position, JMC_VENDOR_ID, JMC_IHSS42_PRODUCT_CODE, 0x6061, 0x00,
            &motor->off.op_mode_display, NULL},
        {}};

    ec_pdo_entry_reg_t* selected_regs;
    if (motor->mode == EC_MODE_PT) {
        selected_regs = pt_regs;
    } else if (motor->mode == EC_MODE_CST) {
        selected_regs = cst_regs;
    } else {
        selected_regs = regs;
    }

    if (ecrt_domain_reg_pdo_entry_list(ctx->domain, selected_regs)) {
        fprintf(stderr, "错误: M%d PDO 条目注册失败!\n", motor_idx);
        return -1;
    }

    // 调试：打印注册后的偏移量
    printf("M%d: PDO 条目注册成功 - 偏移量:\n", motor_idx);
    printf("  ctrl_word=%u, op_mode=%u, target_position=%u\n", motor->off.ctrl_word, motor->off.op_mode,
        motor->off.target_position);
    printf("  profile_vel=%u, target_vel=%u, profile_acc=%u, profile_dec=%u\n", motor->off.profile_velocity,
        motor->off.target_velocity, motor->off.profile_accel, motor->off.profile_decel);
    printf("  status_word=%u, op_mode_display=%u, actual_position=%u\n", motor->off.status_word,
        motor->off.op_mode_display, motor->off.actual_position);

    return 0;
}

void jmc_ihss42_set_profile_params(
    ec_master_ctx_t* ctx, int motor_idx, uint32_t profile_vel, uint32_t profile_acc, uint32_t profile_dec) {
    if (!ctx || motor_idx < 0 || motor_idx >= EC_MAX_MOTORS)
        return;

    ctx->motors[motor_idx].motion.profile_vel = profile_vel;
    ctx->motors[motor_idx].motion.profile_acc = profile_acc;
    ctx->motors[motor_idx].motion.profile_dec = profile_dec;
}

void jmc_ihss42_cyclic_task(ec_master_ctx_t* ctx, int motor_idx) {
    if (!ctx || !ctx->domain_pd || motor_idx < 0 || motor_idx >= ctx->motor_count)
        return;

    ec_motor_t* motor = &ctx->motors[motor_idx];
    uint8_t* pd = ctx->domain_pd;

    // 读取 TxPDO (PP模式: 5个条目)
    uint16_t status_word = EC_READ_U16(pd + motor->off.status_word);
    uint16_t error_code = EC_READ_U16(pd + motor->off.error_code);
    int32_t actual_pos = EC_READ_S32(pd + motor->off.actual_position);
    int32_t actual_vel = EC_READ_S32(pd + motor->off.actual_velocity);
    int16_t actual_torq = EC_READ_S16(pd + motor->off.actual_torque);
    int8_t op_mode_disp = EC_READ_S8(pd + motor->off.op_mode_display);

    int slave_operational = motor->sc_state.operational;

    // 处理 CiA402 状态机
    uint16_t ctrl_word =
        ec_cia402_process(&motor->cia402_state, &motor->fault_reset_count, status_word, slave_operational);

    // 状态机转换与锚定逻辑
    // 方案：PREOP 阶段已设为 CSP，转换期间保持 CSP 并锚定位置，使能后再切目标模式
    int8_t target_op_mode = ec_mode_to_op_mode(motor->mode);
    int8_t hw_op_mode = EC_OP_MODE_CSP;  // 默认为 CSP

    if (motor->cia402_state == EC_STATE_RUNNING) {
        // 已使能，可以切换到目标模式
        hw_op_mode = target_op_mode;

        // 使能瞬间（且尚未被适配器标定前）进行物理起点锚定
        if (motor->motion.start_pos_set == 0) {
            motor->motion.start_pos = actual_pos;
            motor->motion.target_pos = actual_pos;  // 使能瞬间强制 target = current
            motor->motion.start_pos_set = 1;        // 告知适配器可以进行逻辑锚定了
            printf("M%d %s模式使能 - 物理起点锚定: %d\n", motor_idx, ec_mode_name(motor->mode), actual_pos);
        }

        // 在适配器完成逻辑锚定（state
        // 2）之前，持续维持目标等于当前反馈，防止任何非预期移动
        if (motor->motion.start_pos_set < 2) {
            motor->motion.target_pos = actual_pos;
            motor->motion.target_vel = 0;
            motor->motion.target_torque = 0;
        }
    } else {
        // 未使能或转换中，强制 CSP 模式并同步位置（发“无效帧”，即位置不动的轨迹）
        hw_op_mode = EC_OP_MODE_CSP;
        motor->motion.target_pos = actual_pos;
        motor->motion.target_vel = 0;
        motor->motion.target_torque = 0;
        motor->motion.start_pos_set = 0;  // 重置标志位
    }

    if (motor->cia402_state == EC_STATE_RUNNING) {
        // CSP 模式特定逻辑 (目前没有)
        if (motor->mode == EC_MODE_CSP) {
        } else if (motor->mode == EC_MODE_CSV) {
            // CSV 模式特定逻辑 (目前没有)
        } else if (motor->mode == EC_MODE_PV) {
            // PV 模式逻辑
            EC_WRITE_U32(pd + motor->off.profile_accel, motor->motion.profile_acc);
            EC_WRITE_U32(pd + motor->off.profile_decel, motor->motion.profile_dec);
            // PV 模式下，控制字给 0x000F 启动
            ctrl_word = 0x000F;
        } else if (motor->mode == EC_MODE_PP) {
            // PP 模式逻辑 (PDO 控制)
            EC_WRITE_U32(pd + motor->off.profile_velocity, motor->motion.profile_vel);
            EC_WRITE_U32(pd + motor->off.profile_accel, motor->motion.profile_acc);
            EC_WRITE_U32(pd + motor->off.profile_decel, motor->motion.profile_dec);

            switch (motor->motion.pp_state) {
                case 0:  // 空闲
                    break;
                case 1:                   // 发送 New Setpoint
                    ctrl_word |= 0x003F;  // Bit 4: New Setpoint + Bit 5: Change Set Immediately
                    motor->motion.pp_state = 2;
                    motor->motion.pp_wait_count = 0;
                    printf("M%d PP 发送新目标位置 (PDO): %d\n", motor_idx, motor->motion.target_pos);
                    break;
                case 2:  // 等待确认 (Bit 12)
                    ctrl_word |= 0x003F;
                    motor->motion.pp_wait_count++;
                    if (status_word & 0x1000) {  // Bit 12: Setpoint acknowledge
                        motor->motion.pp_state = 3;
                        printf("M%d PP 从站已确认目标位置\n", motor_idx);
                    }
                    // 移除此处的 Bit 10 检查，防止在握手期间因 Bit 10 短暂跳变导致提前终止
                    break;
                case 3:  // 清除 New Setpoint
                    ctrl_word &= ~0x0010;
                    motor->motion.pp_state = 4;
                    break;
                case 4:                          // 等待运动完成
                    if (status_word & 0x0400) {  // Target reached
                        motor->motion.pp_state = 0;
                        printf("M%d PP 到达目标位置: %d\n", motor_idx, actual_pos);
                    }
                    break;
            }
        } else if (motor->mode == EC_MODE_HM) {
            EC_WRITE_S8(pd + motor->off.homing_method, motor->motion.homing_method);
            // 其他 HM 参数通过 SDO 写入 (在 hm_state 1 中触发)

            switch (motor->motion.hm_state) {
                case 0:  // 空闲
                    break;
                case 1:  // 启动回零 - 先发 SDO 参数
                    if (motor->motion.homing_method == 0) {
                        motor->motion.homing_method = 35;  // 默认使用方式 35 (当前位置设为零)
                        printf("M%d HM 使用默认回零方式 35\n", motor_idx);
                    }
                    EC_WRITE_U32(ecrt_sdo_request_data(sdo_ctx[motor_idx].sdo_target_pos), motor->motion.homing_accel);
                    EC_WRITE_U32(
                        ecrt_sdo_request_data(sdo_ctx[motor_idx].sdo_profile_vel), motor->motion.homing_speed_switch);
                    EC_WRITE_U32(
                        ecrt_sdo_request_data(sdo_ctx[motor_idx].sdo_profile_acc), motor->motion.homing_speed_zero);
                    EC_WRITE_S32(
                        ecrt_sdo_request_data(sdo_ctx[motor_idx].sdo_profile_dec), motor->motion.homing_offset);
                    EC_WRITE_S8(
                        ecrt_sdo_request_data(sdo_ctx[motor_idx].sdo_homing_method), motor->motion.homing_method);

                    ecrt_sdo_request_write(sdo_ctx[motor_idx].sdo_target_pos);
                    ecrt_sdo_request_write(sdo_ctx[motor_idx].sdo_profile_vel);
                    ecrt_sdo_request_write(sdo_ctx[motor_idx].sdo_profile_acc);
                    ecrt_sdo_request_write(sdo_ctx[motor_idx].sdo_profile_dec);
                    ecrt_sdo_request_write(sdo_ctx[motor_idx].sdo_homing_method);

                    motor->motion.hm_state = 10;  // 等待 SDO 完成
                    printf("M%d HM 正在发送 SDO 参数...\n", motor_idx);
                    break;
                case 10:  // 等待 SDO 完成
                    if (ecrt_sdo_request_state(sdo_ctx[motor_idx].sdo_target_pos) == EC_REQUEST_SUCCESS &&
                        ecrt_sdo_request_state(sdo_ctx[motor_idx].sdo_profile_vel) == EC_REQUEST_SUCCESS &&
                        ecrt_sdo_request_state(sdo_ctx[motor_idx].sdo_profile_acc) == EC_REQUEST_SUCCESS &&
                        ecrt_sdo_request_state(sdo_ctx[motor_idx].sdo_profile_dec) == EC_REQUEST_SUCCESS &&
                        ecrt_sdo_request_state(sdo_ctx[motor_idx].sdo_homing_method) == EC_REQUEST_SUCCESS) {
                        motor->motion.hm_state = 11;
                    }
                    break;
                case 11:  // 启动回零指令
                    ctrl_word = 0x001F;
                    motor->motion.hm_state = 2;
                    printf("M%d HM 启动回零: 方式=%d\n", motor_idx, motor->motion.homing_method);
                    break;
                case 2:  // 等待确认启动 (Status Bit 12: Homing attained)
                    ctrl_word = 0x001F;
                    if (status_word & 0x2000) {  // Bit 13: Homing error
                        printf("M%d HM 回零报错! SW=0x%04X\n", motor_idx, status_word);
                        motor->motion.hm_state = 100;   // 停止重试
                    } else if (status_word & 0x1000) {  // Bit 12: Homing attained
                        printf("M%d HM 回零成功完成 SW=0x%04X\n", motor_idx, status_word);
                        motor->motion.hm_state = 3;
                    }
                    break;
                case 3:  // 回零完成，清除启动位 (回到 0x000F)
                    ctrl_word = 0x000F;
                    motor->motion.hm_state = 100;  // 进入完成状态，防止循环触发
                    break;
                case 100:  // 完成状态
                    ctrl_word = 0x000F;
                    break;
            }
        } else if (motor->mode == EC_MODE_PT) {
            EC_WRITE_U32(pd + motor->off.torque_slope, motor->motion.torque_slope);

            // PT 模式状态机
            // 0 (空闲) → 1 (写SDO 0x60FF) → 10 (等待SDO完成) → 2 (正常运行)
            // 在 PT 模式电机使能后，先通过 SDO
            // 一次性配置好目标速度参数（因为该参数没有映射到 PDO 中）
            // 配置成功后再进入正常的周期性 PDO 数据交换阶段
            switch (motor->motion.pt_state) {
                case 0:  // 空闲
                    break;
                case 1:  // 使能后发送目标速度 SDO
                    EC_WRITE_U32(ecrt_sdo_request_data(sdo_ctx[motor_idx].sdo_target_velocity), 20000);
                    ecrt_sdo_request_write(sdo_ctx[motor_idx].sdo_target_velocity);
                    motor->motion.pt_state = 10;  // 等待 SDO 完成
                    printf("M%d PT 正在发送目标速度 SDO (0x60FF) = 20000...\n", motor_idx);
                    break;
                case 10:  // 等待 SDO 完成
                    if (ecrt_sdo_request_state(sdo_ctx[motor_idx].sdo_target_velocity) == EC_REQUEST_SUCCESS) {
                        motor->motion.pt_state = 2;
                        printf("\n*******************M%d PT 目标速度 SDO "
                                "发送成功******************\n\n",
                            motor_idx);
                    }
                    break;
                case 2:  // SDO 配置完成，正常运行
                    break;
            }
            EC_WRITE_U32(pd + motor->off.max_speed, motor->motion.max_speed);

            // 启动指令 0x000F
            ctrl_word = 0x000F;

            // 在首次使能时触发 SDO 写入
            if (motor->motion.pt_state == 0 && motor->motion.start_pos_set) {
                motor->motion.pt_state = 1;
            }
        } else if (motor->mode == EC_MODE_CST) {
            // CST 模式：写入最大速度
            EC_WRITE_U32(pd + motor->off.max_speed, motor->motion.max_speed);

            // 启动指令 0x000F
            ctrl_word = 0x000F;

            // CST 模式调试信息
            static int cst_debug_count = 0;
            if (cst_debug_count < 5 || ctx->cycle_count % 400 == 0) {
                printf("M%d CST: 目标转矩=%d, 最大速度=%u, 控制字=0x%04X\n", motor_idx, motor->motion.target_torque,
                    motor->motion.max_speed, ctrl_word);
                printf("M%d CST: 模式显示=%d (应为10), 状态字=0x%04X\n", motor_idx, op_mode_disp, status_word);

                // 检查状态字中的警告位
                if (status_word & 0x0080) {
                    printf("M%d CST 警告: 检测到故障位 (Bit 7)\n", motor_idx);
                }
                if (status_word & 0x2000) {
                    printf("M%d CST 警告: 回零错误 (Bit 13)\n", motor_idx);
                }

                // 显示原始实际转矩数据
                printf("M%d CST: 实际转矩=%d (原始数据: 0x%04X)\n", motor_idx, actual_torq, (uint16_t)actual_torq);

                cst_debug_count++;
            }

            // 检查CST模式是否正确激活
            if (op_mode_disp != EC_OP_MODE_CST && ctx->cycle_count % 200 == 0) {
                printf("M%d CST 警告: 模式显示=%d, 期望=%d (CST)\n", motor_idx, op_mode_disp, EC_OP_MODE_CST);
            }

            // CST 模式转矩测试 - 逐渐增加转矩
            static int torque_test_cycle = 0;
            if (cst_debug_count < 5 && actual_torq == 0) {
                torque_test_cycle++;
                if (torque_test_cycle % 100 == 0) {
                    // 每100个周期增加转矩，最大不超过EC_MAX_TORQUE_LIMIT
                    int test_torque = 500 + (torque_test_cycle / 100) * 100;
                    if (test_torque <= EC_MAX_TORQUE_LIMIT) {
                        motor->motion.target_torque = test_torque;
                        printf(
                            "M%d CST 测试: 设置转矩=%d (最大限制:%d)\n", motor_idx, test_torque, EC_MAX_TORQUE_LIMIT);
                    }
                }
            }

            // 确保转矩不超过最大限制
            if (motor->motion.target_torque > EC_MAX_TORQUE_LIMIT) {
                motor->motion.target_torque = EC_MAX_TORQUE_LIMIT;
                printf("M%d CST: 转矩限制为%d，当前转矩=%d\n", motor_idx, EC_MAX_TORQUE_LIMIT, EC_MAX_TORQUE_LIMIT);
            }
        }
    } else {
        // 未使能时，目标位置跟随实际位置
        motor->motion.target_pos = actual_pos;
        motor->motion.target_vel = 0;
        motor->motion.pp_state = 0;
        motor->motion.pt_state = 0;
        sdo_ctx[motor_idx].sdo_state = 0;
    }

    // 写入最终确定的操作模式
    EC_WRITE_S8(pd + motor->off.op_mode, hw_op_mode);

    // 根据模式写入控制数据
    // 注意：在使能前或模式切换瞬间，这里写入的是已经过物理锚定的 actual_pos 或
    // 0
    if (hw_op_mode == EC_OP_MODE_CSP || hw_op_mode == EC_OP_MODE_PP) {
        EC_WRITE_S32(pd + motor->off.target_position, motor->motion.target_pos);
    } else if (hw_op_mode == EC_OP_MODE_CSV || hw_op_mode == EC_OP_MODE_PV) {
        EC_WRITE_S32(pd + motor->off.target_velocity, motor->motion.target_vel);
    } else if (hw_op_mode == EC_OP_MODE_CST || hw_op_mode == EC_OP_MODE_PT) {
        EC_WRITE_S16(pd + motor->off.target_torque, (int16_t)motor->motion.target_torque);
    }

    // 写入控制字
    EC_WRITE_U16(pd + motor->off.ctrl_word, ctrl_word);

    // 调试：验证写入的数据
    uint16_t written_ctrl = EC_READ_U16(pd + motor->off.ctrl_word);
    int32_t written_tpos = EC_READ_S32(pd + motor->off.target_position);

    // 域分析长度：固定打印 48 字节，确保能看到完整的 TxPDO 数据
    unsigned int pd_size = 48;

    // 调试: 详细的域数据打印
    static int debug_count = 0;
    if (debug_count < 5 || ctx->cycle_count % 400 == 0) {
        printf("\n========== [M%d 完整域数据分析] 周期=%u ==========\n", motor_idx, ctx->cycle_count);
        printf("域信息: 约=%u bytes, 数据指针=%p\n", pd_size, (void*)pd);

        // 打印整个域（以十六进制）
        printf("├─ 域数据 (前96字节 HEX):\n");
        for (unsigned int i = 0; i < 96 && i < pd_size; i += 16) {
            printf("│  [%3u]: ", i);
            for (int j = 0; j < 16 && i + (unsigned int)j < pd_size; j++)
                printf("%02X ", pd[i + j]);
            printf("\n");
        }

        printf("\n├─ [RxPDO映射分析] - 从主站发送到从站\n");
        printf("│  01h: 0x6040 (controlword)      @ offset %u\n", motor->off.ctrl_word);
        printf("│       原始值: 0x%04X  实际数据: ", ctrl_word);
        for (int j = 0; j < 2 && motor->off.ctrl_word + (unsigned int)j < pd_size; j++)
            printf("%02X ", pd[motor->off.ctrl_word + j]);
        printf("\n");

        if (hw_op_mode == EC_OP_MODE_CSP || hw_op_mode == EC_OP_MODE_PP) {
            EC_WRITE_S32(pd + motor->off.target_position, motor->motion.target_pos);
        } else if (hw_op_mode == EC_OP_MODE_CSV || hw_op_mode == EC_OP_MODE_PV) {
            EC_WRITE_S32(pd + motor->off.target_velocity, motor->motion.target_vel);
        } else if (hw_op_mode == EC_OP_MODE_CST || hw_op_mode == EC_OP_MODE_PT) {
            EC_WRITE_S16(pd + motor->off.target_torque, (int16_t)motor->motion.target_torque);
        }

        printf("\n├─ [TxPDO映射分析] - 从从站接收\n");
        printf("│  02h: 0x6041 (status_word)      @ offset %u\n", motor->off.status_word);
        printf("│       读取值: 0x%04X  实际数据: ", status_word);
        for (int j = 0; j < 2 && motor->off.status_word + (unsigned int)j < pd_size; j++)
            printf("%02X ", pd[motor->off.status_word + j]);
        printf("\n");

        printf("│  03h: 0x6061 (op_mode_display)  @ offset %u\n", motor->off.op_mode_display);
        printf("│       读取值: %d  实际数据: %02X\n", op_mode_disp,
            motor->off.op_mode_display < pd_size ? pd[motor->off.op_mode_display] : 0xFF);

        // 对于 CST 模式，显示实际转矩
        if (motor->mode == EC_MODE_CST) {
            printf("│  04h: 0x6077 (actual_torque)    @ offset %u\n", motor->off.actual_torque);
            printf("│       读取值: %d (0x%04X)  实际数据: ", actual_torq, (uint16_t)actual_torq);
            for (int j = 0; j < 2 && motor->off.actual_torque + (unsigned int)j < pd_size; j++)
                printf("%02X ", pd[motor->off.actual_torque + j]);
            printf("\n");
        }

        printf("│  %dh: 0x6064 (actual_position)  @ offset %u\n", motor->mode == EC_MODE_CST ? 5 : 4,
            motor->off.actual_position);
        printf("│       读取值: %d (0x%08X)  实际数据: ", actual_pos, (unsigned int)actual_pos);
        for (int j = 0; j < 4 && motor->off.actual_position + (unsigned int)j < pd_size; j++)
            printf("%02X ", pd[motor->off.actual_position + j]);
        printf("\n");

        printf("│  %dh: 0x606C (actual_velocity)  @ offset %u\n", motor->mode == EC_MODE_CST ? 6 : 5,
            motor->off.actual_velocity);
        printf("│       读取值: %d (0x%08X)  实际数据: ", actual_vel, (unsigned int)actual_vel);
        for (int j = 0; j < 4 && motor->off.actual_velocity + (unsigned int)j < pd_size; j++)
            printf("%02X ", pd[motor->off.actual_velocity + j]);
        printf("\n");

        printf("└─ [摘要]\n");
        if (motor->mode == EC_MODE_CSV || motor->mode == EC_MODE_PV) {
            printf("   RPDO写入: ctrl=0x%04X tvel=%d\n", written_ctrl, motor->motion.target_vel);
            printf("   TPDO读取: status=0x%04X apos=%d avel=%d\n", status_word, actual_pos, actual_vel);
        } else if (motor->mode == EC_MODE_PT) {
            printf("   RPDO写入: ctrl=0x%04X ttrq=%d\n", written_ctrl, motor->motion.target_torque);
            printf("   TPDO读取: status=0x%04X atorq=%d avel=%d\n", status_word, actual_torq, actual_vel);
        } else if (motor->mode == EC_MODE_CST) {
            printf("   RPDO写入: ctrl=0x%04X ttrq=%d\n", written_ctrl, motor->motion.target_torque);
            printf("   TPDO读取: status=0x%04X atorq=%d avel=%d\n", status_word, actual_torq, actual_vel);
        } else {
            printf("   RPDO写入: ctrl=0x%04X tpos=%d\n", written_ctrl, written_tpos);
            printf("   TPDO读取: status=0x%04X apos=%d avel=%d\n", status_word, actual_pos, actual_vel);
        }
        printf("==========================================\n\n");
        debug_count++;
    }

    // 调试: 每秒打印一次写入的数据
    static uint16_t last_ctrl_word = 0xFFFF;
    if (ctx->cycle_count % 400 == 0 || ctrl_word != last_ctrl_word) {
        if (motor->mode == EC_MODE_CSV || motor->mode == EC_MODE_PV) {
            printf("M%d [写入PDO] ctrl=0x%04X tvel=%d\n", motor_idx, ctrl_word, motor->motion.target_vel);
        } else if (motor->mode == EC_MODE_PT || motor->mode == EC_MODE_CST) {
            printf("M%d [写入PDO] ctrl=0x%04X ttrq=%d\n", motor_idx, ctrl_word, motor->motion.target_torque);
        } else {
            printf("M%d [写入PDO] ctrl=0x%04X tpos=%d\n", motor_idx, ctrl_word, motor->motion.target_pos);
        }
        last_ctrl_word = ctrl_word;
    }

    // 每秒打印一次状态
    if (ctx->cycle_count % 400 == 0) {
        if (motor->mode == EC_MODE_PT) {
            printf("M%d [%s] Mode:PT SW:0x%04X Disp:%d Torq:%d TTorq:%d Vel:%d "
                    "OP:%d\n",
                motor_idx, ec_cia402_state_name(status_word), status_word, op_mode_disp, actual_torq,
                motor->motion.target_torque, actual_vel, slave_operational);
        } else if (motor->mode == EC_MODE_CST) {
            printf("M%d [%s] Mode:CST SW:0x%04X Disp:%d Torq:%d TTorq:%d Vel:%d "
                    "OP:%d\n",
                motor_idx, ec_cia402_state_name(status_word), status_word, op_mode_disp, actual_torq,
                motor->motion.target_torque, actual_vel, slave_operational);
        } else {
            printf("M%d [%s] Mode:%s SW:0x%04X Error:0x%04X Disp:%d Pos:%d Vel:%d "
                    "TPos:%d OP:%d\n",
                motor_idx, ec_cia402_state_name(status_word), ec_mode_name(motor->mode), status_word, error_code,
                op_mode_disp, actual_pos, actual_vel, motor->motion.target_pos, slave_operational);
        }
    }
}
