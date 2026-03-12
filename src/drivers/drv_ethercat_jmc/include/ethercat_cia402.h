/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file ethercat_cia402.h
 * @brief 通用 CiA402 EtherCAT 电机控制接口
 *
 * 基于 IGH EtherCAT Master，提供 CiA402 协议的通用电机控制功能
 * 支持模式: PP、PV、PT、HM、CSP、CSV、CST
 */

#ifndef ETHERCAT_CIA402_H
#define ETHERCAT_CIA402_H

#include <ecrt.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/****************************************************************************/
/* 常量定义 */
/****************************************************************************/

// 默认周期时间 (纳秒)
#define EC_DEFAULT_CYCLE_TIME_NS 2500000  // 2.5ms

// 最大电机数量
#define EC_MAX_MOTORS 10

// 操作模式定义 (CiA402 0x6060)
#define EC_OP_MODE_PP 1    // Profile Position
#define EC_OP_MODE_PV 3    // Profile Velocity
#define EC_OP_MODE_PT 4    // Profile Torque
#define EC_OP_MODE_HM 6    // Homing
#define EC_OP_MODE_CSP 8   // Cyclic Synchronous Position
#define EC_OP_MODE_CSV 9   // Cyclic Synchronous Velocity
#define EC_OP_MODE_CST 10  // Cyclic Synchronous Torque

// CiA402 状态机控制字
#define EC_CTRL_SHUTDOWN 0x0006
#define EC_CTRL_SWITCH_ON 0x0007
#define EC_CTRL_ENABLE_OPERATION 0x000F
#define EC_CTRL_DISABLE_VOLTAGE 0x0000
#define EC_CTRL_QUICK_STOP 0x0002
#define EC_CTRL_FAULT_RESET 0x0080

// CiA402 状态字掩码
#define EC_STATUS_NOT_READY 0x0000
#define EC_STATUS_SWITCH_ON_DISABLED 0x0040
#define EC_STATUS_READY_TO_SWITCH_ON 0x0021
#define EC_STATUS_SWITCHED_ON 0x0023
#define EC_STATUS_OPERATION_ENABLED 0x0027
#define EC_STATUS_FAULT 0x0008
#define EC_STATUS_MASK 0x006F

/****************************************************************************/
/* 类型定义 */
/****************************************************************************/

// 应用层模式枚举
typedef enum {
    EC_MODE_PP = 0,   // Profile Position
    EC_MODE_PV = 1,   // Profile Velocity
    EC_MODE_PT = 2,   // Profile Torque
    EC_MODE_HM = 3,   // Homing
    EC_MODE_CSP = 4,  // Cyclic Synchronous Position
    EC_MODE_CSV = 5,  // Cyclic Synchronous Velocity
    EC_MODE_CST = 6,  // Cyclic Synchronous Torque
} ec_app_mode_t;

// CiA402 状态机状态
typedef enum {
    EC_STATE_INIT,
    EC_STATE_WAIT_OP,
    EC_STATE_FAULT_RESET,
    EC_STATE_SWITCH_ON_DISABLED,
    EC_STATE_READY_TO_SWITCH_ON,
    EC_STATE_SWITCHED_ON,
    EC_STATE_OPERATION_ENABLED,
    EC_STATE_RUNNING
} ec_cia402_state_t;

// PDO 偏移量结构 (通用)
typedef struct {
    // RxPDO (主站 -> 从站)
    unsigned int ctrl_word;
    unsigned int op_mode;
    unsigned int target_position;
    unsigned int target_velocity;
    unsigned int profile_velocity;
    unsigned int profile_accel;
    unsigned int profile_decel;
    unsigned int homing_method;
    unsigned int homing_accel;
    unsigned int homing_speed_switch;
    unsigned int homing_speed_zero;
    unsigned int homing_offset;
    unsigned int target_torque;
    unsigned int torque_slope;
    unsigned int max_speed;
    unsigned int touch_probe_func;

    // TxPDO (从站 -> 主站)
    unsigned int status_word;
    unsigned int op_mode_display;
    unsigned int actual_position;
    unsigned int actual_velocity;
    unsigned int actual_torque;
    unsigned int error_code;
    unsigned int touch_probe_status;
    unsigned int touch_probe_pos1;
    unsigned int touch_probe_pos2;
    unsigned int digital_inputs;
} ec_pdo_offset_t;

// 电机运动参数
typedef struct {
    int32_t start_pos;      // 起始物理位置 (轴脉冲)
    int32_t app_start_pos;  // 应用层起始锚点 (轴脉冲)
    int32_t target_pos;     // 目标位置
    int32_t target_vel;     // 目标速度
    int32_t target_torque;  // 目标转矩
    int direction;          // 运动方向 (1/-1)
    int start_pos_set;      // 起始位置是否已设置

    // Profile 模式参数
    uint32_t profile_vel;   // 轮廓速度
    uint32_t profile_acc;   // 加速度
    uint32_t profile_dec;   // 减速度
    uint32_t torque_slope;  // 转矩斜率
    uint32_t max_speed;     // 最大速度
    int pp_state;           // PP模式状态机
    int pp_wait_count;      // PP模式等待计数
    int pt_state;           // PT模式状态机

    // HM 模式参数
    int8_t homing_method;
    uint32_t homing_accel;
    uint32_t homing_speed_switch;
    uint32_t homing_speed_zero;
    int32_t homing_offset;
    int hm_state;  // HM模式状态机
} ec_motion_params_t;

// 单个电机结构
typedef struct {
    ec_slave_config_t* sc;
    ec_slave_config_state_t sc_state;
    ec_slave_config_state_t last_printed_state;

    ec_pdo_offset_t off;
    ec_cia402_state_t cia402_state;
    int fault_reset_count;

    ec_app_mode_t mode;
    ec_motion_params_t motion;

    // 从站信息
    uint32_t vendor_id;
    uint32_t product_code;
    uint16_t alias;
    uint16_t position;
} ec_motor_t;

// 主站配置结构
typedef struct {
    ec_master_t* master;
    ec_master_state_t master_state;

    ec_domain_t* domain;
    ec_domain_state_t domain_state;
    uint8_t* domain_pd;

    ec_motor_t motors[EC_MAX_MOTORS];
    int motor_count;

    // DC 配置
    int enable_dc;
    uint16_t dc_assign_activate;
    uint32_t cycle_time_ns;
    unsigned int dc_sync_counter;

    // 运行状态
    volatile int running;
    uint32_t cycle_count;
} ec_master_ctx_t;

// 从站配置回调函数类型
typedef int (*ec_slave_config_cb)(ec_master_ctx_t* ctx, int motor_idx);

// 从站驱动接口
typedef struct {
    const char* name;
    uint32_t vendor_id;
    uint32_t product_code;

    // PDO 配置
    ec_sync_info_t* sync_info;
    int pdo_entry_count;

    // 回调函数
    ec_slave_config_cb config_slave;  // 配置从站 (SDO等)
    ec_slave_config_cb register_pdo;  // 注册 PDO 条目
} ec_slave_driver_t;

/****************************************************************************/
/* 函数声明 */
/****************************************************************************/

/**
 * @brief 初始化主站上下文
 * @param ctx 主站上下文指针
 * @return 0 成功, -1 失败
 */
int ec_master_init(ec_master_ctx_t* ctx);

/**
 * @brief 请求 EtherCAT 主站
 * @param ctx 主站上下文指针
 * @param master_idx 主站索引 (通常为 0)
 * @return 0 成功, -1 失败
 */
int ec_master_request(ec_master_ctx_t* ctx, unsigned int master_idx);

/**
 * @brief 创建域
 * @param ctx 主站上下文指针
 * @return 0 成功, -1 失败
 */
int ec_domain_create(ec_master_ctx_t* ctx);

/**
 * @brief 配置从站
 * @param ctx 主站上下文指针
 * @param motor_idx 电机索引
 * @param alias 从站别名
 * @param position 从站位置
 * @param driver 从站驱动
 * @return 0 成功, -1 失败
 */
int ec_slave_configure(
    ec_master_ctx_t* ctx, int motor_idx, uint16_t alias, uint16_t position, const ec_slave_driver_t* driver);

/**
 * @brief 激活主站
 * @param ctx 主站上下文指针
 * @return 0 成功, -1 失败
 */
int ec_master_activate(ec_master_ctx_t* ctx);

/**
 * @brief 释放主站
 * @param ctx 主站上下文指针
 */
void ec_master_release(ec_master_ctx_t* ctx);

/**
 * @brief 执行周期任务 (接收/处理/发送)
 * @param ctx 主站上下文指针
 */
void ec_cyclic_task(ec_master_ctx_t* ctx);

/**
 * @brief 处理 CiA402 状态机
 * @param state 当前状态指针
 * @param fault_reset_count 故障复位计数指针
 * @param status_word 状态字
 * @param slave_operational 从站是否 OP
 * @return 控制字
 */
uint16_t ec_cia402_process(
    ec_cia402_state_t* state, int* fault_reset_count, uint16_t status_word, int slave_operational);

/**
 * @brief 获取 CiA402 状态名称
 * @param status_word 状态字
 * @return 状态名称字符串
 */
const char* ec_cia402_state_name(uint16_t status_word);

/**
 * @brief 获取应用模式名称
 * @param mode 应用模式
 * @return 模式名称字符串
 */
const char* ec_mode_name(ec_app_mode_t mode);

/**
 * @brief 将应用模式转换为 CiA402 操作模式值
 * @param mode 应用模式
 * @return CiA402 操作模式值
 */
int8_t ec_mode_to_op_mode(ec_app_mode_t mode);

/**
 * @brief 检查主站状态
 * @param ctx 主站上下文指针
 */
void ec_check_master_state(ec_master_ctx_t* ctx);

/**
 * @brief 检查域状态
 * @param ctx 主站上下文指针
 */
void ec_check_domain_state(ec_master_ctx_t* ctx);

/**
 * @brief 检查从站状态
 * @param ctx 主站上下文指针
 * @param motor_idx 电机索引
 */
void ec_check_slave_state(ec_master_ctx_t* ctx, int motor_idx);

/**
 * @brief 获取当前时间 (纳秒)
 * @return 当前时间
 */
uint64_t ec_get_time_ns(void);

/**
 * @brief 设置电机目标位置 (CSP/PP 模式)
 * @param ctx 主站上下文指针
 * @param motor_idx 电机索引
 * @param position 目标位置
 */
void ec_set_target_position(ec_master_ctx_t* ctx, int motor_idx, int32_t position);

/**
 * @brief 设置电机目标速度 (CSV/PV 模式)
 * @param ctx 主站上下文指针
 * @param motor_idx 电机索引
 * @param velocity 目标速度
 */
void ec_set_target_velocity(ec_master_ctx_t* ctx, int motor_idx, int32_t velocity);

/**
 * @brief 设置电机目标转矩 (CST/PT 模式)
 * @param ctx 主站上下文指针
 * @param motor_idx 电机索引
 * @param torque 目标转矩
 */
void ec_set_target_torque(ec_master_ctx_t* ctx, int motor_idx, int32_t torque);

/**
 * @brief 触发 PP 模式运动
 * @param ctx 主站上下文指针
 * @param motor_idx 电机索引
 * @param position 目标位置
 */
void ec_pp_move_to(ec_master_ctx_t* ctx, int motor_idx, int32_t position);

/**
 * @brief 触发 HM 模式回零
 * @param ctx 主站上下文指针
 * @param motor_idx 电机索引
 */
void ec_hm_start(ec_master_ctx_t* ctx, int motor_idx);

/**
 * @brief 获取电机实际位置
 * @param ctx 主站上下文指针
 * @param motor_idx 电机索引
 * @return 实际位置
 */
int32_t ec_get_actual_position(ec_master_ctx_t* ctx, int motor_idx);

/**
 * @brief 获取电机实际速度
 * @param ctx 主站上下文指针
 * @param motor_idx 电机索引
 * @return 实际速度
 */
int32_t ec_get_actual_velocity(ec_master_ctx_t* ctx, int motor_idx);

/**
 * @brief 获取电机状态字
 * @param ctx 主站上下文指针
 * @param motor_idx 电机索引
 * @return 状态字
 */
uint16_t ec_get_status_word(ec_master_ctx_t* ctx, int motor_idx);

/**
 * @brief 检查电机是否已使能
 * @param ctx 主站上下文指针
 * @param motor_idx 电机索引
 * @return 1 已使能, 0 未使能
 */
int ec_is_motor_enabled(ec_master_ctx_t* ctx, int motor_idx);

/**
 * @brief 设置电机运行模式
 * @param ctx 主站上下文指针
 * @param motor_idx 电机索引
 * @param mode 运行模式
 */
void ec_set_motor_mode(ec_master_ctx_t* ctx, int motor_idx, ec_app_mode_t mode);

#ifdef __cplusplus
}
#endif

#endif /* ETHERCAT_CIA402_H */
