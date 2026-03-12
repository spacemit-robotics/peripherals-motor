/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file ethercat_cia402.c
 * @brief 通用 CiA402 EtherCAT 电机控制实现
 */

#include "ethercat_cia402.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/****************************************************************************/
/* 内部函数 */
/****************************************************************************/

uint64_t ec_get_time_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000000000ULL + ts.tv_nsec;
}

/****************************************************************************/
/* 主站管理 */
/****************************************************************************/

int ec_master_init(ec_master_ctx_t* ctx) {
    if (!ctx)
        return -1;

    memset(ctx, 0, sizeof(ec_master_ctx_t));
    ctx->enable_dc = 1;
    ctx->dc_assign_activate = 0x0300;
    ctx->cycle_time_ns = EC_DEFAULT_CYCLE_TIME_NS;
    ctx->running = 1;

    // 初始化电机默认参数
    for (int i = 0; i < EC_MAX_MOTORS; i++) {
        ctx->motors[i].cia402_state = EC_STATE_INIT;
        ctx->motors[i].mode = EC_MODE_CSP;
        ctx->motors[i].motion.direction = 1;
        ctx->motors[i].motion.profile_vel = 10000;  // 适合短距离运动
        ctx->motors[i].motion.profile_acc = 50000;  // 降低加速度
        ctx->motors[i].motion.profile_dec = 50000;  // 降低减速度
        ctx->motors[i].motion.torque_slope = 1000;  // 默认转矩斜率
        ctx->motors[i].motion.max_speed = 3000;     // 默认最大速度
    }

    return 0;
}

int ec_master_request(ec_master_ctx_t* ctx, unsigned int master_idx) {
    if (!ctx)
        return -1;

    ctx->master = ecrt_request_master(master_idx);
    if (!ctx->master) {
        fprintf(stderr, "错误: 无法请求 EtherCAT 主站 %u!\n", master_idx);
        return -1;
    }
    printf("主站请求成功\n");
    return 0;
}

int ec_domain_create(ec_master_ctx_t* ctx) {
    if (!ctx || !ctx->master)
        return -1;

    ctx->domain = ecrt_master_create_domain(ctx->master);
    if (!ctx->domain) {
        fprintf(stderr, "错误: 无法创建域!\n");
        return -1;
    }
    printf("域创建成功\n");
    return 0;
}

int ec_slave_configure(
    ec_master_ctx_t* ctx, int motor_idx, uint16_t alias, uint16_t position, const ec_slave_driver_t* driver) {
    if (!ctx || !ctx->master || motor_idx < 0 || motor_idx >= EC_MAX_MOTORS || !driver)
        return -1;

    ec_motor_t* motor = &ctx->motors[motor_idx];

    // IGH 把从站推向 PREOP 状态
    motor->sc = ecrt_master_slave_config(ctx->master, alias, position, driver->vendor_id, driver->product_code);
    if (!motor->sc) {
        fprintf(stderr, "错误: 无法配置从站 M%d (alias=%u, pos=%u)!\n", motor_idx, alias, position);
        return -1;
    }

    motor->vendor_id = driver->vendor_id;
    motor->product_code = driver->product_code;
    motor->alias = alias;
    motor->position = position;

    // 配置 PDO
    if (driver->sync_info) {
        printf("M%d: 正在配置 SM/PDO (Syncs via EC_END)\n", motor_idx);
        if (ecrt_slave_config_pdos(motor->sc, EC_END, driver->sync_info)) {
            fprintf(stderr, "警告: M%d PDO 配置失败 (Invalid argument), 将使用默认映射\n", motor_idx);
        } else {
            printf("M%d: PDO 配置成功\n", motor_idx);
        }
    }

    // 调用从站特定配置
    if (driver->config_slave) {
        if (driver->config_slave(ctx, motor_idx) < 0) {
            fprintf(stderr, "警告: M%d 从站配置回调失败\n", motor_idx);
        }
    }

    // 配置 DC 同步
    if (ctx->enable_dc) {
        ecrt_slave_config_dc(motor->sc, ctx->dc_assign_activate, ctx->cycle_time_ns, 0, 0, 0);
    }

    if (motor_idx >= ctx->motor_count) {
        ctx->motor_count = motor_idx + 1;
    }

    return 0;
}

int ec_master_activate(ec_master_ctx_t* ctx) {
    if (!ctx || !ctx->master)
        return -1;

    // 选择 DC 参考时钟
    if (ctx->enable_dc && ctx->motor_count > 0 && ctx->motors[0].sc) {
        if (ecrt_master_select_reference_clock(ctx->master, ctx->motors[0].sc)) {
            fprintf(stderr, "警告: 选择 DC 参考时钟失败\n");
        } else {
            printf("DC 参考时钟: 从站 M0\n");
        }
    }

    // 激活主站
    if (ecrt_master_activate(ctx->master)) {
        fprintf(stderr, "错误: 无法激活主站!\n");
        return -1;
    }
    printf("主站激活成功\n");

    // 获取域数据指针
    ctx->domain_pd = ecrt_domain_data(ctx->domain);
    if (!ctx->domain_pd) {
        fprintf(stderr, "错误: 无法获取域数据指针!\n");
        return -1;
    }
    printf("域数据指针获取成功\n");

    return 0;
}

void ec_master_release(ec_master_ctx_t* ctx) {
    if (ctx && ctx->master) {
        ecrt_release_master(ctx->master);
        ctx->master = NULL;
        printf("主站已释放\n");
    }
}

/****************************************************************************/
/* 状态检查 */
/****************************************************************************/

void ec_check_master_state(ec_master_ctx_t* ctx) {
    if (!ctx || !ctx->master)
        return;

    ec_master_state_t ms;
    ecrt_master_state(ctx->master, &ms);

    if (ms.slaves_responding != ctx->master_state.slaves_responding) {
        printf("从站响应数: %u\n", ms.slaves_responding);
    }
    if (ms.al_states != ctx->master_state.al_states) {
        printf("AL 状态: 0x%02X", ms.al_states);
        switch (ms.al_states) {
            case 0x01:
                printf(" (INIT)");
                break;
            case 0x02:
                printf(" (PREOP)");
                break;
            case 0x04:
                printf(" (SAFEOP)");
                break;
            case 0x08:
                printf(" (OP)");
                break;
        }
        printf("\n");
    }
    if (ms.link_up != ctx->master_state.link_up) {
        printf("链路状态: %s\n", ms.link_up ? "UP" : "DOWN");
    }
    ctx->master_state = ms;
}

void ec_check_domain_state(ec_master_ctx_t* ctx) {
    if (!ctx || !ctx->domain)
        return;

    ec_domain_state_t ds;
    ecrt_domain_state(ctx->domain, &ds);

    if (ds.working_counter != ctx->domain_state.working_counter) {
        printf("域: WC %u\n", ds.working_counter);
    }
    if (ds.wc_state != ctx->domain_state.wc_state) {
        printf("域: 状态 %u\n", ds.wc_state);
    }
    ctx->domain_state = ds;
}

void ec_check_slave_state(ec_master_ctx_t* ctx, int motor_idx) {
    if (!ctx || motor_idx < 0 || motor_idx >= ctx->motor_count)
        return;

    ec_motor_t* motor = &ctx->motors[motor_idx];

    if (motor->sc_state.al_state != motor->last_printed_state.al_state) {
        printf("M%d: AL状态 0x%02X", motor_idx, motor->sc_state.al_state);
        switch (motor->sc_state.al_state) {
            case 1:
                printf(" (INIT)\n");
                break;
            case 2:
                printf(" (PREOP)\n");
                break;
            case 4:
                printf(" (SAFEOP)\n");
                break;
            case 8:
                printf(" (OP)\n");
                break;
            default:
                printf("\n");
                break;
        }
    }
    if (motor->sc_state.online != motor->last_printed_state.online) {
        printf("M%d: %s\n", motor_idx, motor->sc_state.online ? "在线" : "离线");
    }
    if (motor->sc_state.operational != motor->last_printed_state.operational) {
        printf("M%d: %s\n", motor_idx, motor->sc_state.operational ? "已进入OP状态!" : "未进入OP状态");
    }
    motor->last_printed_state = motor->sc_state;
}

/****************************************************************************/
/* CiA402 状态机 */
/****************************************************************************/

const char* ec_cia402_state_name(uint16_t status_word) {
    uint16_t state = status_word & EC_STATUS_MASK;
    if (status_word & EC_STATUS_FAULT)
        return "FAULT";
    if (state == 0x0000)
        return "NOT_READY_TO_SWITCH_ON";
    if (state == 0x0040)
        return "SWITCH_ON_DISABLED";
    if (state == 0x0021)
        return "READY_TO_SWITCH_ON";
    if (state == 0x0023)
        return "SWITCHED_ON";
    if (state == 0x0027)
        return "OPERATION_ENABLED";
    if (state == 0x0007)
        return "QUICK_STOP_ACTIVE";
    return "UNKNOWN";
}

const char* ec_mode_name(ec_app_mode_t mode) {
    static const char* names[] = {"PP", "PV", "PT", "HM", "CSP", "CSV", "CST"};
    if (mode >= 0 && mode <= EC_MODE_CST)
        return names[mode];
    return "UNKNOWN";
}

int8_t ec_mode_to_op_mode(ec_app_mode_t mode) {
    switch (mode) {
        case EC_MODE_PP:
            return EC_OP_MODE_PP;
        case EC_MODE_PV:
            return EC_OP_MODE_PV;
        case EC_MODE_PT:
            return EC_OP_MODE_PT;
        case EC_MODE_HM:
            return EC_OP_MODE_HM;
        case EC_MODE_CSP:
            return EC_OP_MODE_CSP;
        case EC_MODE_CSV:
            return EC_OP_MODE_CSV;
        case EC_MODE_CST:
            return EC_OP_MODE_CST;
        default:
            return EC_OP_MODE_CSP;
    }
}

uint16_t ec_cia402_process(
    ec_cia402_state_t* state, int* fault_reset_count, uint16_t status_word, int slave_operational) {
    uint16_t ctrl_word = 0;
    uint16_t st = status_word & EC_STATUS_MASK;

    // 等待从站进入 OP 状态
    if (!slave_operational) {
        *state = EC_STATE_WAIT_OP;
        return 0x0000;
    }

    // 检查故障状态
    if (status_word & EC_STATUS_FAULT) {
        if (*state != EC_STATE_FAULT_RESET) {
            printf("检测到故障! 状态字: 0x%04X\n", status_word);
            *state = EC_STATE_FAULT_RESET;
            *fault_reset_count = 0;
        }
    }

    // 检查是否意外掉出运行状态 (例如状态变为 0x0000)
    // 仅在逻辑上认为已进入 RUNNING 状态后，且从站 operational 为真时进行监控
    if (*state == EC_STATE_RUNNING && slave_operational) {
        if (st == 0x0000 || (status_word & EC_STATUS_MASK) == 0x0040) {
            printf("[CiA402] 检测到意外运行中断 (SW=0x%04X), "
                    "正在重置状态机为重启准备...\n",
                status_word);
            *state = EC_STATE_SWITCH_ON_DISABLED;
        }
    }

    switch (*state) {
        case EC_STATE_INIT:
        case EC_STATE_WAIT_OP:
            *state = EC_STATE_SWITCH_ON_DISABLED;
            ctrl_word = EC_CTRL_SHUTDOWN;
            printf("[CiA402] 状态: INIT/WAIT_OP -> SWITCH_ON_DISABLED, 发送 ctrl=0x%04X\n", ctrl_word);
            break;

        case EC_STATE_FAULT_RESET:
            if (*fault_reset_count < 100) {
                ctrl_word = 0x0000;
                (*fault_reset_count)++;
            } else if (*fault_reset_count < 200) {
                ctrl_word = EC_CTRL_FAULT_RESET;
                (*fault_reset_count)++;
            } else {
                if (!(status_word & EC_STATUS_FAULT)) {
                    printf("故障已清除\n");
                    *state = EC_STATE_SWITCH_ON_DISABLED;
                } else {
                    *fault_reset_count = 0;
                }
            }
            break;

        case EC_STATE_SWITCH_ON_DISABLED:
            if (status_word == 0x0000) {
                ctrl_word = EC_CTRL_SHUTDOWN;
                static int warn_count = 0;
                if (warn_count++ % 400 == 0) {
                    printf("[CiA402] 警告: SW=0x0000 持续中, 发送 ctrl=0x%04X "
                            "(尝试推动状态转换)\n",
                        ctrl_word);
                }
            } else if ((st & 0x004F) == 0x0040) {
                ctrl_word = EC_CTRL_SHUTDOWN;
                *state = EC_STATE_READY_TO_SWITCH_ON;
                printf("[CiA402] 状态机: SWITCH_ON_DISABLED -> READY_TO_SWITCH_ON\n");
            } else {
                ctrl_word = EC_CTRL_SHUTDOWN;
            }
            break;

        case EC_STATE_READY_TO_SWITCH_ON:
            ctrl_word = EC_CTRL_SHUTDOWN;
            if ((st & 0x006F) == 0x0021) {
                *state = EC_STATE_SWITCHED_ON;
                printf("状态机: READY_TO_SWITCH_ON -> SWITCHED_ON\n");
            }
            break;

        case EC_STATE_SWITCHED_ON:
            ctrl_word = EC_CTRL_SWITCH_ON;
            if ((st & 0x006F) == 0x0023) {
                *state = EC_STATE_OPERATION_ENABLED;
                printf("状态机: SWITCHED_ON -> OPERATION_ENABLED\n");
            }
            break;

        case EC_STATE_OPERATION_ENABLED:
            ctrl_word = EC_CTRL_ENABLE_OPERATION;
            if ((st & 0x006F) == 0x0027) {
                *state = EC_STATE_RUNNING;
                printf("状态机: OPERATION_ENABLED -> RUNNING (电机已使能!)\n");
            }
            break;

        case EC_STATE_RUNNING:
            ctrl_word = EC_CTRL_ENABLE_OPERATION;
            break;
    }

    return ctrl_word;
}

/****************************************************************************/
/* 周期任务 */
/****************************************************************************/

void ec_cyclic_task(ec_master_ctx_t* ctx) {
    if (!ctx || !ctx->master || !ctx->domain)
        return;

    uint64_t now = ec_get_time_ns();

    // DC 同步
    if (ctx->enable_dc) {
        ecrt_master_application_time(ctx->master, now);
        if ((ctx->dc_sync_counter++ % 10) == 0) {
            ecrt_master_sync_reference_clock(ctx->master);
        }
        ecrt_master_sync_slave_clocks(ctx->master);
    }

    // 接收过程数据
    ecrt_master_receive(ctx->master);
    ecrt_domain_process(ctx->domain);

    // 检查状态 (每秒一次)
    if (ctx->cycle_count % 400 == 0) {
        ec_check_master_state(ctx);
        ec_check_domain_state(ctx);
    }

    // 更新从站状态
    for (int i = 0; i < ctx->motor_count; i++) {
        ecrt_slave_config_state(ctx->motors[i].sc, &ctx->motors[i].sc_state);
        if (ctx->cycle_count % 400 == 0) {
            ec_check_slave_state(ctx, i);
        }
    }

    ctx->cycle_count++;
}

/****************************************************************************/
/* 电机控制接口 */
/****************************************************************************/

void ec_set_target_position(ec_master_ctx_t* ctx, int motor_idx, int32_t position) {
    if (!ctx || motor_idx < 0 || motor_idx >= ctx->motor_count)
        return;
    ctx->motors[motor_idx].motion.target_pos = position;
}

void ec_set_target_velocity(ec_master_ctx_t* ctx, int motor_idx, int32_t velocity) {
    if (!ctx || motor_idx < 0 || motor_idx >= ctx->motor_count)
        return;
    ctx->motors[motor_idx].motion.target_vel = velocity;
}

void ec_set_target_torque(ec_master_ctx_t* ctx, int motor_idx, int32_t torque) {
    if (!ctx || motor_idx < 0 || motor_idx >= ctx->motor_count)
        return;
    ctx->motors[motor_idx].motion.target_torque = torque;
}

void ec_pp_move_to(ec_master_ctx_t* ctx, int motor_idx, int32_t position) {
    if (!ctx || motor_idx < 0 || motor_idx >= ctx->motor_count)
        return;
    ec_motor_t* motor = &ctx->motors[motor_idx];

    if (motor->mode == EC_MODE_PP && motor->cia402_state == EC_STATE_RUNNING) {
        motor->motion.target_pos = position;
        motor->motion.pp_state = 1;  // 触发运动
    }
}

void ec_hm_start(ec_master_ctx_t* ctx, int motor_idx) {
    if (!ctx || motor_idx < 0 || motor_idx >= ctx->motor_count)
        return;
    ec_motor_t* motor = &ctx->motors[motor_idx];

    if (motor->mode == EC_MODE_HM && motor->cia402_state == EC_STATE_RUNNING) {
        motor->motion.hm_state = 1;  // 触发回零
    }
}

int32_t ec_get_actual_position(ec_master_ctx_t* ctx, int motor_idx) {
    if (!ctx || !ctx->domain_pd || motor_idx < 0 || motor_idx >= ctx->motor_count)
        return 0;
    return EC_READ_S32(ctx->domain_pd + ctx->motors[motor_idx].off.actual_position);
}

int32_t ec_get_actual_velocity(ec_master_ctx_t* ctx, int motor_idx) {
    if (!ctx || !ctx->domain_pd || motor_idx < 0 || motor_idx >= ctx->motor_count)
        return 0;
    return EC_READ_S32(ctx->domain_pd + ctx->motors[motor_idx].off.actual_velocity);
}

uint16_t ec_get_status_word(ec_master_ctx_t* ctx, int motor_idx) {
    if (!ctx || !ctx->domain_pd || motor_idx < 0 || motor_idx >= ctx->motor_count)
        return 0;
    return EC_READ_U16(ctx->domain_pd + ctx->motors[motor_idx].off.status_word);
}

int ec_is_motor_enabled(ec_master_ctx_t* ctx, int motor_idx) {
    if (!ctx || motor_idx < 0 || motor_idx >= ctx->motor_count)
        return 0;
    return ctx->motors[motor_idx].cia402_state == EC_STATE_RUNNING;
}

void ec_set_motor_mode(ec_master_ctx_t* ctx, int motor_idx, ec_app_mode_t mode) {
    if (!ctx || motor_idx < 0 || motor_idx >= ctx->motor_count)
        return;
    ec_motor_t* motor = &ctx->motors[motor_idx];
    if (motor->mode != mode) {
        motor->mode = mode;
        motor->motion.pp_state = 0;
        motor->motion.hm_state = 0;
        motor->motion.pt_state = 0;
    }
}
