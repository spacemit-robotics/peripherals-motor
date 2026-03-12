/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file test_ethercat_jmc.c
 * @brief JMC IHSS42-EC EtherCAT 电机控制测试程序
 *
 * 使用示例:
 *   ./test_ethercat_jmc --motors 1 --mode csp --dc 0
 *   ./test_ethercat_jmc --motors 2 --mode pp --dc 1
 */
#include <getopt.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "ethercat_cia402.h"
#include "jmc_ihss42.h"


/****************************************************************************/
/* 全局变量 */
/****************************************************************************/

static ec_master_ctx_t g_ctx;

/****************************************************************************/
/* 信号处理 */
/****************************************************************************/

static void signal_handler(int sig) {
    (void)sig;
    g_ctx.running = 0;
}

/****************************************************************************/
/* 辅助函数 */
/****************************************************************************/

static void usage(const char* prog) {
    printf("用法: %s [选项]\n", prog);
    printf("选项:\n");
    printf("  --motors N    电机数量 (默认 1, 最大 %d)\n", EC_MAX_MOTORS);
    printf("  --mode MODE   运行模式: pp|pv|pt|hm|csp|csv|cst (默认 csp)\n");
    printf("  --dc 0|1      是否启用 DC 同步 (默认 1)\n");
    printf("  --cycle N     周期时间 (纳秒, 默认 %d)\n", EC_DEFAULT_CYCLE_TIME_NS);
    printf("  --help        显示帮助信息\n");
    printf("\n模式说明:\n");
    printf("  pp  - Profile Position (轮廓位置模式)\n");
    printf("  pv  - Profile Velocity (轮廓速度模式)\n");
    printf("  pt  - Profile Torque (轮廓转矩模式)\n");
    printf("  hm  - Homing (回零模式)\n");
    printf("  csp - Cyclic Synchronous Position (循环同步位置)\n");
    printf("  csv - Cyclic Synchronous Velocity (循环同步速度)\n");
    printf("  cst - Cyclic Synchronous Torque (循环同步转矩)\n");
}

static ec_app_mode_t parse_mode(const char* str) {
    if (!strcasecmp(str, "pp"))
        return EC_MODE_PP;
    if (!strcasecmp(str, "pv"))
        return EC_MODE_PV;
    if (!strcasecmp(str, "pt"))
        return EC_MODE_PT;
    if (!strcasecmp(str, "hm"))
        return EC_MODE_HM;
    if (!strcasecmp(str, "csp"))
        return EC_MODE_CSP;
    if (!strcasecmp(str, "csv"))
        return EC_MODE_CSV;
    if (!strcasecmp(str, "cst"))
        return EC_MODE_CST;
    return EC_MODE_CSP;
}

static void sleep_until(uint64_t target_ns) {
    uint64_t now = ec_get_time_ns();
    if (now < target_ns) {
        struct timespec ts;
        uint64_t sleep_time = target_ns - now;
        ts.tv_sec = sleep_time / 1000000000ULL;
        ts.tv_nsec = sleep_time % 1000000000ULL;
        nanosleep(&ts, NULL);
    }
}

/****************************************************************************/
/* 轨迹规划示例 */
/****************************************************************************/

// CSP 模式轨迹规划 - 简单往复运动
static void csp_trajectory_update(ec_master_ctx_t* ctx, int motor_idx, int32_t amplitude, int32_t step) {
    ec_motor_t* motor = &ctx->motors[motor_idx];

    if (motor->mode != EC_MODE_CSP || motor->cia402_state != EC_STATE_RUNNING || !motor->motion.start_pos_set)
        return;

    motor->motion.target_pos += motor->motion.direction * step;

    if (motor->motion.target_pos >= motor->motion.start_pos + amplitude) {
        motor->motion.direction = -1;
    } else if (motor->motion.target_pos <= motor->motion.start_pos - amplitude) {
        motor->motion.direction = 1;
    }
}

// PP 模式轨迹规划 - 定点往复运动
static void pp_trajectory_update(ec_master_ctx_t* ctx, int motor_idx, int32_t distance) {
    ec_motor_t* motor = &ctx->motors[motor_idx];

    if (motor->mode != EC_MODE_PP || motor->cia402_state != EC_STATE_RUNNING || !motor->motion.start_pos_set)
        return;

    // 在起始位置和 +distance 之间往复
    if (motor->motion.target_pos == motor->motion.start_pos) {
        ec_pp_move_to(ctx, motor_idx, motor->motion.start_pos + distance);
    } else {
        ec_pp_move_to(ctx, motor_idx, motor->motion.start_pos);
    }
}

// CSV 模式轨迹规划 - 速度斜坡
static void csv_trajectory_update(ec_master_ctx_t* ctx, int motor_idx, int32_t max_vel, int32_t accel) {
    ec_motor_t* motor = &ctx->motors[motor_idx];

    if (motor->mode != EC_MODE_CSV || motor->cia402_state != EC_STATE_RUNNING || !motor->motion.start_pos_set)
        return;

    // 简单速度斜坡
    motor->motion.target_vel += motor->motion.direction * accel;

    if (motor->motion.target_vel >= max_vel) {
        motor->motion.direction = -1;
    } else if (motor->motion.target_vel <= -max_vel) {
        motor->motion.direction = 1;
    }
}

// PV 模式轨迹规划 - 速度阶跃
static void pv_trajectory_update(
    ec_master_ctx_t* ctx, int motor_idx, int32_t target_vel, uint32_t accel, uint32_t decel) {
    ec_motor_t* motor = &ctx->motors[motor_idx];

    if (motor->mode != EC_MODE_PV || motor->cia402_state != EC_STATE_RUNNING || !motor->motion.start_pos_set)
        return;

    // 简单的速度往复
    if (motor->motion.target_vel == 0) {
        motor->motion.target_vel = target_vel;
    } else if (motor->motion.target_vel == target_vel) {
        motor->motion.target_vel = -target_vel;
    } else {
        motor->motion.target_vel = target_vel;
    }

    motor->motion.profile_acc = accel;
    motor->motion.profile_dec = decel;

    printf("M%d PV 轨迹更新: 目标速度=%d, 加速度=%u, 减速度=%u\n", motor_idx, motor->motion.target_vel, accel, decel);
}

// PT 模式轨迹规划 - 转矩阶跃
static void pt_trajectory_update_safe(
    ec_master_ctx_t* ctx, int motor_idx, int32_t target_torque, uint32_t slope, int interval_cycles) {
    ec_motor_t* motor = &ctx->motors[motor_idx];

    // 基础状态检查
    if (motor->mode != EC_MODE_PT || motor->cia402_state != EC_STATE_RUNNING || !motor->motion.start_pos_set)
        return;

    // 使用内部计数器控制翻转频率
    static uint32_t cycle_counter = 0;
    cycle_counter++;

    // 如果目标转矩还未设置（为0），则立即设置目标转矩并重置计数器
    if (cycle_counter >= interval_cycles || motor->motion.target_torque == 0) {
        cycle_counter = 0;  // 重置计数器

        if (motor->motion.target_torque == target_torque) {
            motor->motion.target_torque = -target_torque;  // 翻转为负
        } else {
            motor->motion.target_torque = target_torque;  // 翻转为正
        }

        motor->motion.torque_slope = slope;

        printf("M%d PT 状态切换: 目标转矩设为 %d, 斜率 %u (安全速度限制: %u)\n", motor_idx, motor->motion.target_torque,
            slope, motor->motion.max_speed);
    }
}

// CST 模式轨迹规划 - 周期同步转矩阶跃
static void cst_trajectory_update_safe(
    ec_master_ctx_t* ctx, int motor_idx, int32_t target_torque, uint32_t max_speed, int interval_cycles) {
    ec_motor_t* motor = &ctx->motors[motor_idx];

    // 基础状态检查
    if (motor->mode != EC_MODE_CST || motor->cia402_state != EC_STATE_RUNNING || !motor->motion.start_pos_set)
        return;

    // 使用内部计数器控制翻转频率
    static uint32_t cycle_counter = 0;
    cycle_counter++;

    // 如果目标转矩还未设置（为0），则立即设置目标转矩并重置计数器
    if (cycle_counter >= interval_cycles || motor->motion.target_torque == 0) {
        cycle_counter = 0;  // 重置计数器

        if (motor->motion.target_torque == target_torque) {
            motor->motion.target_torque = -target_torque;  // 翻转为负
        } else {
            motor->motion.target_torque = target_torque;  // 翻转为正
        }

        motor->motion.max_speed = max_speed;

        printf("M%d CST 状态切换: 目标转矩设为 %d (安全速度限制: %u)\n", motor_idx, motor->motion.target_torque,
            max_speed);
    }
}

// HM 模式轨迹规划 - 启动一次性回零
static void hm_trajectory_update(ec_master_ctx_t* ctx, int motor_idx, int8_t method, uint32_t speed_switch,
    uint32_t speed_zero, uint32_t accel, int32_t offset) {
    ec_motor_t* motor = &ctx->motors[motor_idx];

    if (motor->mode != EC_MODE_HM || motor->cia402_state != EC_STATE_RUNNING ||
        motor->motion.hm_state != 0)  // 仅在空闲时启动
        return;

    motor->motion.homing_method = method;
    motor->motion.homing_speed_switch = speed_switch;
    motor->motion.homing_speed_zero = speed_zero;
    motor->motion.homing_accel = accel;
    motor->motion.homing_offset = offset;

    motor->motion.hm_state = 1;  // 触发启动
    printf("M%d HM 任务已分发: 方式=%d\n", motor_idx, method);
}

/****************************************************************************/
/* 主函数 */
/****************************************************************************/

int main(int argc, char** argv) {
    int motor_count = 1;
    ec_app_mode_t default_mode = EC_MODE_CSP;
    int enable_dc = 1;
    uint32_t cycle_time_ns = EC_DEFAULT_CYCLE_TIME_NS;

    // 解析命令行参数
    static struct option long_opts[] = {{"motors", required_argument, 0, 'm'}, {"mode", required_argument, 0, 'o'},
        {"dc", required_argument, 0, 'd'}, {"cycle", required_argument, 0, 'c'}, {"help", no_argument, 0, 'h'},
        {0, 0, 0, 0}};

    int opt;
    while ((opt = getopt_long(argc, argv, "m:o:d:c:h", long_opts, NULL)) != -1) {
        switch (opt) {
            case 'm':
                motor_count = atoi(optarg);
                break;
            case 'o':
                default_mode = parse_mode(optarg);
                break;
            case 'd':
                enable_dc = atoi(optarg) ? 1 : 0;
                break;
            case 'c':
                cycle_time_ns = (uint32_t)strtoul(optarg, NULL, 0);
                break;
            case 'h':
            default:
                usage(argv[0]);
                return 0;
        }
    }

    // 参数检查
    if (motor_count <= 0 || motor_count > EC_MAX_MOTORS) {
        fprintf(stderr, "错误: 电机数量 %d 非法 (范围 1..%d)\n", motor_count, EC_MAX_MOTORS);
        return -1;
    }

    printf("========================================\n");
    printf("JMC IHSS42-EC EtherCAT 测试程序\n");
    printf("========================================\n");
    printf("配置: 电机数量=%d 模式=%s DC=%d 周期=%uns\n", motor_count, ec_mode_name(default_mode), enable_dc,
        cycle_time_ns);
    printf("========================================\n\n");

    // 注册信号处理
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // 初始化主站上下文
    if (ec_master_init(&g_ctx) < 0) {
        fprintf(stderr, "错误: 初始化主站上下文失败\n");
        return -1;
    }

    g_ctx.enable_dc = enable_dc;
    g_ctx.cycle_time_ns = cycle_time_ns;

    // 请求主站
    if (ec_master_request(&g_ctx, 0) < 0) {
        return -1;
    }

    // 创建域
    if (ec_domain_create(&g_ctx) < 0) {
        goto err_release;
    }

    // 配置从站
    for (int i = 0; i < motor_count; i++) {
        // 设置 Profile 参数 (在配置从站之前)
        jmc_ihss42_set_profile_params(&g_ctx, i, 100000, 500000, 500000);

        // 配置从站 (alias=0, position=i)
        if (jmc_ihss42_configure(&g_ctx, i, 0, i, default_mode) < 0) {
            goto err_release;
        }

        // 注册 PDO 条目
        if (jmc_ihss42_register_pdo(&g_ctx, i) < 0) {
            goto err_release;
        }
    }
    printf("从站配置完成: %d 台\n", motor_count);

    // 激活主站
    if (ec_master_activate(&g_ctx) < 0) {
        goto err_release;
    }

    printf("\n开始周期任务 (按 Ctrl+C 退出)...\n");
    printf("等待从站进入 OP 状态...\n\n");

    // 主循环
    uint64_t next_cycle = ec_get_time_ns();
    uint32_t main_cycle_count = 0;

    // 轨迹参数
    int32_t csp_amplitude = 50000;
    int32_t csp_step = 50;
    int csp_update_interval = 10;
    int pp_update_interval = 1200;  // 3秒 (400Hz * 3)
    int32_t pp_distance = 10000;
    int32_t csv_max_vel = 50000;
    int32_t csv_accel = 100;
    int csv_update_interval = 10;

    int32_t pv_target_vel = 50000;
    uint32_t pv_accel = 100000;
    uint32_t pv_decel = 100000;
    int pv_update_interval = 800;  // 2秒 (400Hz * 2)

    int8_t hm_method = 35;  // 35: 当前位置回零 (或 17, 18, 33, 34 等)
    uint32_t hm_speed_switch = 10000;
    uint32_t hm_speed_zero = 5000;
    uint32_t hm_accel = 50000;
    int32_t hm_offset = 0;
    int hm_update_interval = 2000;   // 每 5 秒尝试(仅空闲触发)
    int32_t pt_target_torque = 500;  // 这里的单位取决于驱动器配置，通常是千分之一额定转矩
    uint32_t pt_slope = 1000;
    int pt_update_interval = 800;  // 2秒 (800 * 2.5ms = 2000ms)

    int32_t cst_target_torque = 500;
    uint32_t cst_max_speed = 50000;
    int cst_update_interval = 800;

    while (g_ctx.running) {
        // 等待下一个周期
        sleep_until(next_cycle);
        next_cycle += cycle_time_ns;

        // 执行通用周期任务 (接收/状态检查)
        ec_cyclic_task(&g_ctx);

        // 执行各电机周期任务
        for (int i = 0; i < motor_count; i++) {
            jmc_ihss42_cyclic_task(&g_ctx, i);
        }

        // 轨迹规划更新
        if (main_cycle_count % csp_update_interval == 0) {
            for (int i = 0; i < motor_count; i++) {
                csp_trajectory_update(&g_ctx, i, csp_amplitude, csp_step);
            }
        }

        if (main_cycle_count % pp_update_interval == 0 && main_cycle_count > 0) {
            for (int i = 0; i < motor_count; i++) {
                pp_trajectory_update(&g_ctx, i, pp_distance);
            }
        }

        if (main_cycle_count % csv_update_interval == 0) {
            for (int i = 0; i < motor_count; i++) {
                csv_trajectory_update(&g_ctx, i, csv_max_vel, csv_accel);
            }
        }

        if (main_cycle_count % pv_update_interval == 0 && main_cycle_count > 0) {
            for (int i = 0; i < motor_count; i++) {
                pv_trajectory_update(&g_ctx, i, pv_target_vel, pv_accel, pv_decel);
            }
        }

        if (main_cycle_count % hm_update_interval == 0 && main_cycle_count > 100) {
            for (int i = 0; i < motor_count; i++) {
                hm_trajectory_update(&g_ctx, i, hm_method, hm_speed_switch, hm_speed_zero, hm_accel, hm_offset);
            }
        }

        // PT 模式轨迹规划 - 需要在每个周期循环内连续调用，内部有安全状态机
        for (int i = 0; i < motor_count; i++) {
            pt_trajectory_update_safe(&g_ctx, i, pt_target_torque, pt_slope, pt_update_interval);
        }

        // CST 模式轨迹规划
        for (int i = 0; i < motor_count; i++) {
            cst_trajectory_update_safe(&g_ctx, i, cst_target_torque, cst_max_speed, cst_update_interval);
        }

        // 发送过程数据
        ecrt_domain_queue(g_ctx.domain);
        ecrt_master_send(g_ctx.master);

        main_cycle_count++;
    }

    printf("\n正在停止...\n");

    // 停止电机
    for (int i = 0; i < motor_count; i++) {
        EC_WRITE_U16(g_ctx.domain_pd + g_ctx.motors[i].off.ctrl_word, EC_CTRL_DISABLE_VOLTAGE);
    }
    ecrt_domain_queue(g_ctx.domain);
    ecrt_master_send(g_ctx.master);
    usleep(100000);

err_release:
    ec_master_release(&g_ctx);
    printf("程序退出\n");

    return 0;
}
