/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <getopt.h>
#include <math.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "../include/motor.h"
#include "test_config.h"

#define MAX_CAN_MOTORS 16

/* Motor mechanics parameters */
#define PULSES_PER_REV 10000.0f
#define RAD_PER_REV (2.0f * (float)M_PI)
#define RAD_TO_PULSE (PULSES_PER_REV / RAD_PER_REV)
#define PULSE_TO_RAD (RAD_PER_REV / PULSES_PER_REV)

typedef struct {
    uint32_t index;
    uint32_t subindex;
    uint32_t size;
} sdo_addr_t;

static volatile int g_running = 1;
static int g_verbose = 0;

static void signal_handler(int sig) {
    (void)sig;
    g_running = 0;
}

static void usage(const char* prog) {
    printf("用法: %s [选项]\n", prog);
    printf("选项:\n");
    printf("  -m, --motors N    电机数量 (默认 1, 最大 %d)\n", MAX_CAN_MOTORS);
    printf("  -c, --cycle MS    控制周期 (默认 10 ms)\n");
    printf("  -v, --verbose     启用详细调试打印\n");
    printf("  -q, --quiet       禁用所有非必要打印 (默认)\n");
    printf("  -h, --help        显示帮助信息\n");
}

static void sleep_ms(uint32_t ms) {
    struct timespec ts;
    ts.tv_sec = ms / 1000;
    ts.tv_nsec = (ms % 1000) * 1000000;
    nanosleep(&ts, NULL);
}

int main(int argc, char** argv) {
    int motor_count = 1;
    uint32_t cycle_ms = 10;
    int can_ids[MAX_CAN_MOTORS] = {1, 2, 3, 4}; // default ids

    static struct option long_opts[] = {
        {"motors", required_argument, 0, 'm'},
        {"nums", required_argument, 0, 'n'},
        {"cycle", required_argument, 0, 'c'},
        {"if", required_argument, 0, 1},
        {"iface", required_argument, 0, 1},
        {"id", required_argument, 0, 2},
        {"driver", required_argument, 0, 3},
        {"baud", required_argument, 0, 4},
        {"verbose", no_argument, 0, 'v'},
        {"quiet", no_argument, 0, 'q'},
        {"help", no_argument, 0, 'h'},
        {0, 0, 0, 0}
    };

    int opt;
    opterr = 0; // Disable getopt automatic error printing for unknown options
    while ((opt = getopt_long(argc, argv, "m:n:c:vqh", long_opts, NULL)) != -1) {
        switch (opt) {
            case 'm':
            case 'n':
                motor_count = atoi(optarg);
                break;
            case 'c':
                cycle_ms = atoi(optarg);
                if (cycle_ms == 0) cycle_ms = 10;
                break;
            case 'v':
                g_verbose = 1;
                break;
            case 'q':
                g_verbose = 0;
                break;
            case 1:
            case 2:
            case 3:
            case 4:
            case '?':
                // Ignore these, load_config_and_args will handle them
                break;
            case 'h':
                usage(argv[0]);
                return 0;
            default:
                break;
        }
    }

    const char *driver = "drv_canopen_jmc";
    const char *iface = "can0";
    int baudrate = 1000000;

    // Attempt to load from config
    load_config_and_args(argc, argv, &driver, &iface, &baudrate, can_ids, &motor_count, NULL);

    if (motor_count <= 0 || motor_count > MAX_CAN_MOTORS) {
        fprintf(stderr, "错误: 电机数量 %d 非法 (范围 1..%d)\n", motor_count, MAX_CAN_MOTORS);
        return -1;
    }

    printf("========================================\n");
    printf("CANOpen JMC 测试程序\n");
    printf("驱动=%s, 接口=%s, 周期=%u ms, 电机数=%d\n", driver, iface, cycle_ms, motor_count);
    printf("========================================\n");

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    struct motor_dev* devs[MAX_CAN_MOTORS];

    for (int i = 0; i < motor_count; i++) {
        devs[i] = motor_alloc_can(driver, iface, can_ids[i], NULL);
        if (!devs[i]) {
            fprintf(stderr, "分配电机 %d 失败\n", i);
            return -1;
        }
    }

    if (motor_init(devs, motor_count) < 0) {
        fprintf(stderr, "初始化电机失败\n");
        return -1;
    }

    printf("电机初始化成功，开始运动测试...\n");

    uint32_t loop_count = 0;
    struct motor_cmd cmds[MAX_CAN_MOTORS];
    struct motor_state states[MAX_CAN_MOTORS];

    memset(cmds, 0, sizeof(cmds));
    memset(states, 0, sizeof(states));

    printf("========================================\n");
    printf("等待电机使能并自动锚定逻辑零点...\n");

    int stable_counts = 0;
    while (g_running) {
        motor_get_states(devs, states, motor_count);
        int all_enabled = 1;
        for (int i = 0; i < motor_count; i++) {
            if (((uint16_t)states[i].err & 0x6F) != 0x27) {
                all_enabled = 0;
            }
            cmds[i].mode = MOTOR_MODE_POS;
            cmds[i].pos_des = 0.0f;
        }
        motor_set_cmds(devs, cmds, motor_count);

        if (all_enabled) {
            if (++stable_counts > (100 / cycle_ms)) break;
        } else {
            stable_counts = 0;
            if (g_verbose && loop_count % (500 / cycle_ms) == 0) {
                printf("等待使能中... (M0 SW=%04X)\n", (uint16_t)states[0].err);
            }
            if (loop_count > (5000 / cycle_ms)) {
                fprintf(stderr, "错误: 等待电机使能超时！自动退出。\n");
                motor_free(devs, motor_count);
                return -1;
            }
        }
        sleep_ms(cycle_ms);
        loop_count++;
    }

    printf("\n========================================\n");
    printf("开始参数修改测试 (SDO)\n");
    sdo_addr_t acc_addr = {0x6083, 0x00, 2}; // Profile acceleration
    uint16_t set_acc = 90;

    // Set parameter
    if (motor_set_paras(devs[0], &acc_addr, &set_acc, 2) == 0) {
        printf("尝试修改 0x6083 (加速度) 为 %u ...\n", set_acc);
    } else {
        printf("修改 0x6083 (加速度) 失败\n");
    }

    // Read parameter
    uint16_t get_acc = 0;
    if (motor_get_paras(devs[0], &acc_addr, &get_acc, 2) == 0) {
        printf("成功读取 0x6083 (加速度): %u\n", get_acc);
        if (get_acc == set_acc) {
            printf(">>> 参数修改验证成功！ <<<\n");
        } else {
            printf(">>> 参数修改验证失败！期望 %u, 实际 %u <<<\n", set_acc, get_acc);
        }
    } else {
        printf("读取 0x6083 (加速度) 失败\n");
    }

    printf("\n所有电机已使能且锚定完成，开始 PP 轨迹测试.\n");

    float test_targets[] = {6.283f, 0.0f, -6.283f, 0.0f};  // 1圈, 0, -1圈, 0
    int num_steps = sizeof(test_targets) / sizeof(test_targets[0]);

    for (int s = 0; s < num_steps && g_running; s++) {
        float target = test_targets[s];
        printf("\n---> 下一目标位置 [%d/%d]: %.3f rad\n", s + 1, num_steps, target);

        for (int i = 0; i < motor_count; i++) {
            cmds[i].pos_des = target;
        }

        uint32_t step_timeout = 5000 / cycle_ms;
        int wait_for_clear = 1;
        for (uint32_t t = 0; t < step_timeout && g_running; t++) {
            motor_get_states(devs, states, motor_count);
            motor_set_cmds(devs, cmds, motor_count);

            if (g_verbose && t % (500 / cycle_ms) == 0) {
                printf("[M0] PP模式 - 目标: %.3f, 反馈: %.3f, 状态: 0x%04X\n", target, states[0].pos,
                        (uint16_t)states[0].err);
            }

            if (wait_for_clear) {
                int all_cleared = 1;
                for (int i = 0; i < motor_count; i++) {
                    if ((uint16_t)states[i].err & 0x0400) {
                        all_cleared = 0;
                        break;
                    }
                }
                if (all_cleared || t > (1000 / cycle_ms)) {
                    wait_for_clear = 0;
                }
            } else {
                int all_reached = 1;
                for (int i = 0; i < motor_count; i++) {
                    if (!((uint16_t)states[i].err & 0x0400)) {
                        all_reached = 0;
                        break;
                    }
                }

                if (all_reached) {
                    printf("所有电机已到达目标位置!\n");
                    sleep_ms(200);
                    break;
                }
            }

            sleep_ms(cycle_ms);
            loop_count++;
        }
    }

    // --- PV Test ---
    printf("\n========================================\n");
    printf("开始 PV (速度) 模式测试...\n");

    for (int i = 0; i < motor_count; i++) {
        cmds[i].mode = MOTOR_MODE_VEL;
        cmds[i].vel_des = 6.283f; // ~ 1 rps
    }
    printf("--> 正转测试: 6.283 rad/s\n");
    for (uint32_t t = 0; t < 2000 / cycle_ms && g_running; t++) {
        motor_get_states(devs, states, motor_count);
        motor_set_cmds(devs, cmds, motor_count);
        if (g_verbose && t % (500 / cycle_ms) == 0) {
            printf("[M0] PV模式 - 反馈: %.3f rad, 状态: 0x%04X\n", states[0].pos, (uint16_t)states[0].err);
        }
        sleep_ms(cycle_ms);
    }

    printf("--> 验证 0x6081 (Profile Velocity) 写入是否生效...\n");
    sdo_addr_t vel_addr = {0x6081, 0x00, 4};
    int32_t get_vel = 0;
    if (motor_get_paras(devs[0], &vel_addr, &get_vel, 4) == 0) {
        int32_t expected = (int32_t)(6.283f / (2.0f * (float)M_PI) * 10.0f);
        printf("成功读取 0x6081: %d (期望: %d)\n", get_vel, expected);
        if (get_vel == expected) {
            printf(">>> 速度目标地址 (0x6081) 验证成功！ <<<\n");
        } else {
            printf(">>> 速度目标地址验证失败！ <<<\n");
        }
    } else {
        printf("读取 0x6081 失败\n");
    }

    for (int i = 0; i < motor_count; i++) {
        cmds[i].vel_des = -6.283f; // ~ -1 rps
    }
    printf("--> 反转测试: -6.283 rad/s\n");
    for (uint32_t t = 0; t < 2000 / cycle_ms && g_running; t++) {
        motor_get_states(devs, states, motor_count);
        motor_set_cmds(devs, cmds, motor_count);
        if (g_verbose && t % (500 / cycle_ms) == 0) {
            printf("[M0] PV模式 - 反馈: %.3f rad, 状态: 0x%04X\n", states[0].pos, (uint16_t)states[0].err);
        }
        sleep_ms(cycle_ms);
    }

    for (int i = 0; i < motor_count; i++) {
        cmds[i].vel_des = 0.0f;
    }
    printf("--> 停止测试: 0 rad/s\n");
    for (uint32_t t = 0; t < 1000 / cycle_ms && g_running; t++) {
        motor_get_states(devs, states, motor_count);
        motor_set_cmds(devs, cmds, motor_count);
        sleep_ms(cycle_ms);
    }

    // --- HM Test ---
    printf("\n========================================\n");
    printf("开始 HM (回零) 模式测试...\n");
    for (int i = 0; i < motor_count; i++) {
        cmds[i].mode = MOTOR_MODE_HM;
    }
    printf("--> 触发回零并等待完成\n");
    for (uint32_t t = 0; t < 6000 / cycle_ms && g_running; t++) {
        motor_get_states(devs, states, motor_count);
        motor_set_cmds(devs, cmds, motor_count);
        if (g_verbose && t % (500 / cycle_ms) == 0) {
            printf("[M0] HM模式 - 反馈: %.3f rad, 状态: 0x%04X\n", states[0].pos, (uint16_t)states[0].err);
        }

        int all_homed = 1;
        for (int i = 0; i < motor_count; i++) {
            // Homing attained (bit 12) or Target reached (bit 10) in HM mode
            uint16_t sw = (uint16_t)states[i].err;
            if (!(sw & 0x1000) && !(sw & 0x0400)) {
                all_homed = 0;
            }
        }
        // Wait at least 1 second before checking to let the homing start properly
        if (all_homed && t > (1000 / cycle_ms)) {
            printf("所有电机已完成回零!\n");
            break;
        }
        sleep_ms(cycle_ms);
    }

    // --- IDLE Test ---
    printf("\n========================================\n");
    printf("开始 IDLE 模式 (失能) 测试...\n");
    for (int i = 0; i < motor_count; i++) {
        cmds[i].mode = MOTOR_MODE_IDLE;
    }

    int idle_success = 0;
    for (uint32_t t = 0; t < 2000 / cycle_ms && g_running; t++) {
        motor_get_states(devs, states, motor_count);
        motor_set_cmds(devs, cmds, motor_count);

        int all_disabled = 1;
        for (int i = 0; i < motor_count; i++) {
            uint16_t sw = (uint16_t)states[i].err;
            // 0x27 is Operation Enabled, we expect it to exit this state
            if ((sw & 0x6F) == 0x27) {
                all_disabled = 0;
            }
        }

        if (all_disabled) {
            idle_success = 1;
            break;
        }
        sleep_ms(cycle_ms);
    }

    if (idle_success) {
        printf(">>> IDLE 失能状态流转验证成功！ (M0 SW: 0x%04X) <<<\n", (uint16_t)states[0].err);
    } else {
        printf(">>> IDLE 失能状态流转验证失败！电机未能退出使能状态 (M0 SW: 0x%04X) <<<\n", (uint16_t)states[0].err);
    }

    printf("\n测试停止.\n");
    motor_free(devs, motor_count);
    return 0;
}
