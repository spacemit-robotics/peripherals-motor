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

#define MAX_ECAT_MOTORS 10

/* 电机机械参数 (配置与驱动一致) */
#define PULSES_PER_REV 10000.0f
#define RAD_PER_REV (2.0f * (float)M_PI)
#define RAD_TO_PULSE (PULSES_PER_REV / RAD_PER_REV)
#define PULSE_TO_RAD (RAD_PER_REV / PULSES_PER_REV)

static volatile int g_running = 1;

static void signal_handler(int sig) {
    (void)sig;
    g_running = 0;
}

static void usage(const char* prog) {
    printf("用法: %s [选项]\n", prog);
    printf("选项:\n");
    printf("  --motors N    电机数量 (默认 1, 最大 %d)\n", MAX_ECAT_MOTORS);
    printf("  --cycle MS    控制周期 (默认 2 ms)\n");
    printf("  --help        显示帮助信息\n");
}

static void sleep_ms(uint32_t ms) {
    struct timespec ts;
    ts.tv_sec = ms / 1000;
    ts.tv_nsec = (ms % 1000) * 1000000;
    nanosleep(&ts, NULL);
}

int main(int argc, char** argv) {
    int motor_count = 2;
    uint32_t cycle_ms = 2;
    int app_mode = MOTOR_MODE_POS;

    static struct option long_opts[] = {{"motors", required_argument, 0, 'm'}, {"cycle", required_argument, 0, 'c'},
        {"help", no_argument, 0, 'h'}, {0, 0, 0, 0}};

    int opt;
    while ((opt = getopt_long(argc, argv, "m:c:h", long_opts, NULL)) != -1) {
        switch (opt) {
            case 'm':
                motor_count = atoi(optarg);
                break;
            case 'c':
                cycle_ms = atoi(optarg);
                if (cycle_ms == 0)
                    cycle_ms = 1;
                break;
            case 'h':
            default:
                usage(argv[0]);
                return 0;
        }
    }

    if (motor_count <= 0 || motor_count > MAX_ECAT_MOTORS) {
        fprintf(stderr, "错误: 电机数量 %d 非法 (范围 1..%d)\n", motor_count, MAX_ECAT_MOTORS);
        return -1;
    }

    printf("========================================\n");
    printf("JMC IHSS42-EC EtherCAT 框架测试程序 (PP 模式示例)\n");
    printf("驱动=drv_ethercat_jmc, 周期=%u ms, 电机数=%d\n", cycle_ms, motor_count);
    printf("========================================\n");

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    struct motor_dev* devs[MAX_ECAT_MOTORS];

    // allocate motors
    for (int i = 0; i < motor_count; i++) {
        devs[i] = motor_alloc_ecat("drv_ethercat_jmc", i, (void*)(uintptr_t)cycle_ms);
        if (!devs[i]) {
            fprintf(stderr, "分配电机 %d 失败\n", i);
            return -1;
        }
    }

    // init motors
    if (motor_init(devs, motor_count) < 0) {
        fprintf(stderr, "初始化电机失败\n");
        return -1;
    }

    printf("电机初始化成功，开始运动测试...\n");

    uint32_t loop_count = 0;
    struct motor_cmd cmds[MAX_ECAT_MOTORS];
    struct motor_state states[MAX_ECAT_MOTORS];

    memset(cmds, 0, sizeof(cmds));
    memset(states, 0, sizeof(states));

    // // CSP 模式测试 ------------------------------------------------------
    // printf("========================================\n");
    // printf("开始 CSP 模式测试 (±10 rad 往复运动)...\n");
    // printf("驱动层会自动处理使能前的对齐、指令锚定和安全过滤.\n");

    // float amplitude = 10.0f;
    // float step = 0.01f;
    // float current_pos = 0.0f;
    // int direction = 1;

    // while (g_running) {
    //     // 获取电机状态 (仅用于打印展示)
    //     motor_get_states(devs, states, motor_count);

    //     // 运动逻辑：生成往复运动轨迹
    //     current_pos += direction * step;
    //     if (current_pos > amplitude)
    //         direction = -1;
    //     else if (current_pos < -amplitude)
    //         direction = 1;

    //     for (int i = 0; i < motor_count; ++i) {
    //         cmds[i].mode = MOTOR_MODE_CSP;  // 显式请求 CSP 模式
    //         cmds[i].pos_des = current_pos;
    //     }

    //     // 应用层不再判断使能状态，直接发送期望轨迹。
    //     // 驱动层会自动捕捉使能瞬间的物理位置和指令位置，并进行差值补偿，确保零跳变启动。
    //     motor_set_cmds(devs, cmds, motor_count);

    //     // 每秒打印一次状态
    //     uint32_t print_interval = 1000 / cycle_ms;
    //     if (print_interval == 0) print_interval = 1;
    //     if (loop_count % print_interval == 0) {
    //         for (int i = 0; i < motor_count; i++) {
    //             printf("[M%d] CSP模式 - 反馈: %.3f, 指令: %.3f, 状态: 0x%04X %s\n", i, states[i].pos,
    //             cmds[i].pos_des,
    //                    (uint16_t)states[i].err,
    //                    ((uint16_t)states[i].err & 0x006F) == 0x0027 ? "(RUNNING)" : "(ENABLING...)");
    //         }
    //     }

    //     loop_count++;
    //     sleep_ms(cycle_ms);
    // }

    // // --- CSV 模式测试示例 (Profile Velocity Sine Wave) ---
    // printf("========================================\n");
    // printf("开始 CSV 模式正弦波速度测试...\n");
    // printf("等待电机使能 (Mode: CSV)...\n");

    // float amplitude_csv = 10.0f;  // rad/s (最大速度限制见驱动层)
    // float frequency_csv = 0.5f;   // Hz
    // float time_sec = 0.0f;

    // // 第一步：等待使能
    // while (g_running) {
    //     motor_get_states(devs, states, motor_count);
    //     int all_enabled = 1;
    //     for (int i = 0; i < motor_count; i++) {
    //         if (((uint16_t)states[i].err & 0x006F) != 0x0027) {
    //             all_enabled = 0;
    //             break;
    //         }
    //     }
    //     if (all_enabled) break;

    //     for (int i = 0; i < motor_count; i++) {
    //         cmds[i].mode = MOTOR_MODE_CSV;
    //         cmds[i].vel_des = 0.0f;
    //     }
    //     motor_set_cmds(devs, cmds, motor_count);

    //     if (loop_count % (500 / cycle_ms) == 0) {
    //         printf("等待使能中... (SW=%04X)\n", (uint16_t)states[0].err);
    //     }
    //     sleep_ms(cycle_ms);
    //     loop_count++;
    // }

    // printf("所有电机已使能，开始正弦速度轨迹 (幅值=%.1f rad/s, 频率=%.1f Hz)...\n", amplitude_csv, frequency_csv);

    // // 第二步：执行轨迹
    // while (g_running) {
    //     motor_get_states(devs, states, motor_count);

    //     // 生成正弦速度轨迹
    //     float target_vel = amplitude_csv * sinf(2.0f * M_PI * frequency_csv * time_sec);

    //     for (int i = 0; i < motor_count; i++) {
    //         cmds[i].mode = MOTOR_MODE_CSV;
    //         cmds[i].vel_des = target_vel;
    //     }

    //     motor_set_cmds(devs, cmds, motor_count);

    //     if (loop_count % (1000 / cycle_ms) == 0) {
    //         for (int i = 0; i < motor_count; i++) {
    //             printf("[M%d] CSV 模式 - 反馈: P=%.3f, V=%.3f | 目标速度: %.3f\n", i, states[i].pos, states[i].vel,
    //                    target_vel);
    //         }
    //     }

    //     time_sec += (float)cycle_ms / 1000.0f;
    //     loop_count++;
    //     sleep_ms(cycle_ms);
    // }

    // --- PP 模式测试 (Profile Position) ---
    printf("========================================\n");
    printf("等待电机使能并自动锚定逻辑零点...\n");

    // 第一步：在未使能时持续发送 pos_des=0，驱动会自动将逻辑零点标定在使能瞬间的物理位置
    printf("正在同步所有电机使能状态...\n");
    int stable_counts = 0;
    while (g_running) {
        motor_get_states(devs, states, motor_count);
        int all_enabled = 1;
        for (int i = 0; i < motor_count; i++) {
            if (((uint16_t)states[i].err & 0x006F) != 0x0027) {
                all_enabled = 0;
            }
            cmds[i].mode = MOTOR_MODE_POS;
            cmds[i].pos_des = 0.0f;
        }
        motor_set_cmds(devs, cmds, motor_count);

        if (all_enabled) {
            // 所有电机都已使能，额外等待 100ms 确保适配器内部锚定逻辑彻底完成
            if (++stable_counts > (100 / cycle_ms))
                break;
        } else {
            stable_counts = 0;
            if (loop_count % (500 / cycle_ms) == 0) {
                printf("等待使能中... (M0 SW=%04X, M1 SW=%04X)\n", (uint16_t)states[0].err,
                    (motor_count > 1 ? (uint16_t)states[1].err : 0));
            }
        }

        sleep_ms(cycle_ms);
        loop_count++;
    }

    printf("所有电机已使能且锚定完成，开始 PP 轨迹测试.\n");

    // 第二步：执行往返运动
    float test_targets[] = {6.283f, 0.0f, -6.283f, 0.0f};  // 1圈, 0, -1圈, 0
    int num_steps = sizeof(test_targets) / sizeof(test_targets[0]);

    for (int s = 0; s < num_steps && g_running; s++) {
        float target = test_targets[s];
        printf("\n---> 下一目标位置 [%d/%d]: %.3f rad\n", s + 1, num_steps, target);

        for (int i = 0; i < motor_count; i++) {
            cmds[i].pos_des = target;
        }

        // 持续发送指令并观察到达情况
        uint32_t step_timeout = 3000 / cycle_ms;
        for (uint32_t t = 0; t < step_timeout && g_running; t++) {
            motor_get_states(devs, states, motor_count);
            motor_set_cmds(devs, cmds, motor_count);

            if (t % (500 / cycle_ms) == 0) {
                printf("[M0] PP模式 - 目标: %.3f, 反馈: %.3f, 状态: 0x%04X\n", target, states[0].pos,
                    (uint16_t)states[0].err);
            }

            // 检查所有电机是否都到达目标位置（Target Reached: Bit 10 of statusword in PP mode is 0x0400）
            int all_reached = 1;
            for (int i = 0; i < motor_count; i++) {
                if (!((uint16_t)states[i].err & 0x0400)) {
                    all_reached = 0;
                    break;
                }
            }

            if (all_reached) {
                printf("所有电机已到达目标位置!\n");
                sleep_ms(200);  // 稍微停顿，观察稳态
                break;
            }

            sleep_ms(cycle_ms);
            loop_count++;
        }
    }

    // --- PV 模式测试 (Profile Velocity) ---
    printf("========================================\n");
    printf("开始 PV 模式速度测试...\n");
    printf("等待电机使能 (Mode: VEL)...\n");

    // 第一步：等待使能并确保同步稳定
    int pv_stable_counts = 0;
    while (g_running) {
        motor_get_states(devs, states, motor_count);
        int all_enabled = 1;
        for (int i = 0; i < motor_count; i++) {
            if (((uint16_t)states[i].err & 0x006F) != 0x0027) {
                all_enabled = 0;
            }
            cmds[i].mode = MOTOR_MODE_VEL;
            cmds[i].vel_des = 0.0f;
        }
        motor_set_cmds(devs, cmds, motor_count);

        if (all_enabled) {
            // 所有电机都已使能，额外等待 100ms 确保控制环路稳定
            if (++pv_stable_counts > (100 / cycle_ms))
                break;
        } else {
            pv_stable_counts = 0;
            if (loop_count % (500 / cycle_ms) == 0) {
                printf("PV 等待使能中... (M0 SW=%04X, M1 SW=%04X)\n", (uint16_t)states[0].err,
                    (motor_count > 1 ? (uint16_t)states[1].err : 0));
            }
        }
        sleep_ms(cycle_ms);
        loop_count++;
    }

    printf("PV 模式已就绪，开始速度阶跃测试.\n");

    // 第二步：执行速度阶跃
    float vel_targets[] = {3.14159f, 0.0f, -3.14159f, 0.0f};  // 0.5Hz, 0, -0.5Hz, 0
    int num_vel_steps = sizeof(vel_targets) / sizeof(vel_targets[0]);

    for (int s = 0; s < num_vel_steps && g_running; s++) {
        float target_vel = vel_targets[s];
        printf("\n---> 下一目标速度 [%d/%d]: %.3f rad/s\n", s + 1, num_vel_steps, target_vel);

        for (int i = 0; i < motor_count; i++) {
            cmds[i].vel_des = target_vel;
        }

        // 持续 2 秒
        uint32_t step_duration = 2000 / cycle_ms;
        for (uint32_t t = 0; t < step_duration && g_running; t++) {
            motor_get_states(devs, states, motor_count);
            motor_set_cmds(devs, cmds, motor_count);

            if (t % (500 / cycle_ms) == 0) {
                for (int i = 0; i < motor_count; i++) {
                    printf("[M%d] PV模式 - 目标: %.3f, 反馈V: %.3f, P: %.3f\n", i, target_vel, states[i].vel,
                        states[i].pos);
                }
            }

            sleep_ms(cycle_ms);
            loop_count++;
        }
    }

    // // --- HM 模式测试 (Homing Mode) ---
    // printf("========================================\n");
    // printf("开始 HM 模式回零测试 (方法 35: 当前位置设为零)...\n");
    // printf("等待电机使能 (Mode: HM)...\n");

    // // 第一步：等待使能
    // while (g_running) {
    //     motor_get_states(devs, states, motor_count);
    //     int all_enabled = 1;
    //     for (int i = 0; i < motor_count; i++) {
    //         if (((uint16_t)states[i].err & 0x006F) != 0x0027) {
    //             all_enabled = 0;
    //             break;
    //         }
    //     }
    //     if (all_enabled) break;

    //     for (int i = 0; i < motor_count; i++) {
    //         cmds[i].mode = MOTOR_MODE_HM;
    //     }
    //     // motor_set_cmds 在 HM 模式且状态机就绪时，内部会触发 ec_hm_start()
    //     motor_set_cmds(devs, cmds, motor_count);

    //     if (loop_count % (500 / cycle_ms) == 0) {
    //         printf("等待使能中... (SW=%04X, 当前位置: %.3f)\n", (uint16_t)states[0].err, states[0].pos);
    //     }
    //     sleep_ms(cycle_ms);
    //     loop_count++;
    // }

    // printf("HM 模式已就绪，正在触发回零动作...\n");

    // // 第二步：监控回零进展
    // // 在 HM 模式下，statusword Bit 12 表示 Homing attained，Bit 13 表示 Homing error
    // while (g_running) {
    //     motor_get_states(devs, states, motor_count);
    //     motor_set_cmds(devs, cmds, motor_count);  // 保持模式

    //     uint16_t sw = (uint16_t)states[0].err;

    //     if (loop_count % (500 / cycle_ms) == 0) {
    //         printf("[M0] HM 监控 - SW: 0x%04X, 位置: %.3f\n", sw, states[0].pos);
    //     }

    //     if (sw & 0x2000) {  // Bit 13: Homing error
    //         printf("[M0] ❌ 回零报错! 程序终止.\n");
    //         break;
    //     }

    //     if (sw & 0x1000) {  // Bit 12: Homing attained
    //         printf("[M0] ✅ 回零成功完成! 当前位置已锚定为零.\n");
    //         printf("[M0] 最终反馈位置: %.3f rad\n", states[0].pos);
    //         printf("测试任务完成，正在退出系统...\n");
    //         g_running = 0;  // 终止主循环
    //         break;
    //     }

    //     sleep_ms(cycle_ms);
    //     loop_count++;
    // }

    printf("\n测试停止.\n");
    motor_free(devs, motor_count);
    return 0;
}
