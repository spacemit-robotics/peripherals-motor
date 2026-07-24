/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * 测试用例：验证 drv_can_ddt_p1010b 驱动的多模式切换
 * 基于 motor 抽象层
 *
 * 测试流程：
 *   1. 电压开环控制 (MOTOR_MODE_OPEN)
 *   2. 速度闭环控制 (MOTOR_MODE_VEL)
 *   3. 位置闭环控制 (MOTOR_MODE_POS)
 *   4. 电流闭环控制 (MOTOR_MODE_TRQ)
 *   5. IDLE 失能释放
 */

#include <math.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <net/if.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <signal.h>

#include "motor.h"
#include "test_config.h"

#define LOOP_PERIOD_US 10000   // 10ms

static struct motor_dev** g_devs = NULL;
static int g_num_motors = 0;

static void sigint_handler(int sig) {
    printf("\nCaught signal %d, stopping motor safely...\n", sig);
    if (g_devs && g_num_motors > 0) {
        struct motor_cmd cmd = {0};
        cmd.mode = MOTOR_MODE_IDLE;
        struct motor_cmd cmds[16];
        for (int j = 0; j < g_num_motors; j++) cmds[j] = cmd;
        motor_set_cmds(g_devs, cmds, g_num_motors);
        motor_free(g_devs, g_num_motors);
        g_devs = NULL;
    }
    exit(0);
}

static void print_usage(const char *prog_name) {
    printf("Usage: %s [options]\n", prog_name);
    printf("Options:\n");
    printf("  --driver <name>    Set motor driver (default: drv_can_ddt_p1010b)\n");
    printf("  --if <iface>       Set CAN interface (default: can0)\n");
    printf("  --id <id1,id2...>  Set motor IDs (supports decimal or hex, e.g. 1)\n");
    printf("  -h, --help         Show this help\n");
}

int main(int argc, char** argv) {
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            print_usage(argv[0]);
            return 0;
        }
    }

    struct motor_dev* devs[16];
    struct motor_state states[16];
    struct motor_cmd cmd = {0};

    const char *driver = "drv_can_ddt_p1010b";
    const char *iface = "can0";
    int baudrate = 1000000;
    int ids[16] = {1};
    int num_motors = 1;

    load_config_and_args(argc, argv, &driver, &iface, &baudrate, ids, &num_motors, NULL);

    if (if_nametoindex(iface) == 0) {
        printf("Error: Interface %s does not exist.\n", iface);
        return -1;
    }

    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock >= 0) {
        struct ifreq ifr;
        strncpy(ifr.ifr_name, iface, IFNAMSIZ - 1);
        ifr.ifr_name[IFNAMSIZ - 1] = '\0';
        if (ioctl(sock, SIOCGIFFLAGS, &ifr) == 0) {
            if (!(ifr.ifr_flags & IFF_UP)) {
                printf("Error: Interface %s is DOWN (not open).\n", iface);
                close(sock);
                return -1;
            }
        }
        close(sock);
    }

    printf("=== Allocating motor devices ===\n");
    for (int j = 0; j < num_motors; j++) {
        devs[j] = motor_alloc_can(driver, iface, (uint8_t)ids[j], NULL);
        if (!devs[j]) {
            printf("Error: Failed to allocate motor %d\n", j);
            return -1;
        }
    }

    g_devs = devs;
    g_num_motors = num_motors;
    signal(SIGINT, sigint_handler);
    signal(SIGTERM, sigint_handler);

    printf("=== Initializing ===\n");
    if (motor_init(devs, num_motors) < 0) {
        printf("Error: Init failed\n");
        return -1;
    }
    printf("=== Init done, starting mode switch test ===\n\n");

    float start_pos[16] = {0};
    motor_get_states(devs, states, num_motors);
    for (int j = 0; j < num_motors; j++) {
        start_pos[j] = states[j].pos;
    }
    uint8_t mode_code = 0x09;
    uint8_t abs_pos_code = 0x0D;

    // ========== Phase 1: Voltage Open Loop ==========
    printf("--- Phase 1: MOTOR_MODE_OPEN (Voltage Open Loop) ---\n");
    cmd.mode = MOTOR_MODE_OPEN;
    cmd.vel_des = 0.0f;
    for (int j = 0; j < num_motors; j++) {
        motor_set_cmd_one(devs[j], &cmd);
    }
    usleep(50000); // Wait mode switch

    for (int j = 0; j < num_motors; j++) {
        uint16_t actual_mode = 0xFFFF;
        if (motor_get_paras(devs[j], &mode_code, &actual_mode, 2) == 0) {
            printf("Motor %d Mode: %u (expected 0 for Voltage)\n", j, actual_mode);
        }
    }

    cmd.vel_des = 2.0f; // 2.0V safe voltage
    for (int i = 0; i < 1000; i++) {
        int all_rotated = 1;
        for (int j = 0; j < num_motors; j++) {
            motor_set_cmd_one(devs[j], &cmd);
            motor_get_state_one(devs[j], &states[j]);
            if (i % 20 == 0) {
                uint16_t actual_abs = 0;
                motor_get_paras(devs[j], &abs_pos_code, &actual_abs, 2);
                printf("[Voltage] M%d pos=%.4f vel=%.4f | AbsRaw: %u\n", j, states[j].pos, states[j].vel, actual_abs);
            }
            if (fabs(states[j].pos - start_pos[j]) < 6.28318f) {
                all_rotated = 0;
            }
        }
        if (all_rotated) {
            printf("All motors rotated a full circle in Voltage mode!\n");
            break;
        }
        usleep(LOOP_PERIOD_US);
    }

    // Stop and settle
    cmd.mode = MOTOR_MODE_IDLE;
    for (int j = 0; j < num_motors; j++) motor_set_cmd_one(devs[j], &cmd);
    usleep(500000);

    // ========== Phase 2: Velocity Loop ==========
    printf("\n--- Phase 2: MOTOR_MODE_VEL (Velocity Loop) ---\n");
    motor_get_states(devs, states, num_motors);
    for (int j = 0; j < num_motors; j++) start_pos[j] = states[j].pos;

    cmd.mode = MOTOR_MODE_VEL;
    cmd.vel_des = 0.0f;
    for (int j = 0; j < num_motors; j++) motor_set_cmd_one(devs[j], &cmd);
    usleep(50000);

    for (int j = 0; j < num_motors; j++) {
        uint16_t actual_mode = 0xFFFF;
        if (motor_get_paras(devs[j], &mode_code, &actual_mode, 2) == 0) {
            printf("Motor %d Mode: %u (expected 3 for Velocity)\n", j, actual_mode);
        }
    }

    cmd.vel_des = 2.0f; // 2 rad/s
    for (int i = 0; i < 1000; i++) {
        int all_rotated = 1;
        for (int j = 0; j < num_motors; j++) {
            motor_set_cmd_one(devs[j], &cmd);
            motor_get_state_one(devs[j], &states[j]);
            if (i % 20 == 0) {
                uint16_t actual_abs = 0;
                motor_get_paras(devs[j], &abs_pos_code, &actual_abs, 2);
                printf("[Velocity] M%d pos=%.4f vel=%.4f | AbsRaw: %u\n", j, states[j].pos, states[j].vel, actual_abs);
            }
            if (fabs(states[j].pos - start_pos[j]) < 6.28318f) {
                all_rotated = 0;
            }
        }
        if (all_rotated) {
            printf("All motors rotated a full circle in Velocity mode!\n");
            break;
        }
        usleep(LOOP_PERIOD_US);
    }

    // Stop and settle
    cmd.mode = MOTOR_MODE_IDLE;
    for (int j = 0; j < num_motors; j++) motor_set_cmd_one(devs[j], &cmd);
    usleep(500000);

    // ========== Phase 3: Position Loop ==========
    printf("\n--- Phase 3: MOTOR_MODE_POS (Position Loop) ---\n");
    motor_get_states(devs, states, num_motors);
    for (int j = 0; j < num_motors; j++) start_pos[j] = states[j].pos;

    cmd.mode = MOTOR_MODE_POS;
    for (int j = 0; j < num_motors; j++) {
        cmd.pos_des = start_pos[j];
        motor_set_cmd_one(devs[j], &cmd);
    }
    usleep(50000);

    for (int j = 0; j < num_motors; j++) {
        uint16_t actual_mode = 0xFFFF;
        if (motor_get_paras(devs[j], &mode_code, &actual_mode, 2) == 0) {
            printf("Motor %d Mode: %u (expected 4 for Position)\n", j, actual_mode);
        }
    }

    int steps = 200; // 2 seconds
    for (int i = 0; i <= steps; i++) {
        for (int j = 0; j < num_motors; j++) {
            cmd.pos_des = start_pos[j] + 6.28318f * i / steps;
            motor_set_cmd_one(devs[j], &cmd);
            motor_get_state_one(devs[j], &states[j]);

            if (i % 20 == 0) {
                uint16_t actual_abs = 0;
                motor_get_paras(devs[j], &abs_pos_code, &actual_abs, 2);
                printf("[Position] M%d des=%.4f pos=%.4f vel=%.4f | AbsRaw: %u\n",
                        j, cmd.pos_des, states[j].pos, states[j].vel, actual_abs);
            }
        }
        usleep(LOOP_PERIOD_US);
    }

    printf("Trajectory finished, waiting for motor to settle...\n");
    int settle_steps = 0;
    while (settle_steps < 500) {
        int all_settled = 1;
        for (int j = 0; j < num_motors; j++) {
            cmd.pos_des = start_pos[j] + 6.28318f;
            motor_set_cmd_one(devs[j], &cmd);
            motor_get_state_one(devs[j], &states[j]);
            if (settle_steps % 20 == 0) {
                uint16_t actual_abs = 0;
                motor_get_paras(devs[j], &abs_pos_code, &actual_abs, 2);
                printf("[Settle] M%d pos=%.4f err=%.4f | AbsRaw: %u\n",
                        j, states[j].pos, fabs(states[j].pos - cmd.pos_des), actual_abs);
            }
            if (fabs(states[j].pos - cmd.pos_des) >= 0.05) {
                all_settled = 0;
            }
        }
        if (all_settled) {
            printf("All motors rotated a full circle and settled in Position mode!\n");
            break;
        }
        usleep(LOOP_PERIOD_US);
        settle_steps++;
    }

    // Stop and settle
    cmd.mode = MOTOR_MODE_IDLE;
    for (int j = 0; j < num_motors; j++) motor_set_cmd_one(devs[j], &cmd);
    usleep(500000);

    // ========== Phase 4: Torque (Current) Loop ==========
    printf("\n--- Phase 4: MOTOR_MODE_TRQ (Current Loop) ---\n");
    motor_get_states(devs, states, num_motors);
    for (int j = 0; j < num_motors; j++) start_pos[j] = states[j].pos;

    cmd.mode = MOTOR_MODE_TRQ;
    cmd.trq_des = 0.0f;
    for (int j = 0; j < num_motors; j++) motor_set_cmd_one(devs[j], &cmd);
    usleep(50000);

    for (int j = 0; j < num_motors; j++) {
        uint16_t actual_mode = 0xFFFF;
        if (motor_get_paras(devs[j], &mode_code, &actual_mode, 2) == 0) {
            printf("Motor %d Mode: %u (expected 2 for Current/Torque)\n", j, actual_mode);
        }
    }

    cmd.trq_des = 1.0f; // 1.0A safe current to overcome friction
    for (int i = 0; i < 1000; i++) {
        int all_rotated = 1;
        for (int j = 0; j < num_motors; j++) {
            motor_set_cmd_one(devs[j], &cmd);
            motor_get_state_one(devs[j], &states[j]);
            if (i % 20 == 0) {
                uint16_t actual_abs = 0;
                motor_get_paras(devs[j], &abs_pos_code, &actual_abs, 2);
                printf("[Torque] M%d pos=%.4f vel=%.4f trq=%.4f | AbsRaw: %u\n", j, states[j].pos, states[j].vel, states[j].trq, actual_abs);
            }
            if (fabs(states[j].pos - start_pos[j]) < 6.28318f) {
                all_rotated = 0;
            }
        }
        if (all_rotated) {
            printf("All motors rotated a full circle in Torque mode!\n");
            break;
        }
        usleep(LOOP_PERIOD_US);
    }

    // 钳位 V = 0，迫使电机停止，不再滑行
    // 安全声明：经试验，在 1A 电流下进行 V=0 速度钳位，已有较强的制动效果。
    // 如果测试给定的运行电流超过 1A，直接突变 V=0 可能产生巨大反激电流引发硬件安全隐患，请谨慎使用！
    printf("Braking (V=0) to stop motor...\n");
    cmd.mode = MOTOR_MODE_VEL;
    cmd.vel_des = 0.0f;
    for (int j = 0; j < num_motors; j++) motor_set_cmd_one(devs[j], &cmd);
    usleep(500000); // 等待刹车完全停止

    // ========== Phase 5: IDLE ==========
    printf("\n--- Phase 5: MOTOR_MODE_IDLE ---\n");
    cmd.mode = MOTOR_MODE_IDLE;
    for (int j = 0; j < num_motors; j++) motor_set_cmd_one(devs[j], &cmd);
    printf("All motors disabled\n");

    printf("\n=== Releasing motors ===\n");
    motor_free(devs, num_motors);
    g_devs = NULL;

    printf("=== Test complete ===\n");
    return 0;
}
