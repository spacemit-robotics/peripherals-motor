/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * 测试用例：验证 drv_can_ddt_m152d133 (本末 M15-2D-133) 驱动。
 * 
 * 特点：
 * 1. 仅依赖外部标准头文件与 `<motor.h>` 通用接口，去除了对底层特定头文件的依赖。
 * 2. 移植了稳定的队列排空机制和四种控制模式的连续平滑切换。
 */

#include "motor.h"
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <signal.h>
#include <math.h>

#define K_PI 3.14159265358979323846f

static struct motor_dev *g_dev = NULL;
static volatile sig_atomic_t g_running = 1;

static void signal_handler(int sig) {
    (void)sig;
    g_running = 0;

    if (g_dev) {
        printf("\n[SIGNAL] Caught signal %d, disabling motor...\n", sig);
        struct motor_cmd cmd = {0};
        cmd.mode = MOTOR_MODE_IDLE;
        motor_set_cmd_one(g_dev, &cmd);
        usleep(50000);
        motor_free(&g_dev, 1);
        g_dev = NULL;
    }

    exit(0);
}

int main(int argc, char **argv) {
    if (argc < 2) {
        printf("Usage: %s <can_interface>\n", argv[0]);
        return -1;
    }

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    /* 仅使用 motor.h 暴露的抽象工厂函数，不依赖 motor_core.h 的结构体 */
    struct motor_dev *dev = motor_alloc_can("drv_can_ddt_m152d133", argv[1], 1, NULL);
    if (!dev) {
        printf("Failed to allocate motor\n");
        return -1;
    }

    g_dev = dev;

    if (motor_init_one(dev) != 0) {
        printf("Failed to init motor\n");
        return -1;
    }

    /* 必须设置电机反馈周期为 20ms (参数地址 3)，否则默认的 1ms 上报会瞬间塞满 Linux 缓冲区，
       导致后续所有的反馈数据都是陈旧的历史数据（巨大的延迟和启动位置错误） */
    uint8_t feedback_cfg = 20;
    if (motor_set_paras(dev, (const void *)(uintptr_t)3, &feedback_cfg, sizeof(feedback_cfg)) == 0) {
        printf("Successfully set motor feedback period to 20ms.\n");
    }

    printf("Ensuring motor is disabled and stable before starting...\n");
    struct motor_cmd cmd = {0};
    cmd.mode = MOTOR_MODE_IDLE;
    struct motor_state state = {0};

    int read_failures = 0;
    for (int i = 0; i < 50; i++) {
        motor_set_cmd_one(dev, &cmd);
        if (motor_get_state_one(dev, &state) < 0) {
            read_failures++;
        }
        usleep(2000);
    }

    if (read_failures > 10) {
        printf("Error: Too many state read failures (%d/50).\n", read_failures);
        motor_free(&dev, 1);
        return -1;
    }

    float initial_position = state.pos;
    printf("Initial physical position: %.2f rad (read failures: %d/50)\n", initial_position, read_failures);

    printf("Waiting 3 seconds before enabling motor... Observe if position drifts.\n");
    for (int i = 0; i < 30; i++) {
        if (!g_running) break;
        if (motor_get_state_one(dev, &state) == 0) {
            if (i % 10 == 0) {
                printf("[PRE-ENABLE] pos: %.3f rad, vel: %.3f rad/s, trq: %.3f Nm, err: 0x%02X\n",
                    state.pos, state.vel, state.trq, state.err);
            }
        }
        usleep(100000); /* 100ms * 30 = 3 seconds */
    }

    printf("\n--- Diagnostic: Entering Current Mode with 0 Torque to test PWM engagement ---\n");
    cmd.mode = MOTOR_MODE_TRQ;
    cmd.trq_des = 0.0f;
    motor_set_cmd_one(dev, &cmd);

    for (int i = 0; i < 20; i++) {
        if (!g_running) break;
        motor_get_state_one(dev, &state);
        if (i % 10 == 0) {
            printf("[DIAGNOSTIC] pos: %.3f, trq: %.3f\n", state.pos, state.trq);
        }
        usleep(100000); /* Hold for 2 seconds */
    }

    /* ==================================================================== */
    /* 1. Open Loop                                                         */
    /* ==================================================================== */
    printf("\nSetting mode to Open Loop (0) and holding at 0 for 2 seconds...\n");
    cmd.mode = MOTOR_MODE_OPEN;
    cmd.vel_des = 0.0f;
    for (int i = 0; i < 100; i++) {
        if (!g_running) break;
        motor_set_cmd_one(dev, &cmd);
        motor_get_state_one(dev, &state);
        usleep(20000);
    }

    float safe_open_velocity = 1.0f;

    printf("--- Start Positive Open Loop ---\n");
    cmd.vel_des = safe_open_velocity;
    for (int i = 0; i < 100; i++) {
        if (!g_running) break;
        motor_set_cmd_one(dev, &cmd);
        usleep(20000);
        if (motor_get_state_one(dev, &state) == 0 && (i % 10 == 0)) {
            printf("[Command] vel_des(open): %.2f | [Feedback] pos: %.2f vel: %.2f trq: %.2f err: %u\n",
                cmd.vel_des, state.pos, state.vel, state.trq, state.err);
        }
    }

    printf("--- Start Negative Open Loop ---\n");
    cmd.vel_des = -safe_open_velocity;
    for (int i = 0; i < 100; i++) {
        if (!g_running) break;
        motor_set_cmd_one(dev, &cmd);
        usleep(20000);
        if (motor_get_state_one(dev, &state) == 0 && (i % 10 == 0)) {
            printf("[Command] vel_des(open): %.2f | [Feedback] pos: %.2f vel: %.2f trq: %.2f err: %u\n",
                cmd.vel_des, state.pos, state.vel, state.trq, state.err);
        }
    }

    printf("\nBraking before current loop...\n");
    cmd.vel_des = 0.0f;
    for (int i = 0; i < 50; i++) {
        if (!g_running) break;
        motor_set_cmd_one(dev, &cmd);
        motor_get_state_one(dev, &state);
        usleep(20000);
    }

    /* ==================================================================== */
    /* 2. Current Loop                                                      */
    /* ==================================================================== */
    printf("\nSetting mode to Current Loop (1) and holding torque at 0 for 2 seconds...\n");
    cmd.mode = MOTOR_MODE_TRQ;
    cmd.trq_des = 0.0f;
    for (int i = 0; i < 100; i++) {
        if (!g_running) break;
        motor_set_cmd_one(dev, &cmd);
        motor_get_state_one(dev, &state);
        usleep(20000);
    }

    float safe_torque = 0.88f;

    printf("--- Start Positive Torque ---\n");
    cmd.trq_des = safe_torque;
    for (int i = 0; i < 50; i++) {
        if (!g_running) break;
        motor_set_cmd_one(dev, &cmd);
        usleep(20000);
        if (motor_get_state_one(dev, &state) == 0 && (i % 10 == 0)) {
            printf("[Command] trq_des: %.2f | [Feedback] pos: %.2f vel: %.2f trq: %.2f err: %u\n",
                cmd.trq_des, state.pos, state.vel, state.trq, state.err);
        }
    }

    printf("--- Start Negative Torque ---\n");
    cmd.trq_des = -safe_torque;
    for (int i = 0; i < 50; i++) {
        if (!g_running) break;
        motor_set_cmd_one(dev, &cmd);
        usleep(20000);
        if (motor_get_state_one(dev, &state) == 0 && (i % 10 == 0)) {
            printf("[Command] trq_des: %.2f | [Feedback] pos: %.2f vel: %.2f trq: %.2f err: %u\n",
                cmd.trq_des, state.pos, state.vel, state.trq, state.err);
        }
    }

    printf("\nBraking before velocity loop...\n");
    cmd.trq_des = 0.0f;
    for (int i = 0; i < 50; i++) {
        if (!g_running) break;
        motor_set_cmd_one(dev, &cmd);
        motor_get_state_one(dev, &state);
        usleep(20000);
    }

    /* ==================================================================== */
    /* 3. Velocity Loop                                                     */
    /* ==================================================================== */
    printf("\nSetting mode to Velocity Loop (2) and holding velocity at 0 for 2 seconds...\n");
    cmd.mode = MOTOR_MODE_VEL;
    cmd.vel_des = 0.0f;
    for (int i = 0; i < 100; i++) {
        if (!g_running) break;
        motor_set_cmd_one(dev, &cmd);
        motor_get_state_one(dev, &state);
        usleep(20000);
    }

    float safe_velocity = 1.0f;

    printf("--- Start Positive Velocity ---\n");
    cmd.vel_des = safe_velocity;
    for (int i = 0; i < 100; i++) {
        if (!g_running) break;
        motor_set_cmd_one(dev, &cmd);
        usleep(20000);
        if (motor_get_state_one(dev, &state) == 0 && (i % 10 == 0)) {
            printf("[Command] vel_des: %.2f | [Feedback] pos: %.2f vel: %.2f trq: %.2f err: %u\n",
                cmd.vel_des, state.pos, state.vel, state.trq, state.err);
        }
    }

    printf("--- Start Negative Velocity ---\n");
    cmd.vel_des = -safe_velocity;
    for (int i = 0; i < 100; i++) {
        if (!g_running) break;
        motor_set_cmd_one(dev, &cmd);
        usleep(20000);
        if (motor_get_state_one(dev, &state) == 0 && (i % 10 == 0)) {
            printf("[Command] vel_des: %.2f | [Feedback] pos: %.2f vel: %.2f trq: %.2f err: %u\n",
                cmd.vel_des, state.pos, state.vel, state.trq, state.err);
        }
    }

    printf("\nBraking before position loop...\n");
    cmd.vel_des = 0.0f;
    for (int i = 0; i < 50; i++) {
        if (!g_running) break;
        motor_set_cmd_one(dev, &cmd);
        motor_get_state_one(dev, &state);
        usleep(20000);
    }

    sleep(3);
    /* ==================================================================== */
    /* 4. Position Loop (Moved to end to avoid startup firmware pulse)      */
    /* ==================================================================== */
    motor_get_state_one(dev, &state); // Re-read position to align target
    float current_pos = state.pos;
    cmd.mode = MOTOR_MODE_POS;
    cmd.pos_des = current_pos;

    printf("\nSetting mode to Position Loop (3) and holding current position for 2 seconds...\n");
    for (int i = 0; i < 100; i++) {
        if (!g_running) break;
        motor_set_cmd_one(dev, &cmd);
        motor_get_state_one(dev, &state);
        usleep(20000);
    }

    float current_target = current_pos;
    float step = 0.05f;
    int steps = (int)(K_PI / step);

    printf("--- Start Forward Rotation ---\n");
    for (int i = 0; i < steps; i++) {
        if (!g_running) break;
        current_target += step;
        cmd.pos_des = current_target;
        motor_set_cmd_one(dev, &cmd);
        usleep(20000);

        if (motor_get_state_one(dev, &state) == 0 && (i % 10 == 0)) {
            printf("[Command] pos_des: %.2f | [Feedback] pos: %.2f vel: %.2f trq: %.2f err: %u\n",
                cmd.pos_des, state.pos, state.vel, state.trq, state.err);
        }
    }

    printf("--- Start Backward Rotation ---\n");
    for (int i = 0; i < steps; i++) {
        if (!g_running) break;
        current_target -= step;
        cmd.pos_des = current_target;
        motor_set_cmd_one(dev, &cmd);
        usleep(20000);

        if (motor_get_state_one(dev, &state) == 0 && (i % 10 == 0)) {
            printf("[Command] pos_des: %.2f | [Feedback] pos: %.2f vel: %.2f trq: %.2f err: %u\n",
                cmd.pos_des, state.pos, state.vel, state.trq, state.err);
        }
    }

    /* ==================================================================== */
    /* 5. Disable and Cleanup                                               */
    /* ==================================================================== */
    cmd.mode = MOTOR_MODE_IDLE;
    motor_set_cmd_one(dev, &cmd);
    usleep(50000);

    printf("\n--- Checking State After Disable ---\n");
    if (motor_get_state_one(dev, &state) == 0) {
        printf("[After Disable] pos: %.2f vel: %.2f trq: %.2f err: %u\n",
            state.pos, state.vel, state.trq, state.err);
    } else {
        printf("[After Disable] Failed to get motor state.\n");
    }

    motor_free(&dev, 1);
    g_dev = NULL;
    return 0;
}
