/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * 测试用例：验证 drv_485_ddt_m601c111 (本末 M0601C-111) 驱动。
 *
 * 特点：
 * 1. 仅依赖外部标准头文件与 `<motor.h>` 通用接口，去除了对底层特定头文件的依赖。
 * 2. 依次测试电流环、速度环、位置环。
 * 3. 针对 M0601C 不支持开环的特性，跳过了开环测试。
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
        printf("Usage: %s <tty_device>\n", argv[0]);
        return -1;
    }

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    /* 分配 M0601C 驱动实例 */
    struct motor_dev *dev = motor_alloc_uart("drv_485_ddt_m601c111", argv[1], 115200, 1, NULL);
    if (!dev) {
        printf("Failed to allocate motor\n");
        return -1;
    }

    g_dev = dev;

    if (motor_init_one(dev) != 0) {
        printf("Failed to init motor\n");
        return -1;
    }

    printf("Ensuring motor is disabled and stable before starting...\n");
    struct motor_cmd cmd = {0};
    cmd.mode = MOTOR_MODE_IDLE;
    struct motor_state state = {0};

    int read_failures = 0;
    for (int i = 0; i < 20; i++) {
        motor_set_cmd_one(dev, &cmd);
        if (motor_get_state_one(dev, &state) < 0) {
            read_failures++;
        }
        usleep(50000); // 50ms (由于串口通信为一问一答，无需太快)
    }

    if (read_failures > 5) {
        printf("Error: Too many state read failures (%d/20).\n", read_failures);
        motor_free(&dev, 1);
        return -1;
    }

    float initial_position = state.pos;
    printf("Initial physical position: %.2f rad (read failures: %d/20)\n", initial_position, read_failures);

    printf("Waiting 3 seconds before enabling motor... Observe if position drifts.\n");
    for (int i = 0; i < 30; i++) {
        if (!g_running) break;
        if (motor_get_state_one(dev, &state) == 0) {
            if (i % 10 == 0) {
                printf("[PRE-ENABLE] pos: %.3f rad, vel: %.3f rad/s, trq: %.3f A, err: 0x%02X\n",
                    state.pos, state.vel, state.trq, state.err);
            }
        }
        usleep(100000); /* 100ms * 30 = 3 seconds */
    }

    /* ==================================================================== */
    /* 1. Current Loop                                                      */
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

    float safe_torque = 0.5f; // M0601C 电流量程较大 (8A)，此处使用 0.5A

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
    /* 2. Velocity Loop                                                     */
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

    float safe_velocity = 5.0f; // rad/s

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

    sleep(2);

    /* ==================================================================== */
    /* 3. Position Loop                                                     */
    /* ==================================================================== */
    printf("\nSending IDLE command to safely get current actual position before Position Loop...\n");
    cmd.mode = MOTOR_MODE_IDLE;
    motor_set_cmd_one(dev, &cmd);

    motor_get_state_one(dev, &state); // Re-read position to align target

    // [与 test_once_pos 保持完全一致] 先用上一帧 IDLE 获取到的 pos 作为基准原位使能
    printf("--- Engaging POS mode at current position to lock rotor safely ---\n");
    cmd.mode = MOTOR_MODE_POS;
    cmd.pos_des = state.pos;
    motor_set_cmd_one(dev, &cmd);

    // [与 test_once_pos 保持完全一致] 在锁定状态下获取 0x74，既能更新当前绝对位置，又能防止通信超时
    uint16_t current_pos_raw = 0;
    if (motor_get_paras(dev, (const void *)(uintptr_t)0x74, &current_pos_raw, sizeof(current_pos_raw)) == 0) {
        printf("Current pos_raw: %u (0x%04X)\n", current_pos_raw, current_pos_raw);
        state.pos = ((float)current_pos_raw / 32767.0f) * (2.0f * (float)M_PI);
    }

    // 此时由于 0x74 的问答，电机已经稳定了一小段时间且没有触发超时断联
    float target_pos = state.pos + (float)M_PI;
    printf("--- Start Point-to-Point Position (Target: %.2f) (Forward) ---\n", target_pos);

    cmd.pos_des = target_pos;
    motor_set_cmd_one(dev, &cmd);

    // 位置环模式只需发送一次目标位置，等待运行到位
    sleep(5);

    // 重新发一次指令以触发通信，获取最新状态
    motor_set_cmd_one(dev, &cmd);
    if (motor_get_state_one(dev, &state) == 0) {
        uint16_t pos_raw = (uint16_t)state.temp;
        printf("[Final Feedback] pos: %.2f (raw: %5u, 0x%04X) vel: %.2f trq: %.2f err: %u\n",
            state.pos, pos_raw, pos_raw, state.vel, state.trq, state.err);
    }

    /* ==================================================================== */
    /* 4. Disable and Cleanup                                               */
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
