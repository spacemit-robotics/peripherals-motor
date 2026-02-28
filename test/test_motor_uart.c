/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * @file test_motor_uart.c
 * @brief UART 电机通用测试程序
 *
 * 支持命令行参数配置，用于测试级联在同一总线上的多个 UART 电机。
 * 用法: test_motor_uart [dev_path] [baud] [driver] [num_motors]
 *   默认: /dev/ttyACM1 1000000 feetech 2
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include "motor.h"

int main(int argc, char *argv[])
{
    struct motor_dev *motors[8] = {NULL};
    struct motor_cmd cmds[8];
    struct motor_state states[8];
    uint32_t num_motors = 2;
    uint32_t i;
    int ret = 0;

    const char *dev_path = "/dev/ttyACM1";
    uint32_t baud = 1000000;
    const char *driver = "feetech";

    /* 解析命令行: [dev_path] [baud] [driver] [num_motors] */
    if (argc >= 2) dev_path = argv[1];
    if (argc >= 3) {
        baud = atoi(argv[2]);
        if (baud == 0) {
            fprintf(stderr, "Invalid baud rate: %s\n", argv[2]);
            return 1;
        }
    }
    if (argc >= 4) driver = argv[3];
    if (argc >= 5) {
        num_motors = atoi(argv[4]);
        if (num_motors == 0 || num_motors > 8) {
            fprintf(stderr, "Invalid num_motors: %s (1-8)\n", argv[4]);
            return 1;
        }
    }

    printf("=== UART Motor Test ===\n");
    printf("  dev_path=%s, baud=%u, driver=%s, num_motors=%u\n\n",
           dev_path, baud, driver, num_motors);

    /* 1. 创建电机实例（级联在同一总线上，通过 id 区分） */
    for (i = 0; i < num_motors; i++) {
        uint8_t motor_id = (uint8_t)(i + 1);
        motors[i] = motor_alloc_uart(driver, dev_path, baud, motor_id, NULL);
        if (!motors[i]) {
            fprintf(stderr, "ERROR: Failed to create motor %u on %s\n", i + 1, dev_path);
            ret = -1;
            goto cleanup;
        }
        printf("  Created motor %u (ID=%u)\n", i + 1, motor_id);
    }
    printf("\n");

    /* 2. 初始化 */
    ret = motor_init(motors, num_motors);
    if (ret != 0) {
        fprintf(stderr, "ERROR: Failed to initialize motors: %d\n", ret);
        goto cleanup;
    }
    printf("[2] All motors initialized\n\n");

    /* 3. 位置控制测试 */
    printf("=========================== position ctl ================================\n");
    memset(cmds, 0, sizeof(cmds));
    for (i = 0; i < num_motors; i++) {
        cmds[i].mode = MOTOR_MODE_POS;
        cmds[i].pos_des = 4000;
        cmds[i].vel_des = 2400;
    }
    ret = motor_set_cmds(motors, cmds, num_motors);
    if (ret != 0) {
        fprintf(stderr, "ERROR: Failed to set commands: %d\n", ret);
        goto cleanup;
    }
    sleep(3);

    ret = motor_get_states(motors, states, num_motors);
    if (ret != 0) {
        fprintf(stderr, "ERROR: Failed to get states: %d\n", ret);
        goto cleanup;
    }
    for (i = 0; i < num_motors; i++) {
        printf("Motor %u: Pos=%.2f, Vel=%.2f, Trq=%.2f, Temp=%.2f, Err=0x%X\n",
               i + 1, states[i].pos, states[i].vel, states[i].trq,
               states[i].temp, states[i].err);
    }

    /* 恢复 0 位 */
    memset(cmds, 0, sizeof(cmds));
    for (i = 0; i < num_motors; i++) {
        cmds[i].mode = MOTOR_MODE_POS;
        cmds[i].pos_des = 0;
        cmds[i].vel_des = 2400;
    }
    motor_set_cmds(motors, cmds, num_motors);
    sleep(3);

    ret = motor_get_states(motors, states, num_motors);
    if (ret != 0) {
        fprintf(stderr, "ERROR: Failed to get states: %d\n", ret);
        goto cleanup;
    }
    for (i = 0; i < num_motors; i++) {
        printf("Motor %u: Pos=%.2f, Vel=%.2f, Trq=%.2f, Temp=%.2f, Err=0x%X\n",
               i + 1, states[i].pos, states[i].vel, states[i].trq,
               states[i].temp, states[i].err);
    }

    /* 4. 速度控制测试 */
    printf("=========================== velocity ctl ================================\n");
    memset(cmds, 0, sizeof(cmds));
    for (i = 0; i < num_motors; i++) {
        cmds[i].mode = MOTOR_MODE_VEL;
        cmds[i].vel_des = 2400;
    }
    motor_set_cmds(motors, cmds, num_motors);
    sleep(3);

    motor_get_states(motors, states, num_motors);
    for (i = 0; i < num_motors; i++) {
        printf("Motor %u: Pos=%.2f, Vel=%.2f, Trq=%.2f, Temp=%.2f, Err=0x%X\n",
               i + 1, states[i].pos, states[i].vel, states[i].trq,
               states[i].temp, states[i].err);
    }

    /* 停止 */
    memset(cmds, 0, sizeof(cmds));
    for (i = 0; i < num_motors; i++) {
        cmds[i].mode = MOTOR_MODE_VEL;
        cmds[i].vel_des = 0;
    }
    motor_set_cmds(motors, cmds, num_motors);
    sleep(3);

    ret = motor_get_states(motors, states, num_motors);
    if (ret != 0) {
        fprintf(stderr, "ERROR: Failed to get states: %d\n", ret);
        goto cleanup;
    }
    for (i = 0; i < num_motors; i++) {
        printf("Motor %u: Pos=%.2f, Vel=%.2f, Trq=%.2f, Temp=%.2f, Err=0x%X\n",
               i + 1, states[i].pos, states[i].vel, states[i].trq,
               states[i].temp, states[i].err);
    }

    /* 5. 第二轮位置控制 */
    printf("=========================== position ctl ================================\n");
    memset(cmds, 0, sizeof(cmds));
    for (i = 0; i < num_motors; i++) {
        cmds[i].mode = MOTOR_MODE_POS;
        cmds[i].pos_des = 4000;
        cmds[i].vel_des = 2400;
    }
    ret = motor_set_cmds(motors, cmds, num_motors);
    if (ret != 0) {
        fprintf(stderr, "ERROR: Failed to set commands: %d\n", ret);
        goto cleanup;
    }
    sleep(3);

    ret = motor_get_states(motors, states, num_motors);
    if (ret != 0) {
        fprintf(stderr, "ERROR: Failed to get states: %d\n", ret);
        goto cleanup;
    }
    for (i = 0; i < num_motors; i++) {
        printf("Motor %u: Pos=%.2f, Vel=%.2f, Trq=%.2f, Temp=%.2f, Err=0x%X\n",
               i + 1, states[i].pos, states[i].vel, states[i].trq,
               states[i].temp, states[i].err);
    }

    /* 恢复 0 位 */
    memset(cmds, 0, sizeof(cmds));
    for (i = 0; i < num_motors; i++) {
        cmds[i].mode = MOTOR_MODE_POS;
        cmds[i].pos_des = 0;
        cmds[i].vel_des = 2400;
    }
    motor_set_cmds(motors, cmds, num_motors);
    sleep(3);

    ret = motor_get_states(motors, states, num_motors);
    if (ret != 0) {
        fprintf(stderr, "ERROR: Failed to get states: %d\n", ret);
        goto cleanup;
    }
    for (i = 0; i < num_motors; i++) {
        printf("Motor %u: Pos=%.2f, Vel=%.2f, Trq=%.2f, Temp=%.2f, Err=0x%X\n",
               i + 1, states[i].pos, states[i].vel, states[i].trq,
               states[i].temp, states[i].err);
    }

    printf("=== Test completed ===\n");

cleanup:
    motor_free(motors, num_motors);
    return (ret != 0) ? 1 : 0;
}
