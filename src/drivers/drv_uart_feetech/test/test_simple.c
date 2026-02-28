/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "motor.h"

/*
 * @file test_feetech_motor.c
 * @brief Feetech 舵机驱动测试程序
 *
 * 该测试程序使用 motor.h 框架对 Feetech 舵机进行基本的初始化、
 * 位置控制和状态读取测试。
 *
 *
 */

int main(int argc, char *argv[])
{
    struct motor_dev *motors[4] = {NULL};
    struct motor_cmd cmds[4];
    struct motor_state states[4];
    uint32_t num_motors = 2;  // 默认测试2个电机
    uint32_t i;
    int ret;

    /* 解析命令行参数 */
    const char *dev_path = "/dev/ttyACM1";
    uint32_t baud = 1000000;

    if (argc >= 2) {
        dev_path = argv[1];
    }
    if (argc >= 3) {
        baud = atoi(argv[2]);
        if (baud == 0) {
            fprintf(stderr, "Invalid baud rate: %s\n", argv[2]);
            return 1;
        }
    }

    for (i = 0; i < num_motors; i++) {
        uint8_t motor_id = (uint8_t)(i + 1);
        motors[i] = motor_alloc_uart("feetech", dev_path, baud, motor_id, NULL);
        if (!motors[i]) {
            fprintf(stderr, "ERROR: Failed to create motor %u on %s\n", i + 1, dev_path);
            return 1;
        }
        printf("Created motor %u (ID=%u, baud=%u)\n", i + 1, motor_id, baud);
    }

    // 2. 初始化电机
    ret = motor_init(motors, num_motors);
    if (ret != 0) {
        fprintf(stderr, "ERROR: Failed to initialize motors: %d\n", ret);
        goto cleanup;
    }

    /*
     * 3. 位置控制测试
     * pos_des: 0 ~ 4095 (0.087 度)
     * vel_des: 0 ~ 2400 (0.0146 RPM)
     * trq_des: 0 ~ 1000 (0 ~ 最大负载)

    */

    printf("=========================== position ctl ================================\n");
    memset(cmds, 0, sizeof(cmds));
    for (i = 0; i < num_motors; i++) {
        cmds[i].mode = MOTOR_MODE_POS;  /* 位置控制模式 */
        cmds[i].pos_des = 4000;         /* 目标位置: 2*PI */
        cmds[i].vel_des = 2400;         /* 目标速度 */

        cmds[i].trq_des = 0;
    }
    ret = motor_set_cmds(motors, cmds, num_motors);
    if (ret != 0) {
        fprintf(stderr, "ERROR: Failed to set commands: %d\n", ret);
        goto cleanup;
    }
    sleep(3);  // 等待电机移动

    // 4. 读取状态
    ret = motor_get_states(motors, states, num_motors);
    if (ret != 0) {
        fprintf(stderr, "ERROR: Failed to get states: %d\n", ret);
        goto cleanup;
    }

    for (i = 0; i < num_motors; i++) {
        printf("Motor %u: Pos=%.2f, Vel=%.2f, Trq=%.2f, Temp=%.2f, Err=0x%X\n",
            i + 1,
            states[i].pos,
            states[i].vel,
            states[i].trq,
            states[i].temp,
            states[i].err);
    }

    // 恢复 0 位
    memset(cmds, 0, sizeof(cmds));
    for (i = 0; i < num_motors; i++) {
        cmds[i].mode = MOTOR_MODE_POS;
        cmds[i].pos_des = 0;
        cmds[i].vel_des = 2400;
        cmds[i].trq_des = 0;
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
            i + 1,
            states[i].pos,
            states[i].vel,
            states[i].trq,
            states[i].temp,
            states[i].err);
    }

    // 4. 验证模式切换
    printf("=========================== velocity ctl ================================\n");
    memset(cmds, 0, sizeof(cmds));
    for (i = 0; i < num_motors; i++) {
        cmds[i].mode = 1;       /* 速度控制模式 */
        cmds[i].vel_des = 2400; /* 目标速度 */
    }
    ret = motor_set_cmds(motors, cmds, num_motors);
    if (ret != 0) {
        fprintf(stderr, "ERROR: Failed to set commands: %d\n", ret);
        goto cleanup;
    }
    sleep(3);  // 等待电机移动

    // 读取速度控制状态
    ret = motor_get_states(motors, states, num_motors);
    if (ret != 0) {
        fprintf(stderr, "ERROR: Failed to get states: %d\n", ret);
        goto cleanup;
    }

    for (i = 0; i < num_motors; i++) {
        printf("Motor %u: Pos=%.2f, Vel=%.2f, Trq=%.2f, Temp=%.2f, Err=0x%X\n",
            i + 1,
            states[i].pos,
            states[i].vel,
            states[i].trq,
            states[i].temp,
            states[i].err);
    }

    // 停止电机
    memset(cmds, 0, sizeof(cmds));
    for (i = 0; i < num_motors; i++) {
        cmds[i].mode = 1;    /* 速度控制模式 */
        cmds[i].vel_des = 0; /* 目标速度 */
    }
    ret = motor_set_cmds(motors, cmds, num_motors);
    if (ret != 0) {
        fprintf(stderr, "ERROR: Failed to set commands: %d\n", ret);
        goto cleanup;
    }
    sleep(3);  // 等待电机停止

    // 读取停止后状态
    ret = motor_get_states(motors, states, num_motors);
    if (ret != 0) {
        fprintf(stderr, "ERROR: Failed to get states: %d\n", ret);
        goto cleanup;
    }

    for (i = 0; i < num_motors; i++) {
        printf("Motor %u: Pos=%.2f, Vel=%.2f, Trq=%.2f, Temp=%.2f, Err=0x%X\n",
            i + 1,
            states[i].pos,
            states[i].vel,
            states[i].trq,
            states[i].temp,
            states[i].err);
    }

    printf("=========================== position ctl ================================\n");
    memset(cmds, 0, sizeof(cmds));
    for (i = 0; i < num_motors; i++) {
        cmds[i].mode = 0;              /* 位置控制模式 */
        cmds[i].pos_des = 4000;         /* 目标位置: 2*PI */
        cmds[i].vel_des = 2400;         /* 目标速度 */

        cmds[i].trq_des = 0;
    }
    ret = motor_set_cmds(motors, cmds, num_motors);
    if (ret != 0) {
        fprintf(stderr, "ERROR: Failed to set commands: %d\n", ret);
        goto cleanup;
    }
    sleep(3);  // 等待电机移动

    // 4. 读取状态
    ret = motor_get_states(motors, states, num_motors);
    if (ret != 0) {
        fprintf(stderr, "ERROR: Failed to get states: %d\n", ret);
        goto cleanup;
    }

    for (i = 0; i < num_motors; i++) {
        printf("Motor %u: Pos=%.2f, Vel=%.2f, Trq=%.2f, Temp=%.2f, Err=0x%X\n",
            i + 1,
            states[i].pos,
            states[i].vel,
            states[i].trq,
            states[i].temp,
            states[i].err);
    }

    // 恢复 0 位
    memset(cmds, 0, sizeof(cmds));
    for (i = 0; i < num_motors; i++) {
        cmds[i].mode = MOTOR_MODE_POS;
        cmds[i].pos_des = 0;
        cmds[i].vel_des = 2400;
        cmds[i].trq_des = 0;
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
            i + 1,
            states[i].pos,
            states[i].vel,
            states[i].trq,
            states[i].temp,
            states[i].err);
    }

cleanup:
    // 释放电机资源
    motor_free(motors, num_motors);
}
