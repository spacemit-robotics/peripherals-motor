/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * 测试用例：验证 drv_can_dm1 驱动的多模式切换（双电机同步）
 * 基于 damiao_pack.cpp (DamiaoHW) 抽象层
 *
 * 测试流程：
 *   1. MIT 阻抗控制 - 正弦轨迹跟踪
 *   2. 位置+速度控制 - 定点运动
 *   3. 速度控制 - 匀速旋转
 *   4. 力位混控 - 位置控制 + 力矩限制
 *   5. 参数读写
 *   6. IDLE 失能
 */

#include <math.h>
#include <stdio.h>
#include <unistd.h>

#include "motor.h"

#define LOOP_PERIOD_US 10000   // 10ms
#define PHASE_STEPS 200        // 每个模式运行 200 步 (2s)
#define SETTLE_TIME_US 500000  // 模式切换前预留 500ms 稳定时间
#define NUM_MOTORS 2

static void print_states(const char* phase, int step, struct motor_state* states, int count) {
    if (step % 50 == 0) {
        for (int j = 0; j < count; j++) {
            printf("[%s] step=%3d  M%d  pos=%.3f  vel=%.3f  trq=%.3f\n", phase, step, j, states[j].pos, states[j].vel,
                    states[j].trq);
        }
    }
}

/* 模式切换前：先减速到零，等待电机稳定 */
static void settle_before_switch(struct motor_dev** devs, int count, const char* next_phase) {
    struct motor_cmd cmd = {};
    struct motor_state state;

    printf("\n  [settle] Decelerating before switching to %s ...\n", next_phase);

    for (int j = 0; j < count; j++) {
        motor_get_state_one(devs[j], &state);
        cmd.mode = MOTOR_MODE_HYBRID;
        cmd.pos_des = state.pos;
        cmd.vel_des = 0.0f;
        cmd.trq_des = 0.0f;
        cmd.kp = 30.0f;
        cmd.kd = 3.0f;

        for (int i = 0; i < 50; i++) {
            motor_set_cmd_one(devs[j], &cmd);
            if (j == count - 1) usleep(LOOP_PERIOD_US);
        }
    }

    usleep(SETTLE_TIME_US);
    printf("  [settle] Ready\n");
}

int main(int argc, char** argv) {
    struct motor_dev* devs[NUM_MOTORS];
    struct motor_state states[NUM_MOTORS];
    struct motor_cmd cmd = {};

    // ========== 分配设备 ==========
    printf("=== Allocating motor devices ===\n");
    devs[0] = motor_alloc_can("drv_can_dm", "can0", 0x02, NULL);
    devs[1] = motor_alloc_can("drv_can_dm", "can0", 0x03, NULL);

    for (int j = 0; j < NUM_MOTORS; j++) {
        if (!devs[j]) {
            printf("Error: Failed to allocate motor %d\n", j);
            return -1;
        }
    }

    // ========== 初始化 ==========
    printf("=== Initializing ===\n");
    if (motor_init(devs, NUM_MOTORS) < 0) {
        printf("Error: Init failed\n");
        return -1;
    }

    printf("=== Init done, starting dual-motor mode switch test ===\n\n");

    // ========== Phase 1: MIT 阻抗控制 ==========
    printf("--- Phase 1: MOTOR_MODE_HYBRID (MIT) ---\n");
    for (int i = 0; i < PHASE_STEPS; i++) {
        float t = i * 0.01f;
        float phase = 2.0f * M_PI * t / 2.0f;

        cmd.mode = MOTOR_MODE_HYBRID;
        cmd.pos_des = sinf(phase) * 2.0f;
        cmd.vel_des = cosf(phase) * 2.0f * M_PI;
        cmd.trq_des = 0.0f;
        cmd.kp = 25.0f;
        cmd.kd = 1.5f;

        // 两个电机发送相同指令，同步运动
        struct motor_cmd cmds[NUM_MOTORS] = {cmd, cmd};
        motor_set_cmds(devs, cmds, NUM_MOTORS);
        motor_get_states(devs, states, NUM_MOTORS);
        print_states("MIT", i, states, NUM_MOTORS);
        usleep(LOOP_PERIOD_US);
    }

    // ========== Phase 2: 位置+速度控制 ==========
    settle_before_switch(devs, NUM_MOTORS, "MOTOR_MODE_POS");
    printf("--- Phase 2: MOTOR_MODE_POS (POS_VEL) ---\n");
    for (int i = 0; i < PHASE_STEPS; i++) {
        cmd.mode = MOTOR_MODE_POS;
        cmd.pos_des = 2.0f;
        cmd.vel_des = 1.0f;
        cmd.trq_des = 0.0f;
        cmd.kp = 0.0f;
        cmd.kd = 0.0f;

        struct motor_cmd cmds[NUM_MOTORS] = {cmd, cmd};
        motor_set_cmds(devs, cmds, NUM_MOTORS);
        motor_get_states(devs, states, NUM_MOTORS);
        print_states("POS", i, states, NUM_MOTORS);
        usleep(LOOP_PERIOD_US);
    }

    // ========== Phase 3: 纯速度控制 ==========
    settle_before_switch(devs, NUM_MOTORS, "MOTOR_MODE_VEL");
    printf("--- Phase 3: MOTOR_MODE_VEL ---\n");
    for (int i = 0; i < PHASE_STEPS; i++) {
        cmd.mode = MOTOR_MODE_VEL;
        cmd.pos_des = 0.0f;
        cmd.vel_des = 5.0f;
        cmd.trq_des = 0.0f;
        cmd.kp = 0.0f;
        cmd.kd = 0.0f;

        struct motor_cmd cmds[NUM_MOTORS] = {cmd, cmd};
        motor_set_cmds(devs, cmds, NUM_MOTORS);
        motor_get_states(devs, states, NUM_MOTORS);
        print_states("VEL", i, states, NUM_MOTORS);
        usleep(LOOP_PERIOD_US);
    }

    // ========== Phase 4: 力位混控 ==========
    settle_before_switch(devs, NUM_MOTORS, "MOTOR_MODE_TRQ");
    printf("--- Phase 4: MOTOR_MODE_TRQ (POS_FORCE) ---\n");
    for (int i = 0; i < PHASE_STEPS; i++) {
        cmd.mode = MOTOR_MODE_TRQ;
        cmd.pos_des = 1.0f;
        cmd.vel_des = 10.0f;
        cmd.trq_des = 0.5f;
        cmd.kp = 0.0f;
        cmd.kd = 0.0f;

        struct motor_cmd cmds[NUM_MOTORS] = {cmd, cmd};
        motor_set_cmds(devs, cmds, NUM_MOTORS);
        motor_get_states(devs, states, NUM_MOTORS);
        print_states("TRQ", i, states, NUM_MOTORS);
        usleep(LOOP_PERIOD_US);
    }

    // ========== Phase 5: 参数读写测试（对两个电机） ==========
    printf("\n--- Phase 5: Parameter Read/Write ---\n");
    for (int j = 0; j < NUM_MOTORS; j++) {
        float val = 0.0f;
        uint8_t reg_max_spd = 6;

        if (motor_get_paras(devs[j], (const void*)(uintptr_t)reg_max_spd, &val, sizeof(float)) == 0) {
            printf("Motor %d: Read MAX_SPD = %.2f\n", j, val);
        }

        float new_val = 25.0f;
        if (motor_set_paras(devs[j], (const void*)(uintptr_t)reg_max_spd, &new_val, sizeof(float)) == 0) {
            printf("Motor %d: Write MAX_SPD = %.2f OK\n", j, new_val);
        }
    }

    // ========== Phase 6: IDLE 失能 ==========
    printf("\n--- Phase 6: MOTOR_MODE_IDLE ---\n");
    cmd.mode = MOTOR_MODE_IDLE;
    cmd.pos_des = 0.0f;
    cmd.vel_des = 0.0f;
    cmd.trq_des = 0.0f;
    cmd.kp = 0.0f;
    cmd.kd = 0.0f;
    {
        struct motor_cmd cmds[NUM_MOTORS] = {cmd, cmd};
        motor_set_cmds(devs, cmds, NUM_MOTORS);
    }
    printf("All motors disabled\n");

    // ========== 释放 ==========
    printf("\n=== Releasing motors ===\n");
    motor_free(devs, NUM_MOTORS);

    printf("=== Test complete ===\n");
    return 0;
}
