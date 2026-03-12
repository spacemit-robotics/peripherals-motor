/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * 使用示例：测试 dm_can 驱动在 motor_framework 下的工作情况
 * 仅依赖 motor.h 必须的头文件
 */

#include <math.h>
#include <stdio.h>
#include <unistd.h>

#include "motor.h"

int main(int argc, char** argv) {
    struct motor_dev* dev;
    struct motor_state state;
    struct motor_cmd cmd;

    // 1. 查找并分配驱动 (使用公共 factory function)
    printf("Allocating motor device via motor_alloc_can...\n");
    // 注意：can0 是接口名，0x02 是电机 ID
    dev = motor_alloc_can("motor0", "can0", 0x02, NULL);
    if (!dev) {
        printf("Error: Failed to allocate motor device. Check driver registration.\n");
        return -1;
    }

    // 2. 初始化
    printf("Initializing motor...\n");
    if (motor_init_one(dev) < 0) {
        printf("Error: Init failed.\n");
        return -1;
    }

    printf("Initialized. Starting loop...\n");

    // 3. 控制循环
    float t = 0.0f;
    for (int i = 0; i < 1000; i++) {
        // Get state
        if (motor_get_state_one(dev, &state) == 0) {
            printf("State: pos=%.3f, vel=%.3f, trq=%.3f\n", state.pos, state.vel, state.trq);
        } else {
            // printf("Get state failed (maybe first frame not arrived)\n");
        }

        // Set cmd (Sine wave)
        cmd.mode = MOTOR_MODE_HYBRID;  // MIT
        cmd.pos_des = sin(t) * 10.0f;  // amplitude 10 rad
        cmd.vel_des = cos(t) * 10.0f;
        cmd.trq_des = 0.0f;
        cmd.kp = 20.0f;
        cmd.kd = 2.0f;

        if (motor_set_cmd_one(dev, &cmd) < 0) {
            printf("Set cmd failed.\n");
        }

        t += 0.01f;
        usleep(10000);  // 10ms
    }

    // 4. 释放
    // motor_free 接受 dev 数组指针
    motor_free(&dev, 1);

    return 0;
}
