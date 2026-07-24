/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * M601C position-loop mode verification test.
 *
 * Purpose:
 *   1. Drive the motor in POSITION closed-loop mode (MOTOR_MODE_POS).
 *   2. Command a position trajectory (e.g., 0 to PI rad).
 *   3. Sample position feedback, and raw DATA[6..7] (0~32767) to verify
 *      position control.
 */

#include "motor.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#define TWO_PI 6.28318530717958647692f

int main(int argc, char **argv) {
    if (argc < 2) {
        printf("Usage: %s <tty_device> [rad_count]\n", argv[0]);
        return -1;
    }

    float rad_count = (argc >= 3) ? (float)atof(argv[2]) : 6.28f; // Default 180 degrees
    if (rad_count < 0.1f) rad_count = 0.1f;
    if (rad_count > 3.0f) rad_count = 3.0f; // Limit to <180 deg to prevent wrapping issues

    struct motor_dev *dev = motor_alloc_uart("drv_485_ddt_m601c111", argv[1], 115200, 1, NULL);
    if (!dev) {
        printf("Failed to allocate M0601C motor\n");
        return -1;
    }
    if (motor_init_one(dev) != 0) {
        printf("Failed to init motor\n");
        return -1;
    }

    struct motor_cmd cmd = {0};
    struct motor_state state = {0};

    cmd.mode = MOTOR_MODE_IDLE;

    /* Delay 1s after serial init */
    printf("\nSerial initialized. Delaying 1s in IDLE ...\n");
    for (int i = 0; i < 2; i++) {
        motor_set_cmd_one(dev, &cmd);
        motor_get_state_one(dev, &state);
        uint16_t cur_pos_raw = 0;
        motor_get_paras(dev, (const void *)(uintptr_t)0x74, &cur_pos_raw, sizeof(cur_pos_raw));
        printf("[Init Delay %4dms] pos_raw=%5u (0x%04X) | pos=%.4f rad\n",
            (i + 1) * 500, cur_pos_raw, cur_pos_raw, state.pos);
        usleep(500000);
    }

    uint16_t start_pos_raw = 0;
    motor_get_state_one(dev, &state);
    motor_get_paras(dev, (const void *)(uintptr_t)0x74, &start_pos_raw, sizeof(start_pos_raw));
    float start_pos = state.pos;
    printf("\nInitial before mode switch: pos=%.4f rad | pos_raw=%u (0x%04X)\n",
        start_pos, start_pos_raw, start_pos_raw);

    /* --- Position-loop verification --- */
    float target_pos = start_pos + rad_count;
    // M0601C is a single-turn absolute encoder on the ROTOR.
    // Ensure we do not cross the 0 / 2*PI boundary to prevent the motor from violently reversing.
    if (target_pos > (2.0f * (float)M_PI - 0.1f)) {
        target_pos = start_pos - rad_count;
    }

    printf("\nSwitching to POSITION mode: target delta=%.4f rad ...\n", target_pos - start_pos);
    printf("NOTE: This angle is the ROTOR angle. Due to gear reduction, the physical output shaft will rotate less!\n");

    cmd.mode = MOTOR_MODE_POS;
    cmd.pos_des = start_pos;
    motor_set_cmd_one(dev, &cmd);

    /* Send target position directly (point-to-point) */
    printf("\nSending target position (point-to-point) ...\n");
    float duration_s = 2.0f;
    int step_ms = 20;
    int steps = (int)(duration_s * 1000.0f / step_ms);

    uint16_t pos_raw_min = 0xFFFF;
    uint16_t pos_raw_max = 0;

    cmd.pos_des = target_pos;

    for (int i = 0; i <= steps; i++) {
        motor_set_cmd_one(dev, &cmd);
        motor_get_state_one(dev, &state);

        uint16_t pos_raw = 0;
        motor_get_paras(dev, (const void *)(uintptr_t)0x74, &pos_raw, sizeof(pos_raw));

        if (pos_raw < pos_raw_min) pos_raw_min = pos_raw;
        if (pos_raw > pos_raw_max) pos_raw_max = pos_raw;

        if (i % 10 == 0) {
            printf("[POS] t=%.2fs des_pos=%.4f cur_pos=%.4f pos_raw=%5u (0x%04X)\n",
                i * step_ms / 1000.0f,
                cmd.pos_des, state.pos,
                pos_raw, pos_raw);
        }
        usleep(step_ms * 1000);
    }

    /* Wait for motor to settle at target position */
    printf("\nWaiting for motor to settle ...\n");
    for (int settle = 0; settle < 25; settle++) {
        motor_set_cmd_one(dev, &cmd);
        motor_get_state_one(dev, &state);

        uint16_t pos_raw = 0;
        motor_get_paras(dev, (const void *)(uintptr_t)0x74, &pos_raw, sizeof(pos_raw));
        if (settle % 5 == 0) {
            printf("[Settle] cur_pos=%.4f pos_raw=%5u\n", state.pos, pos_raw);
        }
        usleep(40000);
    }

    /* Switch back to IDLE */
    cmd.mode = MOTOR_MODE_IDLE;
    for (int i = 0; i < 5; i++) {
        motor_set_cmd_one(dev, &cmd);
        motor_get_state_one(dev, &state);
        usleep(50000);
    }

    uint16_t end_pos_raw = 0;
    motor_get_state_one(dev, &state);
    motor_get_paras(dev, (const void *)(uintptr_t)0x74, &end_pos_raw, sizeof(end_pos_raw));

    printf("\n===== Summary =====\n");
    printf("start: pos=%.4f rad | pos_raw=%u\n", start_pos, start_pos_raw);
    printf("target: pos=%.4f rad\n", target_pos);
    printf("end  : pos=%.4f rad | pos_raw=%u\n", state.pos, end_pos_raw);
    printf("delta: %.4f rad | expected %.4f rad\n", state.pos - start_pos, target_pos - start_pos);
    printf("pos error: %.4f rad (%.2f deg)\n",
        state.pos - target_pos, (state.pos - target_pos) * 180.0f / TWO_PI);
    printf("pos_raw range observed: [%u, %u] (0x%04X ~ 0x%04X)\n",
        pos_raw_min, pos_raw_max, pos_raw_min, pos_raw_max);

    motor_free(&dev, 1);
    return 0;
}
