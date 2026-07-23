/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * M603C position-loop mode verification test.
 *
 * Purpose:
 *   1. Drive the motor in POSITION closed-loop mode (MOTOR_MODE_POS).
 *   2. Command a smooth position trajectory (default: 0.5 revolution = PI rad).
 *   3. Sample position feedback, turns, and raw DATA[6..7] (0~32767) to verify
 *      position control accuracy and 0~32767 full-scale range.
 */

#include "motor.h"
#include "motor_core.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#define TWO_PI 6.28318530717958647692f

int main(int argc, char **argv) {
    if (argc < 2) {
        printf("Usage: %s <tty_device> [rev_count]\n", argv[0]);
        return -1;
    }

    float rev_count = (argc >= 3) ? (float)atof(argv[2]) : 0.5f;
    if (rev_count < 0.05f) rev_count = 0.05f;
    if (rev_count > 10.0f) rev_count = 10.0f;

    struct motor_args_uart args = {
        .dev_path = argv[1],
        .baud = 115200,
        .id = 1,
        .args = NULL,
    };

    struct motor_dev *dev = motor_alloc_uart("drv_uart_ddt_m603c111", argv[1], 115200, 1, &args);
    if (!dev) {
        printf("Failed to allocate M0603C motor\n");
        return -1;
    }
    if (motor_init_one(dev) != 0) {
        printf("Failed to init motor\n");
        return -1;
    }

    struct motor_cmd cmd = {0};
    struct motor_state state = {0};

    cmd.mode = MOTOR_MODE_IDLE;

    /* Delay 3s after serial init (querying pos_raw every 500ms) before switching mode. */
    printf("\nSerial initialized. Delaying 3s in IDLE (querying pos_raw every 500ms) ...\n");
    for (int i = 0; i < 6; i++) {
        motor_set_cmd_one(dev, &cmd);
        motor_get_state_one(dev, &state);
        uint16_t cur_pos_raw = 0;
        motor_get_paras(dev, (const void *)(uintptr_t)0x74, &cur_pos_raw, sizeof(cur_pos_raw));
        printf("[Init Delay %4dms] pos_raw=%5u (0x%04X) | pos=%.4f rad\n",
            (i + 1) * 500, cur_pos_raw, cur_pos_raw, state.pos);
        usleep(500000);
    }

    float start_turns = 0.0f;
    uint16_t start_pos_raw = 0;
    motor_get_state_one(dev, &state);
    motor_get_paras(dev, (const void *)(uintptr_t)0x74, &start_turns, sizeof(start_turns));
    motor_get_paras(dev, (const void *)(uintptr_t)0x74, &start_pos_raw, sizeof(start_pos_raw));
    float start_pos = state.pos;
    printf("\nInitial before mode switch: pos=%.4f rad | turns=%.4f | pos_raw=%u (0x%04X)\n",
        start_pos, start_turns, start_pos_raw, start_pos_raw);

    /* --- Position-loop verification --- */
    float target_pos = start_pos + rev_count * TWO_PI;
    printf("\nSwitching to POSITION mode: target delta=%.2f rev (%.4f rad) ...\n",
        rev_count, rev_count * TWO_PI);

    cmd.mode = MOTOR_MODE_POS;
    cmd.pos_des = start_pos;
    motor_set_cmd_one(dev, &cmd);

    /* Ramp position command over duration */
    printf("\nStarting position trajectory ...\n");
    float duration_s = 3.0f * rev_count;
    if (duration_s < 1.5f) duration_s = 1.5f;
    int step_ms = 20;
    int steps = (int)(duration_s * 1000.0f / step_ms);

    uint16_t pos_raw_min = 0xFFFF;
    uint16_t pos_raw_max = 0;

    for (int i = 0; i <= steps; i++) {
        float alpha = (float)i / (float)steps;
        cmd.pos_des = start_pos + alpha * (target_pos - start_pos);

        motor_set_cmd_one(dev, &cmd);
        motor_get_state_one(dev, &state);

        float turns = 0.0f;
        uint16_t pos_raw = 0;
        motor_get_paras(dev, (const void *)(uintptr_t)0x74, &turns, sizeof(turns));
        motor_get_paras(dev, (const void *)(uintptr_t)0x74, &pos_raw, sizeof(pos_raw));

        if (pos_raw < pos_raw_min) pos_raw_min = pos_raw;
        if (pos_raw > pos_raw_max) pos_raw_max = pos_raw;

        if (i % 10 == 0) {
            printf("[POS] t=%.2fs des_pos=%.4f cur_pos=%.4f dturns=%.4f pos_raw=%5u (0x%04X)\n",
                i * step_ms / 1000.0f,
                cmd.pos_des, state.pos,
                turns - start_turns,
                pos_raw, pos_raw);
        }
        usleep(step_ms * 1000);
    }

    /* Wait for motor to settle at target position */
    printf("\nWaiting for motor to settle ...\n");
    for (int settle = 0; settle < 25; settle++) {
        motor_set_cmd_one(dev, &cmd);
        motor_get_state_one(dev, &state);
        usleep(40000);
    }

    /* Switch back to IDLE */
    cmd.mode = MOTOR_MODE_IDLE;
    for (int i = 0; i < 5; i++) {
        motor_set_cmd_one(dev, &cmd);
        motor_get_state_one(dev, &state);
        usleep(50000);
    }

    float end_turns = 0.0f;
    uint16_t end_pos_raw = 0;
    motor_get_state_one(dev, &state);
    motor_get_paras(dev, (const void *)(uintptr_t)0x74, &end_turns, sizeof(end_turns));
    motor_get_paras(dev, (const void *)(uintptr_t)0x74, &end_pos_raw, sizeof(end_pos_raw));

    printf("\n===== Summary =====\n");
    printf("start: pos=%.4f rad | turns=%.4f | pos_raw=%u\n",
        start_pos, start_turns, start_pos_raw);
    printf("target: pos=%.4f rad\n", target_pos);
    printf("end  : pos=%.4f rad | turns=%.4f | pos_raw=%u\n",
        state.pos, end_turns, end_pos_raw);
    printf("delta: %.4f rad (%.4f turns) | expected %.4f rad\n",
        state.pos - start_pos, end_turns - start_turns, target_pos - start_pos);
    printf("pos error: %.4f rad (%.2f deg)\n",
        state.pos - target_pos, (state.pos - target_pos) * 180.0f / TWO_PI);
    printf("pos_raw range observed: [%u, %u] (0x%04X ~ 0x%04X)\n",
        pos_raw_min, pos_raw_max, pos_raw_min, pos_raw_max);

    motor_free(&dev, 1);
    return 0;
}
