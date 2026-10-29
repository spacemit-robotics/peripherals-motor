/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Move one HD1910 a small distance toward its configured zero, then release it.
 * Usage: test_hd1910_small_move [uart-device] [motor-id]
 */
#define _POSIX_C_SOURCE 200809L
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "motor.h"
#include "feetech_hd1910_adapter.h"

enum { MAX_READS = 30 };
static const float kStepRad = 0.3f;
static const float kToleranceRad = 0.02f;

int main(int argc, char **argv) {
    const char *port = argc > 1 ? argv[1] : "/dev/ttyACM0";
    char *end = NULL;
    unsigned long id = argc > 2 ? strtoul(argv[2], &end, 10) : 1;
    if (argc > 3 || (argc > 2 && (argv[2][0] == '-' || !end || *end)) || id >= 0xfe) {
        fprintf(stderr, "Usage: %s [uart-device] [motor-id: 0..253]\n", argv[0]);
        return 2;
    }

    struct feetech_hd1910_config config = {
        .zero_offset_ticks = 2048, .direction = 1, .firmware_p = 5,
    };
    struct motor_dev *motor = motor_alloc_uart(
        "drv_uart_feetech_hd1910", port, 1000000, (uint8_t)id, &config);
    if (!motor) {
        fprintf(stderr, "HD1910 allocation failed: %s id=%lu\n", port, id);
        return 1;
    }

    int result = 1;
    struct motor_state initial = {0};
    if (motor_init_one(motor) != 0)
        fprintf(stderr, "HD1910 initialization read failed; retrying feedback\n");
    int have_feedback = 0;
    for (int i = 0; i < 5; ++i) {
        if (motor_get_state_one(motor, &initial) == 0) {
            have_feedback = 1;
            break;
        }
        struct timespec pause = {.tv_sec = 0, .tv_nsec = 100000000L};
        nanosleep(&pause, NULL);
    }
    if (!have_feedback || !isfinite(initial.pos) || fabsf(initial.pos) > 3.14f) {
        fprintf(stderr, "HD1910 initial feedback unavailable or out of range; no move sent\n");
        goto cleanup;
    }

    const float target = initial.pos + (initial.pos > 0.0f ? -kStepRad : kStepRad);
    struct motor_cmd cmd = {0};
    cmd.mode = MOTOR_MODE_POS;
    cmd.pos_des = target;
    printf("HD1910 id=%lu: initial=%.4f rad, target=%.4f rad\n",
        id, initial.pos, target);
    fflush(stdout);
    if (motor_set_cmd_one(motor, &cmd) != 0) {
        fprintf(stderr, "HD1910 position command failed\n");
        goto cleanup;
    }

    for (int i = 0; i < MAX_READS; ++i) {
        struct timespec pause = {.tv_sec = 0, .tv_nsec = 100000000L};
        nanosleep(&pause, NULL);
        struct motor_state state = {0};
        if (motor_get_state_one(motor, &state) != 0) {
            fprintf(stderr, "HD1910 feedback read failed after movement\n");
            goto cleanup;
        }
        if (fabsf(state.pos - target) <= kToleranceRad) {
            printf("HD1910 reached %.4f rad (target %.4f rad)\n", state.pos, target);
            result = 0;
            break;
        }
        if (i == MAX_READS - 1)
            fprintf(stderr, "HD1910 did not reach target: final=%.4f rad\n", state.pos);
    }

cleanup:
    {
        struct motor_cmd idle = {0};
        idle.mode = MOTOR_MODE_IDLE;
        if (motor_set_cmd_one(motor, &idle) != 0) {
            fprintf(stderr, "HD1910 idle cleanup failed\n");
            result = 1;
        }
    }
    motor_free(&motor, 1);
    return result;
}
