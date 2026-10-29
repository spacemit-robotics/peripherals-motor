/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Move each HD1910 +0.1 rad from its initial position, then read feedback.
 * Usage: test_hd1910_feedback <uart-device> <id> [id ...]
 */
#define _POSIX_C_SOURCE 200809L
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "feetech_hd1910_adapter.h"
#include "motor.h"

enum { MAX_MOTORS = 35, SAMPLES = 5 };

int main(int argc, char **argv) {
    if (argc < 3 || argc - 2 > MAX_MOTORS) {
        fprintf(stderr, "Usage: %s <uart-device> <id> [id ...] (max %d)\n", argv[0], MAX_MOTORS);
        return 2;
    }
    struct motor_dev *motors[MAX_MOTORS] = {0};
    struct motor_state states[MAX_MOTORS] = {0};
    struct motor_cmd cmds[MAX_MOTORS] = {0};
    float initial[MAX_MOTORS] = {0};
    uint8_t ids[MAX_MOTORS] = {0};
    const int count = argc - 2;
    int allocated = 0;
    int result = 1;
    int motion_attempted = 0;
    struct feetech_hd1910_config config = {2048, 1, 5};
    for (int i = 0; i < count; ++i) {
        char *end = NULL;
        unsigned long id = strtoul(argv[i + 2], &end, 10);
        if (!argv[i + 2][0] || !end || *end || id > 253) {
            fprintf(stderr, "Invalid ID: %s\n", argv[i + 2]);
            goto cleanup;
        }
        ids[i] = (uint8_t)id;
        for (int j = 0; j < i; ++j) {
            if (ids[j] == ids[i]) {
                fprintf(stderr, "Duplicate ID: %u\n", ids[i]);
                goto cleanup;
            }
        }
        motors[i] = motor_alloc_uart("drv_uart_feetech_hd1910", argv[1], 1000000, ids[i], &config);
        if (!motors[i]) {
            fprintf(stderr, "Allocation failed: ID %u\n", ids[i]);
            goto cleanup;
        }
        ++allocated;
    }
    for (int i = 0; i < count; ++i) {
        const uint8_t address = 40;
        uint8_t torque = 0;
        if (motor_get_paras(motors[i], &address, &torque, 1) != 0 || torque != 0) {
            fprintf(stderr, "ID %u torque is on or could not be read; no motion sent\n", ids[i]);
            goto cleanup;
        }
    }
    if (feetech_hd1910_get_states_sync(motors, states, (uint32_t)count) != 0) {
        fprintf(stderr, "Initial synchronous feedback failed; no motion sent\n");
        goto cleanup;
    }
    for (int i = 0; i < count; ++i) {
        initial[i] = states[i].pos;
        cmds[i].mode = MOTOR_MODE_POS;
        cmds[i].pos_des = initial[i] + 0.1f;
        printf("id=%u initial=%.5f target=%.5f rad\n", ids[i], initial[i], cmds[i].pos_des);
    }
    fflush(stdout);
    motion_attempted = 1;
    if (feetech_hd1910_set_cmds_sync(motors, cmds, (uint32_t)count) != 0) {
        fprintf(stderr, "Synchronous position command failed\n");
        goto cleanup;
    }
    for (int sample = 0; sample < SAMPLES; ++sample) {
        struct timespec delay = {0, 500000000};
        nanosleep(&delay, NULL);
        if (feetech_hd1910_get_states_sync(motors, states, (uint32_t)count) != 0) {
            fprintf(stderr, "Synchronous feedback failed at sample %d\n", sample + 1);
            goto cleanup;
        }
        for (int i = 0; i < count; ++i) {
            printf("sample=%d id=%u pos=%.5f delta=%.5f target=%.5f rad vel=%.5f rad/s "
                "load=%.0f temp=%.0f err=0x%x\n", sample + 1, ids[i], states[i].pos,
                states[i].pos - initial[i], cmds[i].pos_des, states[i].vel,
                states[i].trq, states[i].temp, states[i].err);
        }
        fflush(stdout);
    }
    result = 0;
    for (int i = 0; i < count; ++i) {
        if (fabsf(states[i].pos - cmds[i].pos_des) > 0.02f) {
            fprintf(stderr, "ID %u did not reach target within 0.02 rad\n", ids[i]);
            result = 1;
        }
    }
cleanup:
    if (motion_attempted) {
        for (int i = 0; i < allocated; ++i) {
            struct motor_cmd idle = {0};
            const uint8_t address = 40, off = 0;
            uint8_t torque = 0xff;
            idle.mode = MOTOR_MODE_IDLE;
            if (motor_set_cmd_one(motors[i], &idle) != 0 ||
                motor_get_paras(motors[i], &address, &torque, 1) != 0 || torque != 0) {
                result = 1;
                /* A failed arm can leave torque on before the driver's armed flag is set. */
                if (motor_set_paras(motors[i], &address, &off, 1) != 0 ||
                    motor_get_paras(motors[i], &address, &torque, 1) != 0 || torque != 0) {
                    fprintf(stderr, "ID %u torque off could not be confirmed\n", ids[i]);
                    result = 1;
                }
            }
        }
    }
    motor_free(motors, (uint32_t)allocated);
    return result;
}
