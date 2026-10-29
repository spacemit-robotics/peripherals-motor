/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Hardware smoke test for the HD1910 motor adapter.
 * Poll feedback at 2 Hz for 5 minutes, even if initialization gets no reply.
 * Usage: test_hd1910_smoke [uart-device] [motor-id]
 */
#define _POSIX_C_SOURCE 200809L
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "motor.h"
#include "feetech_hd1910_adapter.h"

enum { POLL_HZ = 2, DURATION_SECONDS = 5 * 60 };

static int wait_until(const struct timespec *deadline) {
    int error;
    do {
        error = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, deadline, NULL);
    } while (error == EINTR);
    if (error != 0) {
        errno = error;
        perror("HD1910 poll wait failed");
        return -1;
    }
    return 0;
}

int main(int argc, char **argv) {
    const char *port = argc > 1 ? argv[1] : "/dev/ttyACM1";
    char *end = NULL;
    unsigned long parsed_id = argc > 2 ? strtoul(argv[2], &end, 10) : 1;
    if ((argc > 2 && (!end || *end != '\0')) || parsed_id > UINT8_MAX) {
        fprintf(stderr, "Usage: %s [uart-device] [motor-id: 0..255]\n", argv[0]);
        return 2;
    }

    struct feetech_hd1910_config config = {
        .zero_offset_ticks = 2048,
        .direction = 1,
        .firmware_p = 5,
    };
    struct motor_dev *motor = motor_alloc_uart(
        "drv_uart_feetech_hd1910", port, 1000000, (uint8_t)parsed_id, &config);
    if (!motor) {
        fprintf(stderr, "HD1910 allocation failed: port=%s id=%lu\n", port,
                parsed_id);
        return 1;
    }

    int result = 1;
    if (motor_init_one(motor) != 0) {
        fprintf(stderr, "HD1910 initialization read failed; continuing to send "
                        "feedback reads at %d Hz for %d seconds\n",
                POLL_HZ, DURATION_SECONDS);
    }
    struct timespec next_poll;
    if (clock_gettime(CLOCK_MONOTONIC, &next_poll) != 0) {
        perror("HD1910 poll clock failed");
        goto cleanup;
    }

    unsigned int successes = 0, failures = 0;
    const unsigned int total_polls = POLL_HZ * DURATION_SECONDS;
    printf("HD1910 polling: %d Hz for %d seconds (%u reads)\n",
        POLL_HZ, DURATION_SECONDS, total_polls);
    fflush(stdout);
    for (unsigned int i = 0; i < total_polls; ++i) {
        struct motor_state state = {0};
        if (motor_get_state_one(motor, &state) != 0) {
            ++failures;
            fprintf(stderr, "HD1910 feedback read failed [%u/%u]\n",
                i + 1, total_polls);
        } else {
            ++successes;
            printf("HD1910 feedback [%u/%u]: pos=%.4f rad vel=%.4f rad/s "
                "load=%.0f temp=%.0f err=0x%x\n",
                i + 1, total_polls, state.pos, state.vel, state.trq,
                state.temp, state.err);
            fflush(stdout);
        }

        /* Absolute deadlines prevent read latency from accumulating as drift. */
        next_poll.tv_nsec += 1000000000L / POLL_HZ;
        if (next_poll.tv_nsec >= 1000000000L) {
            ++next_poll.tv_sec;
            next_poll.tv_nsec -= 1000000000L;
        }
        if (wait_until(&next_poll) != 0) {
            goto cleanup;
        }
    }
    printf("HD1910 polling complete: success=%u failed=%u\n", successes, failures);
    result = failures == 0 ? 0 : 1;

cleanup:
    /* Ensure any future test extension leaves the motor in the idle mode. */
    {
        struct motor_cmd cmd = {0};
        cmd.mode = MOTOR_MODE_IDLE;
        if (motor_set_cmd_one(motor, &cmd) != 0 && result == 0) {
            fprintf(stderr, "HD1910 idle cleanup failed\n");
            result = 1;
        }
    }
    motor_free(&motor, 1);
    return result;
}
