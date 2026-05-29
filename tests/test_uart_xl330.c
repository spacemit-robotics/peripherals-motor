/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>

#include "motor.h"
#include "test_config.h"

/**
 * @brief Reachy Mini Motor Driver Example (XL330)
 *
 * This example demonstrates how to use the "drv_uart_xl330" driver to control
 * a single motor (e.g., Body Yaw).
 * It implements a sine-wave trajectory (+-170 degrees) to test the motor's
 * full range of movement.
 */

#define BAUDRATE 1000000
#define DEFAULT_PORT "/dev/ttyACM0"

// Parameters for range test
#define AMP_DEG 170.0f
#define FREQ_HZ 0.25f

#define DURATION_S 10.0f
#define CONTROL_PERIOD_US 20000  // 50Hz

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

int main(int argc, char *argv[]) {
    for (int j = 1; j < argc; j++) {
        if (strcmp(argv[j], "-h") == 0 || strcmp(argv[j], "--help") == 0) {
            printf("Usage: %s [options]\n", argv[0]);
            printf("Options:\n");
            printf("  --driver <name>    Set motor driver (default: drv_uart_xl330)\n");
            printf("  --port <port>      Set UART port (default: /dev/ttyACM0)\n");
            printf("  --baud <baudrate>  Set baudrate (default: 1000000)\n");
            printf("  --id <id1,id2...>  Set motor IDs (default: 10)\n");
            printf("  -h, --help         Show this help\n");
            return 0;
        }
    }
    const char *driver = "drv_uart_xl330";
    const char *port = DEFAULT_PORT;
    int baudrate = BAUDRATE;
    int ids[16] = {10};
    int num_motors = 1;

    load_config_and_args(argc, argv, &driver, &port, &baudrate, ids, &num_motors, NULL);

    struct motor_dev *devs[16] = {NULL};
    struct motor_cmd cmds[16];
    struct motor_state states[16];

    printf("[Test] Initializing Reachy Mini motor(s) on %s...\n", port);

    /* 1. Allocate motor device (ID 10 by default) */
    for (int i = 0; i < num_motors; i++) {
        uint8_t id = (uint8_t)ids[i];
        devs[i] = motor_alloc_uart(driver, port, baudrate, id, NULL);
        if (!devs[i]) {
            fprintf(stderr, "Error: Failed to allocate motor ID %d\n", id);
            // Cleanup previously allocated
            motor_free(devs, i);
            return -1;
        }
    }

    /* 2. Initialize motors (set to active mode) */
    if (motor_init(devs, num_motors) != 0) {
        fprintf(stderr, "Error: Failed to initialize motors\n");
        motor_free(devs, num_motors);
        return -1;
    }

    /* Verify initialization success by checking for early async errors (e.g. invalid port) */
    usleep(100000);
    if (motor_get_states(devs, states, num_motors) == 0 && states[0].err != 0) {
        fprintf(stderr, "Error: Motor reported fatal error 0x%X during initialization. Port may be invalid.\n", states[0].err);
        motor_free(devs, num_motors);
        return -1;
    }

    printf("[Test] Motors initialized. Starting sine-wave motion...\n");
    printf("[Test] Amplitude: %.1f deg, Frequency: %.2f Hz, Duration: %.1f s\n", AMP_DEG, FREQ_HZ,
            DURATION_S);

    /* 3. Prepare common command fields */
    for (int i = 0; i < num_motors; i++) {
        cmds[i].mode = MOTOR_MODE_POS;
        cmds[i].vel_des = 0.0f;
        cmds[i].trq_des = 0.0f;
        cmds[i].kp = 0.0f;
        cmds[i].kd = 0.0f;
    }

    struct timespec start_time, current_time;
    clock_gettime(CLOCK_MONOTONIC, &start_time);

    float amp_rad = (float)(AMP_DEG * M_PI / 180.0);

    /* 4. Control Loop */
    while (1) {
        clock_gettime(CLOCK_MONOTONIC, &current_time);
        double elapsed = (current_time.tv_sec - start_time.tv_sec) +
                        (current_time.tv_nsec - start_time.tv_nsec) * 1e-9;

        if (elapsed > DURATION_S) {
            break;
        }

        // Calculate target position (Sine wave)
        float target_pos = amp_rad * sinf((float)(2.0 * M_PI * FREQ_HZ * elapsed));

        for (int i = 0; i < num_motors; i++) {
            cmds[i].pos_des = target_pos;
        }

        // Send commands to all motors
        motor_set_cmds(devs, cmds, num_motors);

        // Read back states
        if (motor_get_states(devs, states, num_motors) == 0) {
            printf("\rTime: %5.2fs | Goal: %6.3f rad | Pos: %6.3f", elapsed, target_pos,
                    states[0].pos);
            if (states[0].err != 0) {
                printf(" | ERR: 0x%X\n", states[0].err);
                fprintf(stderr, "\n[Test] Fatal error (0x%X) detected. Aborting control loop.\n", states[0].err);
                break;
            }
            fflush(stdout);
        } else {
            fprintf(stderr, "\n[Test] Failed to read motor states. Aborting control loop.\n");
            break;
        }

        usleep(CONTROL_PERIOD_US);
    }

    printf("\n[Test] Demo finished. Moving back to zero position...\n");

    /* 5. Return to zero and cleanup */
    for (int i = 0; i < num_motors; i++) {
        cmds[i].pos_des = 0.0f;
    }
    motor_set_cmds(devs, cmds, num_motors);
    sleep(1);

    motor_free(devs, num_motors);
    printf("[Test] All devices released.\n");

    return 0;
}
