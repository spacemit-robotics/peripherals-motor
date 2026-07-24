/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#include "motor.h"
#include <stdio.h>
#include <unistd.h>

int main(int argc, char **argv) {
    if (argc < 2) {
        printf("Usage: %s <tty_device>\n", argv[0]);
        return -1;
    }

    /* Allocate motor device. motor_alloc_uart handles the internal args struct. */
    struct motor_dev *dev = motor_alloc_uart("drv_uart_ddt_m603c111", argv[1], 115200, 1, NULL);
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

    // Spin in velocity mode for a bit
    cmd.mode = MOTOR_MODE_VEL;
    cmd.vel_des = 1.0f; // 1 rad/s

    // Set command once to trigger mode switch
    motor_set_cmd_one(dev, &cmd);

    // Query mode and print
    uint8_t mode_val = 0xFF;
    if (motor_set_paras(dev, (const void *)(uintptr_t)0x75, &mode_val, 1) == 0) {
        printf("Queried Motor Mode: 0x%02X\n", mode_val);
    } else {
        printf("Failed to query Motor Mode\n");
    }

    for (int i = 0; i < 50; i++) {
        motor_set_cmd_one(dev, &cmd);
        motor_get_state_one(dev, &state);
        float raw_turns = 0.0f;
        motor_get_paras(dev, (const void *)(uintptr_t)0x74, &raw_turns, 4);
        printf("State: vel=%.2f trq=%.2f temp=%.1f pos=%.2f rad (%.2f turns)\n",
                state.vel, state.trq, state.temp, state.pos, raw_turns);
        usleep(100000);
    }

    // Smoothly decelerate to 0 in velocity mode first
    printf("\nDecelerating to 0 rad/s...\n");
    cmd.vel_des = 0.0f;
    for (int i = 0; i < 10; i++) {
        motor_set_cmd_one(dev, &cmd);
        usleep(100000); // Wait total 1 second for motor to fully stop
    }

    // Now safely switch to Open-Loop mode
    printf("Switching to Open-Loop Mode...\n");
    cmd.mode = MOTOR_MODE_OPEN;
    cmd.trq_des = 1.5f; // Increased to 1.5A to overcome static friction and inverter dead-zone

    // We loop here just to be absolutely sure the command goes through
    for (int retries = 0; retries < 5; retries++) {
        if (motor_set_cmd_one(dev, &cmd) != 0) {
            printf("Failed to set open loop mode (retry %d)\n", retries);
        } else {
            printf("Successfully set open loop mode!\n");
            break;
        }
        usleep(100000);
    }

    mode_val = 0xFF;
    if (motor_set_paras(dev, (const void *)(uintptr_t)0x75, &mode_val, 1) == 0) {
        printf("Queried Motor Mode: 0x%02X\n", mode_val);
    } else {
        printf("Failed to query Motor Mode\n");
    }

    for (int i = 0; i < 50; i++) {
        motor_set_cmd_one(dev, &cmd);
        motor_get_state_one(dev, &state);
        float raw_turns = 0.0f;
        motor_get_paras(dev, (const void *)(uintptr_t)0x74, &raw_turns, 4);
        printf("State: vel=%.2f trq=%.2f temp=%.1f pos=%.2f rad (%.2f turns)\n",
                state.vel, state.trq, state.temp, state.pos, raw_turns);
        usleep(100000);
    }

    // ---------------------------------------------------------
    // Position Mode Validation
    // ---------------------------------------------------------
    printf("\nStopping motor before Position Mode switch...\n");
    cmd.mode = MOTOR_MODE_IDLE;
    motor_set_cmd_one(dev, &cmd);
    usleep(500000); // 500ms

    motor_get_state_one(dev, &state);
    float start_pos = state.pos;

    printf("\nStarting Position Loop Mode verification...\n");
    cmd.mode = MOTOR_MODE_POS;
    cmd.pos_des = start_pos; // Hold current position

    for (int retries = 0; retries < 5; retries++) {
        if (motor_set_cmd_one(dev, &cmd) != 0) {
            printf("Failed to set position loop mode (retry %d)\n", retries);
        } else {
            printf("Successfully set position loop mode!\n");
            break;
        }
        usleep(100000);
    }

    mode_val = 0xFF;
    if (motor_set_paras(dev, (const void *)(uintptr_t)0x75, &mode_val, 1) == 0) {
        printf("Queried Motor Mode: 0x%02X (expected 0x03)\n", mode_val);
    }

    float target_pos = start_pos + 6.28318f; // One full circle
    int duration_ms = 3000; // 3 seconds
    int step_ms = 10;       // 100 Hz
    int steps = duration_ms / step_ms;

    for (int i = 0; i <= steps; i++) {
        cmd.pos_des = start_pos + (target_pos - start_pos) * i / steps;
        motor_set_cmd_one(dev, &cmd);
        motor_get_state_one(dev, &state);

        if (i % 20 == 0) {
            float raw_turns = 0.0f;
            motor_get_paras(dev, (const void *)(uintptr_t)0x74, &raw_turns, 4);
            printf("Active Query State (Position): des=%.4f pos=%.4f vel=%.4f | turns: %.2f\n",
                    cmd.pos_des, state.pos, state.vel, raw_turns);
        }
        usleep(step_ms * 1000);
    }

    printf("Trajectory finished, waiting for motor to settle...\n");
    for (int settle = 0; settle < 50; settle++) {
        motor_set_cmd_one(dev, &cmd);
        motor_get_state_one(dev, &state);
        if (settle % 10 == 0) {
            printf("Settle: des=%.4f pos=%.4f vel=%.4f\n", cmd.pos_des, state.pos, state.vel);
        }
        float diff = state.pos - target_pos;
        if (diff > -0.05f && diff < 0.05f) {
            printf("Position settled!\n");
            break;
        }
        usleep(100000); // 10Hz
    }

    // Stop
    cmd.mode = MOTOR_MODE_IDLE;
    motor_set_cmd_one(dev, &cmd);

    motor_free(&dev, 1);
    return 0;
}
