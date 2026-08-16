/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file test_encos_protocol.c
 * @brief Offline tests for the Encos frame codec
 */

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "encos_protocol.h"

static struct encos_protocol_limits make_limits(void) {
    const struct encos_protocol_limits limits = {
        .position_min = -12.5f,
        .position_max = 12.5f,
        .velocity_min = -18.0f,
        .velocity_max = 18.0f,
        .torque_min = -90.0f,
        .torque_max = 90.0f,
        .kp_min = 0.0f,
        .kp_max = 500.0f,
        .kd_min = 0.0f,
        .kd_max = 5.0f,
    };

    return limits;
}

int main(void) {
    const struct encos_protocol_limits limits = make_limits();
    struct motor_cmd command = {
        .mode = MOTOR_MODE_HYBRID,
        .pos_des = 0.0f,
        .vel_des = 0.0f,
        .trq_des = 0.0f,
        .kp = 0.0f,
        .kd = 0.0f,
    };
    const uint8_t expected_command[8] = {
        0x00,
        0x00,
        0x00,
        0x7f,
        0xff,
        0x7f,
        0xf7,
        0xff,
    };
    const uint8_t expected_timeout[ENCOS_TIMEOUT_CONFIG_FRAME_SIZE] = {
        0xc0,
        0x0b,
        0x01,
        0xf4,
    };
    uint8_t feedback[8] = {
        0x00,
        0x80,
        0x00,
        0x80,
        0x08,
        0x00,
        0x46,
        0x50,
    };
    struct motor_state state = {0};
    uint8_t encoded[8] = {0};
    uint8_t timeout_encoded[ENCOS_TIMEOUT_CONFIG_FRAME_SIZE] = {0};
    const struct motor_option options[] = {
        {"model", "Encos EC-A8112-P1-18"},
        {"feedback_id", "1"},
        {"can_timeout_ms", "500"},
        {"enable_on_init", "false"},
    };
    const struct motor_option invalid_model_options[] = {
        {"model", "unsupported"},
        {"feedback_id", "1"},
    };
    struct motor_dev *device;

    assert(encos_encode_command(&limits, &command, encoded) == 0);
    assert(memcmp(encoded, expected_command, sizeof(encoded)) == 0);
    assert(encos_encode_timeout_config(500, timeout_encoded) == 0);
    assert(memcmp(timeout_encoded, expected_timeout, sizeof(timeout_encoded)) == 0);
    command.pos_des = limits.position_max + 0.1f;
    assert(encos_encode_command(&limits, &command, encoded) < 0);
    command.pos_des = 0.0f;
    command.kd = limits.kd_max + 0.1f;
    assert(encos_encode_command(&limits, &command, encoded) < 0);
    assert(encos_decode_feedback(&limits, feedback, &state) == 0);
    assert(fabsf(state.pos) < 0.001f);
    assert(fabsf(state.vel) < 0.01f);
    assert(fabsf(state.trq) < 0.05f);
    assert(fabsf(state.temp - 15.0f) < 0.001f);
    assert(state.err == 0);
    feedback[0] = 0x01;
    assert(encos_decode_feedback(&limits, feedback, &state) == 0);
    assert(state.err == 0x01);
    feedback[0] = 0x02;
    assert(encos_decode_feedback(&limits, feedback, &state) == 0);
    assert(state.err == 0x02);
    feedback[0] = 0x03;
    assert(encos_decode_feedback(&limits, feedback, &state) == 0);
    assert(state.err == 0x03);
    feedback[0] = 0x04;
    assert(encos_decode_feedback(&limits, feedback, &state) == 0);
    assert(state.err == 0x04);
    feedback[0] = 0x05;
    assert(encos_decode_feedback(&limits, feedback, &state) == 0);
    assert(state.err == 0x05);
    feedback[0] = 0x06;
    assert(encos_decode_feedback(&limits, feedback, &state) == 0);
    assert(state.err == 0x06);
    feedback[0] = 0x07;
    assert(encos_decode_feedback(&limits, feedback, &state) == 0);
    assert(state.err == 0x07);
    feedback[0] = 0x08;
    assert(encos_decode_feedback(&limits, feedback, &state) == 0);
    assert(state.err == 0x08);
    device = motor_alloc_can_with_options(
        "drv_can_encos", "vcan0", 1, options, sizeof(options) / sizeof(options[0]));
    assert(device != NULL);
    motor_free(&device, 1);
    assert(motor_alloc_can_with_options("drv_can_encos", "vcan0", 1,
        invalid_model_options,
        sizeof(invalid_model_options) / sizeof(invalid_model_options[0])) == NULL);
    puts("Encos protocol tests passed");
    return 0;
}
