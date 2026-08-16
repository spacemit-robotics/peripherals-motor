/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file test_agibot_protocol.c
 * @brief Offline tests for the AGIBOT OmniPicker CAN codec.
 */

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "agibot_protocol.h"

int main(void) {
    const struct agibot_command_profile profile = {
        .force = 0x7f,
        .velocity = 0x7f,
        .acceleration = 0x7f,
        .deceleration = 0x7f,
    };
    const uint8_t open_expected[8] = {0x00, 0xff, 0x7f, 0x7f, 0x7f, 0x7f, 0, 0};
    const uint8_t closed_expected[8] = {0x00, 0x00, 0x7f, 0x7f, 0x7f, 0x7f, 0, 0};
    const uint8_t feedback[8] = {0x00, 0x01, 0x80, 0x40, 0x20, 0, 0, 0};
    const struct motor_option options[] = {
        {"model", "AGIBOT OmniPicker"},
        {"feedback_id", "8"},
    };
    uint8_t data[8] = {0};
    uint8_t motion_state = 0;
    struct motor_state state = {0};
    struct motor_dev *device;

    assert(agibot_encode_position(0.0f, &profile, data) == 0);
    assert(memcmp(data, open_expected, sizeof(data)) == 0);
    assert(agibot_encode_position(AGIBOT_POSITION_MAX_RAD * 0.5f, &profile, data) == 0);
    assert(data[1] == 0x7f);
    assert(agibot_encode_position(AGIBOT_POSITION_MAX_RAD, &profile, data) == 0);
    assert(memcmp(data, closed_expected, sizeof(data)) == 0);
    assert(agibot_encode_position(-0.01f, &profile, data) < 0);
    assert(agibot_decode_feedback(feedback, &state, &motion_state) == 0);
    assert(fabsf(state.pos - 127.0f * AGIBOT_POSITION_MAX_RAD / 255.0f) < 1.0e-6f);
    assert(state.err == 0U);
    assert(motion_state == 1U);

    device = motor_alloc_can_with_options("drv_can_agibot", "vcan0", 8,
        options, sizeof(options) / sizeof(options[0]));
    assert(device != NULL);
    motor_free(&device, 1);
    puts("AGIBOT protocol tests passed");
    return 0;
}
