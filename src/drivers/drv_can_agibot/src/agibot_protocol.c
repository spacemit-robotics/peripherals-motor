/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file agibot_protocol.c
 * @brief AGIBOT OmniPicker CAN frame codec implementation.
 */

#include "agibot_protocol.h"

#include <math.h>
#include <stddef.h>

int agibot_encode_position(float position_rad,
    const struct agibot_command_profile *profile,
    uint8_t data[AGIBOT_CAN_FRAME_SIZE]) {
    float open_ratio;

    if (!profile || !data || !isfinite(position_rad) ||
        position_rad < AGIBOT_POSITION_MIN_RAD ||
        position_rad > AGIBOT_POSITION_MAX_RAD) {
        return -1;
    }

    open_ratio = 1.0f - position_rad / AGIBOT_POSITION_MAX_RAD;
    data[0] = 0;
    data[1] = (uint8_t)(open_ratio * 255.0f);
    data[2] = profile->force;
    data[3] = profile->velocity;
    data[4] = profile->acceleration;
    data[5] = profile->deceleration;
    data[6] = 0;
    data[7] = 0;
    return 0;
}

int agibot_decode_feedback(const uint8_t data[AGIBOT_CAN_FRAME_SIZE],
    struct motor_state *state, uint8_t *motion_state) {
    if (!data || !state) return -1;

    state->pos = (255.0f - data[2]) * AGIBOT_POSITION_MAX_RAD / 255.0f;
    state->vel = 0.0f;
    state->trq = 0.0f;
    state->temp = 0.0f;
    state->err = data[0];
    if (motion_state) *motion_state = data[1];
    return 0;
}
