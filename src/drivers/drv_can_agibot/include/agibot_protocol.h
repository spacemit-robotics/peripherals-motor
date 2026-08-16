/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file agibot_protocol.h
 * @brief AGIBOT OmniPicker CAN frame codec declarations.
 */

#ifndef AGIBOT_PROTOCOL_H
#define AGIBOT_PROTOCOL_H

#include <stdint.h>

#include "agibot.h"
#include "motor.h"

#ifdef __cplusplus
extern "C" {
#endif

#define AGIBOT_CAN_FRAME_SIZE 8
#define AGIBOT_POSITION_MIN_RAD 0.0f
#define AGIBOT_POSITION_MAX_RAD 1.5707963267948966f

int agibot_encode_position(float position_rad,
    const struct agibot_command_profile *profile,
    uint8_t data[AGIBOT_CAN_FRAME_SIZE]);
int agibot_decode_feedback(const uint8_t data[AGIBOT_CAN_FRAME_SIZE],
    struct motor_state *state, uint8_t *motion_state);

#ifdef __cplusplus
}
#endif

#endif  // AGIBOT_PROTOCOL_H
