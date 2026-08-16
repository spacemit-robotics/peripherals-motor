/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file agibot.h
 * @brief Private AGIBOT OmniPicker CAN configuration types.
 */

#ifndef AGIBOT_H
#define AGIBOT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct agibot_command_profile {
    uint8_t force;
    uint8_t velocity;
    uint8_t acceleration;
    uint8_t deceleration;
};

struct motor_can_agibot_config {
    uint16_t feedback_id;
    struct agibot_command_profile command;
};

#ifdef __cplusplus
}
#endif

#endif  // AGIBOT_H
