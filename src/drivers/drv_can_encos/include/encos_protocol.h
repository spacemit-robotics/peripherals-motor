/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file encos_protocol.h
 * @brief Pure Encos CAN frame codec
 */

#ifndef ENCOS_PROTOCOL_H
#define ENCOS_PROTOCOL_H

#include <stdint.h>

#include "encos.h"
#include "motor.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ENCOS_COMMAND_FRAME_SIZE 8
#define ENCOS_FEEDBACK_FRAME_SIZE 8
#define ENCOS_TIMEOUT_CONFIG_FRAME_SIZE 4

int encos_encode_command(const struct encos_protocol_limits *limits,
    const struct motor_cmd *cmd, uint8_t data[ENCOS_COMMAND_FRAME_SIZE]);
int encos_decode_feedback(const struct encos_protocol_limits *limits,
    const uint8_t data[ENCOS_FEEDBACK_FRAME_SIZE], struct motor_state *state);
int encos_encode_timeout_config(uint16_t timeout_ms,
    uint8_t data[ENCOS_TIMEOUT_CONFIG_FRAME_SIZE]);

#ifdef __cplusplus
}
#endif

#endif  // ENCOS_PROTOCOL_H
