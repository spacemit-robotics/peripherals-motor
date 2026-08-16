/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file damiao_protocol.h
 * @brief Pure Damiao MIT frame codec
 */

#ifndef DAMIAO_PROTOCOL_H
#define DAMIAO_PROTOCOL_H

#include <cstdint>

#include "damiao.h"

namespace damiao {

uint16_t FloatToUint(float value, float min, float max, uint8_t bits);
float UintToFloat(uint16_t value, float min, float max, uint8_t bits);
bool EncodeMitCommand(const Limit_param& limits, float position, float velocity,
    float torque, float kp, float kd, uint8_t data[8]);
bool DecodeMitFeedback(const Limit_param& limits, const uint8_t data[8], float* position,
    float* velocity, float* torque, float* temperature, uint32_t* error);

}  // namespace damiao

#endif  // DAMIAO_PROTOCOL_H
