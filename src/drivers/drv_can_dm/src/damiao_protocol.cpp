/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file damiao_protocol.cpp
 * @brief Pure Damiao MIT frame codec implementation.
 */

#include "damiao_protocol.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace damiao {

uint16_t FloatToUint(float value, float min, float max, uint8_t bits) {
    const float normalized = std::clamp((value - min) / (max - min), 0.0f, 1.0f);
    return static_cast<uint16_t>(normalized * static_cast<float>((1U << bits) - 1U));
}

float UintToFloat(uint16_t value, float min, float max, uint8_t bits) {
    return static_cast<float>(value) * (max - min) /
        static_cast<float>((1U << bits) - 1U) + min;
}

bool EncodeMitCommand(const Limit_param& limits, float position, float velocity,
    float torque, float kp, float kd, uint8_t data[8]) {
    if (!data || !std::isfinite(position) || !std::isfinite(velocity) ||
        !std::isfinite(torque) || !std::isfinite(kp) || !std::isfinite(kd) ||
        limits.Q_MAX <= 0.0f || limits.DQ_MAX <= 0.0f || limits.TAU_MAX <= 0.0f ||
        position < -limits.Q_MAX || position > limits.Q_MAX ||
        velocity < -limits.DQ_MAX || velocity > limits.DQ_MAX ||
        torque < -limits.TAU_MAX || torque > limits.TAU_MAX ||
        kp < 0.0f || kp > 500.0f || kd < 0.0f || kd > 5.0f) {
        return false;
    }

    const uint16_t position_uint = FloatToUint(position, -limits.Q_MAX, limits.Q_MAX, 16);
    const uint16_t velocity_uint = FloatToUint(velocity, -limits.DQ_MAX, limits.DQ_MAX, 12);
    const uint16_t torque_uint = FloatToUint(torque, -limits.TAU_MAX, limits.TAU_MAX, 12);
    const uint16_t kp_uint = FloatToUint(kp, 0.0f, 500.0f, 12);
    const uint16_t kd_uint = FloatToUint(kd, 0.0f, 5.0f, 12);

    data[0] = static_cast<uint8_t>(position_uint >> 8);
    data[1] = static_cast<uint8_t>(position_uint);
    data[2] = static_cast<uint8_t>(velocity_uint >> 4);
    data[3] = static_cast<uint8_t>((velocity_uint << 4) | (kp_uint >> 8));
    data[4] = static_cast<uint8_t>(kp_uint);
    data[5] = static_cast<uint8_t>(kd_uint >> 4);
    data[6] = static_cast<uint8_t>((kd_uint << 4) | (torque_uint >> 8));
    data[7] = static_cast<uint8_t>(torque_uint);
    return true;
}

bool DecodeMitFeedback(const Limit_param& limits, const uint8_t data[8], float* position,
    float* velocity, float* torque, float* temperature, uint32_t* error) {
    if (!data || !position || !velocity || !torque || !temperature || !error ||
        limits.Q_MAX <= 0.0f || limits.DQ_MAX <= 0.0f || limits.TAU_MAX <= 0.0f) {
        return false;
    }

    const uint16_t position_uint = (static_cast<uint16_t>(data[1]) << 8) | data[2];
    const uint16_t velocity_uint =
        (static_cast<uint16_t>(data[3]) << 4) | (data[4] >> 4);
    const uint16_t torque_uint =
        (static_cast<uint16_t>(data[4] & 0x0f) << 8) | data[5];
    *position = UintToFloat(position_uint, -limits.Q_MAX, limits.Q_MAX, 16);
    *velocity = UintToFloat(velocity_uint, -limits.DQ_MAX, limits.DQ_MAX, 12);
    *torque = UintToFloat(torque_uint, -limits.TAU_MAX, limits.TAU_MAX, 12);
    *temperature = static_cast<float>(std::max(data[6], data[7]));
    const uint32_t status = data[0] >> 4;
    *error = status > 1U ? status : 0U;
    return true;
}

}  // namespace damiao
