/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file test_damiao_protocol.cpp
 * @brief Offline tests for the Damiao MIT frame codec
 */

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>

#include "damiao_protocol.h"

extern "C" {
#include "motor.h"
}

int main() {
    const damiao::Limit_param limits = {12.566f, 20.0f, 120.0f};
    const std::array<uint8_t, 8> expected = {
        0x7f,
        0xff,
        0x7f,
        0xf0,
        0x00,
        0x00,
        0x07,
        0xff,
    };
    const std::array<uint8_t, 8> feedback = {
        0x30,
        0x80,
        0x00,
        0x80,
        0x08,
        0x00,
        65,
        72,
    };
    std::array<uint8_t, 8> encoded{};
    float position = 0.0f;
    float velocity = 0.0f;
    float torque = 0.0f;
    float temperature = 0.0f;
    uint32_t error = 0;
    const motor_option options[] = {
        {"feedback_id", "17"},
        {"feedback_period_us", "1000"},
        {"can_timeout_ms", "500"},
        {"enable_on_init", "false"},
        {"protocol_limits.position.0", "-12.566"},
        {"protocol_limits.position.1", "12.566"},
        {"protocol_limits.velocity.0", "-20"},
        {"protocol_limits.velocity.1", "20"},
        {"protocol_limits.torque.0", "-120"},
        {"protocol_limits.torque.1", "120"},
        {"protocol_limits.kp.0", "0"},
        {"protocol_limits.kp.1", "500"},
        {"protocol_limits.kd.0", "0"},
        {"protocol_limits.kd.1", "5"},
    };
    constexpr size_t kOptionCount = sizeof(options) / sizeof(options[0]);
    std::array<motor_option, kOptionCount> asymmetric_options{};
    const motor_option model_options[] = {
        {"model", "Damiao DM-J6248P-2EC"},
        {"feedback_id", "17"},
        {"feedback_period_us", "1000"},
        {"can_timeout_ms", "500"},
        {"enable_on_init", "false"},
    };

    assert(damiao::EncodeMitCommand(limits, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, encoded.data()));
    assert(encoded == expected);
    assert(damiao::EncodeMitCommand(limits, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
        encoded.data()));
    const uint16_t torque_raw =
        (static_cast<uint16_t>(encoded[6] & 0x0fU) << 8U) | encoded[7];
    assert(encoded[0] == 0x7f && encoded[1] == 0xff);
    assert(encoded[2] == 0x7f && (encoded[3] & 0xf0U) == 0xf0U);
    assert(encoded[4] == 0x00 && encoded[5] == 0x00);
    assert(torque_raw > 0x07ffU);
    assert(!damiao::EncodeMitCommand(
        limits, limits.Q_MAX + 0.1f, 0.0f, 0.0f, 0.0f, 0.0f, encoded.data()));
    assert(!damiao::EncodeMitCommand(
        limits, 0.0f, 0.0f, 0.0f, 501.0f, 0.0f, encoded.data()));
    assert(damiao::DecodeMitFeedback(limits, feedback.data(), &position, &velocity, &torque,
        &temperature, &error));
    assert(std::fabs(position) < 0.001f);
    assert(std::fabs(velocity) < 0.02f);
    assert(std::fabs(torque) < 0.1f);
    assert(temperature == 72.0f);
    assert(error == 3U);
    std::array<uint8_t, 8> hot_feedback = feedback;
    hot_feedback[6] = 200U;
    hot_feedback[7] = 180U;
    assert(damiao::DecodeMitFeedback(limits, hot_feedback.data(), &position, &velocity, &torque,
        &temperature, &error));
    assert(temperature == 200.0f);
    std::copy_n(options, kOptionCount, asymmetric_options.begin());
    asymmetric_options[4].value = "-12.0";
    assert(motor_alloc_can_with_options("drv_can_dm", "vcan0", 1,
        asymmetric_options.data(), asymmetric_options.size()) == nullptr);
    assert(motor_alloc_can_with_options(
        "drv_can_dm", "vcan0", 0x500, options, kOptionCount) == nullptr);
    motor_dev* device = motor_alloc_can_with_options(
        "drv_can_dm", "vcan0", 1, options, kOptionCount);
    assert(device != nullptr);
    const motor_option duplicate_feedback_options[] = {
        {"model", "Damiao DM-J6248P-2EC"},
        {"feedback_id", "17"},
        {"enable_on_init", "false"},
    };
    assert(motor_alloc_can_with_options("drv_can_dm", "vcan0", 2,
        duplicate_feedback_options,
        sizeof(duplicate_feedback_options) / sizeof(duplicate_feedback_options[0])) == nullptr);
    const motor_option command_feedback_collision_options[] = {
        {"model", "Damiao DM-J6248P-2EC"},
        {"feedback_id", "18"},
        {"enable_on_init", "false"},
    };
    assert(motor_alloc_can_with_options("drv_can_dm", "vcan0", 17,
        command_feedback_collision_options,
        sizeof(command_feedback_collision_options) /
            sizeof(command_feedback_collision_options[0])) == nullptr);
    motor_free(&device, 1);
    device = motor_alloc_can_with_options("drv_can_dm", "vcan0", 1,
        model_options, sizeof(model_options) / sizeof(model_options[0]));
    assert(device != nullptr);
    motor_free(&device, 1);
    std::cout << "Damiao protocol tests passed\n";
    return 0;
}
