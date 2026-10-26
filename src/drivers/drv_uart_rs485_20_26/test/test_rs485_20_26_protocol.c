/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file test_rs485_20_26_protocol.c
 * @brief Offline codec tests, including gain quantization and malformed feedback.
 */
#include "rs485_20_26_protocol.h"

#include <assert.h>
#include <math.h>
#include <string.h>

int main(void) {
    struct rs485_20_26_scale scale = {16.0, 651.898646904, 83443.09729707374, 5.0, 5.0};
    struct rs485_20_26_command command = {0};
    uint8_t bytes[RS485_20_26_COMMAND_SIZE];
    uint8_t feedback_bytes[RS485_20_26_FEEDBACK_SIZE] = {0xfc, 0xee};
    struct rs485_20_26_feedback feedback;
    const uint8_t idle[RS485_20_26_COMMAND_SIZE] = {
        0xfe, 0xee, 0, 0x80, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0xf5, 0x50, 0xa9, 0xf0};

    assert(rs485_20_26_encode(&scale, &command, bytes) == 0);
    assert(memcmp(bytes, idle, sizeof(bytes)) == 0);
    const double saved_scale = scale.torque;
    scale.torque = 0.0;
    assert(rs485_20_26_encode(&scale, &command, bytes) < 0);
    scale.torque = saved_scale;
    command.kd = 0.1;  // Wire truncation: 0.5 -> 0.
    assert(rs485_20_26_encode(&scale, &command, bytes) == 0 && bytes[14] == 0);
    command.kd = (float)0.1;  // A float just above 0.5 still truncates to 0.
    assert(rs485_20_26_encode(&scale, &command, bytes) == 0 && bytes[14] == 0);
    command.kd = 0.3;
    assert(rs485_20_26_encode(&scale, &command, bytes) == 0 && bytes[14] == 1);
    command.torque = 1.9 / scale.torque;
    command.velocity = -1.9 / scale.velocity;
    command.position = -2.9 / scale.position;
    command.kp = 3.9 / scale.kp;
    assert(rs485_20_26_encode(&scale, &command, bytes) == 0);
    assert(bytes[4] == 1 && bytes[5] == 0);
    assert(bytes[6] == 0xff && bytes[7] == 0xff);
    assert(bytes[8] == 0xfe && bytes[9] == 0xff && bytes[10] == 0xff && bytes[11] == 0xff);
    assert(bytes[12] == 3 && bytes[13] == 0);
    command.torque = command.velocity = command.position = command.kp = 0.0;
    command.kd = -0.01;
    assert(rs485_20_26_encode(&scale, &command, bytes) < 0);
    command.kd = 0.0;
    command.position = NAN;
    assert(rs485_20_26_encode(&scale, &command, bytes) < 0);
    command.position = 1.0e20;
    assert(rs485_20_26_encode(&scale, &command, bytes) < 0);
    command.position = 0.0;
    command.id = 15;
    assert(rs485_20_26_encode(&scale, &command, bytes) < 0);

    feedback_bytes[3] = 0xff;
    feedback_bytes[6] = 0xf0;
    feedback_bytes[7] = 0xff;  // -16 / 16 = -1 Nm.
    uint32_t crc = rs485_20_26_crc_words(feedback_bytes + 2, 20);
    for (size_t i = 0; i < 4; ++i) feedback_bytes[22 + i] = (uint8_t)(crc >> (8 * i));
    assert(rs485_20_26_decode(&scale, feedback_bytes, sizeof(feedback_bytes), &feedback) == 0);
    assert(feedback.driver_temperature == -1 && feedback.torque == -1.0);
    assert(rs485_20_26_decode(&scale, feedback_bytes, sizeof(feedback_bytes) - 1, &feedback) < 0);
    feedback_bytes[8] ^= 1;
    assert(rs485_20_26_decode(&scale, feedback_bytes, sizeof(feedback_bytes), &feedback) < 0);
    return 0;
}
