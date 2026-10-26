/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file rs485_20_26_protocol.c
 * @brief Endian-independent RS485 20/26 codec with truncation toward zero.
 */
#include "rs485_20_26_protocol.h"

#include <limits.h>
#include <math.h>
#include <string.h>

static uint32_t read_le(const uint8_t *bytes, size_t size) {
    uint32_t value = 0;
    for (size_t i = 0; i < size; ++i)
        value |= (uint32_t)bytes[i] << (8 * i);
    return value;
}

static int64_t read_signed(const uint8_t *bytes, size_t size) {
    uint32_t value = read_le(bytes, size);
    uint64_t modulus = UINT64_C(1) << (8 * size);
    return value < modulus / 2 ? (int64_t)value : (int64_t)value - (int64_t)modulus;
}

static void write_le(uint8_t *bytes, uint32_t value, size_t size) {
    for (size_t i = 0; i < size; ++i)
        bytes[i] = (uint8_t)(value >> (8 * i));
}


uint32_t rs485_20_26_crc_words(const uint8_t *bytes, size_t size) {
    uint32_t crc = UINT32_MAX;
    if (!bytes || size % 4 != 0)
        return 0;
    for (size_t i = 0; i < size; i += 4) {
        uint32_t word = read_le(bytes + i, 4);
        for (int bit = 31; bit >= 0; --bit) {
            uint32_t high = ((crc >> 31) ^ (word >> bit)) & 1U;
            crc = (crc << 1) ^ (high ? UINT32_C(0x04c11db7) : 0);
        }
    }
    return crc;
}

static int quantize(double value, double scale, int64_t lo, int64_t hi,
    int64_t *result) {
    double scaled = value * scale;
    if (!isfinite(value) || !isfinite(scale) || scale <= 0.0 || !isfinite(scaled) ||
        scaled < (double)lo || scaled > (double)hi)
        return -1;
    // Signed protocol fields truncate toward zero rather than round to nearest.
    *result = (int64_t)scaled;
    return 0;
}

int rs485_20_26_encode(const struct rs485_20_26_scale *scale, const struct rs485_20_26_command *command,
    uint8_t bytes[RS485_20_26_COMMAND_SIZE]) {
    int64_t torque, velocity, position, kp, kd;
    if (!scale || !command || !bytes || command->id > 14 || command->enabled > 1 ||
        command->watchdog > 1 ||
        quantize(command->torque, scale->torque, INT16_MIN, INT16_MAX, &torque) ||
        quantize(command->velocity, scale->velocity, INT16_MIN, INT16_MAX, &velocity) ||
        quantize(command->position, scale->position, INT32_MIN, INT32_MAX, &position) ||
        quantize(command->kp, scale->kp, 0, INT16_MAX, &kp) ||
        quantize(command->kd, scale->kd, 0, INT16_MAX, &kd))
        return -1;
    memset(bytes, 0, RS485_20_26_COMMAND_SIZE);
    bytes[0] = 0xfe;
    bytes[1] = 0xee;
    bytes[2] = command->id | (command->enabled << 4) | (command->watchdog << 7);
    bytes[3] = 0x80;
    write_le(bytes + 4, (uint32_t)torque, 2);
    write_le(bytes + 6, (uint32_t)velocity, 2);
    write_le(bytes + 8, (uint32_t)position, 4);
    write_le(bytes + 12, (uint32_t)kp, 2);
    write_le(bytes + 14, (uint32_t)kd, 2);
    write_le(bytes + 16, rs485_20_26_crc_words(bytes, 16), 4);
    return 0;
}

int rs485_20_26_check_feedback(const uint8_t *bytes, size_t size) {
    if (!bytes || size != RS485_20_26_FEEDBACK_SIZE || bytes[0] != 0xfc || bytes[1] != 0xee)
        return -1;
    if (rs485_20_26_crc_words(bytes + 2, 20) != read_le(bytes + 22, 4))
        return RS485_20_26_DECODE_CRC_ERROR;
    return 0;
}

int rs485_20_26_decode(const struct rs485_20_26_scale *scale, const uint8_t *bytes, size_t size,
    struct rs485_20_26_feedback *feedback) {
    const int checked = rs485_20_26_check_feedback(bytes, size);
    if (checked != 0) return checked;
    if (!scale || !feedback || (bytes[2] & 15U) > 14 ||
        !isfinite(scale->torque) || scale->torque <= 0.0 ||
        !isfinite(scale->velocity) || scale->velocity <= 0.0 ||
        !isfinite(scale->position) || scale->position <= 0.0)
        return -1;
    *feedback = (struct rs485_20_26_feedback){
        .id = bytes[2] & 15U, .status = (bytes[2] >> 4) & 7U,
        .watchdog_latched = bytes[2] >> 7,
        .driver_temperature = (int)read_signed(bytes + 3, 1),
        .winding_temperature = bytes[4], .voltage = bytes[5] / 2.0,
        .torque = read_signed(bytes + 6, 2) / scale->torque,
        .velocity = read_signed(bytes + 8, 2) / scale->velocity,
        .position = read_signed(bytes + 10, 4) / scale->position,
        .error = read_le(bytes + 14, 4)};
    return 0;
}
