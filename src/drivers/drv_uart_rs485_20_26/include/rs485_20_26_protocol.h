/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file rs485_20_26_protocol.h
 * @brief Codec for the RS485 20-byte command / 26-byte feedback format.
 */
#ifndef RS485_20_26_PROTOCOL_H
#define RS485_20_26_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

#define RS485_20_26_COMMAND_SIZE 20U
#define RS485_20_26_FEEDBACK_SIZE 26U
#define RS485_20_26_DECODE_CRC_ERROR (-2)

struct rs485_20_26_scale {
    double torque;
    double velocity;
    double position;
    double kp;
    double kd;
};

struct rs485_20_26_command {
    uint8_t id;
    uint8_t enabled;
    uint8_t watchdog;
    double torque;
    double velocity;
    double position;
    double kp;
    double kd;
};

struct rs485_20_26_feedback {
    uint8_t id;
    uint8_t status;
    uint8_t watchdog_latched;
    int driver_temperature;
    unsigned int winding_temperature;
    double voltage;
    double torque;
    double velocity;
    double position;
    uint32_t error;
};

/* Factors are wire counts per SI unit, supplied by the hardware configuration. */
uint32_t rs485_20_26_crc_words(const uint8_t *bytes, size_t size);
int rs485_20_26_encode(const struct rs485_20_26_scale *scale, const struct rs485_20_26_command *command,
    uint8_t bytes[RS485_20_26_COMMAND_SIZE]);
int rs485_20_26_check_feedback(const uint8_t *bytes, size_t size);
int rs485_20_26_decode(const struct rs485_20_26_scale *scale, const uint8_t *bytes, size_t size,
    struct rs485_20_26_feedback *feedback);

#endif  // RS485_20_26_PROTOCOL_H
