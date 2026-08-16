/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file encos_protocol.c
 * @brief Pure Encos CAN frame codec implementation
 */

#include "encos_protocol.h"

#include <math.h>
#include <stddef.h>

static int range_is_valid(float min, float max) {
    return isfinite(min) && isfinite(max) && min < max;
}

static int limits_are_valid(const struct encos_protocol_limits *limits) {
    if (!limits)
        return 0;
    return range_is_valid(limits->position_min, limits->position_max) &&
        range_is_valid(limits->velocity_min, limits->velocity_max) &&
        range_is_valid(limits->torque_min, limits->torque_max) &&
        range_is_valid(limits->kp_min, limits->kp_max) &&
        range_is_valid(limits->kd_min, limits->kd_max);
}

static uint16_t float_to_uint(float value, float min, float max, int bits) {
    const uint32_t range = (1U << bits) - 1U;

    return (uint16_t)((value - min) * (float)range / (max - min));
}

static int value_is_in_range(float value, float min, float max) {
    return isfinite(value) && value >= min && value <= max;
}

static float uint_to_float(uint16_t value, float min, float max, int bits) {
    const uint32_t range = (1U << bits) - 1U;

    return (float)value * (max - min) / (float)range + min;
}

int encos_encode_command(const struct encos_protocol_limits *limits,
    const struct motor_cmd *cmd, uint8_t data[ENCOS_COMMAND_FRAME_SIZE]) {
    uint16_t position;
    uint16_t velocity;
    uint16_t torque;
    uint16_t kp;
    uint16_t kd;

    if (!limits_are_valid(limits) || !cmd || !data) return -1;
    if (!value_is_in_range(cmd->pos_des, limits->position_min, limits->position_max) ||
        !value_is_in_range(cmd->vel_des, limits->velocity_min, limits->velocity_max) ||
        !value_is_in_range(cmd->trq_des, limits->torque_min, limits->torque_max) ||
        !value_is_in_range(cmd->kp, limits->kp_min, limits->kp_max) ||
        !value_is_in_range(cmd->kd, limits->kd_min, limits->kd_max))
        return -1;

    position = float_to_uint(cmd->pos_des, limits->position_min, limits->position_max, 16);
    velocity = float_to_uint(cmd->vel_des, limits->velocity_min, limits->velocity_max, 12);
    torque = float_to_uint(cmd->trq_des, limits->torque_min, limits->torque_max, 12);
    kp = float_to_uint(cmd->kp, limits->kp_min, limits->kp_max, 12);
    kd = float_to_uint(cmd->kd, limits->kd_min, limits->kd_max, 9);

    data[0] = (uint8_t)(kp >> 7);
    data[1] = (uint8_t)((kp << 1) | (kd >> 8));
    data[2] = (uint8_t)kd;
    data[3] = (uint8_t)(position >> 8);
    data[4] = (uint8_t)position;
    data[5] = (uint8_t)(velocity >> 4);
    data[6] = (uint8_t)((velocity << 4) | (torque >> 8));
    data[7] = (uint8_t)torque;
    return 0;
}

int encos_decode_feedback(const struct encos_protocol_limits *limits,
    const uint8_t data[ENCOS_FEEDBACK_FRAME_SIZE], struct motor_state *state) {
    uint16_t position;
    uint16_t velocity;
    uint16_t torque;
    float rotor_temp;
    float mos_temp;
    uint32_t raw_error;

    if (!limits_are_valid(limits) || !data || !state) return -1;

    position = ((uint16_t)data[1] << 8) | data[2];
    velocity = ((uint16_t)data[3] << 4) | (data[4] >> 4);
    torque = ((uint16_t)(data[4] & 0x0f) << 8) | data[5];
    rotor_temp = ((float)data[6] - 50.0f) * 0.5f;
    mos_temp = ((float)data[7] - 50.0f) * 0.5f;

    state->pos = uint_to_float(position, limits->position_min, limits->position_max, 16);
    state->vel = uint_to_float(velocity, limits->velocity_min, limits->velocity_max, 12);
    state->trq = uint_to_float(torque, limits->torque_min, limits->torque_max, 12);
    state->temp = fmaxf(rotor_temp, mos_temp);
    raw_error = data[0] & 0x1f;
    state->err = raw_error;
    return 0;
}

int encos_encode_timeout_config(uint16_t timeout_ms,
    uint8_t data[ENCOS_TIMEOUT_CONFIG_FRAME_SIZE]) {
    if (!data) return -1;
    data[0] = 0xc0;
    data[1] = 0x0b;
    data[2] = (uint8_t)(timeout_ms >> 8);
    data[3] = (uint8_t)timeout_ms;
    return 0;
}
