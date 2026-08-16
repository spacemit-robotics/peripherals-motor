/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file encos.h
 * @brief Private Encos CAN motor configuration types.
 */

#ifndef ENCOS_H
#define ENCOS_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct encos_protocol_limits {
    float position_min;
    float position_max;
    float velocity_min;
    float velocity_max;
    float torque_min;
    float torque_max;
    float kp_min;
    float kp_max;
    float kd_min;
    float kd_max;
};

struct motor_can_encos_config {
    uint16_t feedback_id;
    uint32_t can_timeout_ms;
    bool enable_on_init;
    struct encos_protocol_limits limits;
};

#ifdef __cplusplus
}
#endif

#endif  // ENCOS_H
