/*
 * Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Public parameter IDs and helper structures for the Benmo M15-2D-133 motor
 * driver (drv_can_ddt_m152d133).
 *
 * These identifiers are passed as the "address" argument of motor_set_paras() /
 * motor_get_paras():
 *
 *   uint32_t id = BM_M152D133_PARAM_COMM_TIMEOUT;
 *   uint16_t timeout_ms = 1000;
 *   motor_set_paras(dev, (const void *)(uintptr_t)id, &timeout_ms,
 *                   sizeof(timeout_ms));
 */

#ifndef DRV_CAN_DDT_M152D133_H
#define DRV_CAN_DDT_M152D133_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Adjustable / queryable parameters.
 *
 * "set" entries are used with motor_set_paras(); "get" entries with
 * motor_get_paras(). The expected data buffer type is documented per entry.
 */
enum bm_m152d133_param {
    /* --- writable (motor_set_paras) --- */
    BM_M152D133_PARAM_ENABLE = 1,     /* uint8_t: 0 = disable, 1 = enable */
    BM_M152D133_PARAM_MODE,           /* uint8_t: 0 open, 1 current, 2 velocity, 3 position */
    BM_M152D133_PARAM_FEEDBACK,       /* uint8_t: bit7 = query mode, low7 = report period (ms) */
    BM_M152D133_PARAM_MOTOR_ID,       /* uint8_t: new CAN ID (1..8), power-cycle to apply */
    BM_M152D133_PARAM_TERM_RES,       /* uint8_t: 0 = off, 1 = on (CAN terminal resistor) */
    BM_M152D133_PARAM_COMM_TIMEOUT,   /* uint16_t: timeout in ms (0 disables) */
    BM_M152D133_PARAM_CURRENT_FILTER, /* uint16_t: 0..1000, actual = value / 1000 */
    BM_M152D133_PARAM_LAST_BYTE,      /* uint8_t: feedback last byte 0 mode / 1 winding temp / 2 MOS temp */
    BM_M152D133_PARAM_PI,             /* struct bm_m152d133_pi_param */
    BM_M152D133_PARAM_SAVE,           /* no data: persist parameters (motor must be disabled) */
    BM_M152D133_PARAM_MECH_ZERO,      /* no data: calibrate current position as mechanical zero */
    BM_M152D133_PARAM_POS_FB_SPAN,    /* uint8_t: 0 = 65535 range (default), 1 = 32767 range (after zero cal) */

    /* --- readable (motor_get_paras) --- */
    BM_M152D133_PARAM_FW_VERSION = 128, /* uint32_t: sw_major<<24 | sw_minor<<16 | hw_major<<8 | hw_minor */
    BM_M152D133_PARAM_VELOCITY,         /* float: rad/s */
    BM_M152D133_PARAM_POSITION,         /* float: rad */
    BM_M152D133_PARAM_CURRENT,          /* float: A */
    BM_M152D133_PARAM_TORQUE,           /* float: Nm */
    BM_M152D133_PARAM_TEMPERATURE,      /* float: Celsius (requires last byte = temperature) */
    BM_M152D133_PARAM_FAULT,            /* uint32_t: fault code */
    BM_M152D133_PARAM_CUR_MODE,         /* uint8_t: current loop mode reported by the motor */
};

/*
 * PI parameter payload for BM_M152D133_PARAM_PI.
 *
 * Divisors are exponents of two (value n meaning 2^n, n <= 15), matching the
 * vendor protocol. Example: P = 0.97 -> p_num = 0x3E8, p_den = 10 (1000/2^10).
 *
 * mode selects which loop / limit set is written:
 *   0x01 current-loop PI       0x02 velocity-loop PI      0x03 position-loop PI
 *   0x11 current-loop limit    0x22 velocity-loop limit   0x33 position-loop limit
 *   0xFF reset all loop PID parameters
 * For the *_limit modes, p_num/p_den carry the max output and i_num/i_den the
 * min output (raw 16-bit values, divisor field unused).
 */
struct bm_m152d133_pi_param {
    uint8_t mode;
    uint16_t p_num;
    uint8_t p_den;
    uint16_t i_num;
    uint8_t i_den;
};

#ifdef __cplusplus
}
#endif

#endif /* DRV_CAN_DDT_M152D133_H */
