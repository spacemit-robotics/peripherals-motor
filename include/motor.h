/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file motor.h
 * @brief Public API for pluggable motor drivers.
 */

#ifndef MOTOR_H
#define MOTOR_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Motor control modes
 */
enum motor_mode {
    MOTOR_MODE_IDLE = 0, /* disabled / freewheeling */
    MOTOR_MODE_POS,      /* position closed-loop (rad) */
    MOTOR_MODE_VEL,      /* velocity closed-loop (rad/s) */
    MOTOR_MODE_TRQ,      /* pure torque control (Nm) */
    MOTOR_MODE_HYBRID,   /* MIT-style impedance control */
    MOTOR_MODE_CSP,      /* cyclic synchronous position */
    MOTOR_MODE_CSV,      /* cyclic synchronous velocity */
    MOTOR_MODE_CST,      /* cyclic synchronous torque */
    MOTOR_MODE_HM,       /* homing, return to home position */
    MOTOR_MODE_OPEN,     /* open-loop voltage control (vel_des as raw speed) */
};

/*
 * struct motor_cmd - command structure
 * @mode:    control mode
 * @pos_des: desired position (rad)
 * @vel_des: desired velocity (rad/s)
 * @trq_des: desired torque (Nm) or feed-forward in HYBRID mode
 * @kp:      stiffness gain (HYBRID mode)
 * @kd:      damping gain (HYBRID mode)
 */
struct motor_cmd {
    uint32_t mode;
    float pos_des;
    float vel_des;
    float trq_des;
    float kp;
    float kd;
};

/*
 * struct motor_state - feedback structure
 * @pos:  current position (rad)
 * @vel:  current velocity (rad/s)
 * @trq:  current torque (Nm)
 * @temp: temperature (Celsius)
 * @err:  error flags
 */
struct motor_state {
    float pos;
    float vel;
    float trq;
    float temp;
    uint32_t err;
};

/** @brief Cumulative bus diagnostics, shared by devices on the same transport.
 * Reads never send commands or clear counters. Unsupported drivers return ENOTSUP.
 * Failure fields retain the last failed transaction across successful retries.
 * Times use CLOCK_MONOTONIC; byte/frame deltas exclude the pre-write drain.
 */
struct motor_transport_diagnostics {
    uint64_t rx_bytes, tx_bytes, valid_frames, crc_errors, decode_errors;
    uint64_t resync_discarded_bytes, unknown_id_frames;
    uint64_t read_errors, write_errors, poll_errors, transactions, timeouts;
    uint64_t last_rx_us;
    uint64_t last_failure_us, failure_elapsed_us, failure_deadline_late_us;
    uint64_t failure_rx_bytes, failure_valid_frames, failure_crc_errors;
    uint32_t failure_expected_bytes, failure_sent_bytes, failure_missing_ids;
    int32_t failure_errno;
};

/**
 * @brief Driver option passed by configuration-driven upper layers.
 *
 * The motor driver owns the option schema and parses values during allocation.
 * Callers only need to keep the strings alive until the allocation call returns.
 */
struct motor_option {
    const char *name;
    const char *value;
};

/* opaque handle */
struct motor_dev;

int motor_get_transport_diagnostics(struct motor_dev *dev,
    struct motor_transport_diagnostics *diagnostics);

/* --- vectorized API --- */

int motor_init(struct motor_dev **devs, uint32_t count);

/**
 * @brief Attempt all commands and return the first negative driver result, or zero.
 *
 * On failure, errno preserves the first failing driver's system error, or zero
 * when the driver supplies no system error. Later commands are still attempted.
 */
int motor_set_cmds(struct motor_dev **devs, const struct motor_cmd *cmds,
                    uint32_t count);
int motor_get_states(struct motor_dev **devs, struct motor_state *states,
    uint32_t count);

/* Batch-capable variants. errors[i] is 0 or a positive errno value; all
 * entries are attempted, including disable commands after another failure.
 * Homogeneous drivers may service multiple buses together. Other drivers use
 * their unchanged single-device callbacks. Failed reads do not refresh state
 * timestamps. Callers must serialize operations and device destruction. */
int motor_get_states_ex(struct motor_dev **devs, struct motor_state *states,
    int *errors, uint32_t count);
int motor_set_cmds_ex(struct motor_dev **devs, const struct motor_cmd *cmds,
    int *errors, uint32_t count);
/** Host monotonic hardware receive times, or zero when unsupported. */
int motor_get_feedback_timestamps(
    struct motor_dev **devs, uint64_t *timestamps_us, uint32_t count);
void motor_free(struct motor_dev **devs, uint32_t count);

/* --- adjust parameters API --- */
int motor_set_paras(struct motor_dev *dev, const void *address,
                    const void *data, uint32_t data_len);

int motor_get_paras(struct motor_dev *dev, const void *address, void *out_data,
                    uint32_t data_len);

/* --- factory functions --- */

struct motor_dev *motor_alloc_pwm(const char *name, uint32_t ch, void *args);

struct motor_dev *motor_alloc_can(const char *name, const char *iface,
                                    uint32_t can_id, void *args);
struct motor_dev *motor_alloc_can_with_options(const char *name,
                                                const char *iface,
                                                uint32_t can_id,
                                                const struct motor_option *options,
                                                uint32_t option_count);
struct motor_dev *motor_alloc_uart(const char *name, const char *dev_path,
    uint32_t baud, uint8_t id, void *args);

/**
 * @brief Allocate a UART motor using driver-specific key/value options.
 * @param name Registered UART driver name.
 * @param dev_path UART device path; allocation does not open the device.
 * @param baud Requested serial baud rate.
 * @param id Command endpoint on the shared bus.
 * @param options Options consumed during allocation, not retained by the driver.
 * @param option_count Number of entries; options may be NULL only when zero.
 * @return Motor handle, or NULL for invalid/unsupported configuration.
 * @note Call motor_init_one() before I/O. Existing motor_alloc_uart() is unchanged.
 */
struct motor_dev *motor_alloc_uart_with_options(const char *name, const char *dev_path,
    uint32_t baud, uint8_t id,
    const struct motor_option *options, uint32_t option_count);
struct motor_dev *motor_alloc_ecat(const char *name, uint16_t slave_idx,
                                    void *args);

/* --- inline helpers for single motor --- */

static inline int motor_init_one(struct motor_dev *dev) {
    return motor_init(&dev, 1);
}

static inline int motor_set_cmd_one(struct motor_dev *dev,
                                    const struct motor_cmd *cmd) {
    return motor_set_cmds(&dev, cmd, 1);
}

static inline int motor_get_state_one(struct motor_dev *dev,
                                        struct motor_state *state) {
    return motor_get_states(&dev, state, 1);
}

/** Return the host monotonic hardware receive time, or zero when unsupported. */
static inline int motor_get_feedback_timestamp_one(struct motor_dev *dev,
                                                    uint64_t *timestamp_us) {
    return motor_get_feedback_timestamps(&dev, timestamp_us, 1);
}

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_H */
