/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Motor control modes
 */
enum motor_mode {
    MOTOR_MODE_IDLE = 0,   /* disabled / freewheeling */
    MOTOR_MODE_VEL,        /* velocity closed-loop (rad/s) */
    MOTOR_MODE_POS,        /* position closed-loop (rad) */
    MOTOR_MODE_TRQ,        /* pure torque control (Nm) */
    MOTOR_MODE_HYBRID,     /* MIT-style impedance control */
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
    float    pos_des;
    float    vel_des;
    float    trq_des;
    float    kp;
    float    kd;
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
    float    pos;
    float    vel;
    float    trq;
    float    temp;
    uint32_t err;
};


/* opaque handle */
struct motor_dev;

/* --- vectorized API --- */

int motor_init(struct motor_dev **devs, uint32_t count);
int motor_set_cmds(struct motor_dev **devs, const struct motor_cmd *cmds,
                    uint32_t count);
int motor_get_states(struct motor_dev **devs, struct motor_state *states,
                    uint32_t count);
void motor_free(struct motor_dev **devs, uint32_t count);

/* --- factory functions --- */

struct motor_dev *motor_alloc_pwm(const char *name, uint32_t ch, void *args);

struct motor_dev *motor_alloc_can(const char *name, const char *iface,
                                uint32_t can_id, void *args);
struct motor_dev *motor_alloc_uart(const char *name, const char *dev_path,
                                uint32_t baud, uint8_t id, void *args);
struct motor_dev *motor_alloc_ecat(const char *name, uint16_t slave_idx,
                                void *args);

/* --- inline helpers for single motor --- */

static inline int motor_init_one(struct motor_dev *dev)
{
    return motor_init(&dev, 1);
}

static inline int motor_set_cmd_one(struct motor_dev *dev,
                                    const struct motor_cmd *cmd)
{
    return motor_set_cmds(&dev, cmd, 1);
}

static inline int motor_get_state_one(struct motor_dev *dev,
                                    struct motor_state *state)
{
    return motor_get_states(&dev, state, 1);
}

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H__ */
