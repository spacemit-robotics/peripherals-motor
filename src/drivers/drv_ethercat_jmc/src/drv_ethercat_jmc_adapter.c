/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>
#include <motor.h>
#include <motor_core.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "ethercat_cia402.h"
#include "jmc_ihss42.h"

#define MAX_ADAPTER_MOTORS EC_MAX_MOTORS

/* Motor mechanics parameters (configurable) */
#define PULSES_PER_REV 10000.0f
#define RAD_PER_REV (2.0f * (float)M_PI)
#define PULSE_TO_RAD (RAD_PER_REV / PULSES_PER_REV)
#define RAD_TO_PULSE (PULSES_PER_REV / RAD_PER_REV)

static ec_master_ctx_t g_ec_ctx;
static bool g_ec_initialized = false;
static pthread_mutex_t g_ec_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_t g_ec_thread;

static int g_configured_motors = 0;
static struct motor_dev* g_ec_devs[MAX_ADAPTER_MOTORS];
static uint16_t g_slave_indices[MAX_ADAPTER_MOTORS];
static uint32_t g_requested_cycle_ms = 2;  // Default 2ms

/* Background thread for EtherCAT cyclic task */
static void* ecat_background_thread(void* arg) {
    uint32_t cycle_time_ns = g_requested_cycle_ms * 1000000;
    uint64_t next_cycle = ec_get_time_ns();

    while (g_ec_ctx.running) {
        pthread_mutex_lock(&g_ec_mutex);
        ec_cyclic_task(&g_ec_ctx);
        for (int i = 0; i < g_ec_ctx.motor_count; i++) {
            jmc_ihss42_cyclic_task(&g_ec_ctx, i);
        }
        ecrt_domain_queue(g_ec_ctx.domain);
        ecrt_master_send(g_ec_ctx.master);
        pthread_mutex_unlock(&g_ec_mutex);

        // Sleep until next cycle
        cycle_time_ns = g_ec_ctx.cycle_time_ns;
        next_cycle += cycle_time_ns;
        uint64_t now = ec_get_time_ns();
        if (now < next_cycle) {
            struct timespec ts;
            uint64_t sleep_time = next_cycle - now;
            ts.tv_sec = sleep_time / 1000000000ULL;
            ts.tv_nsec = sleep_time % 1000000000ULL;
            nanosleep(&ts, NULL);
        }
    }
    return NULL;
}

static ec_app_mode_t convert_mode(uint32_t motor_mode) {
    switch (motor_mode) {
        case MOTOR_MODE_POS:
            return EC_MODE_PP;
        case MOTOR_MODE_VEL:
            return EC_MODE_PV;
        case MOTOR_MODE_TRQ:
            return EC_MODE_PT;
        case MOTOR_MODE_CSP:
            return EC_MODE_CSP;
        case MOTOR_MODE_CSV:
            return EC_MODE_CSV;
        case MOTOR_MODE_CST:
            return EC_MODE_CST;
        case MOTOR_MODE_HM:
            return EC_MODE_HM;
        default:
            return EC_MODE_CSP;
    }
}

static int adapter_init(struct motor_dev* dev) {
    pthread_mutex_lock(&g_ec_mutex);
    if (!g_ec_initialized) {
        if (ec_master_init(&g_ec_ctx) < 0) {
            pthread_mutex_unlock(&g_ec_mutex);
            return -1;
        }
        g_ec_ctx.enable_dc = 1;
        g_ec_ctx.cycle_time_ns = g_requested_cycle_ms * 1000000;
        printf("[Adapter] Starting EtherCAT with bus cycle: %u ms (%u ns)\n", g_requested_cycle_ms,
            g_ec_ctx.cycle_time_ns);

        if (ec_master_request(&g_ec_ctx, 0) < 0 || ec_domain_create(&g_ec_ctx) < 0) {
            pthread_mutex_unlock(&g_ec_mutex);
            return -1;
        }

        /* Configure registered motors */
        for (int i = 0; i < g_configured_motors; i++) {
            jmc_ihss42_set_profile_params(&g_ec_ctx, i, 100000, 500000, 500000);
            if (jmc_ihss42_configure(&g_ec_ctx, i, 0, g_slave_indices[i], EC_MODE_CSP) < 0) {
                pthread_mutex_unlock(&g_ec_mutex);
                return -1;
            }
            if (jmc_ihss42_register_pdo(&g_ec_ctx, i) < 0) {
                pthread_mutex_unlock(&g_ec_mutex);
                return -1;
            }
        }

        if (ec_master_activate(&g_ec_ctx) < 0) {
            pthread_mutex_unlock(&g_ec_mutex);
            return -1;
        }

        // 主站激活后开始周期性任务
        g_ec_ctx.running = 1;
        pthread_create(&g_ec_thread, NULL, ecat_background_thread, NULL);
        g_ec_initialized = true;
    }
    pthread_mutex_unlock(&g_ec_mutex);
    return 0;
}

static int adapter_set_cmd(struct motor_dev* dev, const struct motor_cmd* cmd) {
    if (!dev || !cmd || !g_ec_initialized)
        return -1;

    int motor_idx = (int)(intptr_t)dev->priv_data;

    pthread_mutex_lock(&g_ec_mutex);

    ec_motor_t* motor = &g_ec_ctx.motors[motor_idx];
    ec_app_mode_t ec_mode = convert_mode(cmd->mode);

    if (motor->mode != ec_mode) {
        ec_set_motor_mode(&g_ec_ctx, motor_idx, ec_mode);
        // Reset anchoring state on mode switch to ensure new mode starts correctly
        if (motor->cia402_state == EC_STATE_RUNNING) {
            motor->motion.start_pos_set = 1;
            printf("[Adapter] M%d Mode switch to %s, re-triggering anchor\n", motor_idx, ec_mode_name(ec_mode));
        }
    }

    // Universal Anchoring Logic:
    // When motor is RUNNING and driver has captured physical start (state 1),
    // we perform the logical anchoring required for the current mode.
    if (motor->cia402_state == EC_STATE_RUNNING && motor->motion.start_pos_set == 1) {
        if (ec_mode == EC_MODE_CSP || ec_mode == EC_MODE_PP) {
            // Position anchoring: align application trajectory to physical position
            int32_t cmd_pulses = (int32_t)(cmd->pos_des * RAD_TO_PULSE);
            motor->motion.start_pos = ec_get_actual_position(&g_ec_ctx, motor_idx);
            motor->motion.app_start_pos = cmd_pulses;
            motor->motion.start_pos_set = 2;
            printf("[Adapter] M%d Pos-Anchored: Physical=%d, App=%d\n", motor_idx, motor->motion.start_pos,
                motor->motion.app_start_pos);
        } else {
            // Other modes (Velocity/Torque/Homing): Just unlock the safety gate
            motor->motion.start_pos_set = 2;
            printf("[Adapter] M%d Mode-Anchored: %s\n", motor_idx, ec_mode_name(ec_mode));
        }
    }

    // Only accept motion commands when fully anchored (state 2)
    if (motor->cia402_state != EC_STATE_RUNNING || motor->motion.start_pos_set != 2) {
        pthread_mutex_unlock(&g_ec_mutex);
        return 0;
    }

    if (ec_mode == EC_MODE_CSP || ec_mode == EC_MODE_PP) {
        int32_t cmd_pulses = (int32_t)(cmd->pos_des * RAD_TO_PULSE);
        // Relative command = (Current Cmd - Cmd at Enablement) + Physical Position at Enablement
        int32_t pos_pulses = (cmd_pulses - motor->motion.app_start_pos) + motor->motion.start_pos;

        if (ec_mode == EC_MODE_PP) {
            if (motor->motion.target_pos != pos_pulses) {
                ec_pp_move_to(&g_ec_ctx, motor_idx, pos_pulses);
            }
        } else {
            ec_set_target_position(&g_ec_ctx, motor_idx, pos_pulses);
        }
    } else if (ec_mode == EC_MODE_CSV || ec_mode == EC_MODE_PV) {
        int32_t vel_pulses = (int32_t)(cmd->vel_des * RAD_TO_PULSE);
        ec_set_target_velocity(&g_ec_ctx, motor_idx, vel_pulses);
    } else if (ec_mode == EC_MODE_CST || ec_mode == EC_MODE_PT) {
        int32_t trq_val = (int32_t)(cmd->trq_des * 1000.0f);
        ec_set_target_torque(&g_ec_ctx, motor_idx, trq_val);
    } else if (ec_mode == EC_MODE_HM) {
        if (motor->motion.hm_state == 0) {
            ec_hm_start(&g_ec_ctx, motor_idx);
        }
    }
    pthread_mutex_unlock(&g_ec_mutex);
    return 0;
}

static int adapter_get_state(struct motor_dev* dev, struct motor_state* state) {
    if (!dev || !state || !g_ec_initialized)
        return -1;

    int motor_idx = (int)(intptr_t)dev->priv_data;

    pthread_mutex_lock(&g_ec_mutex);
    state->pos = (float)ec_get_actual_position(&g_ec_ctx, motor_idx) * PULSE_TO_RAD;
    state->vel = (float)ec_get_actual_velocity(&g_ec_ctx, motor_idx) * PULSE_TO_RAD;
    state->trq = 0.0f;
    state->temp = 0.0f;
    state->err = ec_get_status_word(&g_ec_ctx, motor_idx);
    pthread_mutex_unlock(&g_ec_mutex);
    return 0;
}

static void adapter_free(struct motor_dev* dev) {
    if (!dev)
        return;
    /* Optional: reference counting logic for stopping thread and master_release
     */
    free(dev);
}

static const struct motor_ops g_adapter_ops = {
    .init = adapter_init, .set_cmd = adapter_set_cmd, .get_state = adapter_get_state, .free = adapter_free};

static struct motor_dev* adapter_factory(void* args) {
    if (!args)
        return NULL;

    struct motor_args_ecat* ecat_args = (struct motor_args_ecat*)args;

    if (g_configured_motors >= MAX_ADAPTER_MOTORS) {
        return NULL;
    }

    // Use shared cycle time for all motors assigned to this master
    uint32_t val = (uint32_t)(uintptr_t)ecat_args->args;
    if (val > 0) {
        g_requested_cycle_ms = val;
    }

    struct motor_dev* dev = calloc(1, sizeof(struct motor_dev));
    if (!dev)
        return NULL;

    dev->name = "drv_ethercat_jmc";
    dev->ops = &g_adapter_ops;
    dev->priv_data = (void*)(intptr_t)g_configured_motors;

    g_slave_indices[g_configured_motors] = ecat_args->slave_idx;
    g_ec_devs[g_configured_motors] = dev;
    g_configured_motors++;

    return dev;
}

/* Replace flawed REGISTER_MOTOR_DRIVER macro, add __attribute__((used)) and
 * remove static */
__attribute__((used)) struct driver_info __drv_info_adapter_factory = {
    .name = "drv_ethercat_jmc", .type = DRV_TYPE_ECAT, .factory = adapter_factory, .next = 0};
__attribute__((used, constructor)) void __auto_reg_adapter_factory(void) {
    motor_driver_register(&__drv_info_adapter_factory);
}
