/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file test_motor_core.c
 * @brief Offline contract tests for the generic motor core API
 */

#include <assert.h>
#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "motor_core.h"

struct fake_motor {
    int init_result;
    int set_result;
    int set_errno;
    int get_result;
    int set_calls;
    int get_calls;
    uint64_t state_timestamp_us;
};

static int fake_init(struct motor_dev *dev) {
    struct fake_motor *fake = dev ? dev->priv_data : NULL;
    return fake ? fake->init_result : -1;
}

static int fake_set(struct motor_dev *dev, const struct motor_cmd *command) {
    struct fake_motor *fake = dev ? dev->priv_data : NULL;
    if (!fake || !command) return -1;
    ++fake->set_calls;
    if (fake->set_errno != 0) errno = fake->set_errno;
    return fake->set_result;
}

static int fake_get(struct motor_dev *dev, struct motor_state *state) {
    struct fake_motor *fake = dev ? dev->priv_data : NULL;
    if (!fake || !state) return -1;
    ++fake->get_calls;
    state->pos = 1.0f;
    dev->feedback_timestamp_us = fake->state_timestamp_us;
    return fake->get_result;
}

static void fake_free(struct motor_dev *dev) {
    if (!dev) return;
    free(dev->priv_data);
    free(dev);
}

static const struct motor_ops kFakeOps = {
    .init = fake_init,
    .set_cmd = fake_set,
    .get_state = fake_get,
    .free = fake_free,
};

static struct motor_dev *fake_factory(void *args) {
    const struct motor_args_can *can_args = args;
    struct motor_dev *dev;
    struct fake_motor *fake;

    if (!can_args || !can_args->iface || can_args->can_id > 0x7ffU) return NULL;
    if (can_args->options &&
        !motor_option_find(can_args->options, can_args->option_count, "profile")) {
        return NULL;
    }
    dev = calloc(1, sizeof(*dev));
    fake = calloc(1, sizeof(*fake));
    if (!dev || !fake) {
        free(fake);
        free(dev);
        return NULL;
    }
    dev->name = "fake_can";
    dev->ops = &kFakeOps;
    dev->priv_data = fake;
    return dev;
}

static struct fake_motor *fake_data(struct motor_dev *dev) {
    return dev ? dev->priv_data : NULL;
}

int main(void) {
    struct driver_info driver = {
        .name = "fake_can",
        .type = DRV_TYPE_CAN,
        .factory = fake_factory,
        .next = NULL,
    };
    const struct motor_option options[] = {{"profile", "offline"}};
    struct motor_dev *configured;
    struct motor_dev *motors[2];
    struct motor_dev *sparse_motors[2];
    struct motor_cmd commands[2] = {0};
    struct motor_state states[2] = {0};
    uint64_t timestamps_us[2] = {0};

    motor_driver_register(&driver);
    configured = motor_alloc_can_with_options(
        "fake_can", "vcan0", 1, options, sizeof(options) / sizeof(options[0]));
    assert(configured != NULL);
    motor_free(&configured, 1);
    assert(motor_alloc_can_with_options("fake_can", "vcan0", 1, NULL, 1) == NULL);

    motors[0] = motor_alloc_can("fake_can", "vcan0", 1, NULL);
    motors[1] = motor_alloc_can("fake_can", "vcan0", 2, NULL);
    assert(motors[0] != NULL && motors[1] != NULL);
    assert(motor_init(motors, 2) == 0);

    fake_data(motors[1])->set_result = -7;
    assert(motor_set_cmds(motors, commands, 2) == -7);
    assert(fake_data(motors[0])->set_calls == 1);
    assert(fake_data(motors[1])->set_calls == 1);

    fake_data(motors[0])->set_result = -1;
    fake_data(motors[0])->set_errno = ENOBUFS;
    fake_data(motors[1])->set_errno = ENETDOWN;
    assert(motor_set_cmds(motors, commands, 2) == -1);
    assert(errno == ENOBUFS);
    assert(fake_data(motors[0])->set_calls == 2);
    assert(fake_data(motors[1])->set_calls == 2);
    fake_data(motors[0])->set_result = 0;
    fake_data(motors[0])->set_errno = 0;
    assert(motor_set_cmds(motors, commands, 2) == -7);
    assert(errno == ENETDOWN);
    fake_data(motors[1])->set_errno = 0;
    errno = EBUSY;
    assert(motor_set_cmds(motors, commands, 2) == -7);
    assert(errno == 0);
    fake_data(motors[1])->set_result = 0;
    assert(motor_set_cmds(motors, commands, 2) == 0);
    assert(errno == 0);

    fake_data(motors[1])->get_result = -8;
    assert(motor_get_states(motors, states, 2) == -8);
    assert(states[0].pos == 1.0f);
    assert(motor_get_feedback_timestamps(motors, timestamps_us, 2) == 0);
    assert(timestamps_us[0] == 0U);

    sparse_motors[0] = motors[0];
    sparse_motors[1] = NULL;
    fake_data(motors[0])->set_result = 0;
    fake_data(motors[0])->get_result = 0;
    fake_data(motors[0])->state_timestamp_us = 123U;
    assert(motor_init(sparse_motors, 2) == 0);
    assert(motor_set_cmds(sparse_motors, commands, 2) == 0);
    assert(motor_get_states(sparse_motors, states, 2) == 0);
    assert(motor_get_feedback_timestamps(sparse_motors, timestamps_us, 2) == 0);
    assert(timestamps_us[0] == 123U);
    assert(timestamps_us[1] == 0U);

    assert(motor_init(NULL, 1) < 0);
    assert(motor_set_cmds(NULL, commands, 1) < 0);
    assert(motor_get_states(motors, NULL, 1) < 0);
    assert(motor_get_feedback_timestamps(motors, NULL, 1) < 0);
    assert(motor_init(NULL, 0) == 0);
    motor_free(NULL, 1);

    motor_free(motors, 2);
    return 0;
}
