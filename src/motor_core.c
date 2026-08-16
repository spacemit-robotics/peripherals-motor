/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file motor_core.c
 * @brief Motor driver registry and common motor API implementation.
 */

#include "motor_core.h"

#include <errno.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static struct driver_info *g_driver_list = NULL;

/* 注册实现 */
void motor_driver_register(struct driver_info *info) {
    if (!info || !info->name || !info->factory)
        return;
    info->next = g_driver_list;
    g_driver_list = info;
    // printf("[Core] Registered driver: %s (Type: %d)\n", info->name,
    // info->type);
}

/* 查找驱动 */
static struct driver_info *find_driver(const char *name,
    enum driver_type type) {
    struct driver_info *curr = g_driver_list;

    if (!name)
        return NULL;
    while (curr) {
        if (strcmp(curr->name, name) == 0) {
            if (curr->type == type) {
                return curr;
            }
            printf("[Core] Error: Driver '%s' exists but type mismatch "
                "(Expected %d, Got %d)\n", name, type, curr->type);
            return NULL;
        }
        curr = curr->next;
    }
    printf("[Core] Error: Driver '%s' not found!\n", name);
    return NULL;
}

const char *motor_option_find(const struct motor_option *options,
    uint32_t option_count, const char *name) {
    if ((!options && option_count > 0) || !name)
        return NULL;
    for (uint32_t i = 0; i < option_count; ++i) {
        if (options[i].name && strcmp(options[i].name, name) == 0)
            return options[i].value;
    }
    return NULL;
}

int motor_options_validate(const struct motor_option *options,
    uint32_t option_count,
    const char *const *allowed_names,
    uint32_t allowed_count) {
    if ((!options && option_count > 0) || (!allowed_names && allowed_count > 0))
        return -1;
    for (uint32_t i = 0; i < option_count; ++i) {
        bool allowed = false;
        if (!options[i].name || !options[i].name[0] || !options[i].value)
            return -1;
        for (uint32_t j = 0; j < i; ++j) {
            if (strcmp(options[i].name, options[j].name) == 0)
                return -1;
        }
        for (uint32_t j = 0; j < allowed_count; ++j) {
            if (strcmp(options[i].name, allowed_names[j]) == 0) {
                allowed = true;
                break;
            }
        }
        if (!allowed)
            return -1;
    }
    return 0;
}

int motor_option_read_bool(const struct motor_option *options,
    uint32_t option_count, const char *name,
    bool *value) {
    const char *text;
    if (!value)
        return -1;
    text = motor_option_find(options, option_count, name);
    if (!text)
        return 1;
    if (strcmp(text, "true") == 0 || strcmp(text, "1") == 0) {
        *value = true;
        return 0;
    }
    if (strcmp(text, "false") == 0 || strcmp(text, "0") == 0) {
        *value = false;
        return 0;
    }
    return -1;
}

int motor_option_read_u32(const struct motor_option *options,
    uint32_t option_count, const char *name,
    uint32_t *value) {
    const char *text;
    char *end = NULL;
    unsigned long parsed;
    if (!value)
        return -1;
    text = motor_option_find(options, option_count, name);
    if (!text)
        return 1;
    errno = 0;
    parsed = strtoul(text, &end, 0);
    if (errno != 0 || end == text || *end != '\0' || parsed > UINT32_MAX || text[0] == '-')
        return -1;
    *value = (uint32_t)parsed;
    return 0;
}

int motor_option_read_float(const struct motor_option *options,
    uint32_t option_count, const char *name,
    float *value) {
    const char *text;
    char *end = NULL;
    float parsed;
    if (!value)
        return -1;
    text = motor_option_find(options, option_count, name);
    if (!text)
        return 1;
    errno = 0;
    parsed = strtof(text, &end);
    if (errno != 0 || end == text || *end != '\0' || !isfinite(parsed))
        return -1;
    *value = parsed;
    return 0;
}

/* --- Factory Implementations --- */

struct motor_dev *motor_alloc_can(const char *name, const char *iface,
    uint32_t can_id, void *args) {
    struct driver_info *drv = find_driver(name, DRV_TYPE_CAN);
    if (!drv)
        return NULL;

    // 参数打包
    struct motor_args_can args_can = {
        .iface = iface, .can_id = can_id, .args = args,
        .options = NULL, .option_count = 0};
    return drv->factory(&args_can);
}

struct motor_dev *motor_alloc_can_with_options(const char *name,
    const char *iface, uint32_t can_id,
    const struct motor_option *options, uint32_t option_count) {
    struct driver_info *drv;
    struct motor_args_can args_can;

    if ((!options && option_count > 0) || !iface)
        return NULL;
    drv = find_driver(name, DRV_TYPE_CAN);
    if (!drv)
        return NULL;
    args_can.iface = iface;
    args_can.can_id = can_id;
    args_can.args = NULL;
    args_can.options = options;
    args_can.option_count = option_count;
    return drv->factory(&args_can);
}

struct motor_dev *motor_alloc_uart(const char *name, const char *dev_path,
    uint32_t baud, uint8_t id, void *args) {
    struct driver_info *drv = find_driver(name, DRV_TYPE_UART);
    if (!drv)
        return NULL;

    struct motor_args_uart args_uart = {
        .dev_path = dev_path, .baud = baud, .id = id, .args = args};
    return drv->factory(&args_uart);
}

struct motor_dev *motor_alloc_pwm(const char *name, uint32_t ch, void *_args) {
    struct driver_info *drv = find_driver(name, DRV_TYPE_PWM);
    if (!drv)
        return NULL;

    struct motor_args_pwm args = {.ch = ch, .args = _args};
    return drv->factory(&args);
}

struct motor_dev *motor_alloc_ecat(const char *name, uint16_t slave_idx,
    void *args) {
    struct driver_info *drv = find_driver(name, DRV_TYPE_ECAT);
    if (!drv)
        return NULL;

    struct motor_args_ecat args_ecat = {.slave_idx = slave_idx, .args = args};
    return drv->factory(&args_ecat);
}

/* --- API Implementations --- */

int motor_init(struct motor_dev **devs, uint32_t count) {
    if (!devs && count > 0)
        return -1;

    for (uint32_t i = 0; i < count; i++) {
        int ret;

        if (!devs[i] || !devs[i]->ops || !devs[i]->ops->init)
            continue;
        ret = devs[i]->ops->init(devs[i]);
        if (ret < 0)
            return ret;
    }
    return 0;
}

int motor_set_cmds(struct motor_dev **devs, const struct motor_cmd *cmds,
    uint32_t count) {
    int result = 0;

    if ((!devs || !cmds) && count > 0)
        return -1;

    for (uint32_t i = 0; i < count; i++) {
        int ret;

        if (!devs[i] || !devs[i]->ops || !devs[i]->ops->set_cmd) {
            continue;
        }
        ret = devs[i]->ops->set_cmd(devs[i], &cmds[i]);
        if (ret < 0 && result == 0)
            result = ret;
    }
    return result;
}

int motor_get_states(struct motor_dev **devs, struct motor_state *states,
    uint32_t count) {
    int result = 0;

    if ((!devs || !states) && count > 0)
        return -1;

    for (uint32_t i = 0; i < count; i++) {
        int ret;

        if (!devs[i] || !devs[i]->ops || !devs[i]->ops->get_state) {
            continue;
        }
        ret = devs[i]->ops->get_state(devs[i], &states[i]);
        if (ret < 0 && result == 0)
            result = ret;
    }
    return result;
}

void motor_free(struct motor_dev **devs, uint32_t count) {
    if (!devs)
        return;
    for (uint32_t i = 0; i < count; i++) {
        if (devs[i] && devs[i]->ops && devs[i]->ops->free)
            devs[i]->ops->free(devs[i]);
    }
}

int motor_set_paras(struct motor_dev *dev, const void *address,
    const void *data, uint32_t data_len) {
    if (dev && dev->ops && dev->ops->set_paras)
        return dev->ops->set_paras(dev, address, data, data_len);
    return -1;
}

int motor_get_paras(struct motor_dev *dev, const void *address, void *out_data,
    uint32_t data_len) {
    if (dev && dev->ops && dev->ops->get_paras)
        return dev->ops->get_paras(dev, address, out_data, data_len);
    return -1;
}
