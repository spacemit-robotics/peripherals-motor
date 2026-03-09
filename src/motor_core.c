/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#include "motor_core.h"
#include <stddef.h>
#include <stdio.h>
#include <string.h>

static struct driver_info *g_driver_list = NULL;

/* 注册实现 */
void motor_driver_register(struct driver_info *info) {
    info->next = g_driver_list;
    g_driver_list = info;
    // printf("[Core] Registered driver: %s (Type: %d)\n", info->name,
    // info->type);
}

/* 查找驱动 */
static struct driver_info *find_driver(const char *name,
                                        enum driver_type type) {
    struct driver_info *curr = g_driver_list;
    while (curr) {
    if (strcmp(curr->name, name) == 0) {
        if (curr->type == type) {
        return curr;
        } else {
        printf("[Core] Error: Driver '%s' exists but type mismatch (Expected "
                "%d, Got %d)\n",
                name, type, curr->type);
        return NULL;
        }
    }
    curr = curr->next;
    }
    printf("[Core] Error: Driver '%s' not found!\n", name);
    return NULL;
}

/* --- Factory Implementations --- */

struct motor_dev *motor_alloc_can(const char *name, const char *iface,
                                    uint32_t can_id, void *args) {
    struct driver_info *drv = find_driver(name, DRV_TYPE_CAN);
    if (!drv)
    return NULL;

    // 参数打包
    struct motor_args_can args_can = {
        .iface = iface, .can_id = can_id, .args = args};
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
    for (uint32_t i = 0; i < count; i++) {
    if (devs[i] && devs[i]->ops && devs[i]->ops->init)
        devs[i]->ops->init(devs[i]);
    }
    return 0;
}

int motor_set_cmds(struct motor_dev **devs, const struct motor_cmd *cmds,
                    uint32_t count) {
    for (uint32_t i = 0; i < count; i++) {
    if (devs[i] && devs[i]->ops && devs[i]->ops->set_cmd)
        devs[i]->ops->set_cmd(devs[i], &cmds[i]);
    }
    return 0;
}

int motor_get_states(struct motor_dev **devs, struct motor_state *states,
                        uint32_t count) {
    for (uint32_t i = 0; i < count; i++) {
    if (devs[i] && devs[i]->ops && devs[i]->ops->get_state)
        devs[i]->ops->get_state(devs[i], &states[i]);
    }
    return 0;
}

void motor_free(struct motor_dev **devs, uint32_t count) {
    for (uint32_t i = 0; i < count; i++) {
    if (devs[i]) {
        if (devs[i]->ops && devs[i]->ops->free)
        devs[i]->ops->free(devs[i]);
        // free(devs[i]); // dev内存通常由驱动里的free释放，这里取决于设计约定
        // 在本例中，驱动的 free 会释放 dev 结构体本身，所以这里不重复 free
    }
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
