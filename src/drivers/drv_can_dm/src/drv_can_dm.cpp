/*
 * Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * 驱动适配层 - 基于 damiao_pack.cpp (DamiaoHW)
 * 相比 drv_can_dm.cpp 的改进：
 *   - 支持多模式控制（通过 motor_cmd.mode 分发）
 *   - 支持寄存器参数读写
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

extern "C" {
#include "../../../../include/motor.h"
#include "../../../motor_core.h"
}

#include "damiao_pack.h"

// 私有数据
struct dm1_priv {
    char bus_name[32];
    uint16_t can_id;
    struct timespec cmd_send_time;
    bool initialized;
    bool cmd_sent;
};

// ========== 1. 初始化 ==========

static int dm1_init(struct motor_dev* dev) {
    struct dm1_priv* priv = (struct dm1_priv*)dev->priv_data;

    printf("[drv_can_dm1] Motor %s (CAN ID: 0x%02X, Bus: %s) initializing...\n", dev->name, priv->can_id,
            priv->bus_name);

    if (damiao_init_global() != 0) {
        return -1;
    }

    priv->initialized = true;
    priv->cmd_sent = false;
    return 0;
}

// ========== 2. 发送指令（根据 mode 分发） ==========

static int dm1_set_cmd(struct motor_dev* dev, const struct motor_cmd* cmd) {
    struct dm1_priv* priv = (struct dm1_priv*)dev->priv_data;

    clock_gettime(CLOCK_MONOTONIC, &priv->cmd_send_time);

    if (!priv->cmd_sent) {
        printf("[drv_can_dm1] Motor %s (CAN ID: 0x%02X) first cmd at %ld.%09ld\n", dev->name, priv->can_id,
                priv->cmd_send_time.tv_sec, priv->cmd_send_time.tv_nsec);
        priv->cmd_sent = true;
    }

    // 直接透传 mode，由 damiao_set_cmd 内部按模式分发
    return damiao_set_cmd(priv->bus_name, priv->can_id, cmd->mode, cmd->pos_des, cmd->vel_des, cmd->trq_des, cmd->kp,
                            cmd->kd);
}

// ========== 3. 获取状态 ==========

static int dm1_get_state(struct motor_dev* dev, struct motor_state* state) {
    struct dm1_priv* priv = (struct dm1_priv*)dev->priv_data;

    float pos, vel, trq;
    if (damiao_get_state(priv->bus_name, priv->can_id, &pos, &vel, &trq) == 0) {
        state->pos = pos;
        state->vel = vel;
        state->trq = trq;
        state->temp = 0.0f;
        state->err = 0;
        return 0;
    }
    return -1;
}

// ========== 4. 释放电机 ==========

static void dm1_free(struct motor_dev* dev) {
    struct dm1_priv* priv = (struct dm1_priv*)dev->priv_data;

    if (priv && priv->initialized) {
        damiao_release(priv->bus_name, priv->can_id);
    }

    if (dev->priv_data) {
        free(dev->priv_data);
    }
    free(dev);
}

// ========== 5. 获取参数 ==========

static int dm1_get_paras(struct motor_dev* dev, const void* address, void* out_data, uint32_t data_len) {
    if (!dev || !dev->priv_data || !address || !out_data) return -1;

    struct dm1_priv* priv = (struct dm1_priv*)dev->priv_data;
    uint8_t reg_id = (uint8_t)(uintptr_t)address;

    if (data_len >= sizeof(float)) {
        return damiao_get_param(priv->bus_name, priv->can_id, reg_id, reinterpret_cast<float*>(out_data));
    }
    return -1;
}

// ========== 6. 调节参数 ==========

static int dm1_set_paras(struct motor_dev* dev, const void* address, const void* data, uint32_t data_len) {
    if (!dev || !dev->priv_data || !address || !data) return -1;

    struct dm1_priv* priv = (struct dm1_priv*)dev->priv_data;
    uint8_t reg_id = (uint8_t)(uintptr_t)address;

    if (data_len == sizeof(float)) {
        float val = *(const float*)data;
        return damiao_set_param(priv->bus_name, priv->can_id, reg_id, val);
    }
    return -1;
}

// ========== ops 定义 ==========

static const struct motor_ops dm1_ops = {
    .init = dm1_init,
    .set_cmd = dm1_set_cmd,
    .get_state = dm1_get_state,
    .free = dm1_free,
    .set_paras = dm1_set_paras,
    .get_paras = dm1_get_paras,
};

// ========== probe ==========

static struct motor_dev* dm1_probe(void* args) {
    struct motor_args_can* params = (struct motor_args_can*)args;

    struct motor_dev* dev = (struct motor_dev*)calloc(1, sizeof(struct motor_dev));
    if (!dev) return NULL;

    struct dm1_priv* priv = (struct dm1_priv*)calloc(1, sizeof(struct dm1_priv));
    if (!priv) {
        free(dev);
        return NULL;
    }

    dev->name = "dm_can1";
    dev->ops = &dm1_ops;
    dev->priv_data = priv;

    priv->initialized = false;
    priv->cmd_sent = false;

    if (params->iface) {
        snprintf(priv->bus_name, sizeof(priv->bus_name), "%s", params->iface);
    } else {
        snprintf(priv->bus_name, sizeof(priv->bus_name), "can0");
    }
    priv->can_id = params->can_id;

    // 收集配置，等待 init 时统一初始化
    damiao_add_config(priv->bus_name, priv->can_id, 0);

    return dev;
}

// ========== 注册驱动 ==========

REGISTER_MOTOR_DRIVER("drv_can_dm", DRV_TYPE_CAN, dm1_probe);
