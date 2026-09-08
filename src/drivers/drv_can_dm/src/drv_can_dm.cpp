/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file drv_can_dm.cpp
 * @brief Damiao driver registration and generic motor API operations.
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
// 生命周期约定：priv 与其宿主 motor_dev 严格同生共死——
//   - 在 dm1_probe 中一并分配，dev->name 指向 priv->dev_name；
//   - 在 dm1_free 中必须先释放 dev（不再访问 dev->name）再释放 priv，
//     或如当前实现在同一函数内先 free(priv) 后 free(dev) 且其间不再解引用 dev->name。
// 因此 dev->name 不会悬空。请勿在别处单独释放 priv 而保留 dev。
struct dm1_priv {
    char bus_name[32];
    char dev_name[48];  // dev->name 指向此处，生命周期同 priv
    uint16_t can_id;
    struct timespec cmd_send_time;
    bool initialized;
    bool cmd_sent;
    bool enabled;
    bool enable_on_init;
};

static constexpr uint32_t kDefaultCanTimeoutMs = 500;
static constexpr uint32_t kTimeoutCountsPerMs = 20;
static constexpr uint32_t kMaxStandardCanId = 0x7ff;
static constexpr uint32_t kMaxDamiaoCommandId = kMaxStandardCanId - damiao::POS_FORCE_MODE;

static int find_dm_motor_type(const char* model, uint16_t* motor_type) {
    struct ModelEntry {
        const char* name;
        damiao::DM_Motor_Type type;
    };
    static constexpr ModelEntry kModels[] = {
        {"Damiao DM-J4310-2EC", damiao::DMJ4310_2EC},
        {"Damiao DM-J4340P-2EC", damiao::DMJ4340P_2EC},
        {"Damiao DM-J6248P-2EC", damiao::DMJ6248P_2EC},
    };

    if (!model || !motor_type) return -1;
    for (const auto& entry : kModels) {
        if (strcmp(model, entry.name) == 0) {
            *motor_type = static_cast<uint16_t>(entry.type);
            return 0;
        }
    }
    return -1;
}

static int parse_dm_options(const struct motor_args_can* params,
        struct motor_can_dm_config* config) {
    static const char* const kAllowedOptions[] = {
        "model",
        "feedback_id",
        "motor_type",
        "feedback_period_us",
        "can_timeout_ms",
        "enable_on_init",
        "protocol_limits.position.0",
        "protocol_limits.position.1",
        "protocol_limits.velocity.0",
        "protocol_limits.velocity.1",
        "protocol_limits.torque.0",
        "protocol_limits.torque.1",
        "protocol_limits.kp.0",
        "protocol_limits.kp.1",
        "protocol_limits.kd.0",
        "protocol_limits.kd.1",
    };
    static const char* const kLimitNames[] = {
        "protocol_limits.position.0",
        "protocol_limits.position.1",
        "protocol_limits.velocity.0",
        "protocol_limits.velocity.1",
        "protocol_limits.torque.0",
        "protocol_limits.torque.1",
        "protocol_limits.kp.0",
        "protocol_limits.kp.1",
        "protocol_limits.kd.0",
        "protocol_limits.kd.1",
    };
    float* limit_values[] = {
        &config->limits.position_min,
        &config->limits.position_max,
        &config->limits.velocity_min,
        &config->limits.velocity_max,
        &config->limits.torque_min,
        &config->limits.torque_max,
        &config->limits.kp_min,
        &config->limits.kp_max,
        &config->limits.kd_min,
        &config->limits.kd_max,
    };
    uint32_t value = 0;
    uint32_t timeout_ms = kDefaultCanTimeoutMs;
    const char* model;
    size_t limit_count = 0;
    int enable_result;
    int feedback_period_result;
    int timeout_result;

    if (!params || !config || !params->options || params->option_count == 0 ||
        motor_options_validate(params->options, params->option_count, kAllowedOptions,
            sizeof(kAllowedOptions) / sizeof(kAllowedOptions[0])) != 0 ||
        motor_option_read_u32(params->options, params->option_count, "feedback_id", &value) != 0 ||
        value > kMaxStandardCanId) {
        return -1;
    }
    config->feedback_id = static_cast<uint16_t>(value);

    model = motor_option_find(params->options, params->option_count, "model");
    if (model && find_dm_motor_type(model, &config->motor_type) != 0) return -1;
    const int motor_type_result = motor_option_read_u32(
        params->options, params->option_count, "motor_type", &value);
    if (motor_type_result < 0) return -1;
    if (motor_type_result == 0) {
        if (model || value >= damiao::Num_Of_Motor) return -1;
        config->motor_type = static_cast<uint16_t>(value);
    }

    feedback_period_result = motor_option_read_u32(params->options, params->option_count,
        "feedback_period_us", &config->feedback_period_us);
    if (feedback_period_result < 0 ||
        (feedback_period_result == 0 && config->feedback_period_us == 0)) return -1;

    enable_result = motor_option_read_bool(params->options, params->option_count,
        "enable_on_init", &config->enable_on_init);
    if (enable_result < 0) return -1;

    timeout_result = motor_option_read_u32(params->options, params->option_count,
        "can_timeout_ms", &timeout_ms);
    if (timeout_result < 0 || timeout_ms > UINT32_MAX / kTimeoutCountsPerMs)
        return -1;
    config->can_timeout_ms = timeout_ms;
    for (size_t i = 0; i < sizeof(kLimitNames) / sizeof(kLimitNames[0]); ++i) {
        if (motor_option_find(params->options, params->option_count, kLimitNames[i]))
            ++limit_count;
    }
    if (limit_count != 0 && limit_count != sizeof(kLimitNames) / sizeof(kLimitNames[0]))
        return -1;
    if (limit_count > 0) {
        for (size_t i = 0; i < sizeof(kLimitNames) / sizeof(kLimitNames[0]); ++i) {
            if (motor_option_read_float(params->options, params->option_count,
                    kLimitNames[i], limit_values[i]) != 0) {
                return -1;
            }
        }
        config->use_custom_limits = true;
    }
    return 0;
}

// ========== 1. 初始化 ==========

static int dm1_init(struct motor_dev* dev) {
    struct dm1_priv* priv;

    if (!dev || !dev->priv_data) return -1;
    priv = static_cast<struct dm1_priv*>(dev->priv_data);

    printf("[drv_can_dm1] Motor %s (CAN ID: 0x%02X, Bus: %s) initializing...\n", dev->name, priv->can_id,
            priv->bus_name);

    if (damiao_init_global() != 0) {
        return -1;
    }

    priv->initialized = true;
    priv->cmd_sent = false;
    priv->enabled = priv->enable_on_init;
    return 0;
}

// ========== 2. 发送指令（根据 mode 分发） ==========

static int dm1_set_cmd(struct motor_dev* dev, const struct motor_cmd* cmd) {
    struct dm1_priv* priv = dev ? static_cast<struct dm1_priv*>(dev->priv_data) : nullptr;

    if (!priv || !cmd || !priv->initialized) return -1;
    if (cmd->mode > MOTOR_MODE_HYBRID) return -1;
    if (cmd->mode == MOTOR_MODE_HYBRID &&
        damiao_validate_mit_cmd(priv->bus_name, priv->can_id, cmd->pos_des,
            cmd->vel_des, cmd->trq_des, cmd->kp, cmd->kd) != 0) {
        return -1;
    }
    if (cmd->mode == MOTOR_MODE_TRQ &&
        damiao_validate_mit_cmd(priv->bus_name, priv->can_id, 0.0f, 0.0f,
            cmd->trq_des, 0.0f, 0.0f) != 0) {
        return -1;
    }
    if (cmd->mode == MOTOR_MODE_POS &&
        damiao_validate_mit_cmd(priv->bus_name, priv->can_id, cmd->pos_des,
            cmd->vel_des, 0.0f, 0.0f, 0.0f) != 0) {
        return -1;
    }
    if (cmd->mode == MOTOR_MODE_VEL &&
        damiao_validate_mit_cmd(priv->bus_name, priv->can_id, 0.0f,
            cmd->vel_des, 0.0f, 0.0f, 0.0f) != 0) {
        return -1;
    }

    clock_gettime(CLOCK_MONOTONIC, &priv->cmd_send_time);

    if (cmd->mode == MOTOR_MODE_IDLE) {
        if (damiao_disable_once(priv->bus_name, priv->can_id) != 0) return -1;
        priv->enabled = false;
        return 0;
    }

    const int mode_result = damiao_prepare_mode(priv->bus_name, priv->can_id, cmd->mode);
    if (mode_result < 0) return -1;
    if (mode_result > 0) priv->enabled = false;

    if (!priv->enabled) {
        if (cmd->mode == MOTOR_MODE_HYBRID || cmd->mode == MOTOR_MODE_TRQ) {
            const float neutral_position =
                cmd->mode == MOTOR_MODE_HYBRID ? cmd->pos_des : 0.0f;
            if (damiao_set_cmd(priv->bus_name, priv->can_id, cmd->mode,
                neutral_position, 0.0f, 0.0f, 0.0f, 0.0f) != 0) {
                return -1;
            }
        }
        if (damiao_enable(priv->bus_name, priv->can_id) != 0) {
            return -1;
        }
        priv->enabled = true;
    }

    if (!priv->cmd_sent && cmd->mode != MOTOR_MODE_IDLE) {
        printf("[drv_can_dm1] Motor %s (CAN ID: 0x%02X) first cmd at %ld.%09ld\n", dev->name, priv->can_id,
                priv->cmd_send_time.tv_sec, priv->cmd_send_time.tv_nsec);
        priv->cmd_sent = true;
    }

    // 直接透传 mode，由 damiao_set_cmd 内部按模式分发
    const int result = damiao_set_cmd(priv->bus_name, priv->can_id, cmd->mode,
        cmd->pos_des, cmd->vel_des, cmd->trq_des, cmd->kp, cmd->kd);
    if (result < 0 && priv->enabled) {
        if (damiao_disable(priv->bus_name, priv->can_id) == 0)
            priv->enabled = false;
    }
    return result;
}

// ========== 3. 获取状态 ==========

static int dm1_get_state(struct motor_dev* dev, struct motor_state* state) {
    struct dm1_priv* priv;

    if (!dev || !dev->priv_data || !state) return -1;
    priv = static_cast<struct dm1_priv*>(dev->priv_data);

    float pos, vel, trq, temperature;
    uint32_t error;
    uint64_t timestamp_us;
    if (damiao_get_state(priv->bus_name, priv->can_id, &pos, &vel, &trq,
        &temperature, &error, &timestamp_us) == 0) {
        state->pos = pos;
        state->vel = vel;
        state->trq = trq;
        state->temp = temperature;
        state->err = error;
        dev->feedback_timestamp_us = timestamp_us;
        return 0;
    }
    return -1;
}

// ========== 4. 释放电机 ==========

static void dm1_free(struct motor_dev* dev) {
    if (!dev) return;

    struct dm1_priv* priv = (struct dm1_priv*)dev->priv_data;

    if (priv) {
        if (priv->initialized) {
            damiao_disable(priv->bus_name, priv->can_id);
            damiao_release(priv->bus_name, priv->can_id);
        } else {
            damiao_remove_config(priv->bus_name, priv->can_id);
        }
    }

    // dev->name 指向 priv->dev_name：先断开引用，再释放 priv，避免释放后残留悬空指针
    dev->name = NULL;
    if (dev->priv_data) {
        free(dev->priv_data);
        dev->priv_data = NULL;
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
    struct motor_can_dm_config option_config = {};
    const struct motor_can_dm_config* config = nullptr;

    if (!params || !params->iface || params->can_id > kMaxDamiaoCommandId)
        return nullptr;

    if (params->options) {
        if (parse_dm_options(params, &option_config) != 0) return nullptr;
        config = &option_config;
    } else {
        config = static_cast<const struct motor_can_dm_config*>(params->args);
    }

    struct motor_dev* dev = (struct motor_dev*)calloc(1, sizeof(struct motor_dev));
    if (!dev) return NULL;

    struct dm1_priv* priv = (struct dm1_priv*)calloc(1, sizeof(struct dm1_priv));
    if (!priv) {
        free(dev);
        return NULL;
    }

    dev->ops = &dm1_ops;
    dev->priv_data = priv;

    priv->initialized = false;
    priv->cmd_sent = false;
    priv->enabled = false;

    if (params->iface) {
        snprintf(priv->bus_name, sizeof(priv->bus_name), "%s", params->iface);
    } else {
        snprintf(priv->bus_name, sizeof(priv->bus_name), "can0");
    }
    priv->can_id = params->can_id;

    priv->enable_on_init = config ? config->enable_on_init : true;

    // 生成唯一设备名（含总线与 CAN ID），便于多总线/多电机场景区分日志
    snprintf(priv->dev_name, sizeof(priv->dev_name), "dm_%s_0x%02X", priv->bus_name, priv->can_id);
    dev->name = priv->dev_name;

    // 收集配置，等待 init 时统一初始化
    if (damiao_add_config(priv->bus_name, priv->can_id, config) != 0) {
        free(priv);
        free(dev);
        return nullptr;
    }

    return dev;
}

// ========== 注册驱动 ==========

REGISTER_MOTOR_DRIVER("drv_can_dm", DRV_TYPE_CAN, dm1_probe);
