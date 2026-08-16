/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file damiao_pack.cpp
 * @brief Damiao adapter implementation for the generic motor API.
 */
#include "damiao_pack.h"

#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>


extern "C" {
#include "../../../../include/motor.h"
}

// 全局硬件接口
std::shared_ptr<damiao::DamiaoHW> g_damiao_hw = nullptr;

// 待初始化配置
static std::vector<damiao::MotorConfig> g_pending_configs;
static bool g_initialized = false;
static size_t g_clients = 0;
static std::mutex g_state_mutex;

constexpr uint32_t kDefaultFeedbackPeriodUs = 5000;
constexpr uint32_t kDefaultCanTimeoutMs = 500;
constexpr uint32_t kTimeoutCountsPerMs = 20;
constexpr useconds_t kParameterSettleUs = 10000;
constexpr useconds_t kModeSettleUs = 100000;
constexpr uint16_t kMaxStandardCanId = 0x7ff;
constexpr uint16_t kMaxDamiaoCommandId = kMaxStandardCanId - damiao::POS_FORCE_MODE;

static bool range_is_symmetric(float min, float max) {
    const float scale = std::max({1.0f, std::fabs(min), std::fabs(max)});
    return std::fabs(min + max) <= scale * 1.0e-5f;
}

extern "C" {

// ========== 1. 初始化 ==========

int damiao_add_config(const char* bus_name, uint16_t can_id,
        const struct motor_can_dm_config* driver_config) {
    std::lock_guard<std::mutex> lock(g_state_mutex);
    if (!bus_name || !*bus_name || can_id > kMaxDamiaoCommandId || g_initialized)
        return -1;
    damiao::MotorConfig config;
    config.bus_name = bus_name;
    config.can_id = can_id;
    const uint16_t motor_type = driver_config ? driver_config->motor_type : 0;
    if (motor_type >= damiao::Num_Of_Motor) return -1;
    config.motor_type = static_cast<damiao::DM_Motor_Type>(motor_type);
    config.master_id = driver_config ? driver_config->feedback_id : can_id + 0x10;
    if (config.master_id > kMaxStandardCanId) return -1;
    config.control_mode = damiao::MIT_MODE;
    config.feedback_period_us =
        driver_config && driver_config->feedback_period_us
        ? driver_config->feedback_period_us
        : kDefaultFeedbackPeriodUs;
    config.can_timeout_ms = driver_config
        ? driver_config->can_timeout_ms
        : kDefaultCanTimeoutMs;
    if (config.can_timeout_ms > UINT32_MAX / kTimeoutCountsPerMs) return -1;
    config.use_custom_limits = driver_config && driver_config->use_custom_limits;
    config.enable_on_init = !driver_config || driver_config->enable_on_init;
    if (config.use_custom_limits) {
        const struct damiao_protocol_limits& limits = driver_config->limits;
        if (!std::isfinite(limits.position_min) || !std::isfinite(limits.position_max) ||
            !std::isfinite(limits.velocity_min) || !std::isfinite(limits.velocity_max) ||
            !std::isfinite(limits.torque_min) || !std::isfinite(limits.torque_max) ||
            limits.position_min >= 0.0f || limits.position_max <= 0.0f ||
            limits.velocity_min >= 0.0f || limits.velocity_max <= 0.0f ||
            limits.torque_min >= 0.0f || limits.torque_max <= 0.0f ||
            !range_is_symmetric(limits.position_min, limits.position_max) ||
            !range_is_symmetric(limits.velocity_min, limits.velocity_max) ||
            !range_is_symmetric(limits.torque_min, limits.torque_max) ||
            limits.kp_min != 0.0f || limits.kp_max != 500.0f || limits.kd_min != 0.0f ||
            limits.kd_max != 5.0f) {
            return -1;
        }
        config.limits.Q_MAX = limits.position_max;
        config.limits.DQ_MAX = limits.velocity_max;
        config.limits.TAU_MAX = limits.torque_max;
    } else {
        config.limits = damiao::limit_param[config.motor_type];
    }
    for (const auto& existing : g_pending_configs) {
        if (existing.bus_name != config.bus_name) continue;
        if (existing.can_id == config.can_id || existing.master_id == config.master_id ||
            existing.can_id == config.master_id || existing.master_id == config.can_id) {
            return -1;
        }
    }
    g_pending_configs.push_back(config);
    return 0;
}

void damiao_remove_config(const char* bus_name, uint16_t can_id) {
    std::lock_guard<std::mutex> lock(g_state_mutex);
    if (!bus_name || g_initialized) return;
    g_pending_configs.erase(
        std::remove_if(g_pending_configs.begin(), g_pending_configs.end(),
            [bus_name, can_id](const damiao::MotorConfig& config) {
                return config.bus_name == bus_name && config.can_id == can_id;
            }),
        g_pending_configs.end());
}

int damiao_init_global(void) {
    std::lock_guard<std::mutex> lock(g_state_mutex);
    if (g_initialized) {
        ++g_clients;
        return 0;
    }

    if (!g_damiao_hw) {
        g_damiao_hw = std::make_shared<damiao::DamiaoHW>();
    }

    g_damiao_hw->setThreadPriority(95);

    if (!g_damiao_hw->init(g_pending_configs)) {
        std::cerr << "[DamiaoPack] Init failed" << std::endl;
        g_damiao_hw.reset();
        g_pending_configs.clear();
        return -1;
    }

    for (const auto& config : g_pending_configs) {
        damiao::Control_Mode_Code code = damiao::MIT;  // 默认 MIT
        switch (config.control_mode) {
            case damiao::POS_VEL_MODE:
                code = damiao::POS_VEL;
                break;
            case damiao::VEL_MODE:
                code = damiao::VEL;
                break;
            case damiao::POS_FORCE_MODE:
                code = damiao::POS_FORCE;
                break;
            default:
                code = damiao::MIT;
                break;
        }
        if (!g_damiao_hw->disableAllModes(config.bus_name, config.can_id) ||
            !g_damiao_hw->writeParam(config.bus_name, config.can_id, damiao::TIMEOUT,
                static_cast<float>(config.can_timeout_ms * kTimeoutCountsPerMs))) {
            g_damiao_hw->disableAll();
            g_damiao_hw.reset();
            g_pending_configs.clear();
            return -1;
        }
        usleep(kParameterSettleUs);
        if (!g_damiao_hw->switchMode(config.bus_name, config.can_id, code)) {
            g_damiao_hw->disableAll();
            g_damiao_hw.reset();
            g_pending_configs.clear();
            return -1;
        }
        usleep(kModeSettleUs);
        if (config.enable_on_init &&
            !g_damiao_hw->enable(config.bus_name, config.can_id)) {
            g_damiao_hw->disableAll();
            g_damiao_hw.reset();
            g_pending_configs.clear();
            return -1;
        }
    }

    const auto period = std::min_element(g_pending_configs.begin(), g_pending_configs.end(),
        [](const damiao::MotorConfig& lhs, const damiao::MotorConfig& rhs) {
            return lhs.feedback_period_us < rhs.feedback_period_us;
        });
    g_damiao_hw->startAutoRead(period == g_pending_configs.end()
        ? kDefaultFeedbackPeriodUs
        : period->feedback_period_us);

    g_initialized = true;
    g_clients = 1;
    return 0;
}

// ========== 2. 发送指令（根据 mode 分发） ==========

// motor_mode 到达妙 Control_Mode_Code 的映射
static damiao::Control_Mode_Code mode_to_dm_code(uint32_t mode) {
    switch (mode) {
        case MOTOR_MODE_HYBRID:
            return damiao::MIT;
        case MOTOR_MODE_POS:
            return damiao::POS_VEL;
        case MOTOR_MODE_VEL:
            return damiao::VEL;
        case MOTOR_MODE_TRQ:
            return damiao::MIT;
        default:
            return damiao::MIT;
    }
}

// motor_mode 到达妙 Control_Mode 的映射（用于比较当前模式）
static damiao::Control_Mode mode_to_dm_ctrl(uint32_t mode) {
    switch (mode) {
        case MOTOR_MODE_HYBRID:
            return damiao::MIT_MODE;
        case MOTOR_MODE_POS:
            return damiao::POS_VEL_MODE;
        case MOTOR_MODE_VEL:
            return damiao::VEL_MODE;
        case MOTOR_MODE_TRQ:
            return damiao::MIT_MODE;
        default:
            return damiao::MIT_MODE;
    }
}

int damiao_set_cmd(const char* bus_name, uint16_t can_id, uint32_t mode, float pos, float vel, float trq, float kp,
                    float kd) {
    if (!g_damiao_hw || !bus_name) return -1;

    if (mode == MOTOR_MODE_HYBRID &&
        damiao_validate_mit_cmd(bus_name, can_id, pos, vel, trq, kp, kd) != 0)
        return -1;
    if (mode == MOTOR_MODE_TRQ &&
        damiao_validate_mit_cmd(bus_name, can_id, 0.0f, 0.0f, trq, 0.0f, 0.0f) != 0)
        return -1;

    std::string bus(bus_name);

    switch (mode) {
        case MOTOR_MODE_HYBRID:
            if (!g_damiao_hw->controlMit(bus, can_id, pos, vel, trq, kp, kd)) return -1;
            break;

        case MOTOR_MODE_POS:
            if (!g_damiao_hw->controlPosVel(bus, can_id, pos, vel)) return -1;
            break;

        case MOTOR_MODE_VEL:
            if (!g_damiao_hw->controlVel(bus, can_id, vel)) return -1;
            break;

        case MOTOR_MODE_TRQ:
            if (!g_damiao_hw->controlMit(bus, can_id, 0.0f, 0.0f, trq, 0.0f, 0.0f))
                return -1;
            break;

        case MOTOR_MODE_IDLE:
            break;

        default:
            std::cerr << "[DamiaoPack] Unknown mode: " << mode << std::endl;
            return -1;
    }

    return 0;
}

int damiao_prepare_mode(const char* bus_name, uint16_t can_id, uint32_t mode) {
    if (!g_damiao_hw || !bus_name || mode == MOTOR_MODE_IDLE ||
        mode > MOTOR_MODE_HYBRID) {
        return -1;
    }
    const std::string bus(bus_name);
    const damiao::Control_Mode current = g_damiao_hw->getCurrentMode(bus, can_id);
    const damiao::Control_Mode target = mode_to_dm_ctrl(mode);
    if (current == target) return 0;

    if (!g_damiao_hw->disableAllModes(bus, can_id) ||
        !g_damiao_hw->switchMode(bus, can_id, mode_to_dm_code(mode))) {
        return -1;
    }
    usleep(kModeSettleUs);
    return 1;
}

int damiao_validate_mit_cmd(const char* bus_name, uint16_t can_id,
        float pos, float vel, float trq, float kp, float kd) {
    if (!g_damiao_hw || !bus_name) return -1;
    return g_damiao_hw->validateMitCommand(bus_name, can_id, pos, vel, trq, kp, kd) ? 0 : -1;
}

// ========== 3. 获取状态 ==========

int damiao_get_state(const char* bus_name, uint16_t can_id, float* pos, float* vel, float* trq,
        float* temperature, uint32_t* error) {
    if (!g_damiao_hw || !bus_name) return -1;
    return g_damiao_hw->getMotorState(
        bus_name, can_id, pos, vel, trq, temperature, error) ? 0 : -1;
}

// ========== 4. 释放电机 ==========

void damiao_release(const char* bus_name, uint16_t can_id) {
    (void)bus_name;
    (void)can_id;
    std::lock_guard<std::mutex> lock(g_state_mutex);
    if (!g_initialized || g_clients == 0) return;
    if (--g_clients > 0) return;
    g_damiao_hw->stopAutoRead();
    g_damiao_hw.reset();
    g_pending_configs.clear();
    g_initialized = false;
}

void damiao_release_all(void) {
    std::lock_guard<std::mutex> lock(g_state_mutex);
    if (g_damiao_hw) {
        g_damiao_hw->stopAutoRead();
        g_damiao_hw->disableAll();
        g_damiao_hw.reset();
    }
    g_pending_configs.clear();
    g_initialized = false;
    g_clients = 0;
}

int damiao_enable(const char* bus_name, uint16_t can_id) {
    if (!g_damiao_hw || !bus_name) return -1;
    return g_damiao_hw->enable(bus_name, can_id) ? 0 : -1;
}

int damiao_disable(const char* bus_name, uint16_t can_id) {
    if (!g_damiao_hw || !bus_name) return -1;
    return g_damiao_hw->disable(bus_name, can_id) ? 0 : -1;
}

int damiao_disable_once(const char* bus_name, uint16_t can_id) {
    if (!g_damiao_hw || !bus_name) return -1;
    return g_damiao_hw->disableOnce(bus_name, can_id) ? 0 : -1;
}

// ========== 寄存器权限检查 ==========

struct RegInfo {
    const char* name;
    bool writable;
};

static const RegInfo* get_reg_info(uint8_t reg_id) {
    // 寄存器权限表，参考达妙手册 DM-J4310-2EC V1.2
    static const std::unordered_map<uint8_t, RegInfo> reg_table = {
        // RW 寄存器
        {0, {"UV_Value", true}},
        {1, {"KT_Value", true}},
        {2, {"OT_Value", true}},
        {3, {"OC_Value", true}},
        {4, {"ACC", true}},
        {5, {"DEC", true}},
        {6, {"MAX_SPD", true}},
        {7, {"MST_ID", true}},
        {8, {"ESC_ID", true}},
        {9, {"TIMEOUT", true}},
        {10, {"CTRL_MODE", true}},
        {21, {"PMAX", true}},
        {22, {"VMAX", true}},
        {23, {"TMAX", true}},
        {24, {"I_BW", true}},
        {25, {"KP_ASR", true}},
        {26, {"KI_ASR", true}},
        {27, {"KP_APR", true}},
        {28, {"KI_APR", true}},
        {29, {"OV_Value", true}},
        {30, {"GREF", true}},
        {31, {"Deta", true}},
        {32, {"V_BW", true}},
        {33, {"IQ_c1", true}},
        {34, {"VL_c1", true}},
        {35, {"can_br", true}},
        // RO 寄存器
        {11, {"Damp", false}},
        {12, {"Inertia", false}},
        {13, {"hw_ver", false}},
        {14, {"sw_ver", false}},
        {15, {"SN", false}},
        {16, {"NPP", false}},
        {17, {"Rs", false}},
        {18, {"Ls", false}},
        {19, {"Flux", false}},
        {20, {"Gr", false}},
        {36, {"sub_ver", false}},
        {37, {"Boot_ver", false}},
        {55, {"dir", false}},
        {56, {"m_off", false}},
        {59, {"Imax", false}},
        {60, {"VBus", false}},
        {61, {"Tpcb", false}},
        {62, {"Tmtr", false}},
        {63, {"Iu_off", false}},
        {64, {"Iv_off", false}},
        {65, {"Iw_off", false}},
        {80, {"p_m", false}},
        {81, {"xout", false}},
    };

    auto it = reg_table.find(reg_id);
    if (it == reg_table.end()) return nullptr;
    return &it->second;
}

// ========== 5. 获取参数 ==========

int damiao_get_param(const char* bus_name, uint16_t can_id, uint8_t reg_id, float* out_value) {
    if (!g_damiao_hw || !out_value) return -1;

    const RegInfo* info = get_reg_info(reg_id);
    if (!info) {
        std::cerr << "[DamiaoPack] WARNING: Register " << static_cast<int>(reg_id) <<
        " is not a valid register, read denied" << std::endl;
        return -1;
    }

    std::string bus(bus_name);
    g_damiao_hw->readParam(bus, can_id, reg_id);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    *out_value = g_damiao_hw->getCachedParam(bus, can_id, reg_id);
    return 0;
}

// ========== 6. 调节参数 ==========
// 写前进行权限检查
int damiao_set_param(const char* bus_name, uint16_t can_id, uint8_t reg_id, float value) {
    if (!g_damiao_hw) return -1;

    const RegInfo* info = get_reg_info(reg_id);
    if (!info) {
        std::cerr << "[DamiaoPack] WARNING: Register " << static_cast<int>(reg_id) <<
        " is not a valid register, write denied" << std::endl;
        return -1;
    }
    if (!info->writable) {
        std::cerr << "[DamiaoPack] WARNING: Register " << info->name << " (RID=" << static_cast<int>(reg_id)
                    << ") is READ-ONLY, write denied" << std::endl;
        return -1;
    }

    std::string bus(bus_name);
    return g_damiao_hw->writeParam(bus, can_id, reg_id, value) ? 0 : -1;
}

void damiao_save_param(const char* bus_name, uint16_t can_id) {
    if (!g_damiao_hw) return;

    std::string bus(bus_name);
    g_damiao_hw->saveParam(bus, can_id);
}

}  // extern "C"
