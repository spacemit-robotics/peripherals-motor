/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */
#include "damiao_pack.h"

#include <unistd.h>

#include <chrono>
#include <csignal>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>


extern "C" {
#include "../../../../include/motor.h"
}

// 全局硬件接口
std::shared_ptr<damiao::DamiaoHW> g_damiao_hw = nullptr;
std::atomic<bool> g_damiao_running(true);

// 待初始化配置
static std::vector<damiao::MotorConfig> g_pending_configs;
static bool g_initialized = false;

// 信号处理
static void damiao_signal_handler(int signum) {
    g_damiao_running = false;
    std::cout << "\n[DamiaoPack] Signal " << signum << " received, releasing motors..." << std::endl;
    damiao_release_all();
}

extern "C" {

// ========== 1. 初始化 ==========

void damiao_add_config(const char* bus_name, uint16_t can_id, uint16_t motor_type) {
    damiao::MotorConfig config;
    config.bus_name = bus_name;
    config.can_id = can_id;
    config.motor_type = static_cast<damiao::DM_Motor_Type>(motor_type);
    config.master_id = can_id + 0x10;
    config.control_mode = damiao::MIT_MODE;
    g_pending_configs.push_back(config);
}

int damiao_init_global(void) {
    if (g_initialized) return 0;

    signal(SIGINT, damiao_signal_handler);

    if (!g_damiao_hw) {
        g_damiao_hw = std::make_shared<damiao::DamiaoHW>();
    }

    g_damiao_hw->setThreadPriority(95);

    if (!g_damiao_hw->init(g_pending_configs)) {
        std::cerr << "[DamiaoPack] Init failed" << std::endl;
        return -1;
    }

    // 强制切换到配置的控制模式（MIT），确保电机实际模式与代码一致
    // 电机掉电后可能保留上次的模式，不一定是 MIT
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
        g_damiao_hw->switchMode(config.bus_name, config.can_id, code);
    }
    usleep(100000);  // 等待模式切换生效
    g_damiao_hw->enableAll();
    usleep(50000);

    // 启动自动读取线程，5ms 周期刷新总线数据
    g_damiao_hw->startAutoRead(5);

    g_initialized = true;
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
            return damiao::POS_FORCE;
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
            return damiao::POS_FORCE_MODE;
        default:
            return damiao::MIT_MODE;
    }
}

int damiao_set_cmd(const char* bus_name, uint16_t can_id, uint32_t mode, float pos, float vel, float trq, float kp,
                    float kd) {
    if (!g_damiao_hw) return -1;

    std::string bus(bus_name);

    // 模式切换检测：当前模式与目标模式不一致时，先切换模式
    if (mode != MOTOR_MODE_IDLE) {
        damiao::Control_Mode current = g_damiao_hw->getCurrentMode(bus, can_id);
        damiao::Control_Mode target = mode_to_dm_ctrl(mode);

        if (current != target) {
            std::cout << "[DamiaoPack] Switching motor " << can_id << " mode: " << current << " -> " << target
                    << std::endl;
            g_damiao_hw->switchMode(bus, can_id, mode_to_dm_code(mode));
            // 切换后仅重新使能当前电机，避免影响同一 HW 下其他总线/电机的使能状态
            g_damiao_hw->enable(bus, can_id);
            // 等待模式切换生效
            usleep(100000);  // 100ms
        }
    }

    switch (mode) {
        case MOTOR_MODE_HYBRID:
            g_damiao_hw->controlMit(bus, can_id, pos, vel, trq, kp, kd);
            break;

        case MOTOR_MODE_POS:
            g_damiao_hw->controlPosVel(bus, can_id, pos, vel);
            break;

        case MOTOR_MODE_VEL:
            g_damiao_hw->controlVel(bus, can_id, vel);
            break;

        case MOTOR_MODE_TRQ:
            g_damiao_hw->controlPosForce(bus, can_id, pos, vel, trq);
            break;

        case MOTOR_MODE_IDLE:
            // 仅失能当前电机，避免误伤同一 HW 下其他总线/电机
            g_damiao_hw->disable(bus, can_id);
            break;

        default:
            std::cerr << "[DamiaoPack] Unknown mode: " << mode << std::endl;
            return -1;
    }

    return 0;
}

// ========== 3. 获取状态 ==========

int damiao_get_state(const char* bus_name, uint16_t can_id, float* pos, float* vel, float* trq) {
    if (!g_damiao_hw) return -1;

    std::string bus(bus_name);
    damiao::DmActData* data = g_damiao_hw->getMotorData(bus, can_id);
    if (!data) return -1;

    *pos = data->pos;
    *vel = data->vel;
    *trq = data->effort;
    return 0;
}

// ========== 4. 释放电机 ==========

void damiao_release(const char* bus_name, uint16_t can_id) {
    if (!g_damiao_hw) return;

    std::string bus(bus_name);
    g_damiao_hw->setZeroPosition(bus, can_id);
}

void damiao_release_all(void) {
    if (!g_damiao_hw) return;

    g_damiao_hw->stopAutoRead();
    g_damiao_hw->disableAll();
    g_damiao_hw.reset();
    g_initialized = false;
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
