/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file damiao_hw.cpp
 * @brief Damiao multi-bus hardware manager implementation.
 */

#include <unistd.h>

#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "damiao_hw.h"
#include "damiao_protocol.h"

namespace damiao {

// ========== 初始化接口实现 ==========

bool DamiaoHW::init(const std::vector<MotorConfig>& motor_configs) {
    std::unordered_map<std::string, std::vector<MotorConfig>> bus_groups;
    for (const auto& config : motor_configs) {
        bus_groups[config.bus_name].push_back(config);
    }

    for (const auto& group : bus_groups) {
        const std::string& bus_name = group.first;
        const std::vector<MotorConfig>& configs = group.second;

        for (const auto& config : configs) {
            bus_motor_data_[bus_name][config.can_id] = DmActData{.motorType = config.motor_type,
                                                                .mode = config.control_mode,
                                                                .can_id = config.can_id,
                                                                .mst_id = config.master_id,
                                                                .pos = 0,
                                                                .vel = 0,
                                                                .effort = 0,
                                                                .temperature = 0,
                                                                .error = 0,
                                                                .cmd_pos = 0,
                                                                .cmd_vel = 0,
                                                                .cmd_effort = 0,
                                                                .kp = 0,
                                                                .kd = 0,
                                                                .limits = config.limits,
                                                                .use_custom_limits = config.use_custom_limits,
                                                                .feedback_sequence = 0,
                                                                .consumed_sequence = 0};
        }
        try {
            auto controller = std::make_shared<Motor_Control>(bus_name, &bus_motor_data_[bus_name]);
            bus_controllers_[bus_name] = controller;
            std::cout << "[DamiaoHW] Initialized bus: " << bus_name << " with " << configs.size() << " motors"
                    << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "[DamiaoHW] Failed to initialize bus " << bus_name << ": " << e.what() << std::endl;
            return false;
        }
    }

    return true;
}

void DamiaoHW::setThreadPriority(int priority) {
    thread_priority_ = priority;
}

// ========== 数据读写接口实现 ==========

void DamiaoHW::read() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    for (auto& kv : bus_controllers_) {
        kv.second->read();
    }
}

DmActData* DamiaoHW::getMotorData(const std::string& bus_name, uint16_t can_id) {
    auto bus_it = bus_motor_data_.find(bus_name);
    if (bus_it == bus_motor_data_.end()) return nullptr;
    auto motor_it = bus_it->second.find(can_id);
    if (motor_it == bus_it->second.end()) return nullptr;
    return &motor_it->second;
}

bool DamiaoHW::getMotorState(const std::string& bus_name, uint16_t can_id,
        float* position, float* velocity, float* torque, float* temperature, uint32_t* error) {
    if (!position || !velocity || !torque || !temperature || !error) return false;
    std::lock_guard<std::mutex> lock(state_mutex_);
    auto bus_it = bus_motor_data_.find(bus_name);
    if (bus_it == bus_motor_data_.end()) return false;
    auto motor_it = bus_it->second.find(can_id);
    if (motor_it == bus_it->second.end()) return false;
    if (motor_it->second.feedback_sequence == 0 ||
        motor_it->second.feedback_sequence == motor_it->second.consumed_sequence) {
        return false;
    }
    *position = static_cast<float>(motor_it->second.pos);
    *velocity = static_cast<float>(motor_it->second.vel);
    *torque = static_cast<float>(motor_it->second.effort);
    *temperature = static_cast<float>(motor_it->second.temperature);
    *error = motor_it->second.error;
    motor_it->second.consumed_sequence = motor_it->second.feedback_sequence;
    return true;
}

bool DamiaoHW::validateMitCommand(const std::string& bus_name, uint16_t can_id,
        float position, float velocity, float torque, float kp, float kd) {
    auto result = findMotor(bus_name, can_id);
    if (!result.second) return false;
    uint8_t data[8] = {};
    return EncodeMitCommand(
        result.second->get_limit_param(), position, velocity, torque, kp, kd, data);
}

// ========== 辅助函数实现 ==========

std::pair<Motor_Control*, Motor*> DamiaoHW::findMotor(const std::string& bus_name, uint16_t can_id) {
    auto bus_it = bus_controllers_.find(bus_name);
    if (bus_it == bus_controllers_.end()) {
        std::cerr << "[DamiaoHW] Bus not found: " << bus_name << std::endl;
        return std::make_pair(nullptr, nullptr);
    }

    Motor_Control* controller = bus_it->second.get();
    const auto& motors = controller->get_motors();

    auto motor_it = motors.find(can_id);
    if (motor_it == motors.end()) {
        std::cerr << "[DamiaoHW] Motor not found: " << can_id << " on bus " << bus_name << std::endl;
        return std::make_pair(nullptr, nullptr);
    }

    return std::make_pair(controller, motor_it->second.get());
}

// ========== 模式管理接口实现 ==========

bool DamiaoHW::switchMode(const std::string& bus_name, uint16_t can_id, Control_Mode_Code mode) {
    auto result = findMotor(bus_name, can_id);
    if (!result.first || !result.second) return false;

    bool success = result.first->switchControlMode(*result.second, mode);
    if (success) {
        std::cout << "[DamiaoHW] Switched motor " << can_id << " on " << bus_name << " to mode "
                    << static_cast<int>(mode) << std::endl;
    }
    return success;
}

Control_Mode DamiaoHW::getCurrentMode(const std::string& bus_name, uint16_t can_id) {
    auto result = findMotor(bus_name, can_id);
    if (!result.second) return MIT_MODE;
    return result.second->GetMotorMode();
}

// ========== 模式化控制接口实现 ==========

bool DamiaoHW::controlMit(const std::string& bus_name, uint16_t can_id, float pos,
        float vel, float torque, float kp, float kd) {
    auto result = findMotor(bus_name, can_id);
    if (!result.first || !result.second) return false;
    return result.first->control_mit(*result.second, kp, kd, pos, vel, torque);
}

bool DamiaoHW::controlPosVel(const std::string& bus_name, uint16_t can_id, float pos, float vel) {
    auto result = findMotor(bus_name, can_id);
    if (!result.first || !result.second) return false;
    return result.first->control_pos_vel(*result.second, pos, vel);
}

bool DamiaoHW::controlVel(const std::string& bus_name, uint16_t can_id, float vel) {
    auto result = findMotor(bus_name, can_id);
    if (!result.first || !result.second) return false;
    return result.first->control_vel(*result.second, vel);
}

bool DamiaoHW::controlPosForce(const std::string& bus_name, uint16_t can_id, float pos,
        float vel_limit, float current_limit) {
    auto result = findMotor(bus_name, can_id);
    if (!result.first || !result.second) return false;
    return result.first->control_pos_force(*result.second, pos, vel_limit, current_limit);
}

// ========== 参数调节接口实现 ==========

float DamiaoHW::readParam(const std::string& bus_name, uint16_t can_id, uint8_t reg_id) {
    auto result = findMotor(bus_name, can_id);
    if (!result.first || !result.second) return 0.0f;
    return result.first->read_motor_param(*result.second, reg_id);
}

bool DamiaoHW::writeParam(const std::string& bus_name, uint16_t can_id, uint8_t reg_id, float value) {
    auto result = findMotor(bus_name, can_id);
    if (!result.first || !result.second) return false;
    return result.first->change_motor_param(*result.second, reg_id, value);
}

void DamiaoHW::saveParam(const std::string& bus_name, uint16_t can_id) {
    auto result = findMotor(bus_name, can_id);
    if (!result.first || !result.second) return;
    result.first->save_motor_param(*result.second);
    std::cout << "[DamiaoHW] Saved parameters for motor " << can_id << " on " << bus_name << std::endl;
}

float DamiaoHW::getCachedParam(const std::string& bus_name, uint16_t can_id, uint8_t reg_id) {
    auto result = findMotor(bus_name, can_id);
    if (!result.second) return 0.0f;
    if (result.second->is_have_param(reg_id)) {
        return result.second->get_param_as_float(reg_id);
    }
    return 0.0f;
}

// ========== 电机控制接口实现 ==========

void DamiaoHW::enableAll() {
    for (auto& kv : bus_controllers_) {
        kv.second->enable_all();
    }
    std::cout << "[DamiaoHW] All motors enabled" << std::endl;
}

void DamiaoHW::disableAll() {
    for (auto& kv : bus_controllers_) {
        kv.second->disable_all();
    }
    std::cout << "[DamiaoHW] All motors disabled" << std::endl;
}

bool DamiaoHW::enable(const std::string& bus_name, uint16_t can_id) {
    auto result = findMotor(bus_name, can_id);
    if (!result.first || !result.second) return false;
    // CAN 控制器负责总线级重传，实时命令路径不做阻塞式应用层重发。
    uint16_t frame_id = result.second->GetCanId() + result.second->GetMotorMode();
    if (!result.first->send_control_cmd(frame_id, 0xFC)) return false;
    std::cout << "[DamiaoHW] Enabled motor " << can_id << " on " << bus_name << std::endl;
    return true;
}

bool DamiaoHW::disable(const std::string& bus_name, uint16_t can_id) {
    auto result = findMotor(bus_name, can_id);
    if (!result.first || !result.second) return false;
    // 与 Motor_Control::disable_all 一致：cmd 0xFD
    uint16_t frame_id = result.second->GetCanId() + result.second->GetMotorMode();
    bool success = true;
    for (int i = 0; i < 5; i++) {
        success = result.first->send_control_cmd(frame_id, 0xFD) && success;
        usleep(2000);
    }
    if (!success) return false;
    std::cout << "[DamiaoHW] Disabled motor " << can_id << " on " << bus_name << std::endl;
    return true;
}

bool DamiaoHW::disableOnce(const std::string& bus_name, uint16_t can_id) {
    auto result = findMotor(bus_name, can_id);
    if (!result.first || !result.second) return false;
    const uint16_t frame_id = result.second->GetCanId() + result.second->GetMotorMode();
    return result.first->send_control_cmd(frame_id, 0xFD);
}

bool DamiaoHW::disableAllModes(const std::string& bus_name, uint16_t can_id) {
    static const Control_Mode kModes[] = {
        MIT_MODE,
        POS_VEL_MODE,
        VEL_MODE,
        POS_FORCE_MODE,
    };
    auto result = findMotor(bus_name, can_id);
    bool success = true;

    if (!result.first || !result.second) return false;
    for (Control_Mode mode : kModes) {
        for (int attempt = 0; attempt < 2; ++attempt) {
            success = result.first->send_control_cmd(can_id + mode, 0xFD) && success;
            usleep(2000);
        }
    }
    return success;
}

void DamiaoHW::setZeroPosition(const std::string& bus_name, uint16_t can_id) {
    auto result = findMotor(bus_name, can_id);
    if (!result.first || !result.second) return;
    result.first->set_zero_position(*result.second);
    std::cout << "[DamiaoHW] Set zero position for motor " << can_id << " on " << bus_name << std::endl;
}

// ========== 自动读取线程 ==========

void DamiaoHW::startAutoRead(uint32_t period_us) {
    if (read_running_ || period_us == 0) return;
    read_running_ = true;
    read_thread_ = std::thread([this, period_us]() {
        const auto period = std::chrono::microseconds(period_us);
        auto next_read = std::chrono::steady_clock::now();
        while (read_running_) {
            this->read();
            next_read += period;
            const auto now = std::chrono::steady_clock::now();
            if (next_read < now - period) next_read = now;
            std::this_thread::sleep_until(next_read);
        }
    });
}

void DamiaoHW::stopAutoRead() {
    read_running_ = false;
    if (read_thread_.joinable()) {
        read_thread_.join();
    }
}

}  // namespace damiao
