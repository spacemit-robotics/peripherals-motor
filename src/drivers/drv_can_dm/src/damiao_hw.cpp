#include <unistd.h>

#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "damiao_hw.h"

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
                                                                .cmd_pos = 0,
                                                                .cmd_vel = 0,
                                                                .cmd_effort = 0,
                                                                .kp = 0,
                                                                .kd = 0};
        }        try {
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

void DamiaoHW::controlMit(const std::string& bus_name, uint16_t can_id, float pos, float vel, float torque, float kp,
                            float kd) {
    auto result = findMotor(bus_name, can_id);
    if (!result.first || !result.second) return;
    result.first->control_mit(*result.second, kp, kd, pos, vel, torque);
}

void DamiaoHW::controlPosVel(const std::string& bus_name, uint16_t can_id, float pos, float vel) {
    auto result = findMotor(bus_name, can_id);
    if (!result.first || !result.second) return;
    result.first->control_pos_vel(*result.second, pos, vel);
}

void DamiaoHW::controlVel(const std::string& bus_name, uint16_t can_id, float vel) {
    auto result = findMotor(bus_name, can_id);
    if (!result.first || !result.second) return;
    result.first->control_vel(*result.second, vel);
}

void DamiaoHW::controlPosForce(const std::string& bus_name, uint16_t can_id, float pos, float vel_limit,
                                float current_limit) {
    auto result = findMotor(bus_name, can_id);
    if (!result.first || !result.second) return;
    result.first->control_pos_force(*result.second, pos, vel_limit, current_limit);
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

void DamiaoHW::enable(const std::string& bus_name, uint16_t can_id) {
    auto result = findMotor(bus_name, can_id);
    if (!result.first || !result.second) return;
    // 与 Motor_Control::enable_all 一致：cmd 0xFC，id 为 can_id + 当前模式偏移，重发多次确保生效
    uint16_t frame_id = result.second->GetCanId() + result.second->GetMotorMode();
    for (int i = 0; i < 5; i++) {
        result.first->send_control_cmd(frame_id, 0xFC);
        usleep(2000);
    }
    std::cout << "[DamiaoHW] Enabled motor " << can_id << " on " << bus_name << std::endl;
}

void DamiaoHW::disable(const std::string& bus_name, uint16_t can_id) {
    auto result = findMotor(bus_name, can_id);
    if (!result.first || !result.second) return;
    // 与 Motor_Control::disable_all 一致：cmd 0xFD
    uint16_t frame_id = result.second->GetCanId() + result.second->GetMotorMode();
    for (int i = 0; i < 5; i++) {
        result.first->send_control_cmd(frame_id, 0xFD);
        usleep(2000);
    }
    std::cout << "[DamiaoHW] Disabled motor " << can_id << " on " << bus_name << std::endl;
}

void DamiaoHW::setZeroPosition(const std::string& bus_name, uint16_t can_id) {
    auto result = findMotor(bus_name, can_id);
    if (!result.first || !result.second) return;
    result.first->set_zero_position(*result.second);
    std::cout << "[DamiaoHW] Set zero position for motor " << can_id << " on " << bus_name << std::endl;
}

// ========== 自动读取线程 ==========

void DamiaoHW::startAutoRead(unsigned int period_ms) {
    if (read_running_) return;
    read_running_ = true;
    read_thread_ = std::thread([this, period_ms]() {
        while (read_running_) {
            this->read();
            std::this_thread::sleep_for(std::chrono::milliseconds(period_ms));
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
