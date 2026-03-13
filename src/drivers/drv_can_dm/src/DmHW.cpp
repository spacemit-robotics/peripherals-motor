#include "DmHW.h"

#include <iostream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace damiao {
bool DmHW::init(const std::vector<MotorConfig>& motor_configs) {
    // 按总线分组电机配置
    std::unordered_map<std::string, std::vector<MotorConfig>> bus_motors;
    for (const auto& config : motor_configs) {
        bus_motors[config.bus_name].push_back(config);
    }

    // 为每条总线初始化电机
    for (const auto& bus_motor_pair : bus_motors) {
        const std::string& bus_name = bus_motor_pair.first;
        const std::vector<MotorConfig>& motors = bus_motor_pair.second;

        // 为该总线上的每个电机创建数据结构
        for (const auto& motor : motors) {
            bus_id2dm_data_[bus_name].insert(std::make_pair(motor.can_id, DmActData{.motorType = motor.motor_type,
                                                                                .mode = motor.control_mode,
                                                                                .can_id = motor.can_id,
                                                                                .mst_id = motor.master_id,
                                                                                .pos = 0,
                                                                                .vel = 0,
                                                                                .effort = 0,
                                                                                .cmd_pos = 0,
                                                                                .cmd_vel = 0,
                                                                                .cmd_effort = 0}));
        }

        // 为该总线创建 Motor_Control 对象
        motor_ports_.push_back(std::make_shared<Motor_Control>(bus_name, &bus_id2dm_data_[bus_name]));
    }

    return true;
}

void DmHW::read(const sysclock::time_point& time, const duration& period) {
    // 遍历所有Motor_Control 一个Motor_Control代表一个can网络，其实只有一个can0
    for (auto motor_port : motor_ports_) {
        motor_port->read();
    }
}

// 定义DmHW类的write成员函数，该函数用于在给定的时间和周期内更新机器人的硬件状态
void DmHW::write(const sysclock::time_point& time, const duration& period) {
    // 遍历所有Motor_Control 一个Motor_Control代表一个can网络，其实只有一个can0
    for (auto motor_port : motor_ports_) {
        motor_port->write();
    }
}

void DmHW::setCanBusThreadPriority(int thread_priority) {
    thread_priority_ = thread_priority;
}

void DmHW::setMotionParams(const MotionParams& params) {
    // 为所有 Motor_Control 对象设置运动参数
    for (auto motor_port : motor_ports_) {
        motor_port->setMotionParams(params);
    }
    //   std::cout << "[DmHW] Motion parameters set successfully." << std::endl;
}

void DmHW::sendMitCommand(const std::string& bus_name, uint16_t can_id, float p, float v, float t, float kp, float kd) {
    // 查找对应的总线和电机
    // 注意：bus_id2dm_data_ 只是数据映射，控制要通过 motor_ports_
    // Motor_Control 构造时传入了 bus_name。可惜 motor_ports_ 是
    // vector，没有直接按 name 索引。 但是 Motor_Control 的构造函数里有
    // bus_name... 不过它没存为 public 成员。
    //
    // 幸好 init 时我们是按 bus_name 分组创建 Motor_Control 的。
    // 但是 motor_ports_ 并没有保存 bus_name -> Motor_Control 的映射。
    //
    // 简单的办法：遍历 motor_ports_，看谁管理了这个 can_id。
    // Motor_Control::motors 也是 private 的... 但提供了 get_motors()

    for (auto& motor_port : motor_ports_) {
        // 这里假设 Motor_Control 可以根据 can_id 找到电机
        // Motor_Control::motors 是 map<id, shared_ptr<Motor>>
        const auto& motors = motor_port->get_motors();
        if (motors.count(can_id)) {
            auto motor = motors.at(can_id);
            // 发送 MIT 命令
            motor_port->control_mit(*motor, kp, kd, p, v, t);
            return;
        }
    }
    std::cerr << "[DmHW] Error: Motor " << can_id << " not found on any bus." << std::endl;
}

void DmHW::setZeroPosition() {
    // 为所有 Motor_Control 对象设置零点
    for (auto motor_port : motor_ports_) {
        // 获取该总线上的所有电机并设置零点
        const auto& motors = motor_port->get_motors();
        for (const auto& motor_pair : motors) {
            auto motor = motor_pair.second;
            motor_port->set_zero_position(*motor);
            std::cout << "[DmHW] Set zero position for motor ID: " << motor->GetCanId() << std::endl;
        }
    }
    std::cout << "[DmHW] All motors zero position set successfully." << std::endl;
}

void DmHW::emergencyStop() {
    std::cout << "[DmHW] Emergency stop initiated..." << std::endl;

    // 为所有 Motor_Control 对象执行紧急停止
    for (auto motor_port : motor_ports_) {
        // 获取该总线上的所有电机
        const auto& motors = motor_port->get_motors();
        for (const auto& motor_pair : motors) {
            auto motor = motor_pair.second;
            // 使用当前位置作为目标位置，速度设为0，实现平稳停止
            float current_pos = motor->Get_Position();
            motor_port->control_mit(*motor, 20.0, 4.0, current_pos, 0.0, 0.0);
            std::cout << "[DmHW] Emergency stop for motor ID: " << motor->GetCanId() << " at position: " << current_pos
                        << " rad" << std::endl;
        }
    }

    // 等待电机稳定
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    std::cout << "[DmHW] Emergency stop completed." << std::endl;
}

void DmHW::disable() {
    /* 调试 */
    auto now_start = std::chrono::system_clock::now();
    std::time_t now_c_start = std::chrono::system_clock::to_time_t(now_start);
    std::cout << "[DmHW] Disabling all motors at " << std::ctime(&now_c_start);
    /* 调试 */
    for (auto motor_port : motor_ports_) {
        motor_port->disable_all();
    }
    /* 调试 */
    auto now_end = std::chrono::system_clock::now();
    std::time_t now_c_end = std::chrono::system_clock::to_time_t(now_end);
    std::cout << "[DmHW] All motors disabled at " << std::ctime(&now_c_end);
    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now_end - now_start).count();
    std::cout << "[DmHW] disable() elapsed time: " << elapsed_ms << " ms" << std::endl;
    /* 调试 */
}

const std::unordered_map<std::string, std::unordered_map<uint16_t, damiao::DmActData>>& DmHW::getActuatorData() const {
    return bus_id2dm_data_;
}

// 周期性刷新线程
void DmHW::startAutoRead(unsigned int period_ms) {
    if (read_running_)
        return;
    read_running_ = true;
    read_thread_ = std::thread([this, period_ms]() {
        while (read_running_) {
            this->read(std::chrono::system_clock::now(), std::chrono::duration<double>(period_ms / 1000.0));
            std::this_thread::sleep_for(std::chrono::milliseconds(period_ms));
        }
    });
}

void DmHW::stopAutoRead() {
    read_running_ = false;
    if (read_thread_.joinable()) {
        read_thread_.join();
    }
}

}  // namespace damiao
