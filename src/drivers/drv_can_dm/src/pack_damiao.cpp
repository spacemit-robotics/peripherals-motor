/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#include "pack_damiao.h"
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

// 原子标志，用于安全地跨线程修改
std::atomic<bool> running(true);
// 全局硬件接口指针，用于信号处理函数访问
std::shared_ptr<damiao::DmHW> g_motor_hw = nullptr;

/**
 * @brief 4. 电机失能/停止函数
 */
void disable_motors(std::shared_ptr<damiao::DmHW> &hw) {
    if (hw) {
        std::cout << "Disabling motors..." << std::endl;
        // 1. 平稳停止
        hw->emergencyStop();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        /*注意：先失能电机，再保存当前位置为 0，防止电机在回零时跳转*/

        // 2. 显式失能 (发送 0xFD)
        hw->disable();

        // 在测试阶段需要失能，为避免频繁的寄存器操作，此处注释，后续根据需要启用

        // 3. 将当前位置设为零点 (软件层面标定) ****************************
        // 注意：这会将电机当前的机械位置标记为0弧度，以后在该位置上电就是0
        // std::cout << "Setting current position as zero..." << std::endl;
        // hw->setZeroPosition();
        // std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
}

// 全局待初始化配置
static std::vector<damiao::MotorConfig> g_pending_configs;
static bool g_hw_initialized = false;

// C-compatible API impl
extern "C" {

void dm_driver_add_config(const char *bus_name, uint16_t can_id,
        uint16_t motor_type) {
    damiao::MotorConfig config;
    config.bus_name = bus_name;
    config.can_id = can_id;
    config.motor_type =
        (damiao::DM_Motor_Type)(motor_type);  // 简单 cast，注意枚举匹配
    // 关键修正：根据日志推断回复 ID 是发送 ID + 0x10
    config.master_id = can_id + 0x10;
    config.control_mode = damiao::MIT_MODE;
    g_pending_configs.push_back(config);
}

int dm_driver_init_global() {
    if (g_hw_initialized)
        return 0;
    if (!init_motors(g_motor_hw, g_pending_configs)) {
        return -1;
    }
    // 初始化完成后启动自动读取线程，周期5ms刷新总线数据
    if (g_motor_hw) {
        std::printf("\n**********************************************\n");
        g_motor_hw->startAutoRead(5);
    }

    g_hw_initialized = true;
    return 0;
}

void dm_driver_send_cmd(const char *bus_name, uint16_t can_id, float p, float v,
                        float t, float kp, float kd) {
    if (!g_motor_hw)
        return;
    // 使用新增加的接口 sendMitCommand
    g_motor_hw->sendMitCommand(bus_name, can_id, p, v, t, kp, kd);
}

int dm_driver_get_state(const char *bus_name, uint16_t can_id, float *pos,
                        float *vel, float *trq) {
    if (!g_motor_hw)
        return -1;
    // ⚠️ 性能优化：不在此处每次调用都触发 read，总线数据应由外部周期性刷新
    // 假设 g_motor_hw->getActuatorData() 的内容已经由外部循环或线程定时更新为最新数据
    // 可根据需要在此添加条件触发 read，例如传入一个标志位控制是否强制刷新

    const auto &data = g_motor_hw->getActuatorData();
    // 手动查找 bus_name string key
    std::string bname(bus_name);
    if (data.count(bname)) {
        const auto &motors = data.at(bname);
        if (motors.count(can_id)) {
            const auto &act = motors.at(can_id);
            *pos = act.pos;
            *vel = act.vel;
            *trq = act.effort;
            return 0;
        }
    }
    return -1;
}

}  // extern "C"

// Ctrl+C 触发的信号处理函数
void signalHandler(int signum) {
    running = false;
    std::cout << "\nInterrupt signal (" << signum << ") received." << std::endl;
    disable_motors(g_motor_hw);
    std::cout << "Program will exit safely." << std::endl;
}

/**
 * @brief 1. 初始化函数：总线设备初始化+电机使能
 */
bool init_motors(std::shared_ptr<damiao::DmHW> &hw,
        const std::vector<damiao::MotorConfig> &configs) {
    // 注册信号处理函数
    signal(SIGINT, signalHandler);

    if (!hw) {
        hw = std::make_shared<damiao::DmHW>();
    }

    // 设置线程优先级
    hw->setCanBusThreadPriority(95);

    // 初始化总线和电机
    if (!hw->init(configs)) {
        return false;
    }

    return true;
}

/**
 * @brief 2a. 轨迹规划指令发送函数：电机运动参数配置+控制指令发送
 */
void send_trajectory_command(std::shared_ptr<damiao::DmHW> &hw,
        const damiao::MotionParams &params) {
    if (!hw)
        return;

    // 配置运动参数（轨迹规划：圈数、时间、加速度）
    // 注意：在实际高频循环中，如果参数不变，最好不要每次都调用setMotionParams，
    // 但此处为了满足"封装在一个函数中"的要求，每次都调用。
    // 实际该函数使用
    hw->setMotionParams(params);

    // 发送控制指令 (write调用会发送CAN帧)
    // 这里的周期设为0.002s (500Hz)
    hw->write(std::chrono::system_clock::now(),
            std::chrono::duration<double>(0.002));
}

/**
 * @brief 2b. MIT实时控制指令发送函数：直接发送位置/速度/力矩指令
 */
void send_mit_command(std::shared_ptr<damiao::DmHW> &hw,
        const char *bus_name, uint16_t can_id, float pos, float vel,
        float torque, float kp, float kd) {
    if (!hw)
        return;

    // 直接发送MIT控制指令（实时控制：Pdes、Vdes、Tdes）
    hw->sendMitCommand(bus_name, can_id, pos, vel, torque, kp, kd);
}

/**
 * @brief 3. 状态反馈函数：电机状态读取
 */
void get_feedback(std::shared_ptr<damiao::DmHW> &hw) {
    if (!hw)
        return;

    // 从总线读取状态
    hw->read(std::chrono::system_clock::now(),
            std::chrono::duration<double>(0.002));

    // 获取并打印状态
    const auto &data = hw->getActuatorData();

    // 简单的打印逻辑，避免刷屏太快可以加控制，或者这里只负责获取
    // 为了演示，这里打印第一个电机的状态
    static int print_count = 0;
    if (print_count++ % 50 == 0) {  // 每50次调用打印一次 (约10Hz)
        std::cout << "--- Motor Feedback ---" << std::endl;
        for (const auto &bus_pair : data) {
            for (const auto &motor_pair : bus_pair.second) {
                const auto &act = motor_pair.second;
                std::cout << "Bus: " << bus_pair.first
                        << " ID: " << act.can_id
                        << " Pos: " << act.pos
                        << " Vel: " << act.vel
                        << " Torque: " << act.effort << std::endl;
            }
        }
    }
}
