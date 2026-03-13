/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef PACK_DAMIAO_H
#define PACK_DAMIAO_H

#pragma once

#include <atomic>
#include <memory>
#include <vector>

#include "DmHW.h"

// 全局变量声明
extern std::shared_ptr<damiao::DmHW> g_motor_hw;
extern std::atomic<bool> running;

/**
 * @brief 1. 初始化函数：总线设备初始化+电机使能
 * @param hw 硬件接口指针引用
 * @param configs 电机配置列表
 * @return true 成功, false 失败
 */
bool init_motors(std::shared_ptr<damiao::DmHW>& hw, const std::vector<damiao::MotorConfig>& configs);

/**
 * @brief 2a. 轨迹规划指令发送函数：电机运动参数配置+控制指令发送
 * @param hw 硬件接口指针
 * @param params 运动参数（圈数、时间、加速度）
 */
void send_trajectory_command(std::shared_ptr<damiao::DmHW>& hw, const damiao::MotionParams& params);

/**
 * @brief 2b. MIT实时控制指令发送函数：直接发送位置/速度/力矩指令
 * @param hw 硬件接口指针
 * @param bus_name CAN总线名称
 * @param can_id 电机CAN ID
 * @param pos 期望位置 (rad)
 * @param vel 期望速度 (rad/s)
 * @param torque 期望力矩 (Nm)
 * @param kp 位置刚度增益
 * @param kd 阻尼增益
 */
void send_mit_command(std::shared_ptr<damiao::DmHW>& hw, const char* bus_name, uint16_t can_id, float pos, float vel,
    float torque, float kp, float kd);

/**
 * @brief 3. 状态反馈函数：电机状态读取
 * @param hw 硬件接口指针
 */
void get_feedback(std::shared_ptr<damiao::DmHW>& hw);

/**
 * @brief 4. 电机失能/停止函数
 * @param hw 硬件接口指针
 */
void disable_motors(std::shared_ptr<damiao::DmHW>& hw);

// C-compatible API
#ifdef __cplusplus
extern "C" {
#endif

// Opaque handle
typedef void* dm_hw_handle_t;

// Init function that takes simple C-style arrays/structs if needed,
// or simplified init for integration.
// 鉴于 motor_framework 是一一个 dev 初始化，我们创建一个全局配置收集函数
void dm_driver_add_config(const char* bus_name, uint16_t can_id, uint16_t motor_type);

// Initialize the global DmHW instance with collected configs
int dm_driver_init_global();

// Send command wrapper
// p: pos, v: vel, t: torque, kp, kd
void dm_driver_send_cmd(const char* bus_name, uint16_t can_id, float p, float v, float t, float kp, float kd);

// Get state wrapper
int dm_driver_get_state(const char* bus_name, uint16_t can_id, float* pos, float* vel, float* trq);

#ifdef __cplusplus
}
#endif

#endif
