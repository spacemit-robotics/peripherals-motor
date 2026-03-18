/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DAMIAO_PACK_H
#define DAMIAO_PACK_H

#pragma once

#include <atomic>
#include <memory>
#include <vector>

#include "damiao_hw.h"

// 全局变量声明
extern std::shared_ptr<damiao::DamiaoHW> g_damiao_hw;
extern std::atomic<bool> g_damiao_running;

#ifdef __cplusplus
extern "C" {
#endif

/*
 * motor_mode 到达妙模式的映射关系：
 *   MOTOR_MODE_HYBRID   -> MIT_MODE       (0x000)
 *   MOTOR_MODE_POS      -> POS_VEL_MODE   (0x100)
 *   MOTOR_MODE_VEL      -> VEL_MODE       (0x200)
 *   MOTOR_MODE_TRQ      -> POS_FORCE_MODE (0x300)
 *   MOTOR_MODE_IDLE     -> 失能
 */

/* 1. 初始化：收集配置 + 全局初始化 */
void damiao_add_config(const char* bus_name, uint16_t can_id, uint16_t motor_type);
int damiao_init_global(void);

/* 2. 发送指令：根据 motor_cmd.mode 自动分发到对应模式 */
int damiao_set_cmd(const char* bus_name, uint16_t can_id, uint32_t mode, float pos, float vel, float trq, float kp,
                    float kd);

/* 3. 获取状态 */
int damiao_get_state(const char* bus_name, uint16_t can_id, float* pos, float* vel, float* trq);

/* 4. 释放电机 */
void damiao_release(const char* bus_name, uint16_t can_id);
void damiao_release_all(void);

/* 5. 获取参数 */
int damiao_get_param(const char* bus_name, uint16_t can_id, uint8_t reg_id, float* out_value);

/* 6. 调节参数 */
int damiao_set_param(const char* bus_name, uint16_t can_id, uint8_t reg_id, float value);
void damiao_save_param(const char* bus_name, uint16_t can_id);

#ifdef __cplusplus
}
#endif

#endif /* DAMIAO_PACK_H */
