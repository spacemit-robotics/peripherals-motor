/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file damiao_pack.h
 * @brief Damiao driver adapter declarations for the generic motor API.
 */

#ifndef DAMIAO_PACK_H
#define DAMIAO_PACK_H

#include <stdbool.h>
#include <stdint.h>
#include <memory>
#include <vector>

#include "damiao_hw.h"

struct damiao_protocol_limits {
    float position_min;
    float position_max;
    float velocity_min;
    float velocity_max;
    float torque_min;
    float torque_max;
    float kp_min;
    float kp_max;
    float kd_min;
    float kd_max;
};

struct motor_can_dm_config {
    uint16_t feedback_id;
    uint16_t motor_type;
    uint32_t feedback_period_us;
    uint32_t can_timeout_ms;
    bool use_custom_limits;
    bool enable_on_init;
    struct damiao_protocol_limits limits;
};

// 全局变量声明
extern std::shared_ptr<damiao::DamiaoHW> g_damiao_hw;
#ifdef __cplusplus
extern "C" {
#endif

/*
 * motor_mode 到达妙模式的映射关系：
 *   MOTOR_MODE_HYBRID   -> MIT_MODE       (0x000)
 *   MOTOR_MODE_POS      -> POS_VEL_MODE   (0x100)
 *   MOTOR_MODE_VEL      -> VEL_MODE       (0x200)
 *   MOTOR_MODE_TRQ      -> MIT_MODE       (0x000, kp=kd=pos=vel=0)
 *   MOTOR_MODE_IDLE     -> 失能
 */

/* 1. 初始化：收集配置 + 全局初始化 */
int damiao_add_config(const char* bus_name, uint16_t can_id,
        const struct motor_can_dm_config* config);
void damiao_remove_config(const char* bus_name, uint16_t can_id);
int damiao_init_global(void);

/* 2. 发送指令：根据 motor_cmd.mode 自动分发到对应模式 */
int damiao_set_cmd(const char* bus_name, uint16_t can_id, uint32_t mode, float pos, float vel, float trq, float kp,
                    float kd);
int damiao_validate_mit_cmd(const char* bus_name, uint16_t can_id,
        float pos, float vel, float trq, float kp, float kd);
/* Returns 1 when the hardware mode changed, 0 when already selected, -1 on error. */
int damiao_prepare_mode(const char* bus_name, uint16_t can_id, uint32_t mode);

/* 3. 获取状态 */
int damiao_get_state(const char* bus_name, uint16_t can_id, float* pos, float* vel, float* trq,
        float* temperature, uint32_t* error);

/* 4. 释放电机 */
void damiao_release(const char* bus_name, uint16_t can_id);
void damiao_release_all(void);
int damiao_enable(const char* bus_name, uint16_t can_id);
int damiao_disable(const char* bus_name, uint16_t can_id);
int damiao_disable_once(const char* bus_name, uint16_t can_id);

/* 5. 获取参数 */
int damiao_get_param(const char* bus_name, uint16_t can_id, uint8_t reg_id, float* out_value);

/* 6. 调节参数 */
int damiao_set_param(const char* bus_name, uint16_t can_id, uint8_t reg_id, float value);
void damiao_save_param(const char* bus_name, uint16_t can_id);

#ifdef __cplusplus
}
#endif

#endif /* DAMIAO_PACK_H */
