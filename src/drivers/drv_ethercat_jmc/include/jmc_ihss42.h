/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file jmc_ihss42.h
 * @brief JMC IHSS42-EC 步进电机驱动配置
 *
 * 基于 JMC_DRIVE_V2.51 XML 配置
 */

#ifndef JMC_IHSS42_H
#define JMC_IHSS42_H

#include "ethercat_cia402.h"

#ifdef __cplusplus
extern "C" {
#endif

/****************************************************************************/
/* JMC IHSS42 设备信息 */
/****************************************************************************/

#define JMC_VENDOR_ID 0x66668888
#define JMC_IHSS42_PRODUCT_CODE 0x20230321

/****************************************************************************/
/* 函数声明 */
/****************************************************************************/

/**
 * @brief 获取 JMC IHSS42 从站驱动
 * @return 从站驱动指针
 */
const ec_slave_driver_t* jmc_ihss42_get_driver(void);

/**
 * @brief 配置 JMC IHSS42 从站
 * @param ctx 主站上下文指针
 * @param motor_idx 电机索引
 * @param alias 从站别名
 * @param position 从站位置
 * @param mode 运行模式
 * @return 0 成功, -1 失败
 */
int jmc_ihss42_configure(ec_master_ctx_t* ctx, int motor_idx, uint16_t alias, uint16_t position, ec_app_mode_t mode);

/**
 * @brief 注册 JMC IHSS42 PDO 条目到域
 * @param ctx 主站上下文指针
 * @param motor_idx 电机索引
 * @return 0 成功, -1 失败
 */
int jmc_ihss42_register_pdo(ec_master_ctx_t* ctx, int motor_idx);

/**
 * @brief 执行 JMC IHSS42 电机周期任务
 * @param ctx 主站上下文指针
 * @param motor_idx 电机索引
 */
void jmc_ihss42_cyclic_task(ec_master_ctx_t* ctx, int motor_idx);

/**
 * @brief 设置 JMC IHSS42 Profile 参数
 * @param ctx 主站上下文指针
 * @param motor_idx 电机索引
 * @param profile_vel 轮廓速度 (pulse/s)
 * @param profile_acc 加速度 (pulse/s^2)
 * @param profile_dec 减速度 (pulse/s^2)
 */
void jmc_ihss42_set_profile_params(
    ec_master_ctx_t* ctx, int motor_idx, uint32_t profile_vel, uint32_t profile_acc, uint32_t profile_dec);

#ifdef __cplusplus
}
#endif

#endif /* JMC_IHSS42_H */
