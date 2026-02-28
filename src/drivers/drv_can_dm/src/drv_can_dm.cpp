/*
 * Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * dm_hw 驱动适配层 (drv_can_dm.cpp)
 */




// C system headers
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

// C++ system headers
#include <chrono>

// Project headers
extern "C" {
#include "../../../../include/motor.h"
#include "../../../motor_core.h"
}

#include "pack_damiao.h"

// 临时结构体，用于保存每个电机的配置信息
struct dm_priv
{
    // 假设配置信息在 probe 阶段就确定了
    char bus_name[32];
    uint16_t can_id;
    // 控制指令发送时间戳
    struct timespec cmd_send_time;  // 最近一次控制指令发送时间戳
    bool initialized;
    bool cmd_sent;  // 是否已发送过控制指令
                    // 其它配置参数
};

// 初始化
static int dm_init(struct motor_dev *dev)
{
    struct dm_priv *priv = (struct dm_priv *)dev->priv_data;

    printf("Motor %s (CAN ID: 0x%02X) initializing...\n", dev->name, priv->can_id);

    // 在这里触发一次全局初始化
    // 注意：dm_driver_init_global 应该只运行一次，内部需要处理多次调用的情况
    if (dm_driver_init_global() != 0)
    {
        // 如果失败，可能是硬件问题或者配置问题
        // 这里只是单个电机 init 失败
        // 如果全局 DmHW init 失败，整个驱动都不可用
        return -1;
    }

    priv->initialized = true;
    priv->cmd_sent = false;
    return 0;
}

// 发送命令
static int dm_set_cmd(struct motor_dev *dev, const struct motor_cmd *cmd)
{
    struct dm_priv *priv = (struct dm_priv *)dev->priv_data;

    // 记录控制指令发送时间戳
    clock_gettime(CLOCK_MONOTONIC, &priv->cmd_send_time);

    // 如果是第一次发送指令，打印时间戳
    if (!priv->cmd_sent)
    {
        printf("Motor %s (CAN ID: 0x%02X) first command sent at timestamp: "
                "%ld.%09ld\n",
                dev->name, priv->can_id,
                priv->cmd_send_time.tv_sec,
                priv->cmd_send_time.tv_nsec);
        priv->cmd_sent = true;
    }

    // 转换 motor_cmd 到 damiao 的 MIT 命令
    // motor_cmd 定义在 motor.h: pos_des, vel_des, trq_des, kp, kd
    // damiao MIT: p, v, t, kp, kd

    // 注意单位和符号，假设一致 (rad, rad/s, Nm)
    // cmd->mode 忽略 初始化时默认了 MIT。
    // 模式切换需要写 flash, 在高速模式下避免模式切换

    /*
     * 1. MIT:
            (1) Kp = 0, Kd != 0, Vdes = a(定值)，匀速转动
            (2) Kp = 0, Kd = 0, t_ff = a(定值)，给定扭矩输出
            注：位置控制时 Kd 不能给 0，否则会造成电机振荡或失控
            Kp [0,500]; Kd [0,5]
     * 2. p_v:
            (1) 指定目标位置，最大速度，加减速度
            (2) Kd > 0
     * 3. v:
            (1) 指定目标速度，最大加减速度
            (2) Kp > 0, 最佳范围[2.0，10.0]，推荐值 4
     * 4. f_p:
            在 p_v 模式的基础上动态控制输出扭矩的大小
            参数：p,v,I(扭矩电流)
     */

    // 如果需要支持多种模式，需要在 set_cmd 里动态切换，但 DmHW
    // 切换模式较慢（要写参数） 且实时性要求高，暂时只支持 MIT。

    dm_driver_send_cmd(priv->bus_name, priv->can_id,
                        cmd->pos_des, cmd->vel_des,
                        cmd->trq_des, cmd->kp, cmd->kd);

    return 0;
}

// 获取状态
static int dm_get_state(struct motor_dev *dev, struct motor_state *state)
{
    struct dm_priv *priv = (struct dm_priv *)dev->priv_data;

    float pos, vel, trq;
    if (dm_driver_get_state(priv->bus_name, priv->can_id, &pos, &vel, &trq) == 0)
    {
        state->pos = pos;
        state->vel = vel;
        state->trq = trq;
        state->temp = 0.0f;  // Mock
        state->err = 0;
        return 0;
    }
    return -1;
}

// 释放
static void dm_release(struct motor_dev *dev)
{
    // 单个设备释放时，如果全局硬件接口已初始化，则执行电机失能并标定当前位置为零点。
    if (g_motor_hw)
    {
        disable_motors(g_motor_hw);
    }
    if (dev->priv_data)
    {
        free(dev->priv_data);
    }
    free(dev);
}

// 定义 ops
// 必须在 probe 之前定义
static const struct motor_ops dm_ops = {
    .init = dm_init,
    .set_cmd = dm_set_cmd,
    .get_state = dm_get_state,
    .free = dm_release,
};

// 探测函数
static struct motor_dev *dm_probe(void *args)
{
    struct motor_args_can *params = (struct motor_args_can *)args;

    // 分配设备结构体和私有数据
    struct motor_dev *dev = (struct motor_dev *)calloc(1, sizeof(struct motor_dev));
    if (!dev)
    {
        return NULL;
    }

    struct dm_priv *priv = (struct dm_priv *)calloc(1, sizeof(struct dm_priv));
    if (!priv)
    {
        free(dev);
        return NULL;
    }

    dev->name = "dm_can";
    dev->ops = &dm_ops;
    dev->priv_data = priv;

    // 初始化私有数据结构
    priv->initialized = false;
    priv->cmd_sent = false;
    priv->can_id = 0;
    memset(priv->bus_name, 0, sizeof(priv->bus_name));
    priv->cmd_send_time.tv_sec = 0;
    priv->cmd_send_time.tv_nsec = 0;

    // 保存配置
    if (params->iface) {
        snprintf(priv->bus_name, sizeof(priv->bus_name), "%s", params->iface);
    } else {
        snprintf(priv->bus_name, sizeof(priv->bus_name), "can0");
    }
    priv->can_id = params->can_id;

    // 将此电机的配置添加到全局 DmHW 待初始化列表
    // 假设默认使用 DM4310, MIT mode (0)
    // 这里的 0 是 DmHW 枚举 DM4310 的值
    dm_driver_add_config(priv->bus_name, priv->can_id, 0);

    return dev;
}

// 注册驱动
// MOTOR_DRIVER_REGISTER 展开后会自动生成构造函数用于注册
REGISTER_MOTOR_DRIVER("dm_can", DRV_TYPE_CAN, dm_probe);

// 时间戳
extern "C" int dm_get_cmd_timestamp(struct motor_dev *dev, struct timespec *timestamp)
{
    if (!dev || !dev->priv_data || !timestamp)
    {
        return -1;
    }

    struct dm_priv *priv = (struct dm_priv *)dev->priv_data;
    if (!priv->initialized || !priv->cmd_sent)
    {
        return -1;  // 电机尚未初始化或未发送过控制指令
    }

    *timestamp = priv->cmd_send_time;
    return 0;
}

extern "C" void dm_print_cmd_info(struct motor_dev *dev)
{
    if (!dev || !dev->priv_data)
    {
        printf("Invalid motor device\n");
        return;
    }

    struct dm_priv *priv = (struct dm_priv *)dev->priv_data;
    if (!priv->initialized)
    {
        printf("Motor %s not initialized yet\n", dev->name);
        return;
    }

    if (!priv->cmd_sent)
    {
        printf("Motor %s (CAN ID: 0x%02X, Bus: %s): No command sent yet\n",
                dev->name, priv->can_id, priv->bus_name);
        return;
    }

    printf("Motor %s (CAN ID: 0x%02X, Bus: %s):\n",
            dev->name, priv->can_id, priv->bus_name);
    printf("  Last command timestamp: %ld.%09ld seconds\n",
            priv->cmd_send_time.tv_sec,
            priv->cmd_send_time.tv_nsec);
}

