/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file feetech_motor_adapter.cpp
 * @brief Feetech 舵机驱动适配层 - 将 feetech 驱动注册到 motor 框架
 *
 * 该文件实现了 motor.h 框架所需的接口，将 FeetechPack 类封装为
 * motor_dev 结构体，并通过 REGISTER_MOTOR_DRIVER 宏自动注册。
 *
 * 模式默认为位置伺服模式 (mode = 0)
 */

#include <cstdlib>
#include <cstring>
#include <iostream>
#include <mutex>
#include <string>
#include <unordered_map>

#include "feetech_pack.h"

extern "C" {
#include "../../../motor_core.h"
}

// ==============================================================================
// 内部数据结构
// ==============================================================================

int mode_flag = 0;  // 模式修改标志位

// 电机私有数据
struct FeetechPrivData {
    uint8_t motor_id;
    uint8_t current_mode;  // 当前模
    FeetechData data;      // 当前状态数据
    FeetechPack* pack;     // 共享的 FeetechPack 实例指针
    bool is_idle;          // 标记是否处于空闲（卸力）状态
};

// ==============================================================================
// 全局变量
// ==============================================================================

static std::unordered_map<std::string, FeetechPack*> g_feetech_packs;  // 按设备路径管理
static std::unordered_map<uint8_t, FeetechPrivData*> g_motor_map;
static std::mutex g_mutex;

// ==============================================================================
// 辅助函数
// ==============================================================================

// 获取或创建 FeetechPack 实例
static FeetechPack* get_or_create_pack(const char* dev_path, uint32_t baud) {
    std::string key(dev_path);

    auto it = g_feetech_packs.find(key);
    if (it != g_feetech_packs.end()) {
        return it->second;
    }

    // 创建新的 FeetechPack 实例
    FeetechPack* pack = new FeetechPack();
    if (!pack) {
        std::cerr << "[Feetech] Failed to create FeetechPack instance!" << std::endl;
        return nullptr;
    }

    // 初始化
    char* argv[2];
    char arg0[] = "feetech";
    char* arg1 = strdup(dev_path);
    argv[0] = arg0;
    argv[1] = arg1;

    if (!pack->init_pack(2, argv)) {
        std::cerr << "[Feetech] Failed to initialize FeetechPack for " << dev_path << std::endl;
        free(arg1);
        delete pack;
        return nullptr;
    }

    free(arg1);
    g_feetech_packs[key] = pack;
    std::cout << "[Feetech] Initialized pack for device: " << dev_path << std::endl;

    return pack;
}

// ==============================================================================
// motor_ops 回调函数实现
// ==============================================================================

static int feetech_init(struct motor_dev* dev) {
    if (!dev || !dev->priv_data) return -1;

    FeetechPrivData* priv = reinterpret_cast<FeetechPrivData*>(dev->priv_data);

    // 初始化数据结构
    memset(&priv->data, 0, sizeof(FeetechData));
    priv->data.id = priv->motor_id;
    priv->current_mode = 0;
    priv->is_idle = false;

    // 默认模式为位置伺服模式，掉电不丢失
    ModeSwitcher switcher(priv->pack->get_sms_sts());
    switcher.switch_mode(priv->data.id, 0);  // 切换到位置伺服模式

    /**************************************************************************************
     *  上层 motor_core init
     * 函数设计不包含控制参数传递，在本函数实际无法获取用户设置的模式，但先保留此模块
     *  模式切换，默认模式为位置伺服模式 (mode = 0)
     *  ModeSwitcher switcher;
     *
     *  switcher.switch_mode(sms_sts, priv->data.id, priv->data.mode);  // ID
     * 切换到指定模式 priv->current_mode = priv->data.mode;
     **************************************************************************************/
    return 0;
}

static int feetech_set_cmd(struct motor_dev* dev, const struct motor_cmd* cmd) {
    if (!dev || !dev->priv_data || !cmd) return -1;

    FeetechPrivData* priv = reinterpret_cast<FeetechPrivData*>(dev->priv_data);

    std::lock_guard<std::mutex> lock(g_mutex);

    // 检查模式是否匹配，不匹配则切换模式(掉电丢失)
    // 框架层传递 pos vel - 1 2
    // 实际 pos vel - 0 1，映射 - 1

    if (cmd->mode == MOTOR_MODE_IDLE) {
        if (!priv->is_idle) {  // 如果尚未处于卸力状态，则给底层发送卸力指令
            priv->pack->get_sms_sts().EnableTorque(priv->data.id, 0);
            priv->is_idle = true;  // 标记为IDLE，防止下次循环重复发送指令
        }
        return 0;
    }

    // 非掉电模式确保扭矩有效
    if (priv->is_idle) {
        // 从 IDLE 恢复到控制模式，重新使能扭矩
        priv->pack->get_sms_sts().EnableTorque(priv->data.id, 1);
        priv->is_idle = false;
    }

    if ((cmd->mode - 1) != priv->current_mode) {
        ModeSwitcher switcher(priv->pack->get_sms_sts());
        switcher.switch_mode_temp(priv->data.id, cmd->mode - 1);
        priv->current_mode = cmd->mode - 1;
    }

    // 将 motor_cmd 转换为 FeetechData
    // motor_cmd 使用弧度，feetech 使用 0-4095 的位置值

    // 位置
    float pos_rad = cmd->pos_des;
    // 0-4095 对应 0-2π 弧度
    int16_t position = static_cast<int16_t>(pos_rad);
    if (position < 0) position = 0;
    if (position > 4095) position = 4095;

    // feetech 速度单位约为 0.0146rpm，这里不处理
    uint16_t speed = static_cast<uint16_t>(cmd->vel_des);  // 保留原始单位
    if (speed > 2400) speed = 2400;

    // 加速度 (使用默认值)
    uint16_t acceleration = 50;

    priv->data.des_position = position;
    priv->data.des_speed = speed;
    priv->data.des_acceleration = acceleration;

    // 发送指令
    if (priv->pack) {
        priv->pack->send_pack_data(&priv->data, 1);
    }

    return 0;
}

static int feetech_get_state(struct motor_dev* dev, struct motor_state* state) {
    if (!dev || !dev->priv_data || !state) return -1;

    FeetechPrivData* priv = reinterpret_cast<FeetechPrivData*>(dev->priv_data);

    std::lock_guard<std::mutex> lock(g_mutex);

    // 读取状态
    if (priv->pack) {
        priv->pack->recv_unpack_data(&priv->data, 1);
    }

    // 将 FeetechData 转换为 motor_state
    // 位置转换 (0-4095 -> 弧度)
    state->pos = static_cast<float>(priv->data.cur_position);  // raw_position 单位： 0-4095 对应 0-2π rad

    // 速度转换 (feetech units -> rad/s)
    state->vel = static_cast<float>(priv->data.cur_speed);  // raw_speed 单位： 0.0146RPM

    // 力矩/负载转换
    state->trq = static_cast<float>(priv->data.cur_load);  // 0 ~ 1000

    // 温度
    state->temp = static_cast<float>(priv->data.cur_temperature);

    // 错误标志
    state->err = priv->data.err_flag;

    return 0;
}

static void feetech_free(struct motor_dev* dev) {
    if (!dev) return;

    if (dev->priv_data) {
        FeetechPrivData* priv = reinterpret_cast<FeetechPrivData*>(dev->priv_data);

        std::lock_guard<std::mutex> lock(g_mutex);
        g_motor_map.erase(priv->motor_id);

        delete priv;
    }

    free(dev);
}

static int feetech_get_paras(struct motor_dev* dev, const void* address, void* out_data, uint32_t data_len) {
    if (!dev || !dev->priv_data || !address || !out_data || data_len == 0) return -1;

    FeetechPrivData* priv = reinterpret_cast<FeetechPrivData*>(dev->priv_data);
    uint8_t mem_addr = *(const uint8_t*)address;

    // 根据 SMS_STS.h，最大已知寄存器地址是 70
    if (mem_addr + data_len > 71) {
        std::cerr << "[Feetech] Get parameters address out of bounds: " << static_cast<int>(mem_addr) << std::endl;
        return -1;
    }

    std::lock_guard<std::mutex> lock(g_mutex);
    if (!priv->pack) return -1;

    // 调用 SDK 提供的 Read() 方法
    int ret = priv->pack->get_sms_sts().Read(
        priv->motor_id, mem_addr, reinterpret_cast<uint8_t*>(out_data), static_cast<uint8_t>(data_len));
    if (ret != static_cast<int>(data_len)) {
        std::cerr << "[Feetech] Read paras failed! ID: " << static_cast<int>(priv->motor_id)
                    << ", Addr: " << static_cast<int>(mem_addr) << std::endl;
        return -1;
    }
    return 0;
}

static int feetech_set_paras(struct motor_dev* dev, const void* address, const void* data, uint32_t data_len) {
    if (!dev || !dev->priv_data || !address || !data || data_len == 0) return -1;

    FeetechPrivData* priv = reinterpret_cast<FeetechPrivData*>(dev->priv_data);
    uint8_t mem_addr = *(const uint8_t*)address;

    // 地址合法性检查
    if (mem_addr + data_len > 71) {
        std::cerr << "[Feetech] Set parameters address out of bounds: " << static_cast<int>(mem_addr) << std::endl;
        return -1;
    }

    // 检查寄存器读写权限 (根据 SMS_STS 规范)
    // EPROM(只读): 0~4
    // SRAM(只读):  56~70
    bool is_read_only = false;
    bool is_eprom_rw = false;
    for (uint32_t i = 0; i < data_len; i++) {
        uint8_t addr = mem_addr + i;
        if (addr <= 4 || (addr >= 56 && addr <= 70)) {
            is_read_only = true;
        } else if (addr >= 5 && addr <= 39) {
            // EPROM(读写): 5~39，需要专门解锁
            is_eprom_rw = true;
        }
    }

    if (is_read_only) {
        std::cerr << "[Feetech] Error: Cannot write to read-only register at " << static_cast<int>(mem_addr)
                    << std::endl;
        return -1;
    }

    std::lock_guard<std::mutex> lock(g_mutex);
    if (!priv->pack) return -1;

    // 写入 EPROM 区前必须进行解锁操作
    if (is_eprom_rw) {
        priv->pack->get_sms_sts().unLockEprom(priv->motor_id);
    }

    // 调用通用写指令方法 genWrite() （注：不是异步的 regWrite）
    int ret = priv->pack->get_sms_sts().genWrite(
        priv->motor_id, mem_addr, reinterpret_cast<uint8_t*>(const_cast<void*>(data)), static_cast<uint8_t>(data_len));

    // 写完 EPROM 区恢复加锁，防止数据异常覆写
    if (is_eprom_rw) {
        priv->pack->get_sms_sts().LockEprom(priv->motor_id);
    }

    if (ret == -1) {
        std::cerr << "[Feetech] Write paras failed! ID: " << static_cast<int>(priv->motor_id)
                    << ", Addr: " << static_cast<int>(mem_addr) << std::endl;
        return -1;
    }

    return 0;
}

// 虚函数表
static const struct motor_ops feetech_ops = {.init = feetech_init,
    .set_cmd = feetech_set_cmd,
    .get_state = feetech_get_state,
    .free = feetech_free,
    .set_paras = feetech_set_paras,
    .get_paras = feetech_get_paras};

// ==============================================================================
// 工厂函数
// ==============================================================================

static struct motor_dev* feetech_factory(void* args) {
    if (!args) return nullptr;

    struct motor_args_uart* uart_args = reinterpret_cast<struct motor_args_uart*>(args);

    // 分配 motor_dev 结构
    struct motor_dev* dev = reinterpret_cast<struct motor_dev*>(malloc(sizeof(struct motor_dev)));
    if (!dev) return nullptr;

    // 分配私有数据
    FeetechPrivData* priv = new FeetechPrivData();
    if (!priv) {
        free(dev);
        return nullptr;
    }

    priv->motor_id = uart_args->id;
    uint32_t actual_baud = uart_args->baud;
    if (actual_baud == 0) actual_baud = 1000000;  // 默认波特率

    std::cout << "[Feetech] Factory: motor_id=" << static_cast<int>(priv->motor_id) << ", baud=" << actual_baud
                << std::endl;

    // 获取或创建 FeetechPack 实例
    {
        std::lock_guard<std::mutex> lock(g_mutex);
        priv->pack = get_or_create_pack(uart_args->dev_path, actual_baud);
        if (!priv->pack) {
            delete priv;
            free(dev);
            return nullptr;
        }
    }

    // 初始化私有数据
    memset(&priv->data, 0, sizeof(FeetechData));
    priv->data.id = priv->motor_id;

    // 设置 motor_dev
    dev->name = "feetech";
    dev->ops = &feetech_ops;
    dev->priv_data = priv;

    // 添加到全局映射
    {
        std::lock_guard<std::mutex> lock(g_mutex);
        g_motor_map[priv->motor_id] = priv;
    }

    return dev;
}

// ==============================================================================
// 驱动注册 - 自动注册到 motor 框架
// ==============================================================================

REGISTER_MOTOR_DRIVER("feetech", DRV_TYPE_UART, feetech_factory)
