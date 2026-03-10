
/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef FEETECH_PACK_H
#define FEETECH_PACK_H

#include <cstdint>
#include "SMS_STS.h"
#include "SCServo.h"

typedef struct feetech_data
{
    /* data */
    uint8_t id;
    uint8_t mode;  // 舵机模式 0 = 位置伺服，1 = 恒速模式，2 = pwm 开环调速度模式
    uint8_t reg_flag;  // 0x40，此位为 1 表示舵机异步写入寄存器数据未执行
    uint8_t err_flag;  // 0x41，此位为 1 表示舵机有错误
    uint8_t move_flag;  // 0x42，此位为 1 表示舵机正在运动，舵机到达目标位置后该位清 0，舵机无目标位置时保持 0
    uint16_t des_position;
    uint16_t des_speed;
    uint16_t des_acceleration;
    uint16_t cur_position;
    uint16_t cur_speed;
    uint16_t cur_load;
    uint16_t cur_voltage;
    uint8_t cur_temperature;
    uint16_t cur_current;  // 0x45,单位 6.5mA
} FeetechData;

class FeetechPack {
public:
    FeetechPack() {}
    ~FeetechPack() {}
    /*
     * 初始化串口
     */
    int init_pack(int argc, char **argv);

    /*
     * 发送打包数据，调用异步写指令
     * data：数据数组指针
     * num：数据数组长度
     */
    void send_pack_data(FeetechData *data, int num);

    /*
     * 接收打包数据，调用同步读指令
     * data：数据数组指针
     * num：数据数组长度
     */
    void recv_unpack_data(FeetechData *data, int num);

    /*
     * 关闭串口
     */
    void close_pack();


    /*
     * 获取舵机控制对象引用（用于模式切换等操作）
     */
    SMS_STS& get_sms_sts() { return sms_sts; }

private:
    SMS_STS sms_sts;  // 舵机控制对象
    HLSCL hlscl;  // 备用舵机控制对象 == 用于模式切换
};

class ModeSwitcher {
public:
    explicit ModeSwitcher(SMS_STS &sms) : sms_sts(sms) {}
    ~ModeSwitcher() {}
    /*
     * 切换舵机模式
     * id：舵机 ID
     * mode：目标模式
     *   0 = 位置伺服模式，位置反馈
     *   1 = 电机恒速模式，速度反馈，持续运动直到速度为 0
     *       此时位置可反馈，但不作为控制目标，且位置在 4095 范围内循环
     *   2 = 电机开环模式，电流反馈，持续运动直到 PWM 为 0， 此模式在内存表有说明，但官方驱动未实现
     */
    void switch_mode(uint8_t id, uint8_t mode);

    // 临时切换模式（不锁定 EEPROM）
    void switch_mode_temp(uint8_t id, uint8_t mode);

private:
    SMS_STS &sms_sts;  // 引用外部已初始化的舵机控制对象
};

#endif  // FEETECH_PACK_H
