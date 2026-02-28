/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#include "feetech_pack.h"

#include <unistd.h>
#include <ctime>
#include <iostream>


// 获取当前时间戳（纳秒）
static int64_t get_timestamp_ns() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

// 建立串口连接
int FeetechPack::init_pack(int argc, char **argv) {
    if (!sms_sts.begin(1000000, argv[1])) {
        std::cout << "Failed to init serial!" << std::endl;
        return 0;
    }

    return 1;
}

void FeetechPack::send_pack_data(FeetechData *data, int num) {
    int64_t t1, t2, t3;

    t1 = get_timestamp_ns();
    for (int i = 0; i < num; i++) {
        sms_sts.RegWritePosEx(data[i].id, data[i].des_position, data[i].des_speed,
                            data[i].des_acceleration);
    }
    t2 = get_timestamp_ns();

    sms_sts.RegWriteAction(0xfe);  // 广播执行异步写入指令
    t3 = get_timestamp_ns();

    std::cout << "[Timestamp] RegWrite: " << t1 << " ns, done: " << t2
        << " ns (cost " << (t2 - t1) << " ns)" << std::endl;
    std::cout << "[Timestamp] Broadcast: " << t2 << " ns, done: " << t3
        << " ns (cost " << (t3 - t2) << " ns)" << std::endl;
    std::cout << "[Timestamp] Total send cost: " << (t3 - t1) << " ns" << std::endl;
}

void FeetechPack::recv_unpack_data(FeetechData *data, int num) {
    u8 id_list[255];
    u8 rxBuf[15];  // 接收缓冲区
    for (int i = 0; i < num; i++) {
        id_list[i] = data[i].id;
    }
    sms_sts.syncReadBegin(num, 15, 100);            // 每个舵机读取15字节数据，超时100ms
    sms_sts.syncReadPacketTx(id_list, num, 56, 15);  // 从寄存器56(0x38)开始读取
    for (int i = 0; i < num; i++) {
        if (sms_sts.syncReadPacketRx(id_list[i], rxBuf) == 0) {
            std::cout << "Motor " << static_cast<int>(id_list[i]) << " read failed!"
                << std::endl;
            continue;
        }
        // 解析数据 (地址56-70)
        data[i].cur_position = rxBuf[0] | (rxBuf[1] << 8);    // 56-57
        data[i].cur_speed = rxBuf[2] | (rxBuf[3] << 8);       // 58-59
        data[i].cur_load = rxBuf[4] | (rxBuf[5] << 8);        // 60-61
        data[i].cur_voltage = rxBuf[6];                       // 62
        data[i].cur_temperature = rxBuf[7];                   // 63
        data[i].move_flag = rxBuf[10];                        // 66
        data[i].cur_current = rxBuf[13] | (rxBuf[14] << 8);  // 69-70
    }
    sms_sts.syncReadEnd();
}

void FeetechPack::close_pack() { sms_sts.end(); }

void ModeSwitcher::switch_mode(uint8_t id, uint8_t mode) {
    // 解锁 EEPROM
    sms_sts.unLockEprom(id);

    // 设置工作模式
    // mode: 0=Servo模式(位置控制), 1=Wheel模式(速度控制)
    if (mode == 0) {
        sms_sts.ServoMode(id);
    } else if (mode == 1) {
        sms_sts.WheelMode(id);
    }

    // 锁定 EEPROM
    sms_sts.LockEprom(id);
}

// 临时切换模式（不锁定 EEPROM）
void ModeSwitcher::switch_mode_temp(uint8_t id, uint8_t mode) {
    // 设置工作模式
    // mode: 0=Servo模式(位置控制), 1=Wheel模式(速度控制)
    if (mode == 0) {
        uint8_t last_mode = sms_sts.ReadCurMode(id);
        sms_sts.ServoMode(id);

        uint8_t current_mode = sms_sts.ReadCurMode(id);

        std::cout << "[Feetech] SwitchModeTemp: Motor ID " << static_cast<int>(id)
            << ", switched from mode " << static_cast<int>(last_mode)
            << " to mode " << static_cast<int>(mode) << ", current mode is "
            << static_cast<int>(current_mode) << std::endl;
    } else if (mode == 1) {
        uint8_t last_mode = sms_sts.ReadCurMode(id);
        sms_sts.WheelMode(id);
        // 读当前模式

        uint8_t current_mode = sms_sts.ReadCurMode(id);
        std::cout << "[Feetech] SwitchModeTemp: Motor ID " << static_cast<int>(id)
            << ", switched from mode " << static_cast<int>(last_mode)
            << ", switched to mode " << static_cast<int>(mode)
            << ", current mode is " << static_cast<int>(current_mode)
            << std::endl;
    }
}
