/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file test_debug.cpp
 * @brief 飞特舵机通信调试程序
 * 
 * 用于诊断舵机通信问题，包括：
 * 1. Ping 测试
 * 2. ID 扫描
 * 3. 波特率测试
 * 4. 读取模式测试
 */

#include <unistd.h>

#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>

#include "SMS_STS.h"

// 常用波特率列表
static const int BAUD_RATES[] = {1000000, 500000, 250000, 115200, 57600, 38400, 19200, 9600};
static const int NUM_BAUD_RATES = sizeof(BAUD_RATES) / sizeof(BAUD_RATES[0]);

void print_usage(const char *prog_name) {
    printf("Usage: %s <serial_port> [command] [options]\n", prog_name);
    printf("\nCommands:\n");
    printf("  ping <id>        - Ping 指定ID的舵机\n");
    printf("  scan             - 扫描所有舵机ID (1-253)\n");
    printf("  scan_baud <id>   - 扫描指定ID舵机的波特率\n");
    printf("  read_mode <id>   - 读取指定ID舵机的当前模式\n");
    printf("  read_all <id>    - 读取指定ID舵机的所有信息\n");
    printf("\nOptions:\n");
    printf("  -b <baud>        - 指定波特率 (默认: 1000000)\n");
    printf("\nExamples:\n");
    printf("  %s /dev/ttyACM1 ping 1\n", prog_name);
    printf("  %s /dev/ttyACM1 scan -b 1000000\n", prog_name);
    printf("  %s /dev/ttyACM1 scan_baud 1\n", prog_name);
}

int test_ping(SMS_STS &sms_sts, uint8_t id) {
    printf("\n=== Ping 测试 (ID=%d) ===\n", id);

    int result = sms_sts.Ping(id);
    if (result >= 0) {
        printf("✓ Ping 成功! 舵机ID: %d\n", result);
        return 0;
    } else {
        printf("✗ Ping 失败! 错误码: %d\n", result);
        printf("  可能原因:\n");
        printf("  - 舵机ID不正确\n");
        printf("  - 波特率不匹配\n");
        printf("  - 接线问题 (TX/RX)\n");
        printf("  - 舵机未上电\n");
        return -1;
    }
}

// 扫描所有ID
void scan_ids(SMS_STS &sms_sts, int start_id = 1, int end_id = 253) {
    printf("\n=== 扫描舵机ID (%d-%d) ===\n", start_id, end_id);

    int found_count = 0;
    for (int id = start_id; id <= end_id; id++) {
        int result = sms_sts.Ping(id);
        if (result >= 0) {
            printf("✓ 发现舵机 ID: %d\n", result);
            found_count++;
        }
        // 短暂延时避免通信过快
        usleep(10000);  // 10ms
    }

    printf("\n扫描完成，共发现 %d 个舵机\n", found_count);
}

// 扫描波特率
void scan_baud_rate(const char *port, uint8_t id) {
    printf("\n=== 扫描舵机波特率 (ID=%d) ===\n", id);

    SMS_STS sms_sts;

    for (int i = 0; i < NUM_BAUD_RATES; i++) {
        int baud = BAUD_RATES[i];
        printf("尝试波特率: %d ... ", baud);
        fflush(stdout);

        if (!sms_sts.begin(baud, port)) {
            printf("串口打开失败\n");
            continue;
        }

        usleep(50000);  // 等待50ms

        int result = sms_sts.Ping(id);
        if (result >= 0) {
            printf("✓ 成功! 舵机响应\n");
            printf("\n*** 找到正确波特率: %d ***\n", baud);
            sms_sts.end();
            return;
        } else {
            printf("无响应\n");
        }

        sms_sts.end();
        usleep(100000);  // 等待100ms
    }

    printf("\n未找到响应的波特率，请检查:\n");
    printf("  - 舵机ID是否正确\n");
    printf("  - 接线是否正确\n");
    printf("  - 舵机是否上电\n");
}

// 读取舵机模式
int read_mode(SMS_STS &sms_sts, uint8_t id) {
    printf("\n=== 读取舵机模式 (ID=%d) ===\n", id);

    int mode = sms_sts.ReadCurMode(id);
    if (mode >= 0) {
        printf("✓ 读取成功!\n");
        printf("  当前模式: %d ", mode);
        switch (mode) {
            case 0: printf("(位置伺服模式)\n"); break;
            case 1: printf("(恒速模式/轮模式)\n"); break;
            case 2: printf("(PWM开环模式)\n"); break;
            case 3: printf("(步进模式)\n"); break;
            default: printf("(未知模式)\n"); break;
        }
        return mode;
    } else {
        printf("✗ 读取失败! 错误码: %d\n", mode);
        return -1;
    }
}

// 读取舵机所有信息
void read_all_info(SMS_STS &sms_sts, uint8_t id) {
    printf("\n=== 读取舵机完整信息 (ID=%d) ===\n", id);

    // 先 Ping 确认舵机在线
    int ping_result = sms_sts.Ping(id);
    if (ping_result < 0) {
        printf("✗ 舵机无响应，无法读取信息\n");
        return;
    }
    printf("✓ 舵机在线\n\n");

    // 读取各项信息
    printf("--- 状态信息 ---\n");

    int pos = sms_sts.ReadPos(id);
    printf("  位置: %d %s\n", pos, (pos < 0) ? "(读取失败)" : "");

    int speed = sms_sts.ReadSpeed(id);
    printf("  速度: %d %s\n", speed, (speed < 0) ? "(读取失败)" : "");

    int load = sms_sts.ReadLoad(id);
    printf("  负载: %d %s\n", load, (load < 0) ? "(读取失败)" : "");

    int voltage = sms_sts.ReadVoltage(id);
    if (voltage >= 0) {
        printf("  电压: %.1f V\n", voltage / 10.0);
    } else {
        printf("  电压: 读取失败\n");
    }

    int temp = sms_sts.ReadTemper(id);
    printf("  温度: %d °C %s\n", temp, (temp < 0) ? "(读取失败)" : "");

    int moving = sms_sts.ReadMove(id);
    printf("  运动状态: %d %s\n", moving, (moving < 0) ? "(读取失败)" : (moving ? "(运动中)" : "(静止)"));

    int current = sms_sts.ReadCurrent(id);
    if (current >= 0) {
        printf("  电流: %d mA\n", current);
    } else {
        printf("  电流: 读取失败\n");
    }

    int mode = sms_sts.ReadCurMode(id);
    printf("  模式: %d ", mode);
    if (mode >= 0) {
        switch (mode) {
            case 0: printf("(位置伺服模式)\n"); break;
            case 1: printf("(恒速模式)\n"); break;
            case 2: printf("(PWM开环模式)\n"); break;
            case 3: printf("(步进模式)\n"); break;
            default: printf("(未知)\n"); break;
        }
    } else {
        printf("(读取失败)\n");
    }
}

int main(int argc, char *argv[]) {
    if (argc < 3) {
        print_usage(argv[0]);
        return 1;
    }

    const char *port = argv[1];
    const char *cmd = argv[2];
    int baud = 1000000;  // 默认波特率

    // 解析 -b 参数
    for (int i = 3; i < argc; i++) {
        if (strcmp(argv[i], "-b") == 0 && i + 1 < argc) {
            baud = atoi(argv[i + 1]);
            i++;
        }
    }

    // 处理 scan_baud 命令（不需要先打开串口）
    if (strcmp(cmd, "scan_baud") == 0) {
        if (argc < 4) {
            printf("错误: scan_baud 需要指定舵机ID\n");
            return 1;
        }
        uint8_t id = atoi(argv[3]);
        scan_baud_rate(port, id);
        return 0;
    }

    // 其他命令需要先打开串口
    SMS_STS sms_sts;

    printf("打开串口: %s\n", port);
    printf("波特率: %d\n", baud);

    if (!sms_sts.begin(baud, port)) {
        fprintf(stderr, "错误: 无法打开串口 %s\n", port);
        return 1;
    }

    printf("串口打开成功\n");

    // 等待串口稳定
    usleep(100000);  // 100ms

    int ret = 0;

    if (strcmp(cmd, "ping") == 0) {
        if (argc < 4) {
            printf("错误: ping 需要指定舵机ID\n");
            ret = 1;
        } else {
            uint8_t id = atoi(argv[3]);
            ret = test_ping(sms_sts, id);
        }
    } else if (strcmp(cmd, "scan") == 0) {
        scan_ids(sms_sts);
    } else if (strcmp(cmd, "read_mode") == 0) {
        if (argc < 4) {
            printf("错误: read_mode 需要指定舵机ID\n");
            ret = 1;
        } else {
            uint8_t id = atoi(argv[3]);
            ret = (read_mode(sms_sts, id) < 0) ? 1 : 0;
        }
    } else if (strcmp(cmd, "read_all") == 0) {
        if (argc < 4) {
            printf("错误: read_all 需要指定舵机ID\n");
            ret = 1;
        } else {
            uint8_t id = atoi(argv[3]);
            read_all_info(sms_sts, id);
        }
    } else {
        printf("未知命令: %s\n", cmd);
        print_usage(argv[0]);
        ret = 1;
    }

    sms_sts.end();
    return ret;
}
