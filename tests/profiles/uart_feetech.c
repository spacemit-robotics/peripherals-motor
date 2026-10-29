/*
    * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
    * SPDX-License-Identifier: Apache-2.0
    */

/*
    * @file test_motor_uart.c
    * @brief UART 电机通用测试程序
    *
    * 支持命令行参数配置，用于测试级联在同一总线上的多个 UART 电机。
    * 用法: test_motor_uart [--driver name] [--port device] [--baud rate]
    *                      [--id id] [--move-rad delta]
    *   默认: /dev/ttyACM1 1000000 drv_uart_feetech 2,3
    */

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "motor.h"
#include "feetech_hd1910_adapter.h"
#include "test_config.h"

static int run_hd1910(const char *port, int baud, int id, float move_rad) {
    if (baud != 1000000 || id < 0 || id >= 0xfe) {
        fprintf(stderr, "HD1910 requires 1000000 baud and motor ID 0..253\n");
        return 1;
    }
    struct feetech_hd1910_config config = {
        .zero_offset_ticks = 2048, .direction = 1, .firmware_p = 5,
    };
    struct motor_dev *motor = motor_alloc_uart(
        "drv_uart_feetech_hd1910", port, (uint32_t)baud, (uint8_t)id, &config);
    if (!motor) {
        fprintf(stderr, "HD1910 allocation failed: %s ID=%d\n", port, id);
        return 1;
    }

    int result = 1;
    struct motor_state state = {0};
    if (motor_init_one(motor) != 0)
        fprintf(stderr, "HD1910 initialization read failed; retrying feedback\n");
    int have_feedback = 0;
    for (int i = 0; i < 5; ++i) {
        if (motor_get_state_one(motor, &state) == 0) {
            have_feedback = 1;
            break;
        }
        struct timespec pause = {.tv_sec = 0, .tv_nsec = 100000000L};
        nanosleep(&pause, NULL);
    }
    if (!have_feedback || !isfinite(state.pos) || fabsf(state.pos) > 3.14f) {
        fprintf(stderr, "HD1910 initial feedback unavailable or out of range; no move sent\n");
        goto cleanup;
    }
    const float target = state.pos + move_rad;
    struct motor_cmd cmd = {0};
    cmd.mode = MOTOR_MODE_POS;
    cmd.pos_des = target;
    printf("HD1910 ID=%d: initial=%.4f rad, delta=%+.4f rad, target=%.4f rad\n",
        id, state.pos, move_rad, target);
    if (motor_set_cmd_one(motor, &cmd) != 0) {
        fprintf(stderr, "HD1910 position command failed\n");
        goto cleanup;
    }
    for (int i = 0; i < 30; ++i) {
        struct timespec pause = {.tv_sec = 0, .tv_nsec = 100000000L};
        nanosleep(&pause, NULL);
        if (motor_get_state_one(motor, &state) != 0) {
            fprintf(stderr, "HD1910 feedback failed after command\n");
            goto cleanup;
        }
        if (fabsf(state.pos - target) <= 0.02f) {
            printf("HD1910 reached %.4f rad\n", state.pos);
            result = 0;
            break;
        }
    }
    if (result != 0)
        fprintf(stderr, "HD1910 did not reach target: final=%.4f rad\n", state.pos);

cleanup:
    {
        struct motor_cmd idle = {0};
        idle.mode = MOTOR_MODE_IDLE;
        if (motor_set_cmd_one(motor, &idle) != 0) {
            fprintf(stderr, "HD1910 idle cleanup failed\n");
            result = 1;
        }
    }
    motor_free(&motor, 1);
    return result;
}

static void print_usage(const char *prog_name) {
    printf("Usage: %s [options]\n", prog_name);
    printf("Options:\n");
    printf("  --driver <name>    Set motor driver (default: drv_uart_feetech)\n");
    printf("                     STS3215: drv_uart_feetech_sts3215; HD1910: drv_uart_feetech_hd1910\n");
    printf("  --port <port>      Set UART port (default: /dev/ttyACM1)\n");
    printf("  --baud <baudrate>  Set baudrate (default: 1000000)\n");
    printf("  --id <id1,id2...>  Set motor IDs (STS3215 default: 2,3; HD1910 default: 1)\n");
    printf("  --move-rad <delta> HD1910 relative movement, required; -0.5..0.5 rad\n");
    printf("  -h, --help         Show this help\n");
    printf("\nNote: Parameters will be prioritized from config/config_parameters.yaml.\n");
    printf("      Command-line arguments override YAML configurations.\n");
}

int main(int argc, char *argv[]) {
    for (int j = 1; j < argc; j++) {
        if (strcmp(argv[j], "-h") == 0 || strcmp(argv[j], "--help") == 0) {
            print_usage(argv[0]);
            return 0;
        }
    }
    struct motor_dev *motors[16] = {NULL};
    struct motor_cmd cmds[16];
    struct motor_state states[16];
    int num_motors = 2;
    uint32_t i;
    int ret = 0;

    const char *dev_path = "/dev/ttyACM1";
    int baud = 1000000;
    const char *driver = "drv_uart_feetech";
    int ids[16] = {2, 3};
    float move_rad = 0.0f;
    int has_move_rad = 0;

    for (int j = 1; j < argc; ++j) {
        if (strcmp(argv[j], "--move-rad") != 0) continue;
        if (has_move_rad || j + 1 >= argc) {
            fprintf(stderr, "ERROR: --move-rad must be supplied exactly once with a value\n");
            return 2;
        }
        char *end = NULL;
        move_rad = strtof(argv[++j], &end);
        if (end == argv[j] || *end != '\0' || !isfinite(move_rad) ||
            move_rad < -0.5f || move_rad > 0.5f) {
            fprintf(stderr, "ERROR: --move-rad must be within -0.5..0.5 rad\n");
            return 2;
        }
        has_move_rad = 1;
    }

    for (int j = 1; j + 1 < argc; ++j) {
        if (strcmp(argv[j], "--driver") == 0 &&
            strcmp(argv[j + 1], "drv_uart_feetech_hd1910") == 0) {
            ids[0] = 1;
            num_motors = 1;
            break;
        }
    }

    load_config_and_args(argc, argv, &driver, &dev_path, &baud, ids, &num_motors, NULL);

    if (strcmp(driver, "drv_uart_feetech_hd1910") == 0) {
        if (!has_move_rad) {
            fprintf(stderr, "HD1910 requires --move-rad within -0.5..0.5 rad\n");
            return 2;
        }
        if (num_motors != 1) {
            fprintf(stderr, "HD1910 test requires exactly one --id\n");
            return 2;
        }
        return run_hd1910(dev_path, baud, ids[0], move_rad);
    }
    if (has_move_rad) {
        fprintf(stderr, "--move-rad is only supported by drv_uart_feetech_hd1910\n");
        return 2;
    }

    printf("=== UART Motor Test ===\n");
    printf("  dev_path=%s, baud=%d, driver=%s, num_motors=%d\n\n", dev_path, baud,
            driver, num_motors);

    /* 1. 创建电机实例（级联在同一总线上，通过 id 区分） */
    for (i = 0; i < num_motors; i++) {
    uint8_t motor_id = (uint8_t)ids[i];
    motors[i] = motor_alloc_uart(driver, dev_path, (uint32_t)baud, motor_id, NULL);
    if (!motors[i]) {
        fprintf(stderr, "ERROR: Failed to create motor %u on %s\n", i + 1,
                dev_path);
        ret = -1;
        goto cleanup;
    }
    printf("  Created motor %u (ID=%u)\n", i + 1, motor_id);
    }
    printf("\n");

    /* 2. 初始化 */
    ret = motor_init(motors, num_motors);
    if (ret != 0) {
    fprintf(stderr, "ERROR: Failed to initialize motors: %d\n", ret);
    goto cleanup;
    }
    printf("[2] All motors initialized\n\n");

    printf("\n******************************adjust parameters "
            "tests************************\n");
    uint8_t p_addr = 0x15;     // 位置环 P 比例系数 寄存器地址
    uint8_t p_value_write = 0;  // 准备写入的值
    uint8_t p_value_read = 0;  // 准备读取的变量

    // 1. 写入 P 参数 = 1
    printf("Write P -> %d (Addr: 0x%02X)\n", p_value_write, p_addr);
    int ret_set = motor_set_paras(motors[0], &p_addr, &p_value_write,
                                sizeof(p_value_write));
    if (ret_set == 0) {
    printf("Set P successful.\n");
    } else {
    printf("Set P failed! ret = %d\n", ret_set);
    }

    // 2. 读取 P 参数
    int ret_get =
        motor_get_paras(motors[0], &p_addr, &p_value_read, sizeof(p_value_read));
    if (ret_get == 0) {
    printf("Read P successful, Value = %d\n", p_value_read);
    } else {
    printf("Read P failed! ret = %d\n", ret_get);
    }

    // 3. 读取 limit_current 参数(2 字节)
    p_addr = 0x1C;
    uint16_t p_value_read_16 = 0;

    ret_get =
        motor_get_paras(motors[0], &p_addr, &p_value_read_16, sizeof(p_value_read_16));
    if (ret_get == 0) {
    printf("Read limit_current successful, Value = %d\n", p_value_read_16);
    } else {
    printf("Read limit_current failed! ret = %d\n", ret_get);
    }
    printf("*********************************************************************"
            "********\n\n");

    /* 3. 位置控制测试 */
    printf("=========================== position ctl "
            "================================\n");
        const float kPosTargetRad = 6.1374215f;        // 4000 / 4095 * 2π (rad)
        const float kVelTargetRadPerSec = 3.6693802f;  // 2400 units -> rad/s

        memset(cmds, 0, sizeof(cmds));
        for (i = 0; i < num_motors; i++) {
        cmds[i].mode = MOTOR_MODE_POS;
        cmds[i].pos_des = kPosTargetRad;
        cmds[i].vel_des = kVelTargetRadPerSec;
        }

    printf("\nmode =  %d\n", cmds[0].mode);
    ret = motor_set_cmds(motors, cmds, num_motors);
    if (ret != 0) {
    fprintf(stderr, "ERROR: Failed to set commands: %d\n", ret);
    goto cleanup;
    }
    sleep(3);

    ret = motor_get_states(motors, states, num_motors);
    if (ret != 0) {
    fprintf(stderr, "ERROR: Failed to get states: %d\n", ret);
    goto cleanup;
    }
    for (i = 0; i < num_motors; i++) {
    printf("Motor %u: Pos=%.2f, Vel=%.2f, Trq=%.2f, Temp=%.2f, Err=0x%X\n",
            i + 1, states[i].pos, states[i].vel, states[i].trq, states[i].temp,
            states[i].err);
    }

    /* 恢复 0 位 */
    memset(cmds, 0, sizeof(cmds));
    for (i = 0; i < num_motors; i++) {
    cmds[i].mode = MOTOR_MODE_POS;
    cmds[i].pos_des = 0;
        cmds[i].vel_des = kVelTargetRadPerSec;
    }
    motor_set_cmds(motors, cmds, num_motors);
    sleep(3);

    ret = motor_get_states(motors, states, num_motors);
    if (ret != 0) {
    fprintf(stderr, "ERROR: Failed to get states: %d\n", ret);
    goto cleanup;
    }
    for (i = 0; i < num_motors; i++) {
    printf("Motor %u: Pos=%.2f, Vel=%.2f, Trq=%.2f, Temp=%.2f, Err=0x%X\n",
            i + 1, states[i].pos, states[i].vel, states[i].trq, states[i].temp,
            states[i].err);
    }

    /* 4. 速度控制测试 */
    printf("=========================== velocity ctl "
            "================================\n");
        memset(cmds, 0, sizeof(cmds));
        for (i = 0; i < num_motors; i++) {
        cmds[i].mode = MOTOR_MODE_VEL;
        cmds[i].vel_des = kVelTargetRadPerSec;
        }
    motor_set_cmds(motors, cmds, num_motors);
    sleep(3);

    motor_get_states(motors, states, num_motors);
    for (i = 0; i < num_motors; i++) {
    printf("Motor %u: Pos=%.2f, Vel=%.2f, Trq=%.2f, Temp=%.2f, Err=0x%X\n",
            i + 1, states[i].pos, states[i].vel, states[i].trq, states[i].temp,
            states[i].err);
    }

    /* 停止 */
    memset(cmds, 0, sizeof(cmds));
    for (i = 0; i < num_motors; i++) {
    cmds[i].mode = MOTOR_MODE_VEL;
    cmds[i].vel_des = 0;
    }
    motor_set_cmds(motors, cmds, num_motors);
    sleep(3);

    ret = motor_get_states(motors, states, num_motors);
    if (ret != 0) {
    fprintf(stderr, "ERROR: Failed to get states: %d\n", ret);
    goto cleanup;
    }
    for (i = 0; i < num_motors; i++) {
    printf("Motor %u: Pos=%.2f, Vel=%.2f, Trq=%.2f, Temp=%.2f, Err=0x%X\n",
            i + 1, states[i].pos, states[i].vel, states[i].trq, states[i].temp,
            states[i].err);
    }

    /* 5. 第二轮位置控制 */
    printf("=========================== position ctl "
            "================================\n");
        memset(cmds, 0, sizeof(cmds));
        for (i = 0; i < num_motors; i++) {
        cmds[i].mode = MOTOR_MODE_POS;
        cmds[i].pos_des = kPosTargetRad;
        cmds[i].vel_des = kVelTargetRadPerSec;
        }
    ret = motor_set_cmds(motors, cmds, num_motors);
    if (ret != 0) {
    fprintf(stderr, "ERROR: Failed to set commands: %d\n", ret);
    goto cleanup;
    }
    sleep(3);

    ret = motor_get_states(motors, states, num_motors);
    if (ret != 0) {
    fprintf(stderr, "ERROR: Failed to get states: %d\n", ret);
    goto cleanup;
    }
    for (i = 0; i < num_motors; i++) {
    printf("Motor %u: Pos=%.2f, Vel=%.2f, Trq=%.2f, Temp=%.2f, Err=0x%X\n",
            i + 1, states[i].pos, states[i].vel, states[i].trq, states[i].temp,
            states[i].err);
    }

    /* 恢复 0 位 */
    memset(cmds, 0, sizeof(cmds));
    for (i = 0; i < num_motors; i++) {
    cmds[i].mode = MOTOR_MODE_POS;
    cmds[i].pos_des = 0;
        cmds[i].vel_des = kVelTargetRadPerSec;
    }
    motor_set_cmds(motors, cmds, num_motors);
    sleep(3);

    ret = motor_get_states(motors, states, num_motors);
    if (ret != 0) {
    fprintf(stderr, "ERROR: Failed to get states: %d\n", ret);
    goto cleanup;
    }
    for (i = 0; i < num_motors; i++) {
    printf("Motor %u: Pos=%.2f, Vel=%.2f, Trq=%.2f, Temp=%.2f, Err=0x%X\n",
            i + 1, states[i].pos, states[i].vel, states[i].trq, states[i].temp,
            states[i].err);
    }

    printf("=== Test completed ===\n");

cleanup:
    motor_free(motors, num_motors);
    return (ret != 0) ? 1 : 0;
}
