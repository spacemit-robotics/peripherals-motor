/*
 * 最小测试用例：M0601C 电机低频状态查询
 * 编译及运行要求：需链接 motor 库。
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "motor.h"

static void check_and_print_mode(struct motor_dev *dev, const char *context) {
    uint8_t mode = 0xFF;
    if (motor_get_paras(dev, (const void *)(uintptr_t)0x74, &mode, sizeof(mode)) == 0) {
        printf("[%s] Current Mode from device: 0x%02X\n", context, mode);
    } else {
        printf("[%s] Failed to read current mode.\n", context);
    }
}

// 软启动/停止控制函数：给定目标速度和持续时间，并在 50Hz 频率下生成平滑过渡的轨迹
static void run_safe_trajectory(struct motor_dev *dev, float target_vel, int duration_ms, float *current_vel) {
    struct motor_cmd cmd = {0};
    struct motor_state state = {0};
    cmd.mode = MOTOR_MODE_VEL;

    int step_ms = 20; // 控制环路周期 20ms (50 Hz)
    int steps = duration_ms / step_ms;

    // 每次允许的最大加速度跨度：设定为 2.5 rad/s^2 -> 每次 20ms 允许变化 0.05 rad/s
    float max_delta = 0.05f;

    for (int i = 0; i < steps; i++) {
        // 生成斜坡（Ramp）轨迹
        if (*current_vel < target_vel) {
            *current_vel += max_delta;
            if (*current_vel > target_vel) *current_vel = target_vel;
        } else if (*current_vel > target_vel) {
            *current_vel -= max_delta;
            if (*current_vel < target_vel) *current_vel = target_vel;
        }

        cmd.vel_des = *current_vel;
        if (motor_set_cmd_one(dev, &cmd) < 0) {
            // 通信失败时不强行中断，只是报告
        }

        if (i == 0) {
            check_and_print_mode(dev, "Velocity Loop");
        }

        // 降低打印频率至 10Hz（即每 5 次循环打印一次）
        if (i % 5 == 0) {
            motor_get_state_one(dev, &state);
            printf("[Motor State] CmdVel: %6.2f | Pos: %8.4f rad | Vel: %8.4f rad/s | Trq(Curr): %8.4f A\n",
                   *current_vel, state.pos, state.vel, state.trq);
        }
        usleep(step_ms * 1000);
    }
}


int main(int argc, char **argv) {
    const char *dev_path = "/dev/ttyUSB1";
    if (argc > 1) {
        dev_path = argv[1];
    }

    printf("Starting M0601C motor state query test on %s...\n", dev_path);

    // 1. 创建 UART 电机设备实例
    // "drv_485_ddt_m601c111" 为注册的驱动名；波特率 115200；电机 ID 假设为 1
    struct motor_dev *dev = motor_alloc_uart("drv_485_ddt_m601c111", dev_path, 115200, 1, NULL);
    if (!dev) {
        printf("Error: Failed to allocate motor device.\n");
        return -1;
    }

    // 2. 初始化电机 (将打开串口并执行基本配置)
    if (motor_init_one(dev) < 0) {
        printf("Error: Failed to initialize motor.\n");
        motor_free(&dev, 1); // free 虽然通常接收双指针数组，但按 API 标准清理
        return -1;
    }

    uint8_t current_mode = 0xFF;
    if (motor_get_paras(dev, (const void *)(uintptr_t)0x74, &current_mode, sizeof(current_mode)) == 0) {
        printf("Current Mode after init: 0x%02X\n", current_mode);
    } else {
        printf("Failed to read current mode after init.\n");
    }

    printf("Motor initialized successfully. Executing Safe Trajectory: Forward and Reverse x2...\n");
    printf("------------------------------------------------------------\n");

    float current_vel = 0.0f;

    for (int cycle = 1; cycle <= 2; cycle++) {
        printf("\n>>> === Cycle %d/2 === <<<\n", cycle);

        printf("1. Forward Rotation (target: +2.0 rad/s)\n");
        // 旋转 2.5 秒
        run_safe_trajectory(dev, 2.0f, 2500, &current_vel);

        printf("2. Stopping...\n");
        // 停止 1.5 秒
        run_safe_trajectory(dev, 0.0f, 1500, &current_vel);

        printf("3. Reverse Rotation (target: -2.0 rad/s)\n");
        // 旋转 2.5 秒
        run_safe_trajectory(dev, -2.0f, 2500, &current_vel);

        printf("4. Stopping...\n");
        // 停止 1.5 秒
        run_safe_trajectory(dev, 0.0f, 1500, &current_vel);
    }

    printf("\n>>> === Current Loop (Torque Mode) Test === <<<\n");
    printf("Setting mode to Torque (Current)...\n");

    struct motor_cmd trq_cmd = {0};
    struct motor_state trq_state = {0};
    trq_cmd.mode = MOTOR_MODE_TRQ;
    trq_cmd.trq_des = 1.0f; // 1.0A

    for (int i = 0; i < 50; i++) { // 1 second
        motor_set_cmd_one(dev, &trq_cmd);
        if (i == 0) {
            check_and_print_mode(dev, "Torque Loop");
        }
        if (i % 5 == 0) {
            motor_get_state_one(dev, &trq_state);
            printf("[Torque Mode] CmdTrq: %6.2f A | Pos: %8.4f rad | Vel: %8.4f rad/s | Trq(Curr): %8.4f A\n",
                    trq_cmd.trq_des, trq_state.pos, trq_state.vel, trq_state.trq);
        }
        usleep(20000); // 20ms
    }

    printf("Stopping Torque Mode...\n");
    trq_cmd.trq_des = 0.0f;
    for (int i = 0; i < 25; i++) { // 0.5 seconds
        motor_set_cmd_one(dev, &trq_cmd);
        usleep(20000);
    }

    printf("\n>>> === Position Loop Mode Test === <<<\n");
    printf("Setting mode to Position and target to 3.1416 rad (180 deg)...\n");

    struct motor_cmd pos_cmd = {0};
    struct motor_state pos_state = {0};
    pos_cmd.mode = MOTOR_MODE_POS;
    pos_cmd.pos_des = 3.14159f;

    for (int i = 0; i < 50; i++) { // 1 second
        motor_set_cmd_one(dev, &pos_cmd);
        if (i == 0) {
            check_and_print_mode(dev, "Position Loop");
        }
        if (i % 5 == 0) {
            motor_get_state_one(dev, &pos_state);
            printf("[Position Mode] CmdPos: %8.4f rad | Pos: %8.4f rad | Vel: %8.4f rad/s | Trq(Curr): %8.4f A\n",
                    pos_cmd.pos_des, pos_state.pos, pos_state.vel, pos_state.trq);
        }
        usleep(20000); // 20ms
    }

    printf("Setting target to 0 rad (0 deg)...\n");
    pos_cmd.pos_des = 0.0f;
    for (int i = 0; i < 50; i++) { // 1 second
        motor_set_cmd_one(dev, &pos_cmd);
        if (i % 5 == 0) {
            motor_get_state_one(dev, &pos_state);
            printf("[Position Mode] CmdPos: %8.4f rad | Pos: %8.4f rad | Vel: %8.4f rad/s | Trq(Curr): %8.4f A\n",
                    pos_cmd.pos_des, pos_state.pos, pos_state.vel, pos_state.trq);
        }
        usleep(20000); // 20ms
    }

    printf("\nTest complete. Shutting down motor.\n");

    // 安全断电/失能：下发 IDLE 模式
    struct motor_cmd cmd = {0};
    cmd.mode = MOTOR_MODE_IDLE;
    motor_set_cmd_one(dev, &cmd);

    motor_free(&dev, 1);
    return 0;
}
