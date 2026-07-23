#include "motor.h"
#include "motor_core.h"
#include "drv_can_ddt_m152d133.h"
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <signal.h>
#include <stdlib.h>

#define K_PI 3.14159265358979323846f

// 全局设备指针，用于信号处理器中安全释放电机
static struct motor_dev *g_dev = NULL;
static volatile sig_atomic_t g_running = 1;

// 信号处理器：捕获 SIGINT/SIGTERM，安全释放电机
static void signal_handler(int sig) {
    (void)sig;
    g_running = 0;

    if (g_dev) {
        printf("\n[SIGNAL] Caught signal %d, disabling motor...\n", sig);
        struct motor_cmd cmd = {0};
        cmd.mode = MOTOR_MODE_IDLE;
        motor_set_cmd_one(g_dev, &cmd);
        usleep(50000);  // 确保失能报文发送完毕
        motor_free(&g_dev, 1);
        g_dev = NULL;
    }

    exit(0);
}

int main(int argc, char **argv) {
    if (argc < 2) {
        printf("Usage: %s <can_interface>\n", argv[0]);
        return -1;
    }

    // 注册信号处理器，确保异常终止时释放电机
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    struct motor_args_can args = {
        .iface = argv[1],
        .can_id = 1,
        .args = NULL,
    };

    struct motor_dev *dev = motor_alloc_can("drv_can_ddt_m152d133", argv[1], 1, &args);
    if (!dev) {
        printf("Failed to allocate M152D133 motor\n");
        return -1;
    }

    // 保存到全局变量，供信号处理器使用
    g_dev = dev;

    if (motor_init_one(dev) != 0) {
        printf("Failed to init motor\n");
        return -1;
    }

    // 【终极优化】：设置电机反馈周期为 20ms，与本程序的 usleep(20000) 控制循环完美同频，彻底杜绝数据积压滞后
    uint8_t feedback_cfg = 20; // 0~127，代表毫秒
    if (motor_set_paras(dev, (const void *)(uintptr_t)BM_M152D133_PARAM_FEEDBACK, &feedback_cfg, sizeof(feedback_cfg)) == 0) {
        printf("Successfully set motor feedback period to 20ms.\n");
    } else {
        printf("Warning: Failed to set feedback period.\n");
    }

    // 【关键】程序启动时先发送失能命令并等待，确保电机从任意状态平滑过渡
    // 解决问题：失能不断电情况下，程序再次启动时的顿挫
    printf("Ensuring motor is disabled and stable before starting...\n");
    struct motor_cmd cmd = {0};
    cmd.mode = MOTOR_MODE_IDLE;
    struct motor_state state = {0};

    // 【应用层修复】利用 SocketCAN 的阻塞特性排空积压队列 (Drain the Queue)
    // 这里循环 50 次。如果有历史积压帧，底层 read 将瞬间返回，每次循环仅耗时 2ms，能迅速清空队列。
    // 队列清空后，read 会阻塞等待新帧（通常 10~20ms），使循环自动与电机物理反馈频率同步，总体提供约 500~1000ms 的稳定期。
    // 检查反馈读取，多次失败则报错
    int read_failures = 0;
    for (int i = 0; i < 50; i++) {
        motor_set_cmd_one(dev, &cmd);
        if (motor_get_state_one(dev, &state) < 0) {
            read_failures++;
            printf("Warning: Failed to read motor state at iteration %d\n", i);
        }
        usleep(2000);
    }

    // 上述循环结束后，队列已经被排空，且 state 应该是最新的真实物理位置。
    // 如果反馈读取失败次数过多，说明通信/反馈配置可能有问题。
    if (read_failures > 10) {
        printf("Error: Too many state read failures (%d/50). Cannot proceed with valid initial position.\n", read_failures);
        motor_free(&dev, 1);
        return -1;
    }

    float initial_position = state.pos;
    printf("Initial physical position: %.2f rad (read failures: %d/50)\n", initial_position, read_failures);

    // 1. 设置目标位置，并直接通过 motor_set_cmd_one 下发
    // 底层驱动会自动处理 Mode 的切换，保证指令下发和模式切换同步，消除启动抖动。
    cmd.mode = MOTOR_MODE_POS;
    cmd.pos_des = initial_position;

    printf("Setting mode to Position Loop (3) and holding initial position for 2 seconds...\n");
    for (int i = 0; i < 100; i++) {
        motor_set_cmd_one(dev, &cmd);
        motor_get_state_one(dev, &state);
        usleep(20000); // 20ms * 100 = 2秒
    }

    // 设定目标为相对当前位置转动半圈 (+PI)
    // 驱动层现已支持多圈连续反馈，且发令时会自动进行 fmod，所以应用层无需手动截断跨界
    float target_position = initial_position + K_PI;

    float current_target = initial_position;
    float step = 0.05f; // 每次增加 0.05 rad，绝对安全

    printf("--- Start Forward Rotation ---\n");
    // 向前转动直到接近 target_position (此处用步数控制更稳妥，避免跨界大小比较问题)
    int steps = (int)(K_PI / step);
    for (int i = 0; i < steps; i++) {
        current_target += step;
        cmd.pos_des = current_target;
        motor_set_cmd_one(dev, &cmd);

        // 延时等待响应，控制速度
        usleep(20000); // 20ms

        // 终端打印指令帧和反馈帧，降低打印频率 (每 10 个步长打印一次)
        if (motor_get_state_one(dev, &state) == 0) {
            if (i % 10 == 0) {
                printf("[Command] pos_des: %.2f | [Feedback] pos: %.2f vel: %.2f trq: %.2f err: %u\n",
                    cmd.pos_des, state.pos, state.vel, state.trq, state.err);
            }
        }
    }

    printf("--- Start Backward Rotation ---\n");
    for (int i = 0; i < steps; i++) {
        current_target -= step;
        cmd.pos_des = current_target;
        motor_set_cmd_one(dev, &cmd);

        // 延时等待响应，控制速度
        usleep(20000); // 20ms

        // 终端打印指令帧和反馈帧，降低打印频率 (每 10 个步长打印一次)
        if (motor_get_state_one(dev, &state) == 0) {
            if (i % 10 == 0) {
                printf("[Command] pos_des: %.2f | [Feedback] pos: %.2f vel: %.2f trq: %.2f err: %u\n",
                    cmd.pos_des, state.pos, state.vel, state.trq, state.err);
            }
        }
    }

    // 4. 切换到速度环模式
    printf("\nSetting mode to Velocity Loop (2) and holding velocity at 0 for 2 seconds...\n");
    cmd.mode = MOTOR_MODE_VEL;
    cmd.vel_des = 0.0f;
    for (int i = 0; i < 100; i++) {
        motor_set_cmd_one(dev, &cmd);
        motor_get_state_one(dev, &state);
        usleep(20000); // 20ms * 100 = 2秒
    }

    // 绝对安全的速度：1.0 rad/s (约 9.5 rpm)
    float safe_velocity = 1.0f;

    printf("--- Start Positive Velocity ---\n");
    cmd.vel_des = safe_velocity;
    for (int i = 0; i < 100; i++) { // 运行 2 秒
        motor_set_cmd_one(dev, &cmd);
        usleep(20000);

        if (motor_get_state_one(dev, &state) == 0) {
            if (i % 10 == 0) {
                printf("[Command] vel_des: %.2f | [Feedback] pos: %.2f vel: %.2f trq: %.2f err: %u\n",
                    cmd.vel_des, state.pos, state.vel, state.trq, state.err);
            }
        }
    }

    printf("--- Start Negative Velocity ---\n");
    cmd.vel_des = -safe_velocity;
    for (int i = 0; i < 100; i++) { // 运行 2 秒
        motor_set_cmd_one(dev, &cmd);
        usleep(20000);

        if (motor_get_state_one(dev, &state) == 0) {
            if (i % 10 == 0) {
                printf("[Command] vel_des: %.2f | [Feedback] pos: %.2f vel: %.2f trq: %.2f err: %u\n",
                    cmd.vel_des, state.pos, state.vel, state.trq, state.err);
            }
        }
    }

    // 主动制动：确保电机停稳后再进入开环模式
    printf("\nBraking before open loop...\n");
    cmd.vel_des = 0.0f;
    for (int i = 0; i < 50; i++) { // 1秒制动
        motor_set_cmd_one(dev, &cmd);
        motor_get_state_one(dev, &state);
        usleep(20000);
    }

    // 6. 切换到开环模式
    printf("\nSetting mode to Open Loop (0) and holding at 0 for 2 seconds...\n");
    cmd.mode = MOTOR_MODE_OPEN;
    cmd.vel_des = 0.0f;
    for (int i = 0; i < 100; i++) {
        motor_set_cmd_one(dev, &cmd);
        motor_get_state_one(dev, &state);
        usleep(20000); // 20ms * 100 = 2秒
    }

    // 绝对安全的开环速度：1.0 rad/s (约 9.5 rpm，远低于 300rpm 过速保护)
    float safe_open_velocity = 1.0f;

    printf("--- Start Positive Open Loop ---\n");
    cmd.vel_des = safe_open_velocity;
    for (int i = 0; i < 100; i++) { // 运行 2 秒
        motor_set_cmd_one(dev, &cmd);
        usleep(20000);

        if (motor_get_state_one(dev, &state) == 0) {
            if (i % 10 == 0) {
                printf("[Command] vel_des(open): %.2f | [Feedback] pos: %.2f vel: %.2f trq: %.2f err: %u\n",
                    cmd.vel_des, state.pos, state.vel, state.trq, state.err);
            }
        }
    }

    printf("--- Start Negative Open Loop ---\n");
    cmd.vel_des = -safe_open_velocity;
    for (int i = 0; i < 100; i++) { // 运行 2 秒
        motor_set_cmd_one(dev, &cmd);
        usleep(20000);

        if (motor_get_state_one(dev, &state) == 0) {
            if (i % 10 == 0) {
                printf("[Command] vel_des(open): %.2f | [Feedback] pos: %.2f vel: %.2f trq: %.2f err: %u\n",
                    cmd.vel_des, state.pos, state.vel, state.trq, state.err);
            }
        }
    }

    // 主动制动：确保电机停稳后再进入电流环，避免残余速度叠加导致过速
    printf("\nBraking before current loop...\n");
    cmd.vel_des = 0.0f;
    for (int i = 0; i < 50; i++) { // 1秒制动
        motor_set_cmd_one(dev, &cmd);
        motor_get_state_one(dev, &state);
        usleep(20000);
    }

    // 5. 切换到电流环模式
    printf("\nSetting mode to Current Loop (1) and holding torque at 0 for 2 seconds...\n");
    cmd.mode = MOTOR_MODE_TRQ;
    cmd.trq_des = 0.0f;
    for (int i = 0; i < 100; i++) {
        motor_set_cmd_one(dev, &cmd);
        motor_get_state_one(dev, &state);
        usleep(20000); // 20ms * 100 = 2秒
    }

    // 绝对安全的力矩：对应 1A 电流 (0.88 Nm/A * 1A = 0.88 Nm)
    // 运行时间缩短为 1 秒，防止空载电机一直加速导致超速保护
    float safe_torque = 0.88f;

    printf("--- Start Positive Torque ---\n");
    cmd.trq_des = safe_torque;
    for (int i = 0; i < 50; i++) { // 运行 1 秒
        motor_set_cmd_one(dev, &cmd);
        usleep(20000);

        if (motor_get_state_one(dev, &state) == 0) {
            if (i % 10 == 0) {
                printf("[Command] trq_des: %.2f | [Feedback] pos: %.2f vel: %.2f trq: %.2f err: %u\n",
                    cmd.trq_des, state.pos, state.vel, state.trq, state.err);
            }
        }
    }

    printf("--- Start Negative Torque ---\n");
    cmd.trq_des = -safe_torque;
    for (int i = 0; i < 50; i++) { // 运行 1 秒
        motor_set_cmd_one(dev, &cmd);
        usleep(20000);

        if (motor_get_state_one(dev, &state) == 0) {
            if (i % 10 == 0) {
                printf("[Command] trq_des: %.2f | [Feedback] pos: %.2f vel: %.2f trq: %.2f err: %u\n",
                    cmd.trq_des, state.pos, state.vel, state.trq, state.err);
            }
        }
    }

    // 软停止：在断开使能之前先将力矩清零，消除电磁力释放带来的顿挫感
    printf("\nBraking before exit...\n");
    cmd.trq_des = 0.0f;
    for (int i = 0; i < 50; i++) {
        motor_set_cmd_one(dev, &cmd);
        motor_get_state_one(dev, &state);
        printf("[Command] trq_des: %.2f | [Feedback] pos: %.2f vel: %.2f trq: %.2f err: %u\n",
            cmd.trq_des, state.pos, state.vel, state.trq, state.err);
        usleep(20000);
    }

    // 6. 停止电机，返回 IDLE 模式
    cmd.mode = MOTOR_MODE_IDLE;
    motor_set_cmd_one(dev, &cmd);

    usleep(50000); // 确保失能报文发送完毕

    printf("\n--- Checking State After Disable ---\n");
    if (motor_get_state_one(dev, &state) == 0) {
        printf("[After Disable] pos: %.2f vel: %.2f trq: %.2f err: %u\n",
            state.pos, state.vel, state.trq, state.err);
    } else {
        printf("[After Disable] Failed to get motor state.\n");
    }

    uint8_t current_mode_after = 0;
    uint32_t param_mode_get_after = BM_M152D133_PARAM_CUR_MODE;
    if (motor_get_paras(dev, (const void *)(uintptr_t)param_mode_get_after, &current_mode_after, sizeof(current_mode_after)) == 0) {
        printf("[After Disable] Motor Mode: 0x%02X\n", current_mode_after);
    } else {
        printf("[After Disable] Failed to read motor mode.\n");
    }

    motor_free(&dev, 1);
    g_dev = NULL;  // 清理全局指针
    return 0;
}
