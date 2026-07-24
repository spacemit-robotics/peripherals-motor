#include "motor.h"
#include "motor_core.h"
#include "drv_can_ddt_m152d133.h"
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <signal.h>
#include <stdlib.h>

static struct motor_dev *g_dev = NULL;
static volatile sig_atomic_t g_running = 1;

static void signal_handler(int sig) {
    (void)sig;
    g_running = 0;
}

int main(int argc, char **argv) {
    if (argc < 2) {
        printf("Usage: %s <can_interface>\n", argv[0]);
        return -1;
    }

    // 开启底层驱动的 RAW 报文数据打印
    setenv("BM_DEBUG_RAW", "1", 1);

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
    g_dev = dev;

    if (motor_init_one(dev) != 0) {
        printf("Failed to init motor\n");
        return -1;
    }

    // 设置电机反馈周期为 20ms
    uint8_t feedback_cfg = 20;
    if (motor_set_paras(dev, (const void *)(uintptr_t)BM_M152D133_PARAM_FEEDBACK, &feedback_cfg, sizeof(feedback_cfg)) == 0) {
        printf("Successfully set motor feedback period to 20ms.\n");
    } else {
        printf("Warning: Failed to set feedback period.\n");
    }

    // 设置为空闲模式，确保不输出力矩，允许外力转动
    struct motor_cmd cmd = {0};
    cmd.mode = MOTOR_MODE_IDLE;
    motor_set_cmd_one(dev, &cmd);

    printf("Motor disabled. Waiting for external force to rotate rotor...\n");
    printf("Press Ctrl+C to exit.\n");

    struct motor_state state = {0};
    int count = 0;

    // 清空历史缓冲
    for (int i = 0; i < 50; i++) {
        motor_get_state_one(dev, &state);
        usleep(2000);
    }

    while (g_running) {
        // 定期重复发送 IDLE，防止有些电机因为超时无指令而报错
        if (count++ % 50 == 0) {
            motor_set_cmd_one(dev, &cmd);
        }

        // 读取反馈
        if (motor_get_state_one(dev, &state) == 0) {
            printf("[Passive] pos: %.4f rad | vel: %.4f rad/s | err: %u\n",
                state.pos, state.vel, state.err);
        }
        usleep(20000); // 20ms
    }

    printf("\nExiting...\n");
    // 安全释放
    motor_set_cmd_one(dev, &cmd);
    usleep(50000);
    motor_free(&dev, 1);
    g_dev = NULL;
    return 0;
}
