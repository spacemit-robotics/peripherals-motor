/*
 * 初始化后每 0.1 s 发送 IDLE 帧，同步打印 pos_raw, 无限循环发送直到程序被终止
 * 用于测试并观察物理旋转电机时的数值范围，以验证满圈的量程 (32767, 36000, 65535 等)。
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include "motor.h"

static struct motor_dev *g_dev = NULL;
static volatile sig_atomic_t g_running = 1;

static void signal_handler(int sig) {
    (void)sig;
    g_running = 0;
}

int main(int argc, char **argv) {
    const char *dev_path = "/dev/ttyUSB0";
    if (argc > 1) {
        dev_path = argv[1];
    }

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    printf("Starting M0601C state monitor on %s...\n", dev_path);

    struct motor_dev *dev = motor_alloc_uart("drv_485_ddt_m601c111", dev_path, 115200, 1, NULL);
    if (!dev) {
        printf("Error: Failed to allocate motor.\n");
        return -1;
    }
    g_dev = dev;

    if (motor_init_one(dev) < 0) {
        printf("Error: Failed to init motor.\n");
        motor_free(&dev, 1);
        return -1;
    }

    struct motor_cmd cmd = {0};
    cmd.mode = MOTOR_MODE_IDLE;
    struct motor_state state = {0};

    printf("Monitoring started. Please manually rotate the motor. Press Ctrl+C to stop.\n");

    while (g_running) {
        motor_set_cmd_one(dev, &cmd);

        if (motor_get_state_one(dev, &state) == 0) {
            uint16_t pos_raw_temp = (uint16_t)state.temp;

            uint16_t pos_raw_74 = 0;
            if (motor_get_paras(dev, (const void *)(uintptr_t)0x74, &pos_raw_74, sizeof(pos_raw_74)) == 0) {
                printf("[State] pos_raw(0x74): %5u (0x%04X) | pos_raw_unmasked(temp): %5u (0x%04X) | err: %u\n",
                        pos_raw_74, pos_raw_74, pos_raw_temp, pos_raw_temp, state.err);
            } else {
                printf("[State] Failed to get 0x74 paras.\n");
            }
        }
        usleep(100000);
    }

    printf("\nExiting and disabling motor...\n");
    motor_set_cmd_one(dev, &cmd);
    usleep(50000);
    motor_free(&dev, 1);
    g_dev = NULL;
    return 0;
}
