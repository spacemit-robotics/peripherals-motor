#include "motor.h"
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

static volatile sig_atomic_t g_running = 1;

static void signal_handler(int sig) {
    (void)sig;
    g_running = 0;
}

int main(int argc, char **argv) {
    if (argc < 2) {
        printf("Usage: %s <tty_device> [motor_id]\n", argv[0]);
        return -1;
    }

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    uint8_t motor_id = (argc >= 3) ? (uint8_t)atoi(argv[2]) : 1;
    struct motor_dev *dev = motor_alloc_uart("drv_uart_ddt_m603c111", argv[1], 115200, motor_id, NULL);
    if (!dev) {
        printf("Failed to allocate M0603C motor\n");
        return -1;
    }
    if (motor_init_one(dev) != 0) {
        printf("Failed to initialize motor\n");
        motor_free(&dev, 1);
        return -1;
    }

    struct motor_cmd cmd = {0};
    struct motor_state state = {0};
    uint16_t pos_raw = 0;

    // 先下发一次 IDLE 指令，以触发 0x74 帧查询真实编码器位置
    cmd.mode = MOTOR_MODE_IDLE;
    motor_set_cmd_one(dev, &cmd);
    motor_get_state_one(dev, &state);
    motor_get_paras(dev, (const void *)(uintptr_t)0x74, &pos_raw, sizeof(pos_raw));

    float start_pos = state.pos;
    printf("\nInitial state: pos=%.4f rad, pos_raw=%u\n", start_pos, pos_raw);

    // 切换到位置模式，目标位置设为真实初始位置
    cmd.mode = MOTOR_MODE_POS;
    cmd.pos_des = start_pos;
    printf("Switching to POSITION mode with target pos=%.4f rad\n\n", start_pos);

    while (g_running) {
        motor_set_cmd_one(dev, &cmd);
        motor_get_state_one(dev, &state);
        motor_get_paras(dev, (const void *)(uintptr_t)0x74, &pos_raw, sizeof(pos_raw));

        printf("vel: %6.2f rad/s, trq: %6.2f A, temp: %4.1f degC, pos: %8.4f rad, pos_raw: %5u\n",
            state.vel, state.trq, state.temp, state.pos, pos_raw);

        usleep(100000); // 100ms (10Hz) 低频循环
    }

    printf("\nStopping motor and releasing resources...\n");
    cmd.mode = MOTOR_MODE_IDLE;
    motor_set_cmd_one(dev, &cmd);
    motor_free(&dev, 1);
    printf("Motor released cleanly.\n");
    return 0;
}
