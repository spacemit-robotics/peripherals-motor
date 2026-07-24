/*
 * 测试用例：初始化后切换位置模式，读取当前位置，发送一帧目标位置帧，目标位置为半圈后
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include "motor.h"

int main(int argc, char **argv) {
    const char *dev_path = "/dev/ttyUSB0";
    if (argc > 1) {
        dev_path = argv[1];
    }

    printf("Starting M0601C once position test on %s...\n", dev_path);

    // 1. 创建 UART 电机设备实例
    struct motor_dev *dev = motor_alloc_uart("drv_485_ddt_m601c111", dev_path, 115200, 1, NULL);
    if (!dev) {
        printf("Error: Failed to allocate motor device.\n");
        return -1;
    }

    // 2. 初始化电机
    if (motor_init_one(dev) < 0) {
        printf("Error: Failed to initialize motor.\n");
        motor_free(&dev, 1);
        return -1;
    }

    printf("Motor initialized successfully.\n");

    // 3. 初始化后为了安全且避免电机突跳，先发一帧 IDLE 指令获取当前电机的真实位置
    struct motor_cmd init_cmd = {0};
    init_cmd.mode = MOTOR_MODE_IDLE;
    motor_set_cmd_one(dev, &init_cmd);

    struct motor_state state = {0};
    motor_get_state_one(dev, &state);

    // 4. 切换模式：切换到位置模式（目标位置暂时设为当前位置，确保安全平滑）
    struct motor_cmd pos_cmd = {0};
    pos_cmd.mode = MOTOR_MODE_POS;
    pos_cmd.pos_des = state.pos;
    motor_set_cmd_one(dev, &pos_cmd);

    // 5. 打印实际模式
    uint8_t actual_mode = 0xFF;
    if (motor_get_paras(dev, (const void *)(uintptr_t)0x74, &actual_mode, sizeof(actual_mode)) == 0) {
        printf("Actual Mode from device: 0x%02X\n", actual_mode);
    } else {
        printf("Failed to read actual mode.\n");
    }

    // 6. 然后再获取当前位置：通过 get_paras 获取 raw 值并换算
    uint16_t current_pos_raw = 0;
    if (motor_get_paras(dev, (const void *)(uintptr_t)0x74, &current_pos_raw, sizeof(current_pos_raw)) == 0) {
        float converted_pos = ((float)current_pos_raw / 32767.0f) * (2.0f * (float)M_PI);
        printf("Current pos_raw: %u (0x%04X)\n", current_pos_raw, current_pos_raw);
        printf("Current Converted Position: %8.4f rad\n", converted_pos);
        state.pos = converted_pos; // 更新当前位置基准
    } else {
        printf("Failed to get current pos_raw.\n");
    }

    // 7. 发送一帧目标位置帧，目标位置为半圈后 (当前位置 + PI)
    pos_cmd.pos_des = state.pos + M_PI;
    printf("Setting target position to: %8.4f rad (Current + M_PI)...\n", pos_cmd.pos_des);

    if (motor_set_cmd_one(dev, &pos_cmd) < 0) {
        printf("Failed to set position command.\n");
    } else {
        printf("Position command sent successfully.\n");
    }

    // 8. 打印反馈帧及 pos_raw 值
    struct motor_state fb_state = {0};
    if (motor_get_state_one(dev, &fb_state) == 0) {
        printf("[Feedback Frame] Pos: %8.4f rad | Vel: %8.4f rad/s | Trq(Curr): %8.4f A | Err: 0x%02X\n",
                fb_state.pos, fb_state.vel, fb_state.trq, fb_state.err);

        uint16_t fb_pos_raw = 0;
        if (motor_get_paras(dev, (const void *)(uintptr_t)0x74, &fb_pos_raw, sizeof(fb_pos_raw)) == 0) {
            printf("Feedback pos_raw (masked): %u (0x%04X)\n", fb_pos_raw, fb_pos_raw);
        }
        printf("Feedback pos_raw_unmasked (from state.temp): %u (0x%04X)\n",
                (uint16_t)fb_state.temp, (uint16_t)fb_state.temp);
    } else {
        printf("Failed to get feedback state.\n");
    }

    // 给一点时间让指令发送完成
    sleep(3);

    // 9. 再次获取当前位置并打印 pos_raw
    // （由于是问答式协议，需再发一帧同样的位置指令来触发电机回传最新状态）
    motor_set_cmd_one(dev, &pos_cmd);
    if (motor_get_state_one(dev, &fb_state) == 0) {
        printf("\n--- After 3 seconds ---\n");
        printf("[Final State] Pos: %8.4f rad | Vel: %8.4f rad/s | Trq: %8.4f A\n",
                fb_state.pos, fb_state.vel, fb_state.trq);

        uint16_t final_pos_raw = 0;
        if (motor_get_paras(dev, (const void *)(uintptr_t)0x74, &final_pos_raw, sizeof(final_pos_raw)) == 0) {
            printf("Final pos_raw (masked): %u (0x%04X)\n", final_pos_raw, final_pos_raw);
        }
        printf("Final pos_raw_unmasked (from state.temp): %u (0x%04X)\n",
                (uint16_t)fb_state.temp, (uint16_t)fb_state.temp);
    }
    // 安全断电/失能：下发 IDLE 模式
    struct motor_cmd idle_cmd = {0};
    idle_cmd.mode = MOTOR_MODE_IDLE;
    motor_set_cmd_one(dev, &idle_cmd);

    motor_free(&dev, 1);
    return 0;
}
