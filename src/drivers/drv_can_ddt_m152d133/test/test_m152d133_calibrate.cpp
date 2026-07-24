#include "motor.h"
#include "motor_core.h"
#include "drv_can_ddt_m152d133.h"
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>

int main(int argc, char **argv) {
    if (argc < 2) {
        printf("Usage: %s <can_interface>\n", argv[0]);
        return -1;
    }

    // 强制开启底层 RAW 打印，这样驱动内部解析时的原始字节(带最高位标志)就能在终端实时显示
    setenv("BM_DEBUG_RAW", "1", 1);

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

    if (motor_init_one(dev) != 0) {
        printf("Failed to init motor\n");
        return -1;
    }

    // 确保反馈周期为 20ms
    uint8_t feedback_cfg = 20;
    motor_set_paras(dev, (const void *)(uintptr_t)BM_M152D133_PARAM_FEEDBACK, &feedback_cfg, sizeof(feedback_cfg));

    printf("==========================================\n");
    printf("--- Before Calibration ---\n");
    printf("Observing raw feedback for 3 seconds.\n");
    printf("Look for 'pos_raw=...' in the logs. If uncalibrated, it should be >= 32768.\n");
    printf("==========================================\n");

    struct motor_state state;
    for (int i = 0; i < 150; i++) {
        motor_get_state_one(dev, &state);
        usleep(20000);
    }

    printf("\n==========================================\n");
    printf("--- Executing Mechanical Zero Calibration ---\n");
    printf("==========================================\n");

    uint8_t dummy = 1;
    if (motor_set_paras(dev, (const void *)(uintptr_t)BM_M152D133_PARAM_MECH_ZERO, &dummy, 1) == 0) {
        printf("Calibration command sent successfully.\n");
    } else {
        printf("Failed to send calibration command.\n");
    }

    usleep(500000); // 留出 500ms 让固件处理标定内部计算

    // 发送保存参数命令，确保掉电不丢失零位信息
    printf("Saving parameters to motor flash...\n");
    if (motor_set_paras(dev, (const void *)(uintptr_t)BM_M152D133_PARAM_SAVE, &dummy, 1) == 0) {
        printf("Save command sent successfully.\n");
    }

    usleep(500000); // 等待写入 flash 完成

    printf("\n==========================================\n");
    printf("--- After Calibration ---\n");
    printf("Observing raw feedback for another 3 seconds.\n");
    printf("If successful, 'pos_raw=...' should drop down near 0 (without the 32768 offset).\n");
    printf("==========================================\n");

    // 排空一下积压帧
    for (int i = 0; i < 20; i++) {
        motor_get_state_one(dev, &state);
        usleep(2000);
    }

    for (int i = 0; i < 150; i++) {
        motor_get_state_one(dev, &state);
        usleep(20000);
    }

    printf("\nCalibration test complete.\n");
    motor_free(&dev, 1);
    return 0;
}
