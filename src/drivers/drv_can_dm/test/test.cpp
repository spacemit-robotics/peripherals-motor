/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */ 

#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>
#include <vector>

#include "pack_damiao.h"

int main() {
    // 2. 准备电机配置
    std::vector<damiao::MotorConfig> configs;

    // 示例：配置一个连接在 can0 上的 DM4310 电机
    damiao::MotorConfig config1;
    config1.bus_name = "can0";
    config1.can_id = 0x02;    // 电机 ID
    config1.master_id = 0x12;  // 主机 ID (如果有)
    config1.motor_type = damiao::DM4310;
    config1.control_mode = damiao::MIT_MODE;  // 使用 MIT 模式
    configs.push_back(config1);

    // 如果有更多电机，继续添加...
    // configs.push_back(config2);

    std::cout << "Initializing motors..." << std::endl;

    // 3. 初始化电机 (总线初始化 + 使能)
    // g_motor_hw 是在 pack_damiao 内部定义的全局指针，直接传入引用即可
    if (!init_motors(g_motor_hw, configs)) {
        std::cerr << "Failed to initialize motors!" << std::endl;
        return -1;
    }

    std::cout << "Initialization successful. Starting control loop..."
                << std::endl;

    // 4. 设置运动参数
    // 这里设置让电机转 2.5 圈，耗时 6 秒
    damiao::MotionParams params;
    params.total_revolutions = 2.5;  // 目标圈数
    params.motion_time = 6.0;        // 运动总时间 (s)
    params.accel_time = 2.0;         // 加速时间 (s)
    params.kp = 12.0;                // 位置刚度
    params.kd = 2.0;                 // 速度阻尼

    // 5. 进入控制循环
    // running 标志由 signalHandler 控制
    while (running) {
        // 5.1 发送控制指令
        // 注意：内部会调用 setMotionParams 和 write
        send_trajectory_command(g_motor_hw, params);

        // 5.2 获取并打印反馈状态
        // 内部会调用 read 和 getActuatorData
        // 并不是每次调用都会打印，内部有降频打印逻辑
        get_feedback(g_motor_hw);

        // 5.3 控制循环频率
        // 示例中使用 500Hz (2ms)
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    // 退出前确保资源释放
    // (信号处理函数中也会调用，这里作为双重保障或正常退出的处理)
    if (g_motor_hw) {
        std::cout << "Exiting main loop." << std::endl;
        disable_motors(g_motor_hw);
    }

    return 0;
}

