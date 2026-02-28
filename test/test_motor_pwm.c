/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <pthread.h>
#include "../include/motor.h"

static int test_pwm_generic() {
    struct motor_dev *motors[2] = {NULL};
    struct motor_cmd cmds[2];
    struct motor_state states[2];
    int ret;
    uint32_t i;

    printf("=== PWM Motor Vectorized API Test ===\n\n");

    /* 1. 创建多个PWM电机（使用 pwm_demo driver） */
    printf("[1] Creating PWM motor instances...\n");
    motors[0] = motor_alloc_pwm("pwm_demo", 1, NULL);
    motors[1] = motor_alloc_pwm("pwm_demo", 2, NULL);

    for (i = 0; i < 2; i++) {
        if (!motors[i]) {
            fprintf(stderr, "Failed to create PWM motor %u\n", i);
            ret = -1;
            goto cleanup;
        }
        printf("   Created PWM motor %u\n", i);
    }
    printf("\n");

    /* 2. 批量初始化 */
    printf("[2] Initializing all PWM motors (vectorized)...\n");
    ret = motor_init(motors, 2);
    if (ret < 0) {
        fprintf(stderr, "Failed to init motors: %d\n", ret);
        goto cleanup;
    }
    printf("   All 2 motors initialized\n\n");

    /* 3. 批量设置速度命令 */
    printf("[3] Setting velocity commands (vectorized)...\n");
    for (i = 0; i < 2; i++) {
        cmds[i].mode = MOTOR_MODE_VEL;
        cmds[i].vel_des = 30.0f + i * 20.0f;
        cmds[i].pos_des = 0.0f;
        cmds[i].trq_des = 0.0f;
        cmds[i].kp = 0.0f;
        cmds[i].kd = 0.0f;
    }

    ret = motor_set_cmds(motors, cmds, 2);
    if (ret < 0) {
        fprintf(stderr, "Failed to set commands: %d\n", ret);
        goto cleanup;
    }
    for (i = 0; i < 2; i++) {
        printf("   Motor %u: velocity = %.2f rad/s\n", i, cmds[i].vel_des);
    }
    printf("\n");

    /* 4. 测试力矩控制模式 */
    printf("[4] Testing torque control mode...\n");
    cmds[0].mode = MOTOR_MODE_TRQ;
    cmds[0].trq_des = 2.5f;
    cmds[0].vel_des = 0.0f;
    cmds[0].pos_des = 0.0f;

    ret = motor_set_cmd_one(motors[0], &cmds[0]);
    if (ret < 0) {
        fprintf(stderr, "Failed to set torque command: %d\n", ret);
        goto cleanup;
    }
    printf("   Torque command set: %.2f Nm\n\n", cmds[0].trq_des);

    /* 5. 批量获取状态 */
    printf("[5] Reading motor states (vectorized)...\n");
    ret = motor_get_states(motors, states, 2);
    if (ret < 0) {
        fprintf(stderr, "Failed to get states: %d\n", ret);
        goto cleanup;
    }
    for (i = 0; i < 2; i++) {
        printf("   Motor %u state:\n", i);
        printf("    Position: %.3f rad\n", states[i].pos);
        printf("    Velocity: %.3f rad/s\n", states[i].vel);
        printf("    Torque:   %.3f Nm\n", states[i].trq);
        printf("    Temp:     %.1f °C\n", states[i].temp);
    }
    printf("\n");

    /* 6. 测试空闲模式 */
    printf("[6] Testing idle mode...\n");
    cmds[1].mode = MOTOR_MODE_IDLE;
    ret = motor_set_cmd_one(motors[1], &cmds[1]);
    if (ret < 0) {
        fprintf(stderr, "Failed to set idle mode: %d\n", ret);
        goto cleanup;
    }
    printf("   Motor set to idle\n\n");

    printf("=== Test completed successfully ===\n");

cleanup:
    /* 7. 批量释放资源 */
    printf("[7] Cleaning up (vectorized)...\n");
    motor_free(motors, 2);
    printf("   All motors freed\n");
    return (ret < 0) ? -1 : 0;
}

static int run_one_motor_test(int motor_index, int step_gpio, int dir_gpio,
            int enable_gpio, int stop_gpio, int constant_range, int speed){
    struct RoHS_info {
        uint8_t motor_index;    //   电机索引，用于区分多个步进电机
        uint8_t step_gpio;    //   步进脉冲 GPIO 编号
        uint8_t dir_gpio;    //   方向 GPIO 编号
        uint8_t enable_gpio;    //   使能 GPIO 编号
        uint8_t stop_gpio;    //   限位开关 GPIO 编号 (可选, 没有则填0)

        int current_position;        //   当前步数位置
        int constant_range;    //   固定的电机可转动角度范围，根据实际的角度范围设定
        int gpio_max_steps;        //   最大步数限制，首次会动态获取并写入到配置文件里面

        bool enable_gpio_level;         //   使能电平反转， 如果是false， 表示低电平使能
        bool dir_gpio_left_level;       //   指定向左的电平, 如果是false，表示低电平向左
        bool stop_gpio_active_level;    //   限位开关有效电平, 如果是false，表示低电平有效

        int range_steps;  /*没有stop相位开关，则需要配置电机steps步数*/
    } info = {
        .motor_index = motor_index,  //  Motor index comment
        .step_gpio = step_gpio,
        .dir_gpio = dir_gpio,
        .enable_gpio = enable_gpio,
        .stop_gpio = stop_gpio,  //  Optional, fill 0 if no limit switch
        .current_position = -1,  //  Current step position, init to -1 means unknown
        .constant_range = constant_range,  //  Fixed motor rotation angle range
        .gpio_max_steps = -1,  //  Max step limit, dynamically obtained and written to config file on first run
        .dir_gpio_left_level = false,
        .enable_gpio_level = false,
        .stop_gpio_active_level = false,
    };
    struct motor_dev *motor = motor_alloc_pwm("pwm_RoHS", 0, &info);
    if (!motor) {
        printf("错误：电机分配失败\n");
        return -1;
    }

    if (motor_init_one(motor) < 0) {
        printf("错误：电机初始化失败\n");
        motor_free(&motor, 1);
        return -1;
    }

    /*run motor*/
    struct motor_cmd cmd = {
        .mode = MOTOR_MODE_POS,
        .pos_des = 0,  //  Expected position
        .vel_des = speed,  //  Expected speed
    };
    struct motor_state state;

    motor_set_cmd_one(motor, &cmd);
    motor_get_state_one(motor, &state);
    printf("pos=%.2f vel=%.2f trq=%.2f\n", state.pos, state.vel, state.trq);
    usleep(500000);
    cmd.pos_des = 180;
    motor_set_cmd_one(motor, &cmd);
    motor_get_state_one(motor, &state);
    printf("pos=%.2f vel=%.2f trq=%.2f\n", state.pos, state.vel, state.trq);
    usleep(500000);

    cmd.pos_des = 90;
    motor_set_cmd_one(motor, &cmd);
    motor_get_state_one(motor, &state);
    printf("pos=%.2f vel=%.2f trq=%.2f\n", state.pos, state.vel, state.trq);
    usleep(500000);

    cmd.pos_des = 75;
    motor_set_cmd_one(motor, &cmd);
    motor_get_state_one(motor, &state);
    printf("pos=%.2f vel=%.2f trq=%.2f\n", state.pos, state.vel, state.trq);
    usleep(500000);

    cmd.pos_des = 105;
    motor_set_cmd_one(motor, &cmd);
    motor_get_state_one(motor, &state);
    printf("pos=%.2f vel=%.2f trq=%.2f\n", state.pos, state.vel, state.trq);
    usleep(500000);

    cmd.pos_des = 90;
    motor_set_cmd_one(motor, &cmd);
    motor_get_state_one(motor, &state);
    printf("pos=%.2f vel=%.2f trq=%.2f\n", state.pos, state.vel, state.trq);
    usleep(500000);

    motor_free(&motor, 1);

    return 0;
}

//   定义电机参数结构体
typedef struct {
    int motor_index;
    int step_gpio;
    int dir_gpio;
    int enable_gpio;
    int stop_gpio;
    int constant_range;
    int speed;
} MotorParams;

//   线程函数包装器
void* motor_thread_wrapper(void* arg) {
    MotorParams* params = (MotorParams*)arg;

    //   调用实际的电机控制函数
    run_one_motor_test(params->motor_index,
                params->step_gpio,
                params->dir_gpio,
                params->enable_gpio,
                params->stop_gpio,
                params->constant_range,
                params->speed);

    return NULL;
}

static int run_multi_motor_test(){
    pthread_t thread1, thread2, thread3;

    //   为每个电机创建独立的参数结构体
    MotorParams params1 = {1, 46, 43, 42, 83, 60, 3};
    MotorParams params2 = {2, 34, 35, 36, 82, 60, 3};
    MotorParams params3 = {3, 37, 38, 39, 61, 60, 3};
    if (pthread_create(&thread1, NULL, motor_thread_wrapper, &params1) != 0) {
        perror("创建线程1失败");
        return -1;
    }

    if (pthread_create(&thread2, NULL, motor_thread_wrapper, &params2) != 0) {
        perror("创建线程2失败");
        return -1;
    }

    if (pthread_create(&thread3, NULL, motor_thread_wrapper, &params3) != 0) {
        perror("创建线程3失败");
        return -1;
    }

    //   等待线程结束
    pthread_join(thread1, NULL);
    pthread_join(thread2, NULL);
    pthread_join(thread3, NULL);

    return 0;
}

static int test_pwm_gpio()
{
    struct gpio_info {
        uint32_t period;
        uint32_t duty_cycle;
    } info = {
        .period = 100000,  // PWM周期，单位微秒
        .duty_cycle = 50000,  // PWM占空比，单位微秒
    };

    struct motor_dev *motor = motor_alloc_pwm("pwm_gpio", 72, &info);
    if (!motor) {
        printf("错误：电机分配失败\n");
        return -1;
    }

    if (motor_init_one(motor) < 0) {
        printf("错误：电机初始化失败\n");
        motor_free(&motor, 1);
        return -1;
    }

    /*run motor*/
    struct motor_cmd cmd = {
        .mode = MOTOR_MODE_VEL,
        .vel_des = 50,  // 期望速度,0~100
    };

    motor_set_cmd_one(motor, &cmd);
    usleep(2000000);


    cmd.vel_des = 100;
    motor_set_cmd_one(motor, &cmd);
    usleep(2000000);


    cmd.vel_des = 50;
    motor_set_cmd_one(motor, &cmd);
    usleep(2000000);

    cmd.mode = MOTOR_MODE_IDLE;
    motor_set_cmd_one(motor, &cmd);
    usleep(4000000);

    cmd.mode = MOTOR_MODE_VEL;
    cmd.vel_des = 100;
    motor_set_cmd_one(motor, &cmd);
    usleep(2000000);


    cmd.vel_des = 50;
    motor_set_cmd_one(motor, &cmd);
    usleep(2000000);

    cmd.vel_des = 10;
    motor_set_cmd_one(motor, &cmd);
    usleep(2000000);

    cmd.vel_des = 0;
    motor_set_cmd_one(motor, &cmd);
    usleep(2000000);

    motor_free(&motor, 1);

    return 0;
}

int main(void)
{
    int ret = 0;

    // ret = test_pwm_generic();
    ret = test_pwm_gpio();
    ret = run_one_motor_test(1, 46, 43, 42, 83, 60, 2);
    ret = run_one_motor_test(2, 34, 35, 36, 82, 60, 2);
    ret = run_one_motor_test(3, 37, 38, 39, 61, 60, 2);

    ret = run_multi_motor_test();

    return ret;
}
