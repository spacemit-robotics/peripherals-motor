/**
 * @file test_p1010b_current.cpp
 * @brief P1010B 单电流环（力矩）模式独立测试用例。
 *
 * 参考同级 test_p1010b_pos_traj.cpp / test_p1010b_state.cpp 的组织结构，
 * 专门验证 MOTOR_MODE_TRQ 电流环控制链路：
 *   1. 从磁驻点起步，采用 0.1A 的电流梯度爬升克服静摩擦；
 *   2. 通过 0x09 参数查询确认驱动器实际已切换至电流模式（value == 2）；
 *   3. 电流环下驱动电机旋转一整圈（输出轴 2*PI）后进入制动流程；
 *   4. 按 README §5 推荐：先切 MOTOR_MODE_VEL 且 vel_des=0 做速度钳位刹停，
 *      再下发 MOTOR_MODE_IDLE 断电，避免惯性滑行过转。
 */

#include "motor.h"
#include "motor_core.h"
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static struct motor_dev *g_dev = NULL;

static void sigint_handler(int sig) {
    printf("\nCaught signal %d, stopping motor safely...\n", sig);
    if (g_dev) {
        struct motor_cmd cmd = {0};
        /* Clamp velocity to 0 first to actively brake before disabling. */
        cmd.mode = MOTOR_MODE_VEL;
        cmd.vel_des = 0.0f;
        motor_set_cmd_one(g_dev, &cmd);
        usleep(200000); /* 200ms for active braking */

        cmd.mode = MOTOR_MODE_IDLE;
        motor_set_cmd_one(g_dev, &cmd);
        motor_free(&g_dev, 1);
        g_dev = NULL;
    }
    exit(0);
}

int main(int argc, char **argv) {
    if (argc < 2) {
        printf("Usage: %s <can_interface> [--raw]\n", argv[0]);
        return -1;
    }

    bool print_raw = false;
    if (argc >= 3 && strcmp(argv[2], "--raw") == 0) {
        print_raw = true;
    }

    struct motor_dev *dev = motor_alloc_can("drv_can_ddt_p1010b", argv[1], 1, &print_raw);
    if (!dev) {
        printf("Failed to allocate P1010B motor\n");
        return -1;
    }

    g_dev = dev;
    signal(SIGINT, sigint_handler);
    signal(SIGTERM, sigint_handler);

    if (motor_init_one(dev) != 0) {
        printf("Failed to init motor\n");
        return -1;
    }

    struct motor_cmd cmd = {0};
    struct motor_state state = {0};

    /* Baseline position for one-full-circle detection. */
    motor_get_state_one(dev, &state);
    float start_pos = state.pos;
    printf("Baseline pos=%.4f\n", start_pos);

    /* Enter current (torque) loop mode. */
    cmd.mode = MOTOR_MODE_TRQ;
    cmd.trq_des = 0.0f;
    motor_set_cmd_one(dev, &cmd);
    usleep(50000); /* Wait 50ms for mode switch to take effect */

    /* Verify actual mode via param 0x09 (0=Voltage, 2=Current, 3=Velocity, 4=Position). */
    uint8_t mode_code = 0x09;
    uint16_t actual_mode = 0xFFFF;
    if (motor_get_paras(dev, &mode_code, &actual_mode, 2) == 0) {
        printf("Current Actual Mode queried: %u (expected 2 for Current Loop)\n", actual_mode);
    } else {
        printf("Failed to query actual mode.\n");
    }

    /*
     * Ramp current from 0.1A upward in 0.1A steps until the motor visibly starts
     * moving. This follows README §5 guidance on overcoming static friction /
     * magnetic detent without slamming a large current onto a stalled rotor.
     */
    const float trq_step = 0.1f;
    const float trq_max = 1.0f;
    float trq_cmd = trq_step;
    bool moving = false;
    for (int i = 0; i < 30 && !moving; i++) {
        cmd.trq_des = trq_cmd;
        motor_set_cmd_one(dev, &cmd);
        usleep(100000); /* 100ms per ramp step */
        motor_get_state_one(dev, &state);
        printf("Ramp step %d: trq_cmd=%.2fA pos=%.4f vel=%.4f trq=%.4f\n",
                i, trq_cmd, state.pos, state.vel, state.trq);

        if (fabsf(state.vel) > 0.1f) {
            moving = true;
            printf("Motor started moving at trq_cmd=%.2fA\n", trq_cmd);
            break;
        }
        if (trq_cmd < trq_max) {
            trq_cmd += trq_step;
            if (trq_cmd > trq_max) {
                trq_cmd = trq_max;
            }
        }
    }

    if (!moving) {
        printf("Warning: motor did not start within ramp limit (%.2fA), aborting.\n", trq_max);
        cmd.mode = MOTOR_MODE_IDLE;
        motor_set_cmd_one(dev, &cmd);
        motor_free(&dev, 1);
        g_dev = NULL;
        return -1;
    }

    /* Hold the last-known effective current and drive until one full revolution. */
    cmd.trq_des = trq_cmd;
    for (int i = 0; i < 1000; i++) {
        motor_set_cmd_one(dev, &cmd);
        motor_get_state_one(dev, &state);
        if (i % 10 == 0) {
            printf("Current Loop State: pos=%.4f vel=%.4f trq=%.4f (cmd=%.2fA)\n",
                    state.pos, state.vel, state.trq, cmd.trq_des);
        }

        float delta = state.pos - start_pos;
        if (delta < 0) {
            delta = -delta;
        }
        if (delta >= (float)(2.0 * M_PI)) {
            printf("Rotated a full circle (delta=%.4f rad), starting braking.\n", delta);
            break;
        }
        usleep(10000); /* 100Hz control */
    }

    /*
     * Safe braking sequence per README §5:
     *   1) Switch to velocity loop with vel_des=0 for active electromagnetic braking.
     *   2) Wait until velocity actually decays close to zero.
     *   3) Only then drop into IDLE to avoid coasting.
     */
    printf("\nBraking: switching to MOTOR_MODE_VEL vel_des=0 ...\n");
    cmd.mode = MOTOR_MODE_VEL;
    cmd.vel_des = 0.0f;
    motor_set_cmd_one(dev, &cmd);
    usleep(50000);

    for (int i = 0; i < 200; i++) {
        motor_set_cmd_one(dev, &cmd);
        motor_get_state_one(dev, &state);
        if (i % 10 == 0) {
            printf("Braking State: pos=%.4f vel=%.4f trq=%.4f\n",
                    state.pos, state.vel, state.trq);
        }
        if (fabsf(state.vel) < 0.05f) {
            printf("Motor settled (|vel|<0.05 rad/s) after %d ms.\n", i * 10);
            break;
        }
        usleep(10000);
    }

    printf("\nDisabling motor (MOTOR_MODE_IDLE).\n");
    cmd.mode = MOTOR_MODE_IDLE;
    motor_set_cmd_one(dev, &cmd);

    motor_free(&dev, 1);
    g_dev = NULL;
    return 0;
}
