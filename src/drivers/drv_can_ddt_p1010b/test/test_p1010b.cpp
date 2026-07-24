#include "motor.h"
#include "motor_core.h"
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <stdlib.h>
#include <math.h>


static struct motor_dev *g_dev = NULL;

static void sigint_handler(int sig) {
    printf("\nCaught signal %d, stopping motor safely...\n", sig);
    if (g_dev) {
        struct motor_cmd cmd = {0};
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

    // Get baseline state
    motor_get_state_one(dev, &state);
    float start_pos = state.pos;

    // Spin in current loop mode (safe torque) to rotate a full circle
    cmd.mode = MOTOR_MODE_TRQ;
    cmd.trq_des = 1.0f; // Reduced to 1.0A for slower acceleration

    for (int i = 0; i < 1000; i++) {
        motor_set_cmd_one(dev, &cmd);
        motor_get_state_one(dev, &state);
        if (i % 10 == 0) {
            printf("Active Query State: pos=%.4f vel=%.4f trq=%.4f\n", state.pos, state.vel, state.trq);
        }

        float delta = state.pos - start_pos;
        if (delta < 0) delta = -delta; // fabs

        if (delta >= 6.28318f) { // 2 * PI
            printf("Rotated a full circle!\n");
            break;
        }
        usleep(10000); // 100Hz query
    }

    // Stop
    cmd.mode = MOTOR_MODE_IDLE;
    motor_set_cmd_one(dev, &cmd);

    printf("\nMotor set to IDLE. Continuing to monitor coasting (inertia) for 5 seconds...\n");
    for (int i = 0; i < 500; i++) {
        motor_get_state_one(dev, &state);
        if (i % 50 == 0) {
            printf("Coasting State: pos=%.4f vel=%.4f\n", state.pos, state.vel);
        }
        usleep(10000); // 100Hz query
    }

    /*
    // Spin in voltage open-loop mode (safe voltage) to rotate a full circle
    printf("\nStarting Voltage Open-Loop Mode verification...\n");

    cmd.mode = MOTOR_MODE_OPEN;
    cmd.vel_des = 2.0f; // 2.0V is a safe voltage to overcome friction
    motor_set_cmd_one(dev, &cmd);
    usleep(50000); // Wait 50ms for mode switch to take effect

    // Verify mode switch
    uint8_t mode_code = 0x09;
    uint16_t actual_mode = 0xFFFF;
    if (motor_get_paras(dev, &mode_code, &actual_mode, 2) == 0) {
        printf("Current Actual Mode queried: %u (expected 0 for Voltage Open-Loop)\n", actual_mode);
    } else {
        printf("Failed to query actual mode.\n");
    }

    motor_get_state_one(dev, &state);
    start_pos = state.pos;

    uint8_t abs_pos_code = 0x0D;

    for (int i = 0; i < 1000; i++) { // Run until 1 circle or timeout
        motor_set_cmd_one(dev, &cmd);
        motor_get_state_one(dev, &state);
        
        if (i % 10 == 0) {
            uint16_t actual_abs_pos = 0;
            if (motor_get_paras(dev, &abs_pos_code, &actual_abs_pos, 2) == 0) {
                printf("Active Query State (Voltage): pos=%.4f vel=%.4f trq=%.4f | Abs Pos Raw(0x0D): %u\n", 
                       state.pos, state.vel, state.trq, actual_abs_pos);
            } else {
                printf("Active Query State (Voltage): pos=%.4f vel=%.4f trq=%.4f | Failed to read Abs Pos\n", 
                       state.pos, state.vel, state.trq);
            }
        }
        
        float delta = state.pos - start_pos;
        if (delta < 0) delta = -delta; // fabs

        if (delta >= 6.28318f) { // 2 * PI
            printf("Voltage open-loop rotated a full circle! Releasing motor.\n");
            break;
        }
        usleep(10000); // 100Hz query
    }

    // Stop again
    cmd.mode = MOTOR_MODE_IDLE;
    motor_set_cmd_one(dev, &cmd);
    */

    /*
    printf("\nStarting Position Loop Mode verification...\n");

    motor_get_state_one(dev, &state);
    start_pos = state.pos;
    
    cmd.mode = MOTOR_MODE_POS;
    cmd.pos_des = start_pos;
    motor_set_cmd_one(dev, &cmd);
    usleep(50000); // Wait 50ms for mode switch to take effect

    // Verify mode switch
    uint8_t mode_code = 0x09;
    uint16_t actual_mode = 0xFFFF;
    if (motor_get_paras(dev, &mode_code, &actual_mode, 2) == 0) {
        printf("Current Actual Mode queried: %u (expected 4 for Position Loop)\n", actual_mode);
    } else {
        printf("Failed to query actual mode.\n");
    }

    float target_pos = start_pos + 6.28318f;
    uint8_t abs_pos_code = 0x0D;

    int duration_ms = 2000;
    int step_ms = 10;
    int steps = duration_ms / step_ms;

    for (int i = 0; i <= steps; i++) {
        cmd.pos_des = start_pos + (target_pos - start_pos) * i / steps;
        motor_set_cmd_one(dev, &cmd);
        motor_get_state_one(dev, &state);
        
        if (i % 20 == 0) {
            uint16_t actual_abs_pos = 0;
            if (motor_get_paras(dev, &abs_pos_code, &actual_abs_pos, 2) == 0) {
                printf("Active Query State (Position): des=%.4f pos=%.4f vel=%.4f | Abs Pos Raw(0x0D): %u\n", 
                       cmd.pos_des, state.pos, state.vel, actual_abs_pos);
            }
        }
        usleep(step_ms * 1000);
    }

    printf("Trajectory finished, waiting for motor to settle...\n");
    int settle_steps = 0;
    while (settle_steps < 500) {
        motor_set_cmd_one(dev, &cmd);
        if (motor_get_state_one(dev, &state) == 0) {
            if (settle_steps % 20 == 0) {
                uint16_t actual_abs_pos = 0;
                motor_get_paras(dev, &abs_pos_code, &actual_abs_pos, 2);
                printf("Settle %d: des=%.4f pos=%.4f vel=%.4f trq=%.4f | Abs Pos Raw: %u\n", 
                       settle_steps, target_pos, state.pos, state.vel, state.trq, actual_abs_pos);
            }
            if (fabs(state.pos - target_pos) < 0.05) {
                printf("Position loop rotated a full circle and settled! Releasing motor.\n");
                break;
            }
        }
        usleep(step_ms * 1000);
        settle_steps++;
    }

    // Stop again
    cmd.mode = MOTOR_MODE_IDLE;
    motor_set_cmd_one(dev, &cmd);
    */

    /*
    printf("\nStarting Velocity Loop Mode verification...\n");

    motor_get_state_one(dev, &state);
    start_pos = state.pos;
    
    cmd.mode = MOTOR_MODE_VEL;
    cmd.vel_des = 1.0f; // 1 rad/s
    motor_set_cmd_one(dev, &cmd);
    usleep(50000); // Wait 50ms for mode switch to take effect

    // Verify mode switch
    uint8_t mode_code = 0x09;
    uint16_t actual_mode = 0xFFFF;
    if (motor_get_paras(dev, &mode_code, &actual_mode, 2) == 0) {
        printf("Current Actual Mode queried: %u (expected 3 for Velocity Loop)\n", actual_mode);
    } else {
        printf("Failed to query actual mode.\n");
    }

    uint8_t abs_pos_code = 0x0D;

    for (int i = 0; i < 2000; i++) { // Run until 1 circle or timeout
        motor_set_cmd_one(dev, &cmd);
        motor_get_state_one(dev, &state);
        
        if (i % 20 == 0) {
            uint16_t actual_abs_pos = 0;
            if (motor_get_paras(dev, &abs_pos_code, &actual_abs_pos, 2) == 0) {
                printf("Active Query State (Velocity): pos=%.4f vel=%.4f trq=%.4f | Abs Pos Raw(0x0D): %u\n", 
                       state.pos, state.vel, state.trq, actual_abs_pos);
            }
        }
        
        float delta = state.pos - start_pos;
        if (delta < 0) delta = -delta; // fabs

        if (delta >= 6.28318f) { // 2 * PI
            printf("Velocity loop rotated a full circle! Releasing motor.\n");
            break;
        }
        usleep(10000); // 100Hz query
    }

    // Stop again
    cmd.mode = MOTOR_MODE_IDLE;
    motor_set_cmd_one(dev, &cmd);
    */

    motor_free(&dev, 1);
    g_dev = NULL;
    return 0;
}
