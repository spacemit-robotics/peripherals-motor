#include "motor.h"
#include "motor_core.h"
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

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

    if (motor_init_one(dev) != 0) {
        printf("Failed to init motor\n");
        return -1;
    }

    struct motor_cmd cmd = {0};
    struct motor_state state = {0};

    // Make sure we get a baseline position first
    motor_get_state_one(dev, &state);

    // Enable motor by setting it to position mode at current position
    cmd.mode = MOTOR_MODE_POS;
    cmd.pos_des = state.pos;
    motor_set_cmd_one(dev, &cmd);

    // Wait for the motor to be enabled and stabilized
    usleep(500000); // 500ms

    // Trajectory planning: rotate one circle (2 * PI) over 2 seconds
    float start_pos = state.pos;
    float target_pos = start_pos + 2.0 * M_PI;
    int duration_ms = 2000;
    int step_ms = 10;
    int steps = duration_ms / step_ms;

    for (int i = 0; i <= steps; i++) {
        // Linear interpolation
        float current_des = start_pos + (target_pos - start_pos) * i / steps;

        cmd.pos_des = current_des;
        motor_set_cmd_one(dev, &cmd);

        if (motor_get_state_one(dev, &state) == 0) {
            printf("Step %d: des=%.4f pos=%.4f vel=%.4f trq=%.4f\n", i, current_des, state.pos, state.vel, state.trq);
        } else {
            printf("Step %d: Failed to get state\n", i);
        }
        usleep(step_ms * 1000);
    }

    // Wait until the motor actually reaches the target position
    printf("Trajectory finished, waiting for motor to settle...\n");
    int settle_steps = 0;
    while (settle_steps < 500) { // Timeout after 5 seconds of waiting
        motor_set_cmd_one(dev, &cmd);
        if (motor_get_state_one(dev, &state) == 0) {
            printf("Settle %d: des=%.4f pos=%.4f vel=%.4f trq=%.4f\n", settle_steps, target_pos, state.pos, state.vel, state.trq);
            if (fabs(state.pos - target_pos) < 0.05) {
                printf("Target position reached!\n");
                break;
            }
        }
        usleep(step_ms * 1000);
        settle_steps++;
    }

    // Stop motor
    cmd.mode = MOTOR_MODE_IDLE;
    motor_set_cmd_one(dev, &cmd);

    motor_free(&dev, 1);
    return 0;
}
