#include "motor.h"
#include "motor_core.h"
#include <stdio.h>
#include <unistd.h>

int main(int argc, char **argv) {
    if (argc < 2) {
        printf("Usage: %s <can_interface>\n", argv[0]);
        return -1;
    }

    struct motor_dev *dev = motor_alloc_can("drv_can_ddt_p1010b", argv[1], 1, NULL);
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

    // Loop to read state 10 times
    for (int i = 0; i < 10; i++) {
        if (motor_get_state_one(dev, &state) == 0) {
            printf("Read %d: pos=%.4f vel=%.4f trq=%.4f\n", i + 1, state.pos, state.vel, state.trq);
        } else {
            printf("Read %d: Failed to get state\n", i + 1);
        }
        usleep(100000); // 100ms
    }

    // Stop motor
    cmd.mode = MOTOR_MODE_IDLE;
    motor_set_cmd_one(dev, &cmd);

    motor_free(&dev, 1);
    return 0;
}
