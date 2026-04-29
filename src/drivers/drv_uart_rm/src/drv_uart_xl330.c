#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "motor_core.h"

/* --- Rust C-API Forward Declarations --- */
extern int reachy_motor_init(struct motor_dev *dev);
extern int reachy_motor_set_cmd(struct motor_dev *dev,
    const struct motor_cmd *cmd);
extern int reachy_motor_get_state(struct motor_dev *dev,
    struct motor_state *state);
extern void reachy_motor_free(struct motor_dev *dev);

/* --- Cleanup implemented in C to free local allocations --- */
void reachy_motor_free_wrapper(struct motor_dev *dev) {
    if (dev) {
        reachy_motor_free(dev);  // Call Rust cleanup (disable torque)
        if (dev->priv_data) {
            free(dev->priv_data);
        }
        free(dev);
    }
}

/* --- Motor Operations Table --- */
static const struct motor_ops reachy_ops = {
    .init = reachy_motor_init,
    .set_cmd = reachy_motor_set_cmd,
    .get_state = reachy_motor_get_state,
    .free = reachy_motor_free_wrapper,
};

/* --- Private Data Structure --- */
struct reachy_priv {
    uint8_t id;
    char dev_path[64];
};

/* --- Factory Function --- */
static struct motor_dev *reachy_motor_factory(void *args) {
    struct motor_args_uart *u_args = (struct motor_args_uart *)args;

    struct motor_dev *dev = malloc(sizeof(struct motor_dev));
    if (!dev) {
        return NULL;
    }

    struct reachy_priv *priv = malloc(sizeof(struct reachy_priv));
    if (!priv) {
        free(dev);
        return NULL;
    }

    dev->name = "drv_uart_rm";
    dev->ops = &reachy_ops;

    priv->id = u_args->id;
    snprintf(priv->dev_path, sizeof(priv->dev_path), "%s", u_args->dev_path);
    dev->priv_data = priv;

    printf("[ReachyMini Driver] Allocated motor ID: %d on %s\n", priv->id,
        priv->dev_path);
    return dev;
}

/* --- Registration --- */
REGISTER_MOTOR_DRIVER("drv_uart_rm", DRV_TYPE_UART, reachy_motor_factory);
