/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file drv_can_agibot.c
 * @brief AGIBOT OmniPicker driver over SocketCAN.
 */

#include <errno.h>
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include "agibot_protocol.h"
#include "motor_core.h"

#define AGIBOT_MAX_CAN_ID 0x7ffU
#define AGIBOT_DEFAULT_COMMAND_VALUE 0x7fU

struct agibot_priv {
    char interface[IFNAMSIZ];
    int fd;
    uint16_t command_id;
    struct motor_can_agibot_config config;
};

static int agibot_open_socket(struct agibot_priv *priv) {
    struct sockaddr_can address = {0};
    struct can_filter filter = {0};
    struct ifreq request = {0};
    int flags;
    int error;

    if (!priv) return -EINVAL;
    priv->fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (priv->fd < 0) return -errno;

    strncpy(request.ifr_name, priv->interface, IFNAMSIZ - 1);
    if (ioctl(priv->fd, SIOCGIFINDEX, &request) < 0) goto fail;
    filter.can_id = priv->config.feedback_id;
    filter.can_mask = CAN_SFF_MASK;
    if (setsockopt(priv->fd, SOL_CAN_RAW, CAN_RAW_FILTER,
            &filter, sizeof(filter)) < 0) {
        goto fail;
    }
    address.can_family = AF_CAN;
    address.can_ifindex = request.ifr_ifindex;
    if (bind(priv->fd, (struct sockaddr *)&address, sizeof(address)) < 0) goto fail;
    flags = fcntl(priv->fd, F_GETFL, 0);
    if (flags < 0 || fcntl(priv->fd, F_SETFL, flags | O_NONBLOCK) < 0) goto fail;
    return 0;

fail:
    error = errno;
    close(priv->fd);
    priv->fd = -1;
    return -error;
}

static int agibot_init(struct motor_dev *dev) {
    struct agibot_priv *priv = dev ? dev->priv_data : NULL;

    if (!priv) return -EINVAL;
    if (priv->fd >= 0) return 0;
    return agibot_open_socket(priv);
}

static int agibot_set_cmd(struct motor_dev *dev, const struct motor_cmd *command) {
    struct agibot_priv *priv = dev ? dev->priv_data : NULL;
    struct can_frame frame = {0};

    if (!priv || !command || priv->fd < 0) return -EINVAL;
    if (command->mode == MOTOR_MODE_IDLE) return -EOPNOTSUPP;
    if (command->mode != MOTOR_MODE_POS) return -EOPNOTSUPP;
    if (agibot_encode_position(command->pos_des, &priv->config.command, frame.data) < 0)
        return -ERANGE;

    frame.can_id = priv->command_id;
    frame.can_dlc = AGIBOT_CAN_FRAME_SIZE;
    if (write(priv->fd, &frame, sizeof(frame)) == (ssize_t)sizeof(frame)) return 0;
    return errno ? -errno : -EIO;
}

static int agibot_get_state(struct motor_dev *dev, struct motor_state *state) {
    struct agibot_priv *priv = dev ? dev->priv_data : NULL;
    struct can_frame frame;
    struct can_frame latest;
    ssize_t size;
    int found = 0;

    if (!priv || !state || priv->fd < 0) return -EINVAL;
    for (;;) {
        size = read(priv->fd, &frame, sizeof(frame));
        if (size == (ssize_t)sizeof(frame)) {
            if ((frame.can_id & CAN_SFF_MASK) == priv->config.feedback_id &&
                frame.can_dlc == AGIBOT_CAN_FRAME_SIZE) {
                latest = frame;
                found = 1;
            }
            continue;
        }
        if (size < 0 && errno != EAGAIN && errno != EWOULDBLOCK) return -errno;
        break;
    }
    if (!found) return -EAGAIN;
    return agibot_decode_feedback(latest.data, state, NULL);
}

static void agibot_free(struct motor_dev *dev) {
    struct agibot_priv *priv;

    if (!dev) return;
    priv = dev->priv_data;
    if (priv) {
        if (priv->fd >= 0) close(priv->fd);
        free(priv);
    }
    free(dev);
}

static const struct motor_ops kAgibotOps = {
    .init = agibot_init,
    .set_cmd = agibot_set_cmd,
    .get_state = agibot_get_state,
    .free = agibot_free,
};

static int read_byte_option(const struct motor_args_can *args,
    const char *name, uint8_t *value) {
    uint32_t parsed = *value;
    const int result = motor_option_read_u32(
        args->options, args->option_count, name, &parsed);

    if (result < 0 || parsed > UINT8_MAX) return -1;
    *value = (uint8_t)parsed;
    return 0;
}

static int parse_agibot_options(const struct motor_args_can *args,
    struct motor_can_agibot_config *config) {
    static const char *const kAllowedOptions[] = {
        "model",
        "feedback_id",
        "force",
        "velocity",
        "acceleration",
        "deceleration",
    };
    const char *model;
    uint32_t feedback_id;
    int feedback_result;

    if (!args || !config ||
        motor_options_validate(args->options, args->option_count, kAllowedOptions,
            sizeof(kAllowedOptions) / sizeof(kAllowedOptions[0])) != 0) {
        return -1;
    }

    feedback_id = args->can_id;

    model = motor_option_find(args->options, args->option_count, "model");
    if (model && strcmp(model, "AGIBOT OmniPicker") != 0 &&
        strcmp(model, "AGIBOT") != 0) {
        return -1;
    }
    feedback_result = motor_option_read_u32(
        args->options, args->option_count, "feedback_id", &feedback_id);
    if (feedback_result < 0 || feedback_id > AGIBOT_MAX_CAN_ID) return -1;
    config->feedback_id = (uint16_t)feedback_id;
    if (read_byte_option(args, "force", &config->command.force) < 0 ||
        read_byte_option(args, "velocity", &config->command.velocity) < 0 ||
        read_byte_option(args, "acceleration", &config->command.acceleration) < 0 ||
        read_byte_option(args, "deceleration", &config->command.deceleration) < 0) {
        return -1;
    }
    return 0;
}

static struct motor_dev *agibot_probe(void *args) {
    const struct motor_args_can *can_args = args;
    struct motor_can_agibot_config config = {
        .command = {
            .force = AGIBOT_DEFAULT_COMMAND_VALUE,
            .velocity = AGIBOT_DEFAULT_COMMAND_VALUE,
            .acceleration = AGIBOT_DEFAULT_COMMAND_VALUE,
            .deceleration = AGIBOT_DEFAULT_COMMAND_VALUE,
        },
    };
    struct motor_dev *dev;
    struct agibot_priv *priv;

    if (!can_args || !can_args->iface || can_args->can_id > AGIBOT_MAX_CAN_ID)
        return NULL;
    config.feedback_id = (uint16_t)can_args->can_id;
    if (can_args->options) {
        if (parse_agibot_options(can_args, &config) < 0) return NULL;
    } else if (can_args->args) {
        config = *(const struct motor_can_agibot_config *)can_args->args;
        if (config.feedback_id > AGIBOT_MAX_CAN_ID) return NULL;
    }

    dev = calloc(1, sizeof(*dev));
    priv = calloc(1, sizeof(*priv));
    if (!dev || !priv) {
        free(priv);
        free(dev);
        return NULL;
    }
    strncpy(priv->interface, can_args->iface, IFNAMSIZ - 1);
    priv->fd = -1;
    priv->command_id = (uint16_t)can_args->can_id;
    priv->config = config;
    dev->name = "drv_can_agibot";
    dev->ops = &kAgibotOps;
    dev->priv_data = priv;
    return dev;
}

REGISTER_MOTOR_DRIVER("drv_can_agibot", DRV_TYPE_CAN, agibot_probe);
