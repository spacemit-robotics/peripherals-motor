/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file drv_can_encos.c
 * @brief Encos motor driver over shared SocketCAN buses
 */

#include <errno.h>
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <math.h>
#include <net/if.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include "encos_protocol.h"
#include "motor_core.h"

#define ENCOS_MAX_CAN_ID 0x7ffU
#define ENCOS_DEFAULT_TIMEOUT_MS 500U

struct encos_bus {
    char interface[IFNAMSIZ];
    int fd;
    uint32_t references;
    struct can_frame frames[ENCOS_MAX_CAN_ID + 1U];
    uint64_t frame_timestamp_us[ENCOS_MAX_CAN_ID + 1U];
    bool frame_valid[ENCOS_MAX_CAN_ID + 1U];
    pthread_mutex_t mutex;
    struct encos_bus *next;
};

struct encos_priv {
    struct encos_bus *bus;
    struct motor_can_encos_config config;
    uint16_t command_id;
    bool initialized;
    bool enabled;
};

struct encos_model_profile {
    const char *name;
    struct encos_protocol_limits limits;
};

static const struct encos_model_profile kModelProfiles[] = {
    {
        .name = "Encos EC-A10020-P2-24",
        .limits = {-12.5f, 12.5f, -18.0f, 18.0f, -300.0f, 300.0f,
            0.0f, 500.0f, 0.0f, 50.0f},
    },
    {
        .name = "Encos EC-A8116-P1-18",
        .limits = {-12.5f, 12.5f, -18.0f, 18.0f, -150.0f, 150.0f,
            0.0f, 500.0f, 0.0f, 5.0f},
    },
    {
        .name = "Encos EC-A10020-P1-12",
        .limits = {-12.5f, 12.5f, -18.0f, 18.0f, -150.0f, 150.0f,
            0.0f, 500.0f, 0.0f, 50.0f},
    },
    {
        .name = "Encos EC-A8112-P1-18",
        .limits = {-12.5f, 12.5f, -18.0f, 18.0f, -90.0f, 90.0f,
            0.0f, 500.0f, 0.0f, 5.0f},
    },
};

static const uint8_t kEnableCommand[3] = {0x71, 0x03, 0xe8};
static const uint8_t kDisableCommand[3] = {0x6d, 0x00, 0x00};
static struct encos_bus *g_buses;
static pthread_mutex_t g_buses_mutex = PTHREAD_MUTEX_INITIALIZER;

static uint64_t monotonic_time_us(void) {
    struct timespec time;

    if (clock_gettime(CLOCK_MONOTONIC, &time) != 0) return 0;
    return (uint64_t)time.tv_sec * 1000000ULL +
        (uint64_t)time.tv_nsec / 1000ULL;
}

static int get_model_limits(const char *model, struct encos_protocol_limits *limits) {
    size_t index;

    if (!model || !limits) return -1;
    for (index = 0; index < sizeof(kModelProfiles) / sizeof(kModelProfiles[0]); ++index) {
        if (strcmp(model, kModelProfiles[index].name) == 0) {
            *limits = kModelProfiles[index].limits;
            return 0;
        }
    }
    return -1;
}

static struct encos_bus *acquire_bus(const char *interface) {
    struct encos_bus *bus;

    pthread_mutex_lock(&g_buses_mutex);
    for (bus = g_buses; bus; bus = bus->next) {
        if (strcmp(bus->interface, interface) == 0) {
            bus->references++;
            pthread_mutex_unlock(&g_buses_mutex);
            return bus;
        }
    }

    bus = calloc(1, sizeof(*bus));
    if (bus) {
        snprintf(bus->interface, sizeof(bus->interface), "%s", interface);
        bus->fd = -1;
        bus->references = 1;
        pthread_mutex_init(&bus->mutex, NULL);
        bus->next = g_buses;
        g_buses = bus;
    }
    pthread_mutex_unlock(&g_buses_mutex);
    return bus;
}

static void release_bus(struct encos_bus *bus) {
    struct encos_bus **current;

    if (!bus) return;
    pthread_mutex_lock(&g_buses_mutex);
    if (--bus->references > 0) {
        pthread_mutex_unlock(&g_buses_mutex);
        return;
    }
    current = &g_buses;
    while (*current && *current != bus) current = &(*current)->next;
    if (*current) *current = bus->next;
    pthread_mutex_unlock(&g_buses_mutex);

    if (bus->fd >= 0) close(bus->fd);
    pthread_mutex_destroy(&bus->mutex);
    free(bus);
}

static int open_bus(struct encos_bus *bus) {
    struct sockaddr_can address;
    struct ifreq request;
    int flags;

    pthread_mutex_lock(&bus->mutex);
    if (bus->fd >= 0) {
        pthread_mutex_unlock(&bus->mutex);
        return 0;
    }

    bus->fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (bus->fd < 0) goto fail;
    memset(&request, 0, sizeof(request));
    snprintf(request.ifr_name, sizeof(request.ifr_name), "%s", bus->interface);
    if (ioctl(bus->fd, SIOCGIFINDEX, &request) < 0) goto fail;

    memset(&address, 0, sizeof(address));
    address.can_family = AF_CAN;
    address.can_ifindex = request.ifr_ifindex;
    if (bind(bus->fd, (struct sockaddr *)&address, sizeof(address)) < 0) goto fail;
    flags = fcntl(bus->fd, F_GETFL, 0);
    if (flags < 0 || fcntl(bus->fd, F_SETFL, flags | O_NONBLOCK) < 0) goto fail;
    pthread_mutex_unlock(&bus->mutex);
    return 0;

fail:
    if (bus->fd >= 0) close(bus->fd);
    bus->fd = -1;
    pthread_mutex_unlock(&bus->mutex);
    return -1;
}

static int write_frame(struct encos_bus *bus, uint16_t can_id, const uint8_t *data, uint8_t size) {
    struct can_frame frame = {0};
    ssize_t written;
    int write_errno;

    if (!bus || !data || size > CAN_MAX_DLEN) {
        errno = EINVAL;
        return -1;
    }
    if (bus->fd < 0) {
        errno = EBADF;
        return -1;
    }
    frame.can_id = can_id;
    frame.can_dlc = size;
    memcpy(frame.data, data, size);

    pthread_mutex_lock(&bus->mutex);
    written = write(bus->fd, &frame, sizeof(frame));
    write_errno = written < 0 ? errno : EIO;
    pthread_mutex_unlock(&bus->mutex);
    if (written == (ssize_t)sizeof(frame)) return 0;
    errno = write_errno;
    return -1;
}

static int drain_bus(struct encos_bus *bus) {
    struct can_frame frame;
    ssize_t size;

    if (!bus || bus->fd < 0) return -1;
    pthread_mutex_lock(&bus->mutex);
    for (;;) {
        size = read(bus->fd, &frame, sizeof(frame));
        if (size == (ssize_t)sizeof(frame)) {
            const canid_t unsupported_flags = CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG;
            const uint32_t can_id = frame.can_id & CAN_SFF_MASK;
            if ((frame.can_id & unsupported_flags) == 0U &&
                can_id <= ENCOS_MAX_CAN_ID && frame.can_dlc == CAN_MAX_DLEN) {
                bus->frames[can_id] = frame;
                bus->frame_timestamp_us[can_id] = monotonic_time_us();
                bus->frame_valid[can_id] = true;
            }
            continue;
        }
        if (size < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            pthread_mutex_unlock(&bus->mutex);
            return -1;
        }
        break;
    }
    pthread_mutex_unlock(&bus->mutex);
    return 0;
}

static int encos_init(struct motor_dev *dev) {
    struct encos_priv *priv = dev ? dev->priv_data : NULL;
    uint8_t timeout_data[ENCOS_TIMEOUT_CONFIG_FRAME_SIZE];

    if (!priv || open_bus(priv->bus) < 0) return -1;
    if (priv->config.can_timeout_ms > UINT16_MAX ||
        encos_encode_timeout_config((uint16_t)priv->config.can_timeout_ms, timeout_data) < 0 ||
        write_frame(priv->bus, priv->command_id, timeout_data, sizeof(timeout_data)) < 0) {
        return -1;
    }
    priv->initialized = true;
    if (priv->config.enable_on_init) {
        if (write_frame(priv->bus, priv->command_id, kEnableCommand, sizeof(kEnableCommand)) < 0)
            return -1;
        priv->enabled = true;
    }
    return 0;
}

static int encos_set_cmd(struct motor_dev *dev, const struct motor_cmd *cmd) {
    struct encos_priv *priv = dev ? dev->priv_data : NULL;
    struct motor_cmd protocol_cmd;
    uint8_t data[ENCOS_COMMAND_FRAME_SIZE];
    int command_errno;

    if (!priv || !cmd || !priv->initialized) {
        errno = EINVAL;
        return -1;
    }
    if (cmd->mode == MOTOR_MODE_IDLE) {
        if (write_frame(priv->bus, priv->command_id, kDisableCommand, sizeof(kDisableCommand)) < 0)
            return -1;
        priv->enabled = false;
        return 0;
    }
    if (cmd->mode != MOTOR_MODE_HYBRID && cmd->mode != MOTOR_MODE_POS &&
        cmd->mode != MOTOR_MODE_VEL && cmd->mode != MOTOR_MODE_TRQ) {
        errno = EOPNOTSUPP;
        return -1;
    }

    protocol_cmd = *cmd;
    if (cmd->mode == MOTOR_MODE_VEL) {
        protocol_cmd.pos_des = 0.0f;
        protocol_cmd.trq_des = 0.0f;
        protocol_cmd.kp = 0.0f;
    } else if (cmd->mode == MOTOR_MODE_TRQ) {
        protocol_cmd.pos_des = 0.0f;
        protocol_cmd.vel_des = 0.0f;
        protocol_cmd.kp = 0.0f;
        protocol_cmd.kd = 0.0f;
    }
    if (encos_encode_command(&priv->config.limits, &protocol_cmd, data) < 0) {
        errno = ERANGE;
        return -1;
    }
    if (!priv->enabled) {
        if (write_frame(priv->bus, priv->command_id, kEnableCommand, sizeof(kEnableCommand)) < 0)
            return -1;
        priv->enabled = true;
    }
    if (write_frame(priv->bus, priv->command_id, data, sizeof(data)) == 0) return 0;
    command_errno = errno;
    if (write_frame(priv->bus, priv->command_id,
            kDisableCommand, sizeof(kDisableCommand)) == 0) {
        priv->enabled = false;
    }
    errno = command_errno;
    return -1;
}

static int encos_get_state(struct motor_dev *dev, struct motor_state *state) {
    struct encos_priv *priv = dev ? dev->priv_data : NULL;
    struct can_frame frame;
    int result;

    if (!priv || !state || !priv->initialized || drain_bus(priv->bus) < 0) return -1;
    pthread_mutex_lock(&priv->bus->mutex);
    if (!priv->bus->frame_valid[priv->config.feedback_id]) {
        pthread_mutex_unlock(&priv->bus->mutex);
        return -1;
    }
    frame = priv->bus->frames[priv->config.feedback_id];
    dev->feedback_timestamp_us =
        priv->bus->frame_timestamp_us[priv->config.feedback_id];
    priv->bus->frame_valid[priv->config.feedback_id] = false;
    pthread_mutex_unlock(&priv->bus->mutex);
    result = encos_decode_feedback(&priv->config.limits, frame.data, state);
    return result;
}

static void encos_free(struct motor_dev *dev) {
    struct encos_priv *priv;

    if (!dev) return;
    priv = dev->priv_data;
    if (priv) {
        if (priv->initialized)
            write_frame(priv->bus, priv->command_id, kDisableCommand, sizeof(kDisableCommand));
        release_bus(priv->bus);
        free(priv);
    }
    free(dev);
}

static const struct motor_ops kEncosOps = {
    .init = encos_init,
    .set_cmd = encos_set_cmd,
    .get_state = encos_get_state,
    .free = encos_free,
};

static bool encos_range_is_valid(float min, float max) {
    return isfinite(min) && isfinite(max) && min < max;
}

static bool encos_limits_are_valid(const struct encos_protocol_limits *limits) {
    return limits &&
        encos_range_is_valid(limits->position_min, limits->position_max) &&
        encos_range_is_valid(limits->velocity_min, limits->velocity_max) &&
        encos_range_is_valid(limits->torque_min, limits->torque_max) &&
        encos_range_is_valid(limits->kp_min, limits->kp_max) &&
        encos_range_is_valid(limits->kd_min, limits->kd_max);
}

static int parse_encos_options(const struct motor_args_can *can_args,
    struct motor_can_encos_config *config) {
    static const char *const allowed_options[] = {
        "model",
        "feedback_id",
        "can_timeout_ms",
        "enable_on_init",
        "protocol_limits.position.0",
        "protocol_limits.position.1",
        "protocol_limits.velocity.0",
        "protocol_limits.velocity.1",
        "protocol_limits.torque.0",
        "protocol_limits.torque.1",
        "protocol_limits.kp.0",
        "protocol_limits.kp.1",
        "protocol_limits.kd.0",
        "protocol_limits.kd.1",
    };
    static const char *const limit_names[] = {
        "protocol_limits.position.0",
        "protocol_limits.position.1",
        "protocol_limits.velocity.0",
        "protocol_limits.velocity.1",
        "protocol_limits.torque.0",
        "protocol_limits.torque.1",
        "protocol_limits.kp.0",
        "protocol_limits.kp.1",
        "protocol_limits.kd.0",
        "protocol_limits.kd.1",
    };
    float *limit_values[] = {
        &config->limits.position_min,
        &config->limits.position_max,
        &config->limits.velocity_min,
        &config->limits.velocity_max,
        &config->limits.torque_min,
        &config->limits.torque_max,
        &config->limits.kp_min,
        &config->limits.kp_max,
        &config->limits.kd_min,
        &config->limits.kd_max,
    };
    uint32_t feedback_id = 0;
    uint32_t timeout_ms = ENCOS_DEFAULT_TIMEOUT_MS;
    const char *model;
    size_t limit_count = 0;
    int enable_result;
    int timeout_result;

    if (!can_args || !config || !can_args->options || can_args->option_count == 0 ||
        motor_options_validate(can_args->options, can_args->option_count, allowed_options,
            sizeof(allowed_options) / sizeof(allowed_options[0])) != 0 ||
        motor_option_read_u32(can_args->options, can_args->option_count,
            "feedback_id", &feedback_id) != 0 ||
        feedback_id > ENCOS_MAX_CAN_ID) {
        return -1;
    }
    config->feedback_id = (uint16_t)feedback_id;

    enable_result = motor_option_read_bool(can_args->options, can_args->option_count,
        "enable_on_init", &config->enable_on_init);
    if (enable_result < 0) return -1;

    timeout_result = motor_option_read_u32(can_args->options, can_args->option_count,
        "can_timeout_ms", &timeout_ms);
    if (timeout_result < 0 || timeout_ms > UINT16_MAX) return -1;
    config->can_timeout_ms = timeout_ms;

    model = motor_option_find(can_args->options, can_args->option_count, "model");
    if (model && get_model_limits(model, &config->limits) != 0) return -1;
    for (size_t i = 0; i < sizeof(limit_names) / sizeof(limit_names[0]); ++i) {
        if (motor_option_find(can_args->options, can_args->option_count, limit_names[i]))
            ++limit_count;
    }
    if (limit_count != 0 && limit_count != sizeof(limit_names) / sizeof(limit_names[0]))
        return -1;
    if (limit_count > 0) {
        for (size_t i = 0; i < sizeof(limit_names) / sizeof(limit_names[0]); ++i) {
            if (motor_option_read_float(can_args->options, can_args->option_count,
                    limit_names[i], limit_values[i]) != 0) {
                return -1;
            }
        }
    }
    if (!model && limit_count == 0) return -1;
    return encos_limits_are_valid(&config->limits) ? 0 : -1;
}

static struct motor_dev *encos_probe(void *args) {
    const struct motor_args_can *can_args = args;
    const struct motor_can_encos_config *config;
    struct motor_can_encos_config option_config = {0};
    struct motor_dev *dev;
    struct encos_priv *priv;

    if (!can_args || !can_args->iface || can_args->can_id > ENCOS_MAX_CAN_ID)
        return NULL;
    if (can_args->options) {
        if (parse_encos_options(can_args, &option_config) != 0) return NULL;
        config = &option_config;
    } else {
        if (!can_args->args) return NULL;
        config = can_args->args;
    }
    if (config->feedback_id > ENCOS_MAX_CAN_ID ||
        config->can_timeout_ms > UINT16_MAX ||
        !encos_limits_are_valid(&config->limits)) {
        return NULL;
    }

    dev = calloc(1, sizeof(*dev));
    priv = calloc(1, sizeof(*priv));
    if (!dev || !priv) {
        free(priv);
        free(dev);
        return NULL;
    }
    priv->bus = acquire_bus(can_args->iface);
    if (!priv->bus) {
        free(priv);
        free(dev);
        return NULL;
    }
    priv->config = *config;
    priv->command_id = can_args->can_id;
    dev->name = "drv_can_encos";
    dev->ops = &kEncosOps;
    dev->priv_data = priv;
    return dev;
}

REGISTER_MOTOR_DRIVER("drv_can_encos", DRV_TYPE_CAN, encos_probe);
