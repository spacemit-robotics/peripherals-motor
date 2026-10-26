/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file drv_uart_rs485_20_26.c
 * @brief Shared UART transport for RS485 20/26 actuators.
 */
#define _GNU_SOURCE
#include "motor_core.h"
#include "rs485_20_26_protocol.h"

#include <asm/ioctls.h>
#include <asm/termbits.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/serial.h>
#include <math.h>
#include <poll.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

struct rs485_20_26_motor;
struct rs485_20_26_bus {
    char *path;
    int fd;
    uint32_t baud;
    unsigned int references;
    uint8_t stream[RS485_20_26_FEEDBACK_SIZE];
    size_t used;
    struct motor_transport_diagnostics diagnostics;
    struct rs485_20_26_motor *members[15];
    pthread_mutex_t lock;
    struct rs485_20_26_bus *next;
};

struct rs485_20_26_motor {
    struct rs485_20_26_bus *bus;
    struct rs485_20_26_scale scale;
    struct motor_state state;
    uint64_t received_us;
    uint64_t last_read_us;
    uint8_t id;
    uint8_t feedback_id;
    uint32_t timeout_us;
    int initialized;
    int idle_poll;
    int enabled;
};

static pthread_mutex_t registry_lock = PTHREAD_MUTEX_INITIALIZER;
static struct rs485_20_26_bus *buses;

static uint64_t monotonic_us(void) {
    struct timespec now;
    if (clock_gettime(CLOCK_MONOTONIC, &now) != 0)
        return 0;
    return (uint64_t)now.tv_sec * 1000000ULL + (uint64_t)now.tv_nsec / 1000ULL;
}

static int configure_uart(int fd, uint32_t baud) {
    struct termios2 settings;
    struct serial_struct serial = {0};
    if (ioctl(fd, TCGETS2, &settings) != 0)
        return -1;
    settings.c_iflag = 0;
    settings.c_oflag = 0;
    settings.c_lflag = 0;
    settings.c_cflag = CLOCAL | CREAD | CS8 | BOTHER;
    settings.c_ispeed = baud;
    settings.c_ospeed = settings.c_ispeed;
    settings.c_cc[VMIN] = 0;
    settings.c_cc[VTIME] = 0;
    if (ioctl(fd, TCSETS2, &settings) != 0)
        return -1;
    // FTDI resets its receive latency on USB re-enumeration. Request the 1 ms
    // low-latency mode on every open; PTYs do not implement serial ioctls.
    if (ioctl(fd, TIOCGSERIAL, &serial) != 0)
        return errno == ENOTTY ? 0 : -1;
    if (serial.flags & ASYNC_LOW_LATENCY)
        return 0;
    serial.flags |= ASYNC_LOW_LATENCY;
    return ioctl(fd, TIOCSSERIAL, &serial);
}

static int wait_fd(struct rs485_20_26_bus *bus, short events, uint64_t deadline) {
    for (;;) {
        uint64_t now = monotonic_us();
        struct timespec timeout;
        struct pollfd descriptor = {bus->fd, events, 0};
        int result;
        if (!now || now >= deadline) {
            errno = ETIMEDOUT;
            return -1;
        }
        timeout.tv_sec = (time_t)((deadline - now) / 1000000ULL);
        timeout.tv_nsec = (long)((deadline - now) % 1000000ULL) * 1000L;
        result = ppoll(&descriptor, 1, &timeout, NULL);
        if (result < 0 && errno == EINTR)
            continue;
        if (result > 0 && !(descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)))
            return 0;
        if (result >= 0)
            errno = result == 0 ? ETIMEDOUT : EIO;
        if (errno != ETIMEDOUT) ++bus->diagnostics.poll_errors;
        return -1;
    }
}

static void parse_byte(struct rs485_20_26_bus *bus, uint8_t byte, uint64_t received_us) {
    bus->stream[bus->used++] = byte;
    while (bus->used && (bus->stream[0] != 0xfc ||
        (bus->used >= 2 && bus->stream[1] != 0xee))) {
        ++bus->diagnostics.resync_discarded_bytes;
        --bus->used;
        memmove(bus->stream, bus->stream + 1, bus->used);
    }
    if (bus->used == RS485_20_26_FEEDBACK_SIZE) {
        unsigned int id = bus->stream[2] & 15U;
        struct rs485_20_26_motor *motor = id < 15 ? bus->members[id] : NULL;
        struct rs485_20_26_feedback feedback;
        const int decoded = motor
            ? rs485_20_26_decode(&motor->scale, bus->stream, bus->used, &feedback)
            : rs485_20_26_check_feedback(bus->stream, bus->used);
        if (decoded == 0) {
            if (!motor) {
                ++bus->diagnostics.unknown_id_frames;
                bus->used = 0;
                return;
            }
            ++bus->diagnostics.valid_frames;
            motor->state = (struct motor_state){
                .pos = (float)feedback.position, .vel = (float)feedback.velocity,
                .trq = (float)feedback.torque, .temp = (float)feedback.driver_temperature,
                .err = feedback.error | (feedback.watchdog_latched ? UINT32_C(0x80000000) : 0)};
            motor->received_us = received_us;
            bus->used = 0;
        } else {
            if (decoded == RS485_20_26_DECODE_CRC_ERROR) ++bus->diagnostics.crc_errors;
            else ++bus->diagnostics.decode_errors;
            ++bus->diagnostics.resync_discarded_bytes;
            --bus->used;
            memmove(bus->stream, bus->stream + 1, bus->used);
        }
    }
}

static int drain(struct rs485_20_26_bus *bus) {
    for (unsigned int chunk = 0; chunk < 32; ++chunk) {
        uint8_t incoming[256];
        ssize_t size = read(bus->fd, incoming, sizeof(incoming));
        if (size > 0) {
            uint64_t received_us = monotonic_us();
            bus->diagnostics.rx_bytes += (uint64_t)size;
            bus->diagnostics.last_rx_us = received_us;
            if (!received_us) {
                errno = EIO;
                return -1;
            }
            for (ssize_t i = 0; i < size; ++i)
                parse_byte(bus, incoming[i], received_us);
        } else if (size < 0 && errno == EINTR) {
            continue;
        } else if (size == 0 || errno == EAGAIN || errno == EWOULDBLOCK) {
            return 0;
        } else {
            ++bus->diagnostics.read_errors;
            return -1;
        }
    }
    errno = EOVERFLOW;
    ++bus->diagnostics.read_errors;
    return -1;
}

static void record_failure(struct rs485_20_26_bus *bus,
    const struct motor_transport_diagnostics *before, uint64_t start, uint64_t deadline,
    size_t expected, size_t sent, uint32_t missing, int error) {
    const int saved_errno = errno;
    struct motor_transport_diagnostics *stats = &bus->diagnostics;
    const uint64_t now = monotonic_us();
    if (error == ETIMEDOUT) ++stats->timeouts;
    stats->last_failure_us = now;
    stats->failure_elapsed_us = now - start;
    stats->failure_deadline_late_us = deadline && now > deadline ? now - deadline : 0;
    stats->failure_rx_bytes = stats->rx_bytes - before->rx_bytes;
    stats->failure_valid_frames = stats->valid_frames - before->valid_frames;
    stats->failure_crc_errors = stats->crc_errors - before->crc_errors;
    stats->failure_expected_bytes = (uint32_t)expected;
    stats->failure_sent_bytes = (uint32_t)sent;
    stats->failure_missing_ids = missing;
    stats->failure_errno = error;
    errno = saved_errno;
}

/* One bounded transaction. There is no protocol sequence number: the timestamp
 * means host receipt, never a claim that a delayed reply belongs to this write. */
static int exchange(struct rs485_20_26_motor *motor, const struct rs485_20_26_command *command) {
    struct rs485_20_26_bus *bus = motor->bus;
    uint8_t bytes[RS485_20_26_COMMAND_SIZE];
    uint64_t previous;
    uint64_t deadline;
    size_t sent = 0;
    if (!motor->initialized || bus->fd < 0) {
        errno = ENODEV;
        return -1;
    }
    if (rs485_20_26_encode(&motor->scale, command, bytes) != 0) {
        errno = ERANGE;
        return -1;
    }
    if (drain(bus) < 0)
        return -1;
    previous = motor->received_us;
    const uint64_t start = monotonic_us();
    const struct motor_transport_diagnostics before = bus->diagnostics;
    ++bus->diagnostics.transactions;
    deadline = start + motor->timeout_us;
    while (sent < sizeof(bytes)) {
        ssize_t size = write(bus->fd, bytes + sent, sizeof(bytes) - sent);
        if (size > 0) {
            sent += (size_t)size;
            bus->diagnostics.tx_bytes += (uint64_t)size;
        } else if (size < 0 && errno == EINTR) {
            continue;
        } else if (size < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            if (wait_fd(bus, POLLOUT, deadline) < 0)
                goto failed;
        } else {
            if (size == 0) errno = EIO;
            ++bus->diagnostics.write_errors;
            goto failed;
        }
    }
    /* Suppress idle queries as soon as a command may have reached the actuator,
     * even if its reply times out. A read must never silently disable that motor. */
    motor->enabled = command->enabled;
    while (motor->received_us <= previous) {
        if (wait_fd(bus, POLLIN, deadline) < 0 || drain(bus) < 0)
            goto failed;
    }
    return 0;
failed:
    record_failure(bus, &before, start, deadline, sizeof(bytes), sent,
        motor->received_us <= previous ? 1U << motor->feedback_id : 0, errno);
    return -1;
}

static int rs485_20_26_init(struct motor_dev *dev) {
    struct rs485_20_26_motor *motor = dev->priv_data;
    struct rs485_20_26_bus *bus = motor->bus;
    int result = 0;
    pthread_mutex_lock(&bus->lock);
    if (bus->fd < 0) {
        bus->fd = open(bus->path, O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
        if (bus->fd < 0 || flock(bus->fd, LOCK_EX | LOCK_NB) < 0 ||
            configure_uart(bus->fd, bus->baud) < 0) {
            int saved = errno;
            if (bus->fd >= 0) close(bus->fd);
            bus->fd = -1;
            errno = saved;
            result = -1;
        }
    }
    if (result == 0) motor->initialized = 1;
    pthread_mutex_unlock(&bus->lock);
    return result;
}

static int rs485_20_26_set_cmd(struct motor_dev *dev, const struct motor_cmd *cmd) {
    struct rs485_20_26_motor *motor = dev->priv_data;
    struct rs485_20_26_command command = {.id = motor->id};
    int result;
    int saved;
    if (cmd->mode == MOTOR_MODE_HYBRID || cmd->mode == MOTOR_MODE_TRQ) {
        command.enabled = 1;
        command.watchdog = 1;
        command.torque = cmd->trq_des;
        if (cmd->mode == MOTOR_MODE_HYBRID) {
            command.velocity = cmd->vel_des;
            command.position = cmd->pos_des;
            command.kp = cmd->kp;
            command.kd = cmd->kd;
        }
    } else if (cmd->mode != MOTOR_MODE_IDLE) {
        errno = ENOTSUP;
        return -1;
    }
    pthread_mutex_lock(&motor->bus->lock);
    result = exchange(motor, &command);
    saved = errno;
    pthread_mutex_unlock(&motor->bus->lock);
    errno = saved;
    return result;
}

static int rs485_20_26_get_state(struct motor_dev *dev, struct motor_state *state) {
    struct rs485_20_26_motor *motor = dev->priv_data;
    int result = 0;
    int saved;
    pthread_mutex_lock(&motor->bus->lock);
    if (!motor->initialized) {
        errno = ENODEV;
        result = -1;
    } else if (motor->idle_poll && !motor->enabled && motor->received_us <= motor->last_read_us) {
        struct rs485_20_26_command idle = {.id = motor->id};
        result = exchange(motor, &idle);
    } else {
        result = drain(motor->bus);
    }
    if (result == 0 && motor->received_us == 0) {
        errno = EAGAIN;
        result = -1;
    }
    if (result == 0) {
        *state = motor->state;
        dev->feedback_timestamp_us = motor->received_us;
        motor->last_read_us = motor->received_us;
    }
    saved = errno;
    pthread_mutex_unlock(&motor->bus->lock);
    errno = saved;
    return result;
}

static void rs485_20_26_free(struct motor_dev *dev) {
    struct rs485_20_26_motor *motor = dev->priv_data;
    struct rs485_20_26_bus *bus = motor->bus;
    pthread_mutex_lock(&registry_lock);
    bus->members[motor->feedback_id] = NULL;
    if (--bus->references == 0) {
        struct rs485_20_26_bus **link = &buses;
        while (*link != bus) link = &(*link)->next;
        *link = bus->next;
        if (bus->fd >= 0) close(bus->fd);
        pthread_mutex_destroy(&bus->lock);
        free(bus->path);
        free(bus);
    }
    pthread_mutex_unlock(&registry_lock);
    free(motor);
    free(dev);
}

/* The vector path submits every independent UART before waiting. Each UART
 * carries one contiguous request batch; replies are routed once by feedback ID.
 * No per-motor USB round trips, worker threads, background retries or RX flush.
 * A missing reply remains a per-motor timeout, never a successful fresh read.
 * This still measures host receipt: the wire has no transaction sequence ID. */
#define RS485_20_26_BATCH_MAX 64
struct rs485_20_26_transfer {
    struct rs485_20_26_bus *bus;
    uint8_t bytes[15 * RS485_20_26_COMMAND_SIZE];
    size_t size;
    size_t sent;
    uint64_t deadline;
    int error;
    struct motor_transport_diagnostics before;
};

static int build_command(struct rs485_20_26_motor *motor, const struct motor_cmd *cmd,
    struct rs485_20_26_command *command) {
    *command = (struct rs485_20_26_command){.id = motor->id};
    if (!cmd || cmd->mode == MOTOR_MODE_IDLE) return 0;
    if (cmd->mode != MOTOR_MODE_HYBRID && cmd->mode != MOTOR_MODE_TRQ) return ENOTSUP;
    command->enabled = 1;
    command->watchdog = 1;
    command->torque = cmd->trq_des;
    if (cmd->mode == MOTOR_MODE_HYBRID) {
        command->velocity = cmd->vel_des;
        command->position = cmd->pos_des;
        command->kp = cmd->kp;
        command->kd = cmd->kd;
    }
    return 0;
}

static void rs485_20_26_transfer_all(struct motor_dev **devs, const struct motor_cmd *cmds,
    struct motor_state *states, int *errors, uint32_t count) {
    struct rs485_20_26_transfer transfers[RS485_20_26_BATCH_MAX] = {0};
    struct pollfd descriptors[RS485_20_26_BATCH_MAX];
    uint32_t bus_index[RS485_20_26_BATCH_MAX];
    uint64_t previous[RS485_20_26_BATCH_MAX] = {0};
    size_t frame_end[RS485_20_26_BATCH_MAX] = {0};
    uint8_t enabled[RS485_20_26_BATCH_MAX] = {0};
    uint32_t bus_count = 0;
    int invalid = count > RS485_20_26_BATCH_MAX;
    for (uint32_t i = 0; i < count; ++i) {
        errors[i] = 0;
        for (uint32_t j = 0; j < i; ++j)
            if (devs[i] == devs[j]) invalid = 1;
    }
    if (invalid) {
        for (uint32_t i = 0; i < count; ++i) errors[i] = EINVAL;
        return;
    }
    /* Serialize lock acquisition, not I/O. Another vector call cannot hold a
     * conflicting subset of bus locks. Lifetime is serialized by the caller. */
    pthread_mutex_lock(&registry_lock);
    for (uint32_t i = 0; i < count; ++i) {
        struct rs485_20_26_motor *motor = devs[i]->priv_data;
        uint32_t b = 0;
        while (b < bus_count && transfers[b].bus != motor->bus) ++b;
        if (b == bus_count) {
            transfers[b].bus = motor->bus;
            pthread_mutex_lock(&motor->bus->lock);
            ++bus_count;
        }
        bus_index[i] = b;
    }
    pthread_mutex_unlock(&registry_lock);
    for (uint32_t b = 0; b < bus_count; ++b) {
        struct rs485_20_26_transfer *transfer = &transfers[b];
        if (transfer->bus->fd < 0) transfer->error = ENODEV;
        else if (drain(transfer->bus) < 0) transfer->error = errno;
    }
    const uint64_t start = monotonic_us();
    for (uint32_t b = 0; b < bus_count; ++b)
        transfers[b].before = transfers[b].bus->diagnostics;
    for (uint32_t i = 0; i < count; ++i) {
        struct rs485_20_26_motor *motor = devs[i]->priv_data;
        struct rs485_20_26_transfer *transfer = &transfers[bus_index[i]];
        struct rs485_20_26_command command;
        previous[i] = motor->received_us;
        if (!motor->initialized || transfer->error) {
            errors[i] = transfer->error ? transfer->error : ENODEV;
            continue;
        }
        if (!cmds && (!motor->idle_poll || motor->enabled ||
            motor->received_us > motor->last_read_us)) continue;
        errors[i] = build_command(motor, cmds ? &cmds[i] : NULL, &command);
        if (errors[i]) continue;
        if (rs485_20_26_encode(&motor->scale, &command, transfer->bytes + transfer->size) < 0) {
            errors[i] = ERANGE;
            continue;
        }
        transfer->size += RS485_20_26_COMMAND_SIZE;
        frame_end[i] = transfer->size;
        enabled[i] = command.enabled;
        const uint64_t deadline = start + motor->timeout_us;
        if (!transfer->deadline || deadline < transfer->deadline) transfer->deadline = deadline;
    }
    for (uint32_t b = 0; b < bus_count; ++b)
        if (transfers[b].size) ++transfers[b].bus->diagnostics.transactions;
    for (;;) {
        uint64_t next_deadline = UINT64_MAX;
        uint32_t pending_buses = 0;
        for (uint32_t b = 0; b < bus_count; ++b) {
            struct rs485_20_26_transfer *transfer = &transfers[b];
            descriptors[b] = (struct pollfd){.fd = -1};
            if (transfer->error || !transfer->size) continue;
            if (monotonic_us() >= transfer->deadline) {
                transfer->error = ETIMEDOUT;
                continue;
            }
            if (transfer->sent < transfer->size) {
                /* If any bytes might be sent, a future read must not issue an
                 * idle query for a motor that might already have been enabled. */
                for (uint32_t i = 0; i < count; ++i) {
                    if (bus_index[i] == b && frame_end[i] && enabled[i])
                        ((struct rs485_20_26_motor *)devs[i]->priv_data)->enabled = 1;
                }
                ssize_t size = write(transfer->bus->fd, transfer->bytes + transfer->sent,
                    transfer->size - transfer->sent);
                if (size > 0) {
                    transfer->sent += (size_t)size;
                    transfer->bus->diagnostics.tx_bytes += (uint64_t)size;
                } else if (size == 0 ||
                    (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)) {
                    transfer->error = size == 0 ? EIO : errno;
                    ++transfer->bus->diagnostics.write_errors;
                }
                for (uint32_t i = 0; i < count; ++i) {
                    if (bus_index[i] == b && frame_end[i] &&
                        frame_end[i] <= transfer->sent && !enabled[i])
                        ((struct rs485_20_26_motor *)devs[i]->priv_data)->enabled = 0;
                }
            }
            if (transfer->error) continue;
            int pending = transfer->sent < transfer->size;
            for (uint32_t i = 0; i < count; ++i) {
                struct rs485_20_26_motor *motor = devs[i]->priv_data;
                if (bus_index[i] == b && frame_end[i] && motor->received_us <= previous[i])
                    pending = 1;
            }
            if (!pending) continue;
            descriptors[b] = (struct pollfd){.fd = transfer->bus->fd,
                .events = POLLIN | (transfer->sent < transfer->size ? POLLOUT : 0)};
            ++pending_buses;
            if (transfer->deadline < next_deadline) next_deadline = transfer->deadline;
        }
        if (!pending_buses) break;
        const uint64_t now = monotonic_us();
        if (now >= next_deadline) continue;
        struct timespec timeout = {
            .tv_sec = (time_t)((next_deadline - now) / 1000000),
            .tv_nsec = (long)((next_deadline - now) % 1000000) * 1000};
        int result = ppoll(descriptors, bus_count, &timeout, NULL);
        if (result < 0 && errno == EINTR) continue;
        for (uint32_t b = 0; b < bus_count; ++b) {
            if (descriptors[b].fd < 0) continue;
            if (result < 0) {
                transfers[b].error = errno;
                ++transfers[b].bus->diagnostics.poll_errors;
            } else if (descriptors[b].revents & (POLLERR | POLLHUP | POLLNVAL)) {
                transfers[b].error = EIO;
                ++transfers[b].bus->diagnostics.poll_errors;
            } else if ((descriptors[b].revents & POLLIN) && drain(transfers[b].bus) < 0)
                transfers[b].error = errno;
        }
    }
    for (uint32_t i = 0; i < count; ++i) {
        struct rs485_20_26_motor *motor = devs[i]->priv_data;
        struct rs485_20_26_transfer *transfer = &transfers[bus_index[i]];
        if (errors[i]) continue;
        if (frame_end[i] && (transfer->sent < frame_end[i] || motor->received_us <= previous[i]))
            errors[i] = transfer->error ? transfer->error : ETIMEDOUT;
        else if (!motor->received_us) errors[i] = EAGAIN;
        if (errors[i]) continue;
        if (states) {
            states[i] = motor->state;
            motor->last_read_us = motor->received_us;
        }
        devs[i]->feedback_timestamp_us = motor->received_us;
    }
    for (uint32_t b = 0; b < bus_count; ++b) {
        struct rs485_20_26_transfer *transfer = &transfers[b];
        uint32_t missing = 0;
        int error = 0;
        for (uint32_t i = 0; i < count; ++i) {
            if (bus_index[i] != b || !frame_end[i] || !errors[i]) continue;
            struct rs485_20_26_motor *motor = devs[i]->priv_data;
            missing |= 1U << motor->feedback_id;
            error = errors[i];
        }
        if (error || (!transfer->size && transfer->error))
            record_failure(transfer->bus, &transfer->before, start, transfer->deadline,
                transfer->size, transfer->sent, missing, error ? error : transfer->error);
    }
    for (uint32_t b = bus_count; b > 0; --b)
        pthread_mutex_unlock(&transfers[b - 1].bus->lock);
}

/* Concatenated frames require consecutive IDs starting at zero. Other
 * selections use one addressed frame per bus per round, without sending
 * to unselected endpoints. Independent buses are scheduled together. */
static void rs485_20_26_transfer_selected(struct motor_dev **devs, const struct motor_cmd *cmds,
    struct motor_state *states, int *errors, uint32_t count) {
    int sparse = 0;
    int invalid = count > RS485_20_26_BATCH_MAX;
    for (uint32_t i = 0; i < count && !invalid; ++i) {
        const struct rs485_20_26_motor *motor = devs[i]->priv_data;
        unsigned int preceding = 0;
        for (uint32_t j = 0; j < i; ++j) {
            if (devs[i] == devs[j]) invalid = 1;
            if (((struct rs485_20_26_motor *)devs[j]->priv_data)->bus == motor->bus) ++preceding;
        }
        if (motor->id != preceding) sparse = 1;
    }
    if (!sparse || invalid) {
        rs485_20_26_transfer_all(devs, cmds, states, errors, count);
        return;
    }
    uint8_t done[RS485_20_26_BATCH_MAX] = {0};
    uint32_t remaining = count;
    while (remaining) {
        struct motor_dev *round_devs[RS485_20_26_BATCH_MAX];
        struct motor_cmd round_cmds[RS485_20_26_BATCH_MAX];
        struct motor_state round_states[RS485_20_26_BATCH_MAX];
        int round_errors[RS485_20_26_BATCH_MAX];
        uint32_t indices[RS485_20_26_BATCH_MAX];
        uint32_t size = 0;
        for (uint32_t i = 0; i < count; ++i) {
            if (done[i]) continue;
            const struct rs485_20_26_bus *bus = ((struct rs485_20_26_motor *)devs[i]->priv_data)->bus;
            uint32_t j = 0;
            while (j < size && ((struct rs485_20_26_motor *)round_devs[j]->priv_data)->bus != bus) ++j;
            if (j < size) continue;
            round_devs[size] = devs[i];
            if (cmds) round_cmds[size] = cmds[i];
            indices[size++] = i;
            done[i] = 1;
            --remaining;
        }
        rs485_20_26_transfer_all(round_devs, cmds ? round_cmds : NULL,
            states ? round_states : NULL, round_errors, size);
        for (uint32_t i = 0; i < size; ++i) {
            errors[indices[i]] = round_errors[i];
            if (states && !round_errors[i]) states[indices[i]] = round_states[i];
        }
    }
}

static void rs485_20_26_get_states(struct motor_dev **devs, struct motor_state *states,
    int *errors, uint32_t count) {
    rs485_20_26_transfer_selected(devs, NULL, states, errors, count);
}

static void rs485_20_26_set_cmds(struct motor_dev **devs, const struct motor_cmd *cmds,
    int *errors, uint32_t count) {
    rs485_20_26_transfer_selected(devs, cmds, NULL, errors, count);
}

static int rs485_20_26_get_transport_diagnostics(struct motor_dev *dev,
    struct motor_transport_diagnostics *diagnostics) {
    struct rs485_20_26_bus *bus = ((struct rs485_20_26_motor *)dev->priv_data)->bus;
    pthread_mutex_lock(&bus->lock);
    *diagnostics = bus->diagnostics;
    pthread_mutex_unlock(&bus->lock);
    return 0;
}

static const struct motor_ops rs485_20_26_ops = {
    .init = rs485_20_26_init, .set_cmd = rs485_20_26_set_cmd, .get_state = rs485_20_26_get_state, .free = rs485_20_26_free,
    .get_states = rs485_20_26_get_states, .set_cmds = rs485_20_26_set_cmds,
    .get_transport_diagnostics = rs485_20_26_get_transport_diagnostics};

/* Wire conversion factors belong to the actuator configuration, not joint names. */
static int read_scale(const struct motor_args_uart *uart, const char *key, double *value) {
    const char *text = motor_option_find(uart->options, uart->option_count, key);
    char *end = NULL;
    if (!text || !text[0]) return -1;
    errno = 0;
    *value = strtod(text, &end);
    return errno || end == text || *end || !isfinite(*value) || *value <= 0.0 ? -1 : 0;
}

static struct motor_dev *rs485_20_26_create(void *args) {
    static const char *const allowed[] = {
        "model", "feedback_id", "idle_poll", "transaction_timeout_us",
        "protocol_scales.torque", "protocol_scales.velocity", "protocol_scales.position",
        "protocol_scales.kp", "protocol_scales.kd"};
    const struct motor_args_uart *uart = args;
    struct rs485_20_26_scale scale;
    struct motor_dev *dev;
    struct rs485_20_26_motor *motor;
    struct rs485_20_26_bus *bus;
    uint32_t feedback_id;
    uint32_t timeout_us = 2000;
    bool idle_poll = false;
    if (!uart || uart->args || !uart->dev_path || uart->dev_path[0] != '/' ||
        uart->baud == 0 || uart->id > 14 ||
        motor_options_validate(uart->options, uart->option_count, allowed,
            sizeof(allowed) / sizeof(allowed[0])) < 0)
        return NULL;
    feedback_id = uart->id;
    if (read_scale(uart, "protocol_scales.torque", &scale.torque) < 0 ||
        read_scale(uart, "protocol_scales.velocity", &scale.velocity) < 0 ||
        read_scale(uart, "protocol_scales.position", &scale.position) < 0 ||
        read_scale(uart, "protocol_scales.kp", &scale.kp) < 0 ||
        read_scale(uart, "protocol_scales.kd", &scale.kd) < 0 ||
        motor_option_read_u32(uart->options, uart->option_count,
            "feedback_id", &feedback_id) < 0 || feedback_id > 14 ||
        motor_option_read_u32(uart->options, uart->option_count,
            "transaction_timeout_us", &timeout_us) < 0 || timeout_us < 100 || timeout_us > 100000 ||
        motor_option_read_bool(uart->options, uart->option_count, "idle_poll", &idle_poll) < 0)
        return NULL;
    dev = calloc(1, sizeof(*dev));
    motor = calloc(1, sizeof(*motor));
    if (!dev || !motor) {
        free(dev);
        free(motor);
        return NULL;
    }
    pthread_mutex_lock(&registry_lock);
    for (bus = buses; bus && strcmp(bus->path, uart->dev_path) != 0; bus = bus->next) {}
    if (bus && (bus->baud != uart->baud || bus->members[feedback_id]))
        goto fail;
    if (bus) {
        for (size_t i = 0; i < 15; ++i)
            if (bus->members[i] && bus->members[i]->id == uart->id) goto fail;
    } else {
        bus = calloc(1, sizeof(*bus));
        if (!bus) goto fail;
        bus->path = strdup(uart->dev_path);
        if (!bus->path || pthread_mutex_init(&bus->lock, NULL) != 0) {
            free(bus->path);
            free(bus);
            goto fail;
        }
        bus->fd = -1;
        bus->baud = uart->baud;
        bus->next = buses;
        buses = bus;
    }
    *motor = (struct rs485_20_26_motor){.bus = bus, .scale = scale, .id = uart->id,
        .feedback_id = (uint8_t)feedback_id, .timeout_us = timeout_us, .idle_poll = idle_poll};
    bus->members[feedback_id] = motor;
    ++bus->references;
    pthread_mutex_unlock(&registry_lock);
    dev->name = "drv_uart_rs485_20_26";
    dev->ops = &rs485_20_26_ops;
    dev->priv_data = motor;
    return dev;
fail:
    pthread_mutex_unlock(&registry_lock);
    free(motor);
    free(dev);
    errno = EINVAL;
    return NULL;
}

REGISTER_MOTOR_DRIVER("drv_uart_rs485_20_26", DRV_TYPE_UART, rs485_20_26_create);
