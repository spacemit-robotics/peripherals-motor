/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file test_rs485_20_26_batch.c
 * @brief Multi-UART batch contracts on PTYs only, never robot hardware.
 */
#define _GNU_SOURCE
#include "motor.h"
#include "rs485_20_26_protocol.h"

#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <stdlib.h>
#include <unistd.h>

struct emulator {
    int fd[2];
    int omit;
    int enable;
    int corrupt;
    int corrupt_only;
};

static void read_exact(int fd, uint8_t *bytes, size_t count) {
    while (count) {
        struct pollfd descriptor = {fd, POLLIN, 0};
        assert(poll(&descriptor, 1, 1000) == 1);
        ssize_t size = read(fd, bytes, count);
        assert(size > 0);
        bytes += size;
        count -= (size_t)size;
    }
}

static void *emulate(void *argument) {
    struct emulator *emulator = argument;
    uint8_t commands[2][3 * RS485_20_26_COMMAND_SIZE];
    // Neither bus replies until BOTH batches have arrived: per-bus blocking
    // and single-motor waits cannot pass this test.
    for (int bus = 0; bus < 2; ++bus)
        read_exact(emulator->fd[bus], commands[bus], sizeof(commands[bus]));
    for (int bus = 0; bus < 2; ++bus) {
        for (int i = 2; i >= 0; --i) {
            const uint8_t *command = commands[bus] + i * RS485_20_26_COMMAND_SIZE;
            assert(command[0] == 0xfe && command[1] == 0xee);
            assert((command[2] & 15U) == (unsigned int)i);
            assert(((command[2] >> 4) & 1) == emulator->enable);
            if (!emulator->enable) {
                assert(command[3] == 0x80);
                for (int j = 4; j < 16; ++j) assert(command[j] == 0);
            }
            if (bus == 0 && i == emulator->omit) continue;
            uint8_t reply[RS485_20_26_FEEDBACK_SIZE] = {0xfc, 0xee};
            reply[2] = (uint8_t)i;
            reply[3] = (uint8_t)(30 + bus * 3 + i);
            uint32_t crc = rs485_20_26_crc_words(reply + 2, 20);
            for (int j = 0; j < 4; ++j) reply[22 + j] = (uint8_t)(crc >> (8 * j));
            assert(write(emulator->fd[bus], "junk", 4) == 4);
            if (emulator->corrupt && bus == 0) {
                reply[22] ^= 1;
                assert(write(emulator->fd[bus], reply, sizeof(reply)) == sizeof(reply));
                reply[22] ^= 1;
                if (emulator->corrupt_only) continue;
                // Valid frame from an unconfigured address is not a CRC error.
                uint8_t unknown[RS485_20_26_FEEDBACK_SIZE] = {0xfc, 0xee, 14};
                const uint32_t other_crc = rs485_20_26_crc_words(unknown + 2, 20);
                for (int j = 0; j < 4; ++j)
                    unknown[22 + j] = (uint8_t)(other_crc >> (8 * j));
                assert(write(emulator->fd[bus], unknown, sizeof(unknown)) == sizeof(unknown));
            }
            assert(write(emulator->fd[bus], reply, 9) == 9);
            assert(write(emulator->fd[bus], reply + 9, sizeof(reply) - 9) == sizeof(reply) - 9);
        }
    }
    return NULL;
}

static void *emulate_sparse(void *argument) {
    struct emulator *emulator = argument;
    for (unsigned int id = 1; id <= 2; ++id) {
        for (int bus = 0; bus < 2; ++bus) {
            uint8_t command[RS485_20_26_COMMAND_SIZE];
            read_exact(emulator->fd[bus], command, sizeof(command));
            assert(command[0] == 0xfe && command[1] == 0xee && command[2] == id);
            for (int j = 4; j < 16; ++j) assert(command[j] == 0);
            struct pollfd descriptor = {emulator->fd[bus], POLLIN, 0};
            assert(poll(&descriptor, 1, 0) == 0);  // Never concatenate a sparse pair.
        }
        for (int bus = 0; bus < 2; ++bus) {
            if (bus == 0 && (int)id == emulator->omit) continue;
            uint8_t reply[RS485_20_26_FEEDBACK_SIZE] = {0xfc, 0xee};
            reply[2] = (uint8_t)id;
            reply[3] = (uint8_t)(30 + bus * 3 + id);
            const uint32_t crc = rs485_20_26_crc_words(reply + 2, 20);
            for (int j = 0; j < 4; ++j) reply[22 + j] = (uint8_t)(crc >> (8 * j));
            assert(write(emulator->fd[bus], reply, sizeof(reply)) == sizeof(reply));
        }
    }
    return NULL;
}

int main(void) {
    struct emulator emulator = {.omit = -1};
    struct motor_dev *motors[6];
    struct motor_state states[6] = {0};
    struct motor_cmd commands[6] = {0};
    uint64_t before[6], after[6];
    int errors[6];
    pthread_t worker;
    const struct motor_option options[] = {
        {"protocol_scales.torque", "16"}, {"protocol_scales.velocity", "651.898646904"},
        {"protocol_scales.position", "83443.0972851327"},
        {"protocol_scales.kp", "5"}, {"protocol_scales.kd", "5"},
        {"idle_poll", "true"}, {"transaction_timeout_us", "50000"}};
    for (int bus = 0; bus < 2; ++bus) {
        emulator.fd[bus] = posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK);
        assert(emulator.fd[bus] >= 0);
        assert(grantpt(emulator.fd[bus]) == 0 && unlockpt(emulator.fd[bus]) == 0);
        for (int i = 0; i < 3; ++i) {
            motors[bus * 3 + i] = motor_alloc_uart_with_options("drv_uart_rs485_20_26",
                ptsname(emulator.fd[bus]), 6000000, i, options,
                sizeof(options) / sizeof(options[0]));
            assert(motors[bus * 3 + i]);
        }
    }
    assert(motor_init(motors, 6) == 0);
    assert(pthread_create(&worker, NULL, emulate, &emulator) == 0);
    assert(motor_get_states_ex(motors, states, errors, 6) == 0);
    assert(pthread_join(worker, NULL) == 0);
    assert(motor_get_feedback_timestamps(motors, before, 6) == 0);
    for (int i = 0; i < 6; ++i) {
        assert(errors[i] == 0 && before[i] > 0 && states[i].temp == 30 + i);
    }
    emulator.omit = 1;
    states[1].temp = -99;
    assert(pthread_create(&worker, NULL, emulate, &emulator) == 0);
    assert(motor_get_states_ex(motors, states, errors, 6) < 0 && errno == ETIMEDOUT);
    assert(pthread_join(worker, NULL) == 0);
    assert(motor_get_feedback_timestamps(motors, after, 6) == 0);
    assert(errors[1] == ETIMEDOUT && states[1].temp == -99 && after[1] == before[1]);
    for (int i = 0; i < 6; ++i)
        if (i != 1) assert(errors[i] == 0 && after[i] > before[i]);
    struct motor_transport_diagnostics diagnostics, other;
    assert(motor_get_transport_diagnostics(motors[0], &diagnostics) == 0);
    assert(diagnostics.valid_frames == 5 && diagnostics.resync_discarded_bytes == 20);
    assert(diagnostics.timeouts == 1 && diagnostics.failure_missing_ids == (1U << 1));
    assert(diagnostics.failure_errno == ETIMEDOUT && diagnostics.failure_elapsed_us >= 50000);
    assert(diagnostics.failure_sent_bytes == 3 * RS485_20_26_COMMAND_SIZE);
    assert(diagnostics.failure_valid_frames == 2 && diagnostics.failure_crc_errors == 0);
    assert(motor_get_transport_diagnostics(motors[3], &other) == 0);
    assert(other.timeouts == 0 && other.last_failure_us == 0);

    // A missing reply on the first bus must not suppress disables on bus two.
    for (int i = 0; i < 6; ++i) commands[i].mode = MOTOR_MODE_IDLE;
    assert(pthread_create(&worker, NULL, emulate, &emulator) == 0);
    assert(motor_set_cmds_ex(motors, commands, errors, 6) < 0);
    assert(pthread_join(worker, NULL) == 0);
    for (int i = 0; i < 6; ++i) assert(errors[i] == (i == 1 ? ETIMEDOUT : 0));

    // A successful disable already obtains feedback. Its next read consumes
    // that reply without polling the same motors a second time in the cycle.
    emulator.omit = -1;
    assert(pthread_create(&worker, NULL, emulate, &emulator) == 0);
    assert(motor_set_cmds_ex(motors, commands, errors, 6) == 0);
    assert(pthread_join(worker, NULL) == 0);
    assert(motor_get_states_ex(motors, states, errors, 6) == 0);
    for (int bus = 0; bus < 2; ++bus) {
        struct pollfd descriptor = {emulator.fd[bus], POLLIN, 0};
        assert(poll(&descriptor, 1, 0) == 0);
    }

    // Enabled PTY motors: reads only drain, never insert a disable query.
    emulator.enable = 1;
    for (int i = 0; i < 6; ++i) commands[i].mode = MOTOR_MODE_HYBRID;
    assert(pthread_create(&worker, NULL, emulate, &emulator) == 0);
    assert(motor_set_cmds_ex(motors, commands, errors, 6) == 0);
    assert(pthread_join(worker, NULL) == 0);
    assert(motor_get_feedback_timestamps(motors, before, 6) == 0);
    assert(motor_get_states_ex(motors, states, errors, 6) == 0);
    assert(motor_get_feedback_timestamps(motors, after, 6) == 0);
    for (int i = 0; i < 6; ++i) assert(before[i] == after[i]);
    for (int bus = 0; bus < 2; ++bus) {
        struct pollfd descriptor = {emulator.fd[bus], POLLIN, 0};
        assert(poll(&descriptor, 1, 0) == 0);
    }
    struct motor_dev *duplicates[] = {motors[0], motors[0]};
    assert(motor_get_states_ex(duplicates, states, errors, 2) < 0 && errno == EINVAL);

    struct motor_dev *subset[] = {motors[1], motors[2], motors[4], motors[5]};
    struct motor_cmd subset_commands[4] = {0};
    assert(pthread_create(&worker, NULL, emulate_sparse, &emulator) == 0);
    assert(motor_set_cmds_ex(subset, subset_commands, errors, 4) == 0);
    assert(pthread_join(worker, NULL) == 0);
    assert(motor_get_states_ex(subset, states, errors, 4) == 0);  // Consume write feedback.
    assert(pthread_create(&worker, NULL, emulate_sparse, &emulator) == 0);
    assert(motor_get_states_ex(subset, states, errors, 4) == 0);
    assert(pthread_join(worker, NULL) == 0);
    assert(states[0].temp == 31 && states[1].temp == 32 &&
        states[2].temp == 34 && states[3].temp == 35);
    emulator.omit = 1;
    assert(pthread_create(&worker, NULL, emulate_sparse, &emulator) == 0);
    assert(motor_set_cmds_ex(subset, subset_commands, errors, 4) < 0 && errno == ETIMEDOUT);
    assert(pthread_join(worker, NULL) == 0);
    for (int i = 0; i < 4; ++i) assert(errors[i] == (i == 0 ? ETIMEDOUT : 0));

    emulator.omit = -1;
    emulator.enable = 0;
    emulator.corrupt = 1;
    emulator.corrupt_only = 1;
    for (int i = 0; i < 6; ++i) commands[i].mode = MOTOR_MODE_IDLE;
    assert(pthread_create(&worker, NULL, emulate, &emulator) == 0);
    assert(motor_set_cmds_ex(motors, commands, errors, 6) < 0 && errno == ETIMEDOUT);
    assert(pthread_join(worker, NULL) == 0);
    assert(motor_get_transport_diagnostics(motors[0], &diagnostics) == 0);
    assert(diagnostics.crc_errors == 3 && diagnostics.failure_crc_errors == 3);
    assert(diagnostics.failure_valid_frames == 0 && diagnostics.failure_missing_ids == 7);
    assert(diagnostics.failure_rx_bytes == 3 * (RS485_20_26_FEEDBACK_SIZE + 4));
    const uint64_t failure_time = diagnostics.last_failure_us;
    emulator.corrupt_only = 0;
    assert(pthread_create(&worker, NULL, emulate, &emulator) == 0);
    assert(motor_set_cmds_ex(motors, commands, errors, 6) == 0);
    assert(pthread_join(worker, NULL) == 0);
    assert(motor_get_transport_diagnostics(motors[2], &other) == 0);
    assert(other.crc_errors == 6 && other.unknown_id_frames == 3);
    assert(other.last_failure_us == failure_time && other.failure_crc_errors == 3);
    // Reading diagnostics neither transmits nor clears evidence.
    assert(motor_get_transport_diagnostics(motors[0], &diagnostics) == 0);
    assert(diagnostics.crc_errors == other.crc_errors && diagnostics.tx_bytes == other.tx_bytes);
    for (int bus = 0; bus < 2; ++bus) {
        struct pollfd descriptor = {emulator.fd[bus], POLLIN, 0};
        assert(poll(&descriptor, 1, 0) == 0);
    }
    motor_free(motors, 6);
    for (int bus = 0; bus < 2; ++bus) close(emulator.fd[bus]);
    return 0;
}
