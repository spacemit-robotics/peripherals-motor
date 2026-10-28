/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file test_rs485_20_26_uart.c
 * @brief PTY-only integration test; no hardware paths or motor devices are opened.
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
#include <string.h>
#include <time.h>
#include <unistd.h>

ssize_t __real_write(int fd, const void *data, size_t size);
int __real_clock_gettime(clockid_t clock, struct timespec *value);
static int interrupt_writes;
static unsigned int write_attempts;
static uint64_t clock_advance_us;

ssize_t __wrap_write(int fd, const void *data, size_t size) {
    if (!interrupt_writes) return __real_write(fd, data, size);
    ++write_attempts;
    clock_advance_us += 10000;
    if (interrupt_writes == 2 && write_attempts == 1)
        return __real_write(fd, data, size > 3 ? 3 : size);
    // Bound the mock itself so a missing driver deadline fails, not hangs.
    errno = write_attempts < 100 ? EINTR : EIO;
    return -1;
}

int __wrap_clock_gettime(clockid_t clock, struct timespec *value) {
    const int result = __real_clock_gettime(clock, value);
    if (!result && interrupt_writes && clock == CLOCK_MONOTONIC) {
        value->tv_nsec += (long)(clock_advance_us % 1000000) * 1000;
        value->tv_sec += (time_t)(clock_advance_us / 1000000) + value->tv_nsec / 1000000000;
        value->tv_nsec %= 1000000000;
    }
    return result;
}

struct emulator {
    int fd;
    unsigned int commands;
};

static void *reply_once(void *arg) {
    struct emulator *emulator = arg;
    uint8_t command[RS485_20_26_COMMAND_SIZE];
    uint8_t feedback[RS485_20_26_FEEDBACK_SIZE] = {0xfc, 0xee};
    size_t used = 0;
    while (used < sizeof(command)) {
        struct pollfd ready = {emulator->fd, POLLIN, 0};
        assert(poll(&ready, 1, 1000) == 1);
        ssize_t size = read(emulator->fd, command + used, sizeof(command) - used);
        assert(size > 0);
        used += (size_t)size;
    }
    assert(command[0] == 0xfe && command[1] == 0xee);
    ++emulator->commands;
    feedback[2] = command[2] & 15U;
    feedback[3] = 30;
    feedback[10] = 1;
    uint32_t crc = rs485_20_26_crc_words(feedback + 2, 20);
    for (size_t i = 0; i < 4; ++i) feedback[22 + i] = (uint8_t)(crc >> (8 * i));
    // Junk + a fragmented reply must resynchronize, not produce invented data.
    assert(write(emulator->fd, "noise", 5) == 5);
    assert(write(emulator->fd, feedback, 7) == 7);
    assert(write(emulator->fd, feedback + 7, sizeof(feedback) - 7) == (ssize_t)sizeof(feedback) - 7);
    return NULL;
}

int main(void) {
    int master = posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK);
    assert(master >= 0 && grantpt(master) == 0 && unlockpt(master) == 0);
    const char *path = ptsname(master);
    struct motor_option options[] = {
        {"model", "test actuator"},
        {"protocol_scales.torque", "16"}, {"protocol_scales.velocity", "651.898646904"},
        {"protocol_scales.position", "83443.09729707374"},
        {"protocol_scales.kp", "5"}, {"protocol_scales.kd", "5"},
        {"transaction_timeout_us", "50000"}};
    const uint32_t option_count = sizeof(options) / sizeof(options[0]);
    struct motor_dev *dev = motor_alloc_uart_with_options(
        "drv_uart_rs485_20_26", path, 6000000, 4, options, option_count);
    struct motor_state state;
    uint64_t before, after;
    struct emulator emulator = {master, 0};
    pthread_t worker;
    struct motor_cmd command = {.mode = MOTOR_MODE_HYBRID, .kd = 0.0f};
    assert(dev != NULL);
    assert(motor_alloc_uart_with_options("drv_uart_rs485_20_26", path,
        6000000, 3, NULL, 0) == NULL);  // No implicit model profile or scale defaults.
    options[1].value = "nan";
    assert(motor_alloc_uart_with_options("drv_uart_rs485_20_26", path,
        6000000, 3, options, option_count) == NULL);
    options[1].value = "16";
    assert(motor_alloc_uart_with_options("drv_uart_rs485_20_26", path,
        115200, 3, options, option_count) == NULL);  // Shared bus must agree on baud.
    assert(motor_alloc_uart_with_options("drv_uart_rs485_20_26", path,
        6000000, 4, options, option_count) == NULL);  // Duplicate endpoint rejected.
    // PTYs reject TIOCGSERIAL with ENOTTY; UART initialization must still work.
    assert(motor_init_one(dev) == 0);
    struct pollfd empty = {master, POLLIN, 0};
    assert(poll(&empty, 1, 0) == 0);  // No transmit from alloc/init.
    assert(motor_get_state_one(dev, &state) < 0);
    assert(pthread_create(&worker, NULL, reply_once, &emulator) == 0);
    assert(motor_set_cmd_one(dev, &command) == 0);
    assert(pthread_join(worker, NULL) == 0 && emulator.commands == 1);
    assert(motor_get_state_one(dev, &state) == 0 && state.temp == 30.0f);
    assert(motor_get_feedback_timestamp_one(dev, &before) == 0 && before > 0);
    assert(motor_get_state_one(dev, &state) == 0);
    assert(motor_get_feedback_timestamp_one(dev, &after) == 0 && before == after);
    command.mode = MOTOR_MODE_POS;
    assert(motor_set_cmd_one(dev, &command) < 0 && errno == ENOTSUP);
    command.mode = MOTOR_MODE_HYBRID;
    assert(motor_set_cmd_one(dev, &command) < 0 && errno == ETIMEDOUT);
    assert(motor_get_feedback_timestamp_one(dev, &after) == 0 && before == after);
    struct motor_transport_diagnostics diagnostics;
    assert(motor_get_transport_diagnostics(dev, &diagnostics) == 0);
    assert(diagnostics.transactions == 2 && diagnostics.timeouts == 1);
    assert(diagnostics.valid_frames == 1 && diagnostics.resync_discarded_bytes == 5);
    assert(diagnostics.failure_errno == ETIMEDOUT && diagnostics.failure_missing_ids == (1U << 4));
    assert(diagnostics.failure_sent_bytes == RS485_20_26_COMMAND_SIZE && diagnostics.failure_rx_bytes == 0);
    assert(diagnostics.poll_errors == 0);
    for (int mode = 1; mode <= 2; ++mode) {
        interrupt_writes = mode;
        write_attempts = 0;
        clock_advance_us = 0;
        assert(motor_set_cmd_one(dev, &command) < 0 && errno == ETIMEDOUT);
        interrupt_writes = 0;
        assert(write_attempts > 0 && write_attempts <= 5);
        assert(motor_get_transport_diagnostics(dev, &diagnostics) == 0);
        assert(diagnostics.timeouts == (uint64_t)(1 + mode));
        assert(diagnostics.failure_errno == ETIMEDOUT);
        assert(diagnostics.failure_sent_bytes == (uint32_t)(mode == 2 ? 3 : 0));
        assert(diagnostics.write_errors == 0);
    }
    motor_free(&dev, 1);
    close(master);
    return 0;
}
