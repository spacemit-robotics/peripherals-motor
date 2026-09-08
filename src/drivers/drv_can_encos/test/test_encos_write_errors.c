/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file test_encos_write_errors.c
 * @brief Offline SocketCAN error injection for Encos command writes.
 */

#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <stdarg.h>
#include <stddef.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include "motor.h"

#define TEST_SOCKET_FD 1234

static int write_errors[4];
static size_t write_calls;
static int short_write;

int __wrap_socket(int domain, int type, int protocol) {
    assert(domain == PF_CAN && type == SOCK_RAW && protocol == CAN_RAW);
    return TEST_SOCKET_FD;
}

int __wrap_ioctl(int fd, unsigned long request, ...) {
    va_list args;
    struct ifreq *interface;
    assert(fd == TEST_SOCKET_FD && request == SIOCGIFINDEX);
    va_start(args, request);
    interface = va_arg(args, struct ifreq *);
    interface->ifr_ifindex = 1;
    va_end(args);
    return 0;
}

int __wrap_fcntl(int fd, int command, ...) {
    assert(fd == TEST_SOCKET_FD);
    assert(command == F_GETFL || command == F_SETFL);
    return 0;
}

int __wrap_bind(int fd, const struct sockaddr *address, socklen_t size) {
    assert(fd == TEST_SOCKET_FD && address != NULL && size == sizeof(struct sockaddr_can));
    return 0;
}

int __wrap_close(int fd) {
    assert(fd == TEST_SOCKET_FD);
    return 0;
}

ssize_t __wrap_write(int fd, const void *buffer, size_t size) {
    const struct can_frame *frame = buffer;
    int error;
    assert(fd == TEST_SOCKET_FD && size == sizeof(*frame));
    assert(frame->can_id == 2);
    assert(write_calls < sizeof(write_errors) / sizeof(write_errors[0]));
    error = write_errors[write_calls++];
    errno = error;
    if (error != 0) return -1;
    return short_write ? (ssize_t)size - 1 : (ssize_t)size;
}

static void plan_writes(int first, int second, int third) {
    memset(write_errors, 0, sizeof(write_errors));
    write_errors[0] = first;
    write_errors[1] = second;
    write_errors[2] = third;
    write_calls = 0;
    short_write = 0;
}

int main(void) {
    const struct motor_option options[] = {
        {"model", "Encos EC-A8116-P1-18"},
        {"feedback_id", "2"},
        {"enable_on_init", "false"},
    };
    struct motor_dev *motor = motor_alloc_can_with_options("drv_can_encos", "test_can", 2,
        options, sizeof(options) / sizeof(options[0]));
    struct motor_cmd command = {0};
    assert(motor != NULL);
    assert(motor_init_one(motor) == 0);
    assert(write_calls == 1);

    plan_writes(ENOBUFS, 0, 0);
    assert(motor_set_cmd_one(motor, &command) == -1);
    assert(errno == ENOBUFS && write_calls == 1);

    plan_writes(0, 0, 0);
    short_write = 1;
    assert(motor_set_cmd_one(motor, &command) == -1);
    assert(errno == EIO && write_calls == 1);

    plan_writes(0, 0, 0);
    command.mode = MOTOR_MODE_OPEN;
    assert(motor_set_cmd_one(motor, &command) == -1);
    assert(errno == EOPNOTSUPP && write_calls == 0);

    command.mode = MOTOR_MODE_HYBRID;
    plan_writes(ENETDOWN, 0, 0);
    assert(motor_set_cmd_one(motor, &command) == -1);
    assert(errno == ENETDOWN && write_calls == 1);

    plan_writes(0, ENOBUFS, ENETDOWN);
    assert(motor_set_cmd_one(motor, &command) == -1);
    assert(errno == ENOBUFS && write_calls == 3);

    plan_writes(EAGAIN, 0, 0);
    assert(motor_set_cmd_one(motor, &command) == -1);
    assert(errno == EAGAIN && write_calls == 2);

    plan_writes(0, 0, 0);
    assert(motor_set_cmd_one(motor, &command) == 0);
    assert(errno == 0 && write_calls == 2);
    plan_writes(0, 0, 0);
    motor_free(&motor, 1);
    return 0;
}
