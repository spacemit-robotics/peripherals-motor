/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */
#define _XOPEN_SOURCE 600
#include "motor.h"

#include <assert.h>
#include <errno.h>
#include <dirent.h>
#include <fcntl.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static void test_init_failure(void) {
    /* Allocation/registration needs no hardware; /dev/null is not a UART. */
    struct motor_dev *dev = motor_alloc_uart(
        "drv_485_m8010a", "/dev/null", 4000000, 0, NULL);
    struct motor_dev *peer = motor_alloc_uart(
        "drv_485_m8010a", "/dev/null", 4000000, 1, NULL);
    assert(dev != NULL);
    assert(peer != NULL);
    /* ID 15 is rejected by the driver; this is not an allocation-count limit. */
    assert(motor_alloc_uart("drv_485_m8010a", "/dev/null", 4000000, 15, NULL) == NULL);
    errno = 0;
    assert(motor_init_one(dev) < 0);
    assert(errno == ENOTTY);
    motor_free(&dev, 1);
    motor_free(&peer, 1);
}

static unsigned int count_open_path(const char *path) {
    DIR *directory = opendir("/proc/self/fd");
    struct dirent *entry;
    unsigned int count = 0;

    assert(directory != NULL);
    while ((entry = readdir(directory)) != NULL) {
        char fd_path[512];
        char target[512];
        ssize_t length;

        if (entry->d_name[0] == '.')
            continue;
        snprintf(fd_path, sizeof(fd_path), "/proc/self/fd/%s", entry->d_name);
        length = readlink(fd_path, target, sizeof(target) - 1U);
        if (length < 0)
            continue;
        target[length] = '\0';
        if (strcmp(target, path) == 0)
            ++count;
    }
    closedir(directory);
    return count;
}

static void test_pty_lifecycle(void) {
    int master = posix_openpt(O_RDWR | O_NOCTTY);
    const char *slave;
    struct motor_dev *devs[2];

    assert(master >= 0);
    assert(grantpt(master) == 0);
    assert(unlockpt(master) == 0);
    slave = ptsname(master);
    assert(slave != NULL);
    assert(count_open_path(slave) == 0U);

    devs[0] = motor_alloc_uart("drv_485_m8010a", slave, 4000000, 0, NULL);
    devs[1] = motor_alloc_uart("drv_485_m8010a", slave, 4000000, 1, NULL);
    assert(devs[0] != NULL);
    assert(devs[1] != NULL);
    assert(motor_init(devs, 2) == 0);
    assert(count_open_path(slave) == 1U);

    /* Releasing one motor must keep the shared UART alive for its peer. */
    motor_free(&devs[0], 1);
    assert(count_open_path(slave) == 1U);
    assert(motor_init_one(devs[1]) == 0);
    motor_free(&devs[1], 1);
    assert(count_open_path(slave) == 0U);

    /* A new device must be able to open the bus after the last user releases it. */
    devs[0] = motor_alloc_uart("drv_485_m8010a", slave, 4000000, 0, NULL);
    assert(devs[0] != NULL);
    assert(motor_init_one(devs[0]) == 0);
    assert(count_open_path(slave) == 1U);
    motor_free(&devs[0], 1);
    assert(count_open_path(slave) == 0U);
    assert(close(master) == 0);
}

int main(void) {
    test_init_failure();
    test_pty_lifecycle();
    return 0;
}
