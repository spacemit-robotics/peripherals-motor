/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <string.h>
static const char *driver_arg(int argc, char **argv) {
    const char *driver = NULL;
    for (int i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "--driver")) {
            if (driver || ++i >= argc) return NULL;
            driver = argv[i];
        }
    }
    return driver;
}
static int run_fixed(int argc, char **argv, int (*profile)(int, char **), const char *option, const char *fallback) {
    const char *path = fallback;
    for (int i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "--driver")) { ++i; continue; }
        if (!strcmp(argv[i], option) && i + 1 < argc) { path = argv[++i]; continue; }
        fprintf(stderr, "Unsupported option for this profile: %s\n", argv[i]);
        return 2;
    }
    char *args[] = {argv[0], (char *)path, NULL};
    return profile(2, args);
}
#ifdef HAVE_FEETECH
int profile_uart_feetech(int argc, char **argv);
#endif
#ifdef HAVE_FEETECH
int profile_uart_feetech(int argc, char **argv);
#endif
#ifdef HAVE_FEETECH
int profile_uart_feetech(int argc, char **argv);
#endif
#ifdef HAVE_XL330
int profile_uart_xl330(int argc, char **argv);
#endif
#ifdef HAVE_UART_M603
int profile_uart_m603(int argc, char **argv);
#endif
int main(int argc, char **argv) {
    const char *driver = driver_arg(argc, argv);
    if (!driver) {
        fprintf(stderr, "Specify exactly one --driver <name>; no motor has been started.\n");
        return 2;
    }
#ifdef HAVE_FEETECH
    if (!strcmp(driver, "drv_uart_feetech")) return profile_uart_feetech(argc, argv);
#endif
#ifdef HAVE_FEETECH
    if (!strcmp(driver, "drv_uart_feetech_sts3215")) return profile_uart_feetech(argc, argv);
#endif
#ifdef HAVE_FEETECH
    if (!strcmp(driver, "drv_uart_feetech_hd1910")) return profile_uart_feetech(argc, argv);
#endif
#ifdef HAVE_XL330
    if (!strcmp(driver, "drv_uart_xl330")) return profile_uart_xl330(argc, argv);
#endif
#ifdef HAVE_UART_M603
    if (!strcmp(driver, "drv_uart_ddt_m603c111")) return run_fixed(argc, argv, profile_uart_m603, "--port", "/dev/ttyUSB0");
#endif
    fprintf(stderr, "Unsupported or unbuilt driver: %s\n", driver);
    return 2;
}
