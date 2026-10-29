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
#ifdef HAVE_RS485_M601
int profile_rs485_m601(int argc, char **argv);
#endif
int main(int argc, char **argv) {
    const char *driver = driver_arg(argc, argv);
    if (!driver) {
        fprintf(stderr, "Specify exactly one --driver <name>; no motor has been started.\n");
        return 2;
    }
#ifdef HAVE_RS485_M601
    if (!strcmp(driver, "drv_485_ddt_m601c111")) return run_fixed(argc, argv, profile_rs485_m601, "--port", "/dev/ttyUSB0");
#endif
    fprintf(stderr, "Unsupported or unbuilt driver: %s\n", driver);
    return 2;
}
