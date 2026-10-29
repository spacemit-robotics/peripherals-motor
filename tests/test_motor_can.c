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
#ifdef HAVE_CAN_DM
int profile_can_dm(int argc, char **argv);
#endif
#ifdef HAVE_CAN_M152
int profile_can_m152d133(int argc, char **argv);
#endif
#ifdef HAVE_CAN_P1010
int profile_can_p1010b(int argc, char **argv);
#endif
#ifdef HAVE_CANOPEN
int profile_canopen_jmc(int argc, char **argv);
#endif
int main(int argc, char **argv) {
    const char *driver = driver_arg(argc, argv);
    if (!driver) {
        fprintf(stderr, "Specify exactly one --driver <name>; no motor has been started.\n");
        return 2;
    }
#ifdef HAVE_CAN_DM
    if (!strcmp(driver, "drv_can_dm")) return profile_can_dm(argc, argv);
#endif
#ifdef HAVE_CAN_M152
    if (!strcmp(driver, "drv_can_ddt_m152d133")) return run_fixed(argc, argv, profile_can_m152d133, "--if", "can0");
#endif
#ifdef HAVE_CAN_P1010
    if (!strcmp(driver, "drv_can_ddt_p1010b")) return profile_can_p1010b(argc, argv);
#endif
#ifdef HAVE_CANOPEN
    if (!strcmp(driver, "drv_canopen_jmc")) return profile_canopen_jmc(argc, argv);
#endif
    fprintf(stderr, "Unsupported or unbuilt driver: %s\n", driver);
    return 2;
}
