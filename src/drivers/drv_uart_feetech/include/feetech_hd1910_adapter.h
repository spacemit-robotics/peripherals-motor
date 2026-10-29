/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef FEETECH_HD1910_ADAPTER_H
#define FEETECH_HD1910_ADAPTER_H
#include <stdint.h>
/* Optional motor_args_uart::args. The structure is copied at allocation time.
 * zero_offset_ticks defaults to 2048; direction must be +1 or -1; firmware_p
 * defaults to 5 (the HD1910 self-trained deployment contract). */
struct feetech_hd1910_config {
    int32_t zero_offset_ticks;
    int32_t direction;
    uint8_t firmware_p;
};
/* All devices in a batch must be HD1910s on the same UART. */
struct motor_dev;
struct motor_cmd;
struct motor_state;
#ifdef __cplusplus
extern "C" {
#endif
int feetech_hd1910_set_cmds_sync(struct motor_dev **devs,
    const struct motor_cmd *cmds, uint32_t count);
/* On any missing, invalid or error reply returns -1 and leaves all states unchanged. */
int feetech_hd1910_get_states_sync(struct motor_dev **devs,
    struct motor_state *states, uint32_t count);
#ifdef __cplusplus
}
#endif
#endif
