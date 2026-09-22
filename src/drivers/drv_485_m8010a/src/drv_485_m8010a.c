/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */
#include "motor_core.h"

/* Implemented by libdrv_485_m8010a.so, built against the same motor ABI. */
extern struct motor_dev *m8010a_create(void *args);

REGISTER_MOTOR_DRIVER("drv_485_m8010a", DRV_TYPE_UART, m8010a_create)
