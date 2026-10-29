#!/usr/bin/env bash
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
set -euo pipefail
# One explicitly selected driver per invocation. This script can cause motion.
driver=""
for ((i=1; i<=$#; i++)); do
    if [[ "${!i}" == "--driver" ]]; then
        ((i+=1))
        if (( i > $# )) || [[ -n "$driver" ]]; then echo "Invalid --driver" >&2; exit 2; fi
        driver="${!i}"
    fi
done
case "$driver" in
    drv_can_dm|drv_can_ddt_m152d133|drv_can_ddt_p1010b|drv_canopen_jmc) tool=test_motor_can ;;
    drv_uart_feetech|drv_uart_feetech_sts3215|drv_uart_feetech_hd1910|drv_uart_xl330|drv_uart_ddt_m603c111) tool=test_motor_uart ;;
    drv_485_ddt_m601c111) tool=test_motor_485 ;;
    pwm_gpio|pwm_RoHS) tool=test_motor_pwm ;;
    drv_ethercat_jmc) tool=test_motor_ecat ;;
    *) echo "Specify a supported --driver; no motor started" >&2; exit 2 ;;
esac
exec "$tool" "$@"
