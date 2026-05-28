#!/usr/bin/env bash
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0

set -euo pipefail

function log_info() { echo -e "\033[32m[INFO] $1\033[0m"; }
function log_err()  { echo -e "\033[31m[ERROR] $1\033[0m"; }
function log_warn() { echo -e "\033[33m[WARN] $1\033[0m"; }

echo "=========================================="
echo "    Motor 组件 - 硬件手动测试脚本"
echo "=========================================="
echo "请确保硬件连接正常，观察对应电机的运动情况。"
echo ""

failed_count=0

function run_test_cmd() {
    local cmd="$1"
    log_info "执行命令: ${cmd}"
    
    if ! command -v "${cmd}" >/dev/null 2>&1; then
        log_err "命令 '${cmd}' 未找到 (可能未参与编译或未加入 PATH)!"
        failed_count=$((failed_count + 1))
        return 0
    fi

    if ! ${cmd}; then
        log_err "程序 '${cmd}' 未启动成功或执行报错！"
        log_warn "参数配置可能有误，请引导检查配置文件："
        log_warn " -> components/peripherals/motor/config/config_parameters.yaml"
        failed_count=$((failed_count + 1))
    fi
    sleep 2
}

# 1. pwm
log_info "测试 1/5: PWM 电机测试"
run_test_cmd "test_motor_pwm"

# 2. can
log_info "测试 2/5: CAN 电机测试 (Damiao, 默认 can0, ID: 2)"
run_test_cmd "test_motor_can"

# 3. uart - xl330
log_info "测试 3/5: UART 电机测试 (Reachy Mini xl330, 默认串口 /dev/ttyACM0)"
run_test_cmd "test_motor_xl330"

# 4. uart - feetech
log_info "测试 4/5: UART 舵机测试 (Feetech, 默认串口 /dev/ttyACM0)"
run_test_cmd "test_motor_uart"

# 5. ethercat
log_info "测试 5/5: EtherCAT 伺服电机测试 (默认两台电机)"
run_test_cmd "test_motor_ecat"

echo "=========================================="
if [[ ${failed_count} -gt 0 ]]; then
    log_err "全部手动测试流程执行完毕。共计有 ${failed_count} 个测试未通过！"
    exit 1
else
    log_info "全部手动测试流程执行完毕，无报错退出。"
    exit 0
fi
