#!/bin/bash
# tests/test_functional.sh
# 非法参数注入测试

TOTAL_TESTS=0
FAILED_TESTS=0

function log_info() { echo -e "\033[32m[INFO] $1\033[0m"; }
function log_err() { echo -e "\033[31m[ERROR] $1\033[0m"; }

# 验证程序不挂起并正常退出
function test_timeout() {
    local cmd="$1"
    local desc="$2"
    ((TOTAL_TESTS++))
    log_info "--- 测试: $desc ---"
    
    # 设定 3 秒超时，使用 bash -c 避免直接变量展开的 SC2086 警告
    timeout 3 bash -c "$cmd" > /dev/null 2>&1
    local exit_code=$?

    # 124 代表 timeout 触发（即程序挂起了）
    if [ "$exit_code" -eq 124 ]; then
        log_err "FAIL: 程序挂起或阻塞 (Timeout) -> $cmd"
        FAILED_TESTS=$((FAILED_TESTS+1))
        return 1
    fi
    log_info "PASS: 程序未挂起并正常退出 (Exit code: $exit_code)"
    return 0
}

# 1. pwm
# 即使给 PWM 程序传入无关参数，它也应该很快退出（或报错退出），而不应阻塞
test_timeout "test_motor_pwm invalid_arg" "PWM 非法参数注入"

# 2. can
# - 输入一个非法 can 口，执行 can 电机测试程序
test_timeout "test_motor_can --driver drv_can_dm --if not_exist_can --id 2" "CAN 非法网口注入"

# 3. uart
# - xl330 输入非法 id、非法串口, 执行测试程序
test_timeout "test_motor_xl330 --port /dev/not_exist_port --id 10" "UART xl330 非法串口注入"
# - feetech 输入非法 id、非法串口，执行测试测试
test_timeout "test_motor_uart --port /dev/not_exist_port --baud 1000000 --driver drv_uart_feetech --id 1" "UART feetech 非法串口注入"

# 4. ethercat
# - 即使网络不存在或者参数非法，程序也应处理后立刻退出
test_timeout "test_motor_ecat --nums -1 -c -1" "EtherCAT 非法参数注入"

echo "=========================================="
if [ $FAILED_TESTS -eq 0 ]; then
    log_info "所有的 $TOTAL_TESTS 个非法注入测试全部通过！"
    exit 0
else
    log_err "测试失败！ 共计 $TOTAL_TESTS 个用例，其中 $FAILED_TESTS 个未通过（出现挂起）。"
    exit 1
fi
