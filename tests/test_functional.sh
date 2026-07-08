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
    
    local -a args
    read -r -a args <<< "$cmd"
    local tool="${args[0]}"
    local tool_path
    if ! tool_path=$(command -v "$tool" 2>/dev/null); then
        log_err "FAIL: 命令不存在 (未编译或不在 PATH) -> $tool"
        FAILED_TESTS=$((FAILED_TESTS+1))
        return 1
    fi

    # 设定 3 秒超时，并使用数组方式安全传递参数，避免分词风险
    timeout 3 "$tool_path" "${args[@]:1}" > /dev/null 2>&1
    local exit_code=$?

    # 124 代表 timeout 触发（即程序挂起了）
    # 127 代表 shell 找不到命令
    # 139 代表 段错误 (Segfault)
    if [ "$exit_code" -eq 124 ]; then
        log_err "FAIL: 程序挂起或阻塞 (Timeout) -> $cmd"
        FAILED_TESTS=$((FAILED_TESTS+1))
        return 1
    elif [ "$exit_code" -eq 127 ]; then
        log_err "FAIL: 命令在执行时未找到 (127) -> $cmd"
        FAILED_TESTS=$((FAILED_TESTS+1))
        return 1
    elif [ "$exit_code" -eq 139 ]; then
        log_err "FAIL: 程序发生崩溃/段错误 (139) -> $cmd"
        FAILED_TESTS=$((FAILED_TESTS+1))
        return 1
    elif [ "$exit_code" -eq 0 ]; then
        log_err "FAIL: 程序未能识别非法参数，错误地返回了成功状态 (0) -> $cmd"
        FAILED_TESTS=$((FAILED_TESTS+1))
        return 1
    fi
    log_info "PASS: 程序未挂起并正确抛错退出 (Exit code: $exit_code)"
    return 0
}

# 1. can
# - 输入一个非法 can 口，执行 can 电机测试程序
test_timeout "test_motor_can --driver drv_can_dm --if not_exist_can --id 2" "CAN 非法网口注入"

# 2. uart
# - xl330 输入非法 id、非法串口, 执行测试程序
test_timeout "test_motor_xl330 --port /dev/not_exist_port --id 10" "UART xl330 非法串口注入"
# - feetech 输入非法 id、非法串口，执行测试测试
test_timeout "test_motor_uart --port /dev/not_exist_port --baud 1000000 --driver drv_uart_feetech --id 1" "UART feetech 非法串口注入"

# 3. ethercat
# - 即使网络不存在或者参数非法，程序也应处理后立刻退出
test_timeout "test_motor_ecat --nums -1 -c -1" "EtherCAT 非法参数注入"

# 4. canopen
# - JMC CANopen 输入一个非法 can 口，执行测试程序
test_timeout "test_motor_canopen_jmc --driver drv_canopen_jmc --if not_exist_can --id 1" "CANopen JMC 非法网口注入"

echo "=========================================="
if [ $FAILED_TESTS -eq 0 ]; then
    log_info "所有的 $TOTAL_TESTS 个非法注入测试全部通过！"
    exit 0
else
    log_err "测试失败！ 共计 $TOTAL_TESTS 个用例，其中 $FAILED_TESTS 个未通过（出现异常：挂起、崩溃、命令缺失或未正确拒绝非法参数）。"
    exit 1
fi
