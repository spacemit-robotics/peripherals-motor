# Motor 组件使用说明

## 项目简介

Motor 组件是一个统一的电机控制框架，为不同类型的电机（PWM、CAN、UART、EtherCAT）提供标准化的 API 接口。该组件解决了多协议电机驱动集成复杂、接口不统一的问题，通过抽象层设计实现"一次开发，多协议适配"的目标。

## 功能特性

### 支持的电机类型

**组件可灵活注册各类驱动以支持不同类型、不同型号的电机**，当前组件已支持型号如下
- **PWM 电机**：支持步进电机、直流电机等 PWM 控制类型
  - `pwm_demo` - 通用 PWM 驱动
  - `pwm_RoHS` - RoHS 步进电机驱动
- **CAN 电机**：支持 Can 总线通信的电机 
  - `dm_can` - 达妙 DM-j4310-2EC 驱动
- **UART 电机**：支持串口通信的舵机
  - `feetech` - Feetech STS3215 舵机驱动
  
- **EtherCAT 电机**：支持 EtherCAT 总线电机
  

### 控制模式
- `MOTOR_MODE_IDLE` - 空闲模式（电机自由转动）
- `MOTOR_MODE_VEL` - 速度闭环控制（rad/s）
- `MOTOR_MODE_POS` - 位置闭环控制（rad）
- `MOTOR_MODE_TRQ` - 纯力矩控制（Nm）
- `MOTOR_MODE_HYBRID` - MIT 风格阻抗控制

### API 特性
- **向量化 API**：支持多电机批量操作
- **单电机 API**：提供便捷的单电机操作接口
- **统一接口**：所有电机类型使用相同的 API
- **自动注册**：驱动程序自动发现和注册

## 快速开始

### 环境准备

#### 系统依赖
```bash
# Ubuntu/Debian
sudo apt-get install cmake build-essential libgpiod-dev

# CentOS/RHEL
sudo yum install cmake gcc gcc-c++ gpiod-devel
```

#### 硬件准备
- PWM 电机：连接 GPIO 和 PWM 输出
- CAN 电机：配置 CAN 接口（can0, can1 等）
- UART 电机：连接串口设备（/dev/ttyUSB0, /dev/ttyACM0 等）
- EtherCAT 电机：配置 EtherCAT 主站，本组件基于 **IGH 主站** 进行开发

### 构建编译

```bash
# 1. 进入 motor 组件目录
cd /path/to/motor

# 2. 创建构建目录
mkdir build && cd build

# 3. 配置编译（可选择特定驱动）
cmake .. -DSROBOTIS_PERIPHERALS_MOTOR_ENABLED_DRIVERS=drv_can_dm;drv_pwm_generic

# 4. 编译
make -j$(nproc)

```

### 运行示例

#### PWM 电机测试
```bash
# 运行 PWM 电机测试
./build/test_motor_pwm

# 运行 RoHS 步进电机测试
./build/test_motor_pwm
```

#### CAN 电机测试
```bash
# 默认配置测试（dm_can, can0, ID: 0x02, 0x03）
./build/test_motor_can

# 自定义配置
./build/test_motor_can --driver dm_can --if can0 --id 0x02 --id 0x03 --loops 100 --period_ms 50
```
**注意**：电机 ID 必须与 总线上电机的实际ID完全匹配

#### UART 舵机测试
```bash
# UART 舵机测试
./build/test_uart_feetech /dev/ttyACM1 1000000 1


```
**注意**：电机波特率与串口波特率必须一致，电机 ID 必须与硬件 ID 匹配

#### EtherCAT 电机测试
```bash
# EtherCAT 电机测试
./build/test_motor_ecat
```

## 详细使用

### API 简要说明

#### 核心数据结构

**电机控制命令结构体**
```c
struct motor_cmd {
    uint32_t mode;    // 控制模式
    float    pos_des; // 目标位置 (rad)
    float    vel_des; // 目标速度 (rad/s)  
    float    trq_des; // 目标力矩 (Nm) 或前馈力矩
    float    kp;      // 刚度增益 (HYBRID 模式)
    float    kd;      // 阻尼增益 (HYBRID 模式)
};
```

**电机状态反馈结构体**
```c
struct motor_state {
    float    pos;  // 当前位置 (rad)
    float    vel;  // 当前速度 (rad/s)
    float    trq;  // 当前力矩 (Nm)
    float    temp; // 温度 (°C)
    uint32_t err;  // 错误标志
};
```

#### 主要 API 函数

**向量化 API（推荐用于多电机控制）**
```c
// 批量初始化电机
int motor_init(struct motor_dev **devs, uint32_t count);
// devs: 电机设备数组指针, count: 电机数量

// 批量设置控制命令
int motor_set_cmds(struct motor_dev **devs, const struct motor_cmd *cmds, uint32_t count);
// devs: 电机设备数组, cmds: 命令数组, count: 电机数量

// 批量读取电机状态
int motor_get_states(struct motor_dev **devs, struct motor_state *states, uint32_t count);
// devs: 电机设备数组, states: 状态数组(输出), count: 电机数量

// 批量释放电机资源
void motor_free(struct motor_dev **devs, uint32_t count);
// devs: 电机设备数组指针, count: 电机数量
```

**电机工厂函数**
```c
// 创建 PWM 电机
struct motor_dev *motor_alloc_pwm(const char *name, uint32_t ch, void *args);
// name: 驱动名称, ch: PWM通道, args: 额外参数

// 创建 CAN 电机
struct motor_dev *motor_alloc_can(const char *name, const char *iface, uint32_t can_id, void *args);
// name: 驱动名称, iface: CAN接口, can_id: 电机ID, args: 额外参数

// 创建 UART 电机
struct motor_dev *motor_alloc_uart(const char *name, const char *dev_path, uint32_t baud, uint8_t id, void *args);
// name: 驱动名称, dev_path: 串口设备, baud: 波特率, id: 电机ID, args: 额外参数

// 创建 EtherCAT 电机  
struct motor_dev *motor_alloc_ecat(const char *name, uint16_t slave_idx, void *args);
// name: 驱动名称, slave_idx: 从站索引, args: 额外参数
```

**单电机便捷 API**
```c
// 单电机初始化
int motor_init_one(struct motor_dev *dev);
// dev: 单个电机设备指针

// 单电机设置命令
int motor_set_cmd_one(struct motor_dev *dev, const struct motor_cmd *cmd);
// dev: 电机设备, cmd: 控制命令

// 单电机读取状态
int motor_get_state_one(struct motor_dev *dev, struct motor_state *state);
// dev: 电机设备, state: 状态结构体(输出)
```

#### 控制模式说明

| 模式 | 常量 | 说明 | 适用场景 |
|------|------|------|----------|
| 空闲 | `MOTOR_MODE_IDLE` | 电机自由转动，无控制 | 电机停止、紧急停止 |
| 速度 | `MOTOR_MODE_VEL` | 速度闭环控制 | 恒速运动、调速控制 |
| 位置 | `MOTOR_MODE_POS` | 位置闭环控制 | 精确定位、轨迹跟踪 |
| 力矩 | `MOTOR_MODE_TRQ` | 纯力矩控制 | 力控应用、恒力输出 |
| 阻抗 | `MOTOR_MODE_HYBRID` | MIT 风格阻抗控制 | 柔性控制、人机交互 |

#### 使用原则

1. **优先使用向量化 API**：多电机控制时使用批量操作，提高效率
2. **错误处理**：所有 API 返回值应检查，负值表示错误
3. **资源管理**：使用 `motor_free()` 释放分配的电机资源
4. **模式切换**：切换控制模式前建议先设为 `MOTOR_MODE_IDLE`
5. **参数范围**：确保命令参数在电机允许范围内

### 基本使用流程

#### 1. 创建电机实例
```c
#include "motor.h"

// PWM 电机
struct motor_dev *pwm_motor = motor_alloc_pwm("pwm_demo", 1, NULL);

// CAN 电机
struct motor_dev *can_motor = motor_alloc_can("dm_can", "can0", 0x02, NULL);

// UART 舵机
struct motor_dev *uart_motor = motor_alloc_uart("feetech", "/dev/ttyACM0", 1000000, 1, NULL);

// EtherCAT 电机
struct motor_dev *ecat_motor = motor_alloc_ecat("ecat_demo", 1, NULL);
```

#### 2. 初始化电机
```c
// 单电机初始化
int ret = motor_init_one(pwm_motor);

// 批量初始化
struct motor_dev *motors[] = {pwm_motor, can_motor};
ret = motor_init(motors, 2);
```

#### 3. 设置控制命令
```c
// 位置控制
struct motor_cmd cmd = {
    .mode = MOTOR_MODE_POS,
    .pos_des = 3.14f,    // 目标位置：π rad
    .vel_des = 1.0f,     // 目标速度：1 rad/s
    .trq_des = 0.0f,     // 前馈力矩
    .kp = 0.0f,          // 刚度增益（HYBRID 模式使用）
    .kd = 0.0f           // 阻尼增益（HYBRID 模式使用）
};

// 单电机设置
motor_set_cmd_one(pwm_motor, &cmd);

// 批量设置
motor_set_cmds(motors, cmds, 2);
```

#### 4. 读取电机状态
```c
struct motor_state state;

// 单电机读取
motor_get_state_one(pwm_motor, &state);
printf("位置: %.3f rad, 速度: %.3f rad/s, 力矩: %.3f Nm\n", 
       state.pos, state.vel, state.trq);

// 批量读取
struct motor_state states[2];
motor_get_states(motors, states, 2);
```

#### 5. 释放资源
```c
// 单电机释放
motor_free(&pwm_motor, 1);

// 批量释放
motor_free(motors, 2);
```

### 高级用法

#### 多电机同步控制
```c
// 创建多个电机
struct motor_dev *motors[4];
for (int i = 0; i < 4; i++) {
    motors[i] = motor_alloc_can("dm_can", "can0", 0x02 + i, NULL);
}

// 批量初始化
motor_init(motors, 4);

// 同步控制
struct motor_cmd cmds[4];
for (int i = 0; i < 4; i++) {
    cmds[i].mode = MOTOR_MODE_POS;
    cmds[i].pos_des = i * 0.5f;  // 不同目标位置
    cmds[i].vel_des = 1.0f;
}

motor_set_cmds(motors, cmds, 4);
```

#### 阻抗控制（MIT 风格）
```c
struct motor_cmd cmd = {
    .mode = MOTOR_MODE_HYBRID,
    .pos_des = 1.57f,    // 目标位置
    .vel_des = 0.0f,     // 目标速度
    .trq_des = 0.1f,     // 前馈力矩
    .kp = 20.0f,         // 位置刚度
    .kd = 2.0f           // 速度阻尼
};

motor_set_cmd_one(motor, &cmd);
```

#### 实时控制循环
```c
// 50Hz 控制循环
while (running) {
    // 设置控制命令
    motor_set_cmds(motors, cmds, motor_count);
    
    // 读取状态反馈
    motor_get_states(motors, states, motor_count);
    
    // 处理状态数据
    for (int i = 0; i < motor_count; i++) {
        printf("Motor %d: pos=%.3f, vel=%.3f\n", i, states[i].pos, states[i].vel);
    }
    
    usleep(20000); // 50Hz = 20ms
}
```

## 常见问题

### 编译问题

#### Q: 找不到 libgpiod
```bash
# 解决方案：安装 libgpiod 开发包
sudo apt-get install libgpiod-dev  # Ubuntu/Debian
sudo yum install gpiod-devel       # CentOS/RHEL
```

#### Q: 驱动未找到
```bash
# 检查驱动文件是否存在
ls src/drivers/

# 确认驱动名称正确
cmake .. -DSROBOTIS_PERIPHERALS_MOTOR_ENABLED_DRIVERS=drv_can_dm
```

### 运行时问题

#### Q: PWM 电机无响应
```bash
# 检查 GPIO 权限
sudo usermod -a -G gpio $USER
# 或使用 sudo 运行
sudo ./test_motor_pwm
```

#### Q: CAN 接口无法打开
```bash
# 配置 CAN 接口
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can0 up
```

#### Q: UART 设备权限不足
```bash
# 添加用户到 dialout 组
sudo usermod -a -G dialout $USER
# 重新登录后生效
```

### 开发问题

#### 目录结构
```
motor
├── CMakeLists.txt
├── include
│   └── motor.h
├── LICENSE
├── NOTICE
├── package.xml
├── README.md
├── src
│   ├── drivers
│   │   ├── drv_can_dm
│   │   ├── drv_pwm_generic.c
│   │   ├── drv_pwm_RoHS.c
│   │   └── drv_uart_feetech
│   ├── motor_core.c
│   └── motor_core.h
└── test
    ├── test_motor_can.c
    ├── test_motor_pwm.c
    └── test_motor_uart.c

```



#### Q: 如何添加新的电机驱动？
1. 在 `src/drivers/` 目录下创建驱动目录
2. 实现驱动接口函数，匹配 motor.h 架构，详见 `src/drivers/motor_core.h`
3. 在 CMakeLists.txt 中添加编译规则
4. 使用 `MOTOR_DRIVER_REGISTER` 宏注册驱动



## 版本与发布

版本以本目录 `package.xml` 中的 `<version>` 为准。

| 版本   | 日期       | 说明 |
| ------ | ---------- | ---- |
| 0.1.0  | 2026-02-28 | 初始版本，支持 PWM、CAN、UART 电机。 |

**兼容性**：C99 / C++17（部分驱动），Linux（推荐 Ubuntu 20.04+），x86_64 / ARM64 / ARMv7。

## 贡献方式

欢迎参与贡献：提交 Issue 反馈问题，或通过 Pull Request 提交代码。

- **编码规范**：本组件 C/C++ 代码遵循 [Google C++ 风格指南](https://google.github.io/styleguide/cppguide.html)，请按该规范编写与修改代码，添加适当的注释和文档。
- **提交前检查**：请在提交前运行本仓库的 lint 脚本，确保通过风格检查：
  ```bash
  # 在仓库根目录执行（检查全仓库）
  bash scripts/lint/lint_cpp.sh

  # 仅检查本组件
  bash scripts/lint/lint_cpp.sh components/peripherals/motor
  ```
  脚本路径：`scripts/lint/lint_cpp.sh`。若未安装 `cpplint`，可先执行：`pip install cpplint` 或 `pipx install cpplint`。
- **提交说明**：报告问题时请在 Issue 中描述现象、复现步骤、环境信息及错误日志。贡献者与维护者名单见 `CONTRIBUTORS.md`。

## License

本组件源码文件头声明为 Apache-2.0，最终以本目录 `LICENSE` 文件为准。
