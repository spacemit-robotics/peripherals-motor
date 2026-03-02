# Feetech 舵机驱动

## 项目简介

Feetech 舵机驱动是基于统一 `motor.h` 框架的串口舵机控制组件，为 Feetech SMS/STS 系列舵机提供标准化的 C 语言接口。

**解决的问题：**
- 统一多电机控制接口，简化上层应用开发
- 自动管理串口资源共享，支持同一总线多电机通信
- 提供位置控制和速度控制两种工作模式
- 内置通信重试机制，提高通信可靠性

## 功能特性

**支持的功能：**
- ✅ 位置控制模式（精度 0.087°，范围 0-360°）
- ✅ 速度控制模式（范围 0-35 RPM）
- ✅ 实时状态反馈（位置、速度、负载、温度）
- ✅ 多电机同步控制
- ✅ 串口资源自动管理
- ✅ 通信失败自动重试（最多 3 次）



## 快速开始

### 环境准备

**硬件要求：**
- Feetech SMS/STS 系列舵机
- USB 转串口模块或板载串口
- Linux 系统（已测试 Ubuntu）

**软件依赖：**
- CMake >= 3.10
- GCC/G++ 编译器
- 串口设备访问权限

**权限配置：**
```bash
# 方式 1：临时权限（每次重启后失效）
sudo chmod 666 /dev/ttyACM1

# 方式 2：永久权限（推荐）
sudo usermod -aG dialout $USER
# 注销后重新登录生效
```

### 构建编译

```bash
cd components/peripherals/motor
mkdir build && cd build
cmake ..
make
```

### 运行示例

**基本使用示例：**

```c
#include "motor.h"

int main() {
    // 1. 创建电机实例（ID=1，波特率 1000000）
    uint32_t encoded_baud = (1000000 & 0xFFFFFF00) | 1;
    struct motor_dev *motor = motor_alloc_uart("feetech", "/dev/ttyACM1", encoded_baud, NULL);
    
    // 2. 初始化
    motor_init(&motor, 1);
    
    // 3. 位置控制
    struct motor_cmd cmd = {0};
    cmd.mode = MOTOR_MODE_POS;
    cmd.pos_des = 2048;  // 180°
    cmd.vel_des = 1200;  // 中速
    motor_set_cmds(&motor, &cmd, 1);
    
    // 4. 读取状态
    struct motor_state state;
    motor_get_states(&motor, &state, 1);
    printf("Position: %.2f, Temp: %.2f°C\n", state.pos, state.temp);
    
    // 5. 释放资源
    motor_free(&motor, 1);
    return 0;
}
```

**运行测试程序：**
```bash
# 位置/速度控制测试
sudo ./test_motor_uart /dev/ttyACM1 1000000

```

## 详细使用

### API 参考

#### 创建电机实例

### API 参考

#### 创建电机实例

```c
struct motor_dev* motor_alloc_uart(const char *driver_name, 
                                   const char *dev_path, 
                                   uint32_t baud,
                                   void *args);
```

**参数：**
- `driver_name`: 驱动名称，固定为 `"feetech"`
- `dev_path`: 串口设备路径，如 `"/dev/ttyACM1"`
- `baud`: 编码后的波特率，格式：`(波特率 & 0xFFFFFF00) | 电机ID`

**返回值：** 电机设备指针，失败返回 `NULL`

**示例：**
```c
// 创建 ID=1 的电机，波特率 1000000
uint32_t encoded = (1000000 & 0xFFFFFF00) | 1;
struct motor_dev *motor = motor_alloc_uart("feetech", "/dev/ttyACM1", encoded, NULL);
```

---

#### 初始化电机

```c
int motor_init(struct motor_dev **motors, uint32_t num);
```

**参数：**
- `motors`: 电机设备指针数组
- `num`: 电机数量

**返回值：** 成功返回 `0`，失败返回 `-1`

**说明：** 必须在发送控制命令前调用，用于建立与电机的通信连接

---

#### 设置控制命令

```c
int motor_set_cmds(struct motor_dev **motors, 
                   const struct motor_cmd *cmds, 
                   uint32_t num);
```

**参数：**
- `motors`: 电机设备指针数组
- `cmds`: 控制命令数组
- `num`: 电机数量

**motor_cmd 结构体：**
```c
struct motor_cmd {
    uint8_t  mode;     // 控制模式：0=位置控制, 1=速度控制
    float    pos_des;  // 目标位置 (0-4095)
    float    vel_des;  // 目标速度 (0-2400)
    float    trq_des;  // 目标力矩 (保留，暂未使用)
};
```

**返回值：** 成功返回 `0`，失败返回 `-1`

**说明：** 
- 位置控制模式下，`pos_des` 和 `vel_des` 都需要设置
- 速度控制模式下，只需设置 `vel_des`

---

#### 读取电机状态

```c
int motor_get_states(struct motor_dev **motors, 
                     struct motor_state *states, 
                     uint32_t num);
```

**参数：**
- `motors`: 电机设备指针数组
- `states`: 状态数据数组（输出）
- `num`: 电机数量

**motor_state 结构体：**
```c
struct motor_state {
    float    pos;   // 当前位置 (0-4095)
    float    vel;   // 当前速度 (0-2400)
    float    trq;   // 当前负载 (0-1000)
    float    temp;  // 当前温度 (℃)
    uint32_t err;   // 错误标志
};
```

**返回值：** 成功返回 `0`，失败返回 `-1`

**说明：** 可以周期性调用以监控电机状态

---

#### 释放资源

```c
void motor_free(struct motor_dev **motors, uint32_t num);
```

**参数：**
- `motors`: 电机设备指针数组
- `num`: 电机数量

**说明：** 程序退出前必须调用，用于释放串口资源和内存

---

### 控制模式说明

#### 位置控制模式 (mode = 0)

- 电机运动到指定位置后停止并保持
- 位置范围：0 - 4095 (对应 0° - 360°)
- 速度范围：0 - 2400 (约 0 - 35 RPM)

**示例：**
```c
cmd.mode = MOTOR_MODE_POS;  // 或 0
cmd.pos_des = 2048;         // 中间位置 (180°)
cmd.vel_des = 1200;         // 运动速度
```

#### 速度控制模式 (mode = 1)

- 电机以恒定速度持续旋转
- 速度范围：0 - 2400 (约 0 - 35 RPM)
- 位置会在 0-4095 范围内循环

**示例：**
```c
cmd.mode = 1;               // 速度控制
cmd.vel_des = 2400;         // 全速旋转

// 停止电机
cmd.vel_des = 0;
```

---

### 参数范围

| 参数 | 范围 | 单位 | 说明 |
|------|------|------|------|
| 位置 | 0 - 4095 | 0.087° | 对应 0° - 360° |
| 速度 | 0 - 2400 | 0.0146 RPM | 约 0 - 35 RPM |
| 负载 | 0 - 1000 | - | 相对值 |
| 温度 | - | ℃ | 实际温度 |
| 电压 | - | V | 实际电压 |
| 电流 | - | 6.5mA | 实际电流 |

---

### 注意事项

1. **串口共享**：同一串口上的多个电机共享一个 `FeetechPack` 实例，自动管理
2. **模式切换**：首次切换模式时会有短暂延迟（约 0.7s），后续切换更快
3. **初始化**：首次通信可能失败，驱动内部已实现自动重试（最多3次）
4. **ID 编码**：电机 ID 必须编码到波特率参数中：`(baud & 0xFFFFFF00) | motor_id`
5. **错误处理**：所有 API 返回 `-1` 表示失败，应检查返回值

## 常见问题

**Q1: 创建电机失败，返回 NULL**
```
A: 检查串口路径是否正确
   ls /dev/ttyACM*  # 查看可用串口
   sudo chmod 666 /dev/ttyACM1  # 临时授权
```

**Q2: 初始化失败，返回 -1**
```
A: 可能原因：
   1. 权限不足 → 使用 sudo 运行或加入 dialout 组
   2. 电机 ID 错误 → 确认电机实际 ID（默认为 1）
   3. 波特率不匹配 → 确认电机波特率配置（默认 1000000）
```

**Q3: 模式切换很慢（首次约 0.7s）**
```
A: 这是正常现象，首次切换需要写入 EEPROM
   后续切换会快很多（约 0.1s）
```

**Q4: 通信偶尔超时**
```
A: 驱动已内置重试机制（最多 3 次）
   如频繁超时，检查：
   1. 串口线缆质量
   2. 电源供电是否稳定
   3. 总线负载（建议单总线 ≤ 10 个电机）
```

**Q5: 如何控制多个电机？**
```c
// 同一串口多电机示例
struct motor_dev *motors[3];
for (int i = 0; i < 3; i++) {
    uint32_t encoded = (1000000 & 0xFFFFFF00) | (i + 1);
    motors[i] = motor_alloc_uart("feetech", "/dev/ttyACM1", encoded, NULL);
}
motor_init(motors, 3);
// ... 后续操作
```

**Q6: Permission denied 错误**
```bash
# 临时解决
sudo chmod 666 /dev/ttyACM1

# 永久解决
sudo usermod -aG dialout $USER
# 注销后重新登录
```

**Q7: 如何使用示例程序？**
```bash
# 示例位置：drv_uart_feetech/test/examples/SMS_STS/
# 包含：Broadcast, SyncRead, SyncWritePos, WritePos, WriteSpe 等

# 编译步骤
cd drv_uart_feetech/test/examples/SMS_STS/WritePos
mkdir build && cd build
cmake ..
make

# 运行
sudo ./WritePos /dev/ttyACM1
```

## 版本与发布

**当前版本：** v1.0.0

**变更记录：**
- v1.0.0 (2026-02-26)
  - 初始版本发布
  - 支持位置控制和速度控制模式
  - 实现串口资源自动管理
  - 添加通信重试机制

**兼容性说明：**
- 支持 Feetech SMS/STS 系列舵机
- 已测试型号：STS3215
- Linux 内核 >= 4.4
- 遵循统一 `motor.h` 接口规范

**已知问题：**
- 首次模式切换延迟较高（约 0.7s）
- 暂不支持力矩控制模式

## 贡献方式

我们欢迎各种形式的贡献：

**报告问题：**
- 在 Issue 中描述问题现象、复现步骤和环境信息
- 提供相关日志和错误信息

**提交代码：**
1. Fork 本仓库
2. 创建特性分支 (`git checkout -b feature/your-feature`)
3. 提交更改 (`git commit -am 'Add some feature'`)
4. 推送到分支 (`git push origin feature/your-feature`)
5. 创建 Pull Request

**代码规范：**
- 遵循项目现有代码风格
- 添加必要的注释和文档
- 确保通过 cpplint 检查

贡献者与维护者名单见：`CONTRIBUTORS.md`

## License

本组件源码文件头声明为 Apache-2.0，最终以本目录 `LICENSE` 文件为准。

---

## 附录：技术细节
### 独立功能验证
```

### 示例程序列表

位于 `drv_uart_feetech/test/examples/SMS_STS/`：

| 示例 | 功能说明 |
|------|----------|
| WritePos | 单电机位置控制 |
| WriteSpe | 单电机速度控制 |
| SyncWritePos | 多电机同步位置控制 |
| SyncRead | 多电机同步状态读取 |
| Broadcast | 广播命令（所有电机） |
| RegWritePos | 寄存器写入位置 |
| FeedBack | 状态反馈测试 |
| Ping | 电机连接测试 |
| CalibrationOfs | 零点校准 |
| ProgramEprom | EEPROM 编程 |

**编译示例程序：**

1. 编译源码生成静态库
```bash
cd src
mkdir build && cd build
cmake ..
make
```

2. 编译示例程序
```bash
cd drv_uart_feetech/test/examples/SMS_STS/WritePos
mkdir build && cd build
cmake ..
make
```

3. 运行示例
```bash
sudo ./WritePos /dev/ttyACM1
```

### 驱动源码结构

```
components/peripherals/motor/src/drivers/drv_uart_feetech/
├── CMakeLists.txt
├── include
│   ├── feetech_pack.h
│   ├── HLSCL.h
│   ├── INST.h
│   ├── SCSCL.h
│   ├── SCSerial.h
│   ├── SCServo.h
│   ├── SCS.h
│   └── SMS_STS.h
├── LICENSE
├── src
│   ├── CMakeLists.txt
│   ├── feetech_motor_adapter.cpp
│   ├── feetech_pack.cpp
│   ├── HLSCL.cpp
│   ├── SCSCL.cpp
│   ├── SCS.cpp
│   ├── SCSerial.cpp
│   └── SMS_STS.cpp
└── test
    ├── examples
    ├── test_debug.cpp
    └── test_simple.c
```