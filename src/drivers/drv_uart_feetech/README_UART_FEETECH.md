# Feetech 舵机驱动

## 项目简介

Feetech 舵机驱动基于统一 `motor.h` 框架。STS3215 适配注册名为 `drv_uart_feetech_sts3215`，旧名 `drv_uart_feetech` 保持兼容；HD1910 使用独立注册名 `drv_uart_feetech_hd1910`。前面的通用位置与速度示例主要适用于 STS3215，HD1910 的模式、接口和测试用法见[专用驱动](#hd1910-专用驱动)。

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
    struct motor_dev *motor = motor_alloc_uart("drv_uart_feetech_sts3215",
                                                "/dev/ttyACM1", 1000000, 1, NULL);
    
    // 2. 初始化
    motor_init(&motor, 1);
    
    // 3. 位置控制
    struct motor_cmd cmd = {0};
    cmd.mode = MOTOR_MODE_POS;
    cmd.pos_des = 3.14159f;  // 180°，单位：弧度
    cmd.vel_des = 1.5f;      // 目标角速度，单位：rad/s
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
sudo ./test_motor_uart --driver drv_uart_feetech_sts3215 --port /dev/ttyACM1 --baud 1000000 --id 1

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
                                   uint8_t id,
                                   void *args);
```

**参数：**
- `driver_name`: STS3215 为 `"drv_uart_feetech_sts3215"`（兼容旧名 `"drv_uart_feetech"`），HD1910 为 `"drv_uart_feetech_hd1910"`
- `dev_path`: 串口设备路径，如 `"/dev/ttyACM1"`
- `baud`: 串口波特率，例如 `1000000`
- `id`: 舵机总线 ID，与波特率分开传递

**返回值：** 电机设备指针，失败返回 `NULL`

**示例：**
```c
// 创建 ID=1 的 STS3215，波特率 1000000
struct motor_dev *motor = motor_alloc_uart("drv_uart_feetech_sts3215",
                                            "/dev/ttyACM1", 1000000, 1, NULL);
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
    float    pos_des;  // 目标位置 (单位：rad，范围 0~2π)
    float    vel_des;  // 目标速度 (单位：rad/s)
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
    float    pos;   // 当前位置 (单位：rad)
    float    vel;   // 当前速度 (单位：rad/s)
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
- 位置输入范围：0 - 2π（单位：rad），内部会自动映射到 0-4095
- 速度输入：任意非负 rad/s 值，驱动内部自动映射到原始速度寄存器
    - 建议范围：0 ~ 4 rad/s（对应 Feetech 最大约 35 RPM），超过后驱动会自动饱和

**示例：**
```c
cmd.mode = MOTOR_MODE_POS;  // 或 0
cmd.pos_des = 3.14159f;     // 中间位置 (180°)，单位：rad
cmd.vel_des = 1.5f;         // 运动速度，单位：rad/s
```

#### 速度控制模式 (mode = 1)

- 电机以恒定速度持续旋转
- 速度输入：任意非负 rad/s 值
- 位置会在 0-2π（映射到 0-4095）范围内循环
    - 建议范围：0 ~ 4 rad/s，内部超过后会被限制在硬件允许的 35 RPM 左右

**示例：**
```c
cmd.mode = 1;               // 速度控制
cmd.vel_des = 3.67f;        // 示例速度，单位：rad/s

// 停止电机
cmd.vel_des = 0;
```

---

### 参数范围

| 参数 | 范围 | 单位 | 说明 |
|------|------|------|------|
| 位置 | [0, 2π) | rad | 对应 0° - 360°，内部会自动映射到 0-4095 |
| 速度 | [0, 4.0] | rad/s | 对应 0 - 35 RPM，上层可输入更大数值但会被限制 |
| 加速度 | 固定 50 | - | 当前驱动默认值，可在源码中修改 |
| 负载 | 0 - 1000 | - | 相对值 |
| 温度 | - | ℃ | 实际温度 |
| 电压 | - | V | 实际电压 |
| 电流 | - | 6.5mA | 实际电流 |

---

### 注意事项

1. **串口共享**：同一串口上的多个电机共享一个 `FeetechPack` 实例，自动管理
2. **模式切换**：首次切换模式时会有短暂延迟（约 0.7s），后续切换更快
3. **初始化**：首次通信可能失败，驱动内部已实现自动重试（最多3次）
4. **ID 参数**：电机 ID 是 `motor_alloc_uart` 的独立参数，不需要编码到波特率中
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
    motors[i] = motor_alloc_uart("drv_uart_feetech_sts3215",
                                 "/dev/ttyACM1", 1000000, i + 1, NULL);
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
│   ├── feetech_sts3215_adapter.cpp
│   ├── feetech_hd1910_adapter.cpp
│   └── common/
│       ├── feetech_pack.cpp
│       ├── HLSCL.cpp
│       ├── SCSCL.cpp
│       ├── SCS.cpp
│       ├── SCSerial.cpp
│       └── SMS_STS.cpp
└── test
    ├── examples
    ├── test_debug.cpp
    └── test_simple.c
```

## HD1910 专用驱动

HD1910 使用独立注册名 `drv_uart_feetech_hd1910`，不经过通用 SMS/STS 的模式切换。该适配在内部实现 FT-SCS 串口事务：按完整请求帧过滤可选的本机回显，接收完整应答并检查 ID、长度、校验和与状态字节；其他 Feetech 驱动仍使用公共协议层。适配采用 HD1910 的 42–47 六字节目标位置块、位置反馈有符号解码，以及使能前增益/力矩限制读回确认。

### HD-1910-C001 厂家规格

以下数据来自飞特《HD-1910-C001 产品规格书》A/0（2026-09-07），产品名称为
HD1910M。规格参数可能随固件和出厂配置变化，部署时应以舵机实际控制表为准。

| 项目 | 厂家规格 |
| --- | --- |
| 工作电压 | 4–8.4 V；规格列出 4.8 V、6 V、7.4 V 三个典型测试点 |
| 运行温度 | -20–60 ℃ |
| 保存温度 | -30–80 ℃ |
| 6 V 性能 | 空载速度 0.109 s/60°（92 RPM）；堵转扭矩 12 kg·cm；堵转电流 1.6 A；额定负载 3 kg·cm；额定电流 690 mA |
| 通信 | TTL 半双工异步串行，8 数据位、1 停止位、无校验 |
| ID 与波特率 | ID 0–253，出厂默认 1；38400 bps–1 Mbps，出厂默认 1 Mbps |
| 位置 | 中位 2048；0–4095 对应 360°；分辨率 0.088°/pulse |
| 厂家反馈项 | 输入电压、负载、工作电流、工作速度、工作温度、位置 |
| 最大位置更新率 | 1 kHz |

运行温度上限 60 ℃是厂家规定的环境使用条件；80 ℃是过热保护动作阈值。80 ℃不能
作为允许持续运行的温度。堵转扭矩和堵转电流也是短时极限参数，不应当作连续工作点。

### 电子保护

| 保护项 | 厂家行为 | 出厂状态与恢复 |
| --- | --- | --- |
| 过流 | 运行电流大于 0.5 A 并持续 2 s 后关闭输出，进入过流保护；电流阈值和持续时间可配置 | 出厂默认关闭；重新发送位置指令会清除过流保护标志 |
| 电压 | 大于 10 V 或小于 4 V 时进入保护；保护电压范围可配置 | 出厂默认关闭；电压恢复正常范围后自动解除 |
| 过热 | 大于 80 ℃时关闭扭矩输出 | 出厂默认开启；规格书未说明恢复温度、温度滞回或是否需要重新使能扭矩 |

当前 HD1910 适配器会在 `motor_state.temp` 中返回温度，但尚未在软件中主动实施 60 ℃运行
上限或 80 ℃停机策略；成功反馈的 `motor_state.err` 当前置为 0，也未公开输入电压和工作
电流。上层应持续监控温度，并在接近运行上限时停止运动和发送 IDLE，不能依赖硬件过热
关断作为日常温控手段。若要依赖过流或电压保护，应先确认实际舵机已启用相应保护。

### 厂家运行模式

| 模式 | 厂家说明 | 当前 HD1910 适配器 |
| --- | --- | --- |
| 模式 0：角度伺服 | 0–360°绝对位置，可基于目标电流限力 | 未由适配器切换或公开配置 |
| 模式 1：电机恒速 | 随负载增加维持设定速度 | 不支持 `MOTOR_MODE_VEL` |
| 模式 2：电机恒流 | 随负载增加维持设定电流 | 不支持 `MOTOR_MODE_TRQ` |
| 模式 3：PWM 开环调速 | 占空比调压调速，无速度闭环，转速随负载波动 | 不支持 |
| 模式 4：纯位置 PD（Sim2Real） | 比例-微分位置闭环；规格书标注为出厂默认模式 | 适配器仅发送位置目标，不写模式寄存器 |
| 多圈模式 | 最高精度下支持正负 7 圈绝对位置；掉电不保存圈数；降低分辨率可扩大圈数 | 当前适配器限制目标为 0–4095，不提供多圈控制 |

当前适配器只接受 `MOTOR_MODE_POS` 和 `MOTOR_MODE_IDLE`。初始化不会读取或切换厂家模式，
因此运行前应确认舵机仍处于与部署固件和控制策略相符的模式；不要把厂家模式编号直接作为
`motor_mode` 枚举值传入。

### Microduck 训练指定模式与 PD 调整

Microduck 的 HD-1910 训练与部署按厂家 **模式 4「纯位置 PD（Sim2Real）」** 使用；
它是舵机控制表地址 33（`0x21`）的值，不是 `motor_cmd.mode`。本适配器的
`MOTOR_MODE_POS` 只表示发送位置目标，**不会设置或核对地址 33**。装机或更换舵机后，
先在舵机卸力时读回模式，确认值为 4；如果不是 4，应在单颗舵机上完成模式配置、重新读回，
再接入训练策略。通用 STS 内存表列出的模式范围是 0–3，HD-1910-C001 的模式 4 是
型号定制项，不能按通用表把它改成模式 0。

PD 有两组地址，作用不同：

| 地址（十进制） | 用途 | 本适配器行为 |
| --- | --- | --- |
| 21 / 22 | EEPROM 中的默认 P / D；23 是 I | 运行时不修改 |
| 50 / 51 | HD-1910 定制固件的运行时 RAM P / D | 上力前把 P 写为 `feetech_hd1910_config.firmware_p`、D 写为 0，并逐项读回；卸力时恢复原 P，D 保持 0 |
| 48–49 | 运行时 RAM 转矩限制，小端 `uint16` | 上力前写 1000 并读回 |

`firmware_p` 应与所用训练模型的 `kp_fw` 合约一致，默认值 5 只是适配器的缺省值，
不能替代模型核对。当前 `feetech_hd1910_config` 只提供 P 配置；D=0 和转矩限制 1000
在适配器中固定，**没有通过 `motor.h` 提供任意 D 调节或厂家模式切换 API**。
需要试验其他运行时 P/D 时，应在卸力、独占串口的条件下通过独立串口工具写 RAM 并读回；
之后再次通过本适配器上力会将它们恢复为上述部署值。改 EEPROM 默认 P/D 会持久改变
舵机参数，不能当作日常调参方式。

以下是 **ID 20（`0x14`）** 的 FT-SCS 十六进制帧示例。使用 1 Mbps、8N1、TTL 半双工，
串口工具须按十六进制字节发送；更换 ID、地址或数值时须重算末尾校验和：
`CHK = ~(ID + LEN + INST + 所有参数) & 0xFF`，`LEN = 参数字节数 + 2`。

| 操作 | 发送帧 | 预期读回 |
| --- | --- | --- |
| 读厂家模式（地址 33） | `FF FF 14 04 02 21 01 C3` | 数据字节 `04` |
| 关扭矩（地址 40） | `FF FF 14 04 03 28 00 BC` | 写入后读回 `00` |
| 解锁 EEPROM（地址 55） | `FF FF 14 04 03 37 00 AD` | 写入后读回 `00` |
| 将厂家模式设为 4 | `FF FF 14 04 03 21 04 BF` | 再读地址 33，应为 `04` |
| 重新上锁 EEPROM | `FF FF 14 04 03 37 01 AC` | 写入后读回 `01` |
| 临时设运行时 P5 / D0（地址 50–51） | `FF FF 14 05 03 32 05 00 AC` | 用下一帧读取 |
| 读运行时 P / D | `FF FF 14 04 02 32 02 B1` | 数据字节 `05 00` |
| 临时设限力 1000 / P5 / D0（地址 48–51） | `FF FF 14 07 03 30 E8 03 05 00 C1` | 读地址 48、长度 4，应为 `E8 03 05 00` |
| 读运行时限力 / P / D | `FF FF 14 04 02 30 04 B1` | 数据字节 `E8 03 05 00` |

厂家模式和 EEPROM P/D 写入时先关扭矩、解锁，写完上锁并读回；模式 4
只在配置不符时写一次。运行时 RAM 48–51 不需要解锁。单播 `WRITE` 帧在应答级别为 1 时
应有状态应答，**应答成功仍需读回核对**。不要在 `robotd` 或本驱动占用同一串口时手动发帧，
也不要向已装机的多舵机总线广播模式切换。上述 RAM 50/51 是 HD-1910 定制用法，
通用 STS 内存表将 50–54 标为“无定义”。

| 项目 | HD1910 行为 |
| --- | --- |
| 串口 | 1 Mbps；总线 ID 0–253；半双工转换由 UART 模块或外部电路负责 |
| 位置控制 | `MOTOR_MODE_POS`，`pos_des` 单位 rad；相对零点换算为 0–4095 ticks，目标写入 42–47（位置、电流 0、速度 0） |
| 使能 | 检查扭矩状态与 P 值；设置 D=0、力矩限制 1000；首次使能先以当前位置为目标，再开启扭矩 |
| 卸力 | `MOTOR_MODE_IDLE` 关闭扭矩，并在需要时恢复原 P 值 |
| 反馈 | 读取位置、速度、负载、温度；串口事务失败返回错误 |

上力过程中发生通信或校验失败时，驱动会尝试关闭扭矩并读回确认，再恢复原 P；
如果关扭矩或恢复失败，命令返回错误并保留待恢复状态，调用方应在保持串口可用时重试
`MOTOR_MODE_IDLE`。释放对象时也会尝试清理并在失败时输出错误；释放接口没有返回值，
不能仅凭释放成功判断舵机已卸力。批量上力中途失败时，会回滚本次新上力的舵机。

无硬件回归测试通过伪串口模拟应答丢失及关扭矩失败：启用
`BUILD_TESTS` 和 `drv_uart_feetech_hd1910` 构建后，执行
`ctest --test-dir <构建目录> -R hd1910_rollback --output-on-failure`。

`vel_des` 不参与 HD1910 的位置命令；`MOTOR_MODE_VEL` 等其他控制模式不受支持。零点、方向和 P 契约必须与机构实际配置一致。

```c
struct feetech_hd1910_config config = {
    .zero_offset_ticks = 2048, .direction = 1, .firmware_p = 5
};
struct motor_dev *motor = motor_alloc_uart("drv_uart_feetech_hd1910",
                                            "/dev/ttyACM1", 1000000, 1, &config);
```

可选 `args` 指向 `feetech_hd1910_config`（见 `include/feetech_hd1910_adapter.h`），用于设置 `zero_offset_ticks`、`direction` 和策略契约 P 值；默认分别为 2048、+1 和 5。请先按实际机构确认零位、方向、ID 和反馈，再发送位置命令。

### 多舵机同步读写

包含 `feetech_hd1910_adapter.h`，使用 HD1910 专用批量接口：

```c
/* motors 和 cmds 的顺序一一对应，所有舵机使用同一个串口。 */
int ret = feetech_hd1910_set_cmds_sync(motors, cmds, count);
if (ret == 0)
    ret = feetech_hd1910_get_states_sync(motors, states, count);
```

同步写只接受 `MOTOR_MODE_POS`，一帧 `SYNC WRITE`（`0x83`）包含各舵机的 ID 和
地址 42–47 的位置目标，最多 35 颗。首次上力仍逐颗检查和配置参数、对齐目标及使能；
随后目标合并发送。上力中途失败时之前已上力的舵机可能保持使能，应由调用方检查并卸力。
同步写无舵机应答，返回成功只表示帧已发送。

同步读发送一帧 `SYNC READ`（`0x82`），从地址 56 读取每颗 15 字节反馈，按应答 ID
归并，最多 251 颗；全部有效才更新 `states`，缺包、错误状态或超时返回失败并保留原数组。
两种接口都要求非空、ID 不重复且全部属于同一串口。通用 `motor_set_cmds()` 和
`motor_get_states()` 仍逐颗调用，不会自动使用这两个接口。2026-10-29 在 `/dev/ttyACM0`
对 ID 10–14、20–24、30–34 做过 5 轮同步读取，15 颗均有反馈；仅验证了短时读取，
驱动同步写接口上力后的运动和长期总线时序尚未实机验证。

`test/test_hd1910_feedback.c` 为同步运动与反馈用例，在 `drv_uart_feetech` 目录构建：

```bash
cmake -S . -B build
cmake --build build --target test_hd1910_feedback
./build/test_hd1910_feedback /dev/ttyACM0 31 32 33
```

最后一行会**使能舵机并实际运动**：程序先确认每颗舵机已卸力并读取初始位置，再向每颗
同步发送相对初始位置 +0.1 rad 的目标；每 0.5 秒读取一次，共 5 次，输出 ID、位置、
位移、目标、速度、负载和温度。最后一次位置与目标相差超过 0.02 rad、通信失败或
卸力无法确认时返回非零。运动尝试后会逐颗尝试卸力并读回；异常时检查所有舵机的扭矩状态。

运行前确认各关节有足够运动空间且目标在允许范围内，列表中每个 ID 都必须在线且唯一。
默认零位 2048、方向 +1、P=5 仅为用例配置，不代表实际机构标定或训练模型参数。
出现 `Synchronous feedback failed` 时，先逐颗核对 ID、供电和连接；同步读取要求所有
列出的 ID 都返回有效反馈。例如 `./build/test_hd1910_feedback /dev/ttyACM0 31` 也会
驱动 ID 31 移动 0.1 rad，不能当作只读探测。上文的 `test_hd1910_smoke` 适合单颗只读反馈检查。

硬件冒烟用例位于 `test/test_hd1910_smoke.c`。在本目录运行 `cmake -S . -B build && cmake --build build --target test_hd1910_smoke` 可单独构建，运行方式为 `test_hd1910_smoke [uart-device] [motor-id]`。根工程默认不编译 `drv_uart_feetech/test` 下的硬件用例；如需从 motor 根目录构建，显式设置 `-DBUILD_TESTS=ON -DMOTOR_BUILD_FEETECH_DRIVER_TESTS=ON -DSROBOTIS_PERIPHERALS_MOTOR_ENABLED_DRIVERS=drv_uart_feetech_hd1910`。该用例以 2 Hz 读取反馈，持续 5 分钟；初始化或单次读取失败时仍继续发送请求，最后发送 IDLE 收尾命令，不发送位置目标。每次读取只发送一个请求，不因回显而重发。半双工总线的驱动释放必须由 UART 模块或外部转换电路完成；回显过滤不改变引脚电气状态。


小位移用例位于 `test/test_hd1910_small_move.c`，目标名为 `test_hd1910_small_move`。运行 `test_hd1910_small_move /dev/ttyACM0 1`：读取当前位置后，向配置零点方向发送一次 0.3 rad（约 17°）位置命令，最多等待 3 秒确认反馈进入目标 ±0.02 rad，最后发送 IDLE 卸力。测试会实际驱动舵机，运行前需确保机构有足够活动空间。可在本目录单独构建，或按上述选项从 motor 根目录显式启用。

根工程的 `test_motor_uart` 支持通过 `--driver` 选择舵机适配器。HD1910 示例：`test_motor_uart --driver drv_uart_feetech_hd1910 --port /dev/ttyACM0 --baud 1000000 --id 1 --move-rad 0.3`。`--move-rad` 表示相对当前位置的有符号位移，必须显式指定，允许范围为 −0.5～0.5 rad；无效或超范围时在打开串口前退出。此模式只运行单舵机位置控制流程，不执行 STS3215 的参数写入或速度测试。
