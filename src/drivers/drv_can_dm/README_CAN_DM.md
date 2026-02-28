# 达妙 CAN 电机驱动

## 项目简介

达妙 CAN 电机驱动是基于统一 `motor.h` 框架的 CAN 总线电机控制组件，为达妙（Damiao）系列电机提供标准化的控制与状态反馈接口。

**解决的问题：**
- 统一 CAN 电机驱动入口，便于上层应用接入
- 提供标准化的控制模式与状态读取接口
- 封装 SocketCAN 通信细节，简化集成成本

## 功能特性

**支持的功能：**
- ✅ 支持达妙系列电机（DM4310）
- ✅ 支持 MIT 控制模式（位置/速度/力矩）
- ✅ CAN 总线多电机配置与管理
- ✅ 电机状态反馈（位置/速度/力矩）


## 快速开始

### 环境准备

**硬件要求：**
- 达妙系列 CAN 电机，DM-j4310-2EC
- CAN 适配器（SocketCAN 支持）
- Linux 系统（支持 SocketCAN）

**软件依赖：**
- CMake >= 3.16
- GCC/G++ 编译器
- 已配置 CAN 接口（如 can0）

**CAN 口配置示例：**
```bash
# 以 1Mbps 为例
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can0 up
```

**注意：can 口通信频率必须与电机频率一致**

### 构建编译

```bash
cd components/peripherals/motor
mkdir -p build && cd build
cmake ..
make
```

### 运行示例

**使用统一 CAN 测试程序：**
```bash
# 默认 DM 驱动，can0，默认 ID=0x02/0x03
sudo ./test_motor_can

# 指定驱动、接口与多个电机 ID
sudo ./test_motor_can --driver dm_can --if can0 --id 0x02 --id 0x03
```
**注意： 程序内定义的电机 ID 必须与总线上的 电机实际 ID 完全匹配**



**直接运行驱动测试程序：**
```bash
# 进入驱动目录构建
cd components/peripherals/motor/src/drivers/drv_can_dm
mkdir -p build && cd build
cmake ..
make

# 运行测试（示例程序 test_dm_motor）
./test_dm_motor
```

## 详细使用

详细 API 和控制模式说明请参考后续官方文档：
- 达妙电机官方手册
- 项目统一 `motor.h` 接口文档

### 参数配置说明

**电机配置项（示例代码：`test/test.cpp`）：**
- `bus_name`：CAN 总线名称，例如 `can0`
- `can_id`：电机 CAN ID（与电机实际 ID 一致）
- `master_id`：主机 ID（部分电机需要，按电机手册设置）
- `motor_type`：电机型号（如 `DM4310`）
- `control_mode`：控制模式（如 `MIT_MODE`）


**MIT 直接控制参数（示例接口：`send_mit_command`）：**
- `pos`：期望位置（rad）
- `vel`：期望速度（rad/s）
- `torque`：期望力矩（Nm）
- `kp`：位置刚度增益
- `kd`：阻尼增益

## 常见问题

**Q1: 初始化失败，无法通信**
```
A: 确认 can0 已正确配置并 up
   sudo ip link set can0 up type can bitrate 1000000
```

**Q2: 电机无响应**
```
A: 检查 CAN ID 是否正确，驱动默认不会扫描 ID
```

**Q3: 多电机控制异常**
```
A: 确认总线上 ID 不冲突，线缆与终端电阻正确
```

## 版本与发布

**当前版本：** v1.0.0

**变更记录：**
- v1.0.0 (2026-02-27)
  - 初始版本发布
  - 支持达妙电机 CAN 驱动接入
  - 支持 MIT 控制模式与状态反馈

**兼容性说明：**
- Linux SocketCAN
- 达妙 DM 系列电机

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

贡献者与维护者名单见：`CONTRIBUTORS.md`

## License

本组件源码文件头声明为 Apache-2.0，最终以本目录 `LICENSE` 文件为准。
