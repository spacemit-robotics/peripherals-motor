# JMC CANOpen 电机驱动使用 (drv_canopen_jmc)

本文档介绍了如何使用 `drv_canopen_jmc` 驱动来控制 JMC 系列的 CANOpen 伺服电机，并简要介绍了驱动底层的协议映射逻辑。

## 一、支持的特性与模式

基于 CiA 301（通信协议）与 CiA 402（驱动器设备配置文件），本驱动为 JMC 伺服电机封装了完整的生命周期控制：
- **状态机自动托管**：由独立后台线程通过解析 TPDO (StatusWord)，自动控制电机完成从 `Switch On Disabled` -> `Ready to Switch On` -> `Operation Enabled` 的有限状态机流转。
- **运行模式**：
  - **PP (Profile Position, 模式 1)**：轮廓位置模式，基于内置的加减速度生成平滑的位移曲线。
  - **PV (Profile Velocity, 模式 3)**：轮廓速度模式，支持通过 SDO/RPDO 不断下发实时速度指令。
  - **HM (Homing, 模式 6)**：自动回零模式，结合硬限位或者原点开关自动找零点。
- **读写参数 (SDO)**：支持完整的阻塞式参数读取 `motor_get_paras` 与写入 `motor_set_paras`。

---

## 二、CANOpen 协议在驱动中的应用简述

CANOpen 协议在驱动开发中主要分为三类核心报文：NMT（网络管理）、SDO（服务数据）和 PDO（过程数据）。

### 1. NMT (网络管理) - `ID: 0x000`
- 驱动在程序启动时向总线广播 `0x01` 报文，要求所有电机进入 **Operational（操作）** 状态。
- 在程序结束、清理资源时，驱动广播 `0x80` 报文，强制电机回退到 **Pre-operational（预操作）** 状态，**该设计避免了主程序闪退后电机继续飞车失控的风险。**

### 2. SDO (服务数据对象) - `ID: 0x600+节点ID / 0x580+节点ID`
SDO 主要用于非实时的系统参数配置，例如修改电机的最大转速、加速度等（读写对象字典 Object Dictionary）。
- **同步拦截**：因为底层 SocketCAN 是单通道，驱动通过后台线程持续监控 `0x580+NodeID` 的反馈报文，当应用层调用 `motor_set_paras` 写入参数时，驱动层会进入最大 50ms 的同步等待。
- **JMC 特有坑点记录**：根据标准 CiA402 协议，加速度 `0x6083` 应该是 4 字节的 Unsigned32。但在 JMC 伺服上，**它被实现为了 2 字节！** 下发 4 字节的写指令会导致电机回复 Abort (`0x80`) 拒收。因此在使用 `motor_set_paras` 修改 `0x6083` 时，请务必传入长度 `2`。

### 3. PDO (过程数据对象) - `实时循环`
PDO 是“无需请求、自动上报/执行”的高速帧，非常适合 10ms 甚至更小周期的运动控制。
- **TPDO (电机到主机)**：绑定到 `0x280 + NodeID`，包含 `StatusWord` (当前状态) 和 `ActualPosition` (当前物理位置)。后台线程会不断解析它以更新 `motor_state`。
- **RPDO (主机到电机)**：绑定到 `0x400 + NodeID`，包含 `ControlWord` (控制位，如复位、使能、新目标标记) 和 `TargetPosition` (目标位置)。

---

## 三、快速测试与用例验证

工程内置了 `test_motor_canopen_jmc` 用于验证各项功能。

### 1. 运行测试
假设电机 ID 配置为 1，运行如下指令：
```bash
test_motor_canopen_jmc --motors 1 --cycle 10 -v
```

### 2. 测试用例工作流
该测试程序自动执行以下流程（代码位于 `tests/test_motor_canopen_jmc.c`）：
1. **自动使能 (Auto-Enable)**：等待底层的后台线程捕获到所有电机回复的 `StatusWord & 0x6F == 0x27`，确认全部成功上电并自动对齐零点。
2. **参数读写测试 (SDO)**：使用 `motor_set_paras` 向 `0x6083` 写入 2 字节宽度的加速度测试值（如 `90`），并立刻使用 `motor_get_paras` 读取校验。
3. **位置轨迹测试 (PP)**：电机正转 1 圈（6.283 rad），归零，再反转 1 圈，最后归零。每次下发坐标后会监控 `StatusWord` 的 `Target Reached (bit 10)` 判断是否到达位姿。
4. **速度巡航测试 (PV)**：动态切换到 `MOTOR_MODE_VEL`，给定 6.283 rad/s 的正反转速度，持续数秒。
5. **回零测试 (HM)**：切换到 `MOTOR_MODE_HM`，电机通过控制字 bit 4 开始找零。

---

## 四、应用层代码示例

```c
#include "motor.h"

int main() {
    // 1. 初始化电机对象 (绑定 ID 1)
    struct motor_dev* dev = motor_alloc_can("drv_canopen_jmc", "can0", 1, NULL);
    struct motor_dev* devs[] = { dev };
    motor_init(devs, 1); // 启动后台总线线程与状态机

    // 2. 修改加速度参数 (JMC的加速度为 2 字节)
    uint32_t index = 0x6083; uint32_t sub = 0x00;
    uint32_t size = 2;   // 数据长度
    // 构造 SDO 地址
    uint32_t sdo_addr[3] = {index, sub, size}; 
    uint16_t acc_val = 100;
    motor_set_paras(dev, sdo_addr, &acc_val, 2);

    // 3. 在循环中下发控制指令
    struct motor_cmd cmd = {0};
    cmd.mode = MOTOR_MODE_POS; // 设定为位置模式
    cmd.pos_des = 3.14f;       // 目标位置(弧度)
    
    while (1) {
        motor_set_cmds(devs, &cmd, 1); // 下发目标
        usleep(10000); // 10ms 周期
    }
    return 0;
}
```
