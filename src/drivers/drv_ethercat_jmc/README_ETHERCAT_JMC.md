# JMC IHSS42 EtherCAT 驱动程序说明文档

## 1. 概述
本驱动程序专门用于 JMC IHSS42-EC 集成式步进伺服电机，基于 EtherCAT 通讯协议，支持标准的 CiA402 状态机控制。驱动层通过 `motor_dev` 接口向应用层提供统一的操作方法。

## 2. 核心控制模式说明

根据不同的应用场景，驱动支持以下五种主要控制模式：

| 模式 | 全称 | 类型 | 适用场景 |
| :--- | :--- | :--- | :--- |
| **CSP** | Cyclic Synchronous Position | 同步位置 | 高动响应的正弦轨迹、多轴联动插补 |
| **CSV** | Cyclic Synchronous Velocity | 同步速度 | 旋转泵、传送带、连续扫描运动 |
| **PP** | Profile Position | 梯形加减速位置 | 点到点定位、简单的往复摆动 |
| **PV** | Profile Velocity | 梯形加减速速度 | 恒速控制、大惯量阶跃调速 |
| **HM** | Homing Mode | 回零模式 | 系统上电寻找原点、标定物理零位 |

---

## 3. **至关重要：等待使能 (Wait for Enable)**

在 EtherCAT 框架下，应用层在发送运动指令前，**必须执行等待使能逻辑**。

### 3.1 为什么要等待？
1.  **逻辑零点锚定 (Anchoring)**：PP 和 CSP 模式需要将应用层的“起始位置”与使能瞬间电机的“物理位置”对齐。如果不等待使能就发送大距离指令，电机在开启瞬间可能发生剧烈跳变。
2.  **适配器门控机制**：驱动内部设有安全闸门（Adapter Gate），只有检测到从站进入 `OPERATION_ENABLED` 状态后，真正的运动数据才会写入 PDO。
3.  **多轴同步性**：不同电机的使能速度可能存在毫秒级差异。等待所有电机全部进入就绪态并保持一小段稳定期（建议 100ms），是保证多轴协同运动不产生相位偏差的前提。
4.  **SDO 配置延迟**：HM 模式等需要通过 SDO 下发参数，必须在使能过程中完成握手。

### 3.2 推荐的编程模式 (以多轴为例)
```c
// 等待所有电机使能
int stable_counts = 0;
while (g_running) {
    motor_get_states(devs, states, motor_count);
    int all_enabled = 1;
    for (int i = 0; i < motor_count; i++) {
        // 0x0027 为 CiA402 的 Operation Enabled 状态
        if (((uint16_t)states[i].err & 0x006F) != 0x0027) {
            all_enabled = 0;
        }
    }
    
    if (all_enabled) {
        if (++stable_counts > 100 / cycle_ms) break; // 稳定 100ms 后退出
    } else {
        stable_counts = 0;
    }
    
    // 等待期间应持续下发 0 指令以维持锚定
    for (int i = 0; i < motor_count; i++) {
        cmds[i].mode = MOTOR_MODE_POS; // 或对应模式
        cmds[i].pos_des = 0.0f;
    }
    motor_set_cmds(devs, cmds, motor_count);
    sleep_ms(cycle_ms);
}
```

---

## 4. 模式使用说明
示例于 motor/test/test_motor_ecat.c 中

### 4.1 CSP & CSV (同步循环模式)
- **特点**：驱动不负责加减速逻辑，应用层每个周期需下发平滑的点位/速度。
- **注意**：CSP 下，`pos_des` 是相对于使能瞬间位置的增量。驱动层自动处理了平滑过渡。

### 4.2 PP (梯形加减速位置)
- **目标响应**：下发一个目标点，电机根据从站内部设定的 `Profile Velocity` 和 `Acceleration` 自动运行。
- **状态监控**：通过 `(status_word & 0x0400)` 判断是否到达目标（Target Reached）。建议在多轴场景下，确认所有轴均 Reached 后再下发下一目标。

### 4.3 PV (梯形加减速速度)
- **行为**：电机加速至目标速度平稳运行。
- **示例**：适合需要精确速度反馈且不关心绝对位置的场景。

### 4.4 HM (回零模式)
- **流程**：
    1.  应用层下发 `MOTOR_MODE_HM`。
    2.  等待使能完成后，驱动内部触发回零启动信号。
    3.  监控 `status_word`：
        - `Bit 12 (0x1000)`: Homing attained (成功)。
        - `Bit 13 (0x2000)`: Homing error (失败)。
    4.  成功后，电机物理位置会被强制设为零。

---

## 5. 多轴同步建议
- 始终使用同一个 `loop_count` 管理打印和逻辑逻辑。
- 在 `test_motor_ecat.c` 中参考 `memset` 初始化 `cmds` 和 `states`，防止脏数据干扰。
- 对于多轴往返运动，务必等待所有从站的 `Target Reached` 位。

---

## 6. 常见状态字 (Status Word) 参考
- `0x0027`: 电机已使能 (Operation Enabled)。
- `0x0227`: 电机运行中且已使能。
- `0x0400 (Bit 10)`: [PP] 目标已到达。
- `0x1000 (Bit 12)`: [HM] 回零完成。
- `0x0008 (Bit 3)`: 电机故障 (Fault)。
