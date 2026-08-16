# Encos CAN 电机驱动

`drv_can_encos` 实现 Encos 电机的 MIT 风格 8 字节控制帧、状态反馈、使能/失能和
CAN watchdog 配置。

内置型号：

- `Encos EC-A10020-P2-24`
- `Encos EC-A8116-P1-18`
- `Encos EC-A10020-P1-12`
- `Encos EC-A8112-P1-18`

配置优先使用 `model` 选择协议量程。为兼容现有 YAML，也可完整提供
`protocol_limits.position/velocity/torque/kp/kd`，但不能只覆盖其中一部分。
其他配置项为 `feedback_id`、`can_timeout_ms` 和 `enable_on_init`。
`can_timeout_ms` 默认 `500 ms`，仅离线调试时设为 `0`。

反馈帧低 5 位原样写入 `motor_state.err`：`0` 无故障，`1` 过热，`2` 过流，
`3` 过压，`4` 欠压，`5` 编码器故障，`6` 抱闸电压过高，`7` 驱动故障。
未定义的非零值仍保留给上层处理。
