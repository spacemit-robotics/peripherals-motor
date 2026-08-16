# AGIBOT CAN 电机驱动

`drv_can_agibot` 对接 AGIBOT OmniPicker 对外公开的经典 CAN 协议。

- 支持 `MOTOR_MODE_POS`，位置范围为 `0~pi/2 rad`：`0` 完全张开，`pi/2` 完全闭合。
- `force`、`velocity`、`acceleration`、`deceleration` 为协议原始 `0~255` 配置值。
- 反馈中的位置转换为 rad；协议未给出速度和力的物理量换算，因此通用状态中的
  `vel`、`trq` 保持为 `0`。
- 官方 CAN 协议没有使能/失能帧，`MOTOR_MODE_IDLE` 返回不支持，不会把“张开”伪装成失能。

可选配置项：`model`、`feedback_id`、`force`、`velocity`、`acceleration`、
`deceleration`。`model` 支持 `AGIBOT OmniPicker` 和厂商配置中的 `AGIBOT`。
