# drv_485_m8010a

version: 0.0.1.202610121419

M8010-A RS485 驱动通过通用 `motor.h` 使用，消费仓库只保留注册代码与预编译库。

随附 `lib/riscv64/libdrv_485_m8010a.so` SHA-256：`f7903f582a9fe8a5d2eeddbc8ccbe9dd19e3c790269838b0782c4a4edfb14628`。
更新库时，同步更新版本号及库 SHA-256。

```text
src/drv_485_m8010a.c             驱动注册
lib/riscv64/libdrv_485_m8010a.so  K3 完整驱动实现
```

```c
#include "motor.h"

struct motor_dev *motor = motor_alloc_uart(
    "drv_485_m8010a", "/dev/ttyUSB0", 4000000, 0, NULL);
```

`motor_set_cmd_one()` 执行命令收发，`motor_get_state_one()` 返回最近一次有效反馈。
同一设备路径和波特率的多个设备共享串口和收发锁，最后一个设备释放时关闭串口。
`motor_get_paras()` 支持 `feedback_id`（uint8_t）、`foot_force_raw`（uint16_t）和
`command_sent_us`（uint64_t，最近一次完整命令写入的单调时钟微秒时间）。
完整发送后即使回包超时，发送时间仍会更新；查询或未完成发送不会更新时间。

原生 `ttyS` 串口首字节等待为 `3ms`，整次发送与接收截止时间为 `10ms`；
USB 串口保留 `100ms` 等待。仅首字节等待超时时，原生串口保留接收队列和残帧，
迟到回包可在后续事务中解析并采用；整次事务达到截止时间时清理未读字节和残帧。
收发采用的已解析缓存反馈必须距接收时间不足 `10ms`，采用时保留原接收时间戳。
`motor_set_cmd_one()` 超时返回 `-ETIMEDOUT`；`motor_get_state_one()` 仍返回最后一次有效状态，
不按上述 `10ms` 限制自动失效，应用应通过 `motor_get_feedback_timestamp_one()` 判断反馈新鲜度。

## 构建与更新

由维护者提供对应版本与架构的库文件。
库使用对应组件的 `motor.h` 和 `motor_core.h` 编译；接口布局或语义变化时配套更新库。

CMake 自动按目标架构读取 `lib/${CMAKE_SYSTEM_PROCESSOR}/libdrv_485_m8010a.so`，
K3（riscv64）使用 `lib/riscv64/`，无需手动指定库路径。
交叉编译时目标架构由工具链配置决定，其他架构需要自行提供相应二进制。
需要测试替代库时，可使用 `-DMOTOR_M8010A_LIBRARY=/absolute/path/libdrv_485_m8010a.so`
覆盖默认选择，指定的库必须匹配目标架构及运行环境，路径无效会报错。
安装组件时库同时安装到 SDK 的 lib 目录。
未指定驱动列表、自动发现驱动时，缺少当前平台的库会提示并跳过 M8010-A，其他驱动照常选择。
明确选择 M8010-A 但当前平台没有库时，会告警并跳过该驱动；
riscv64 缺失库属于仓库损坏，明确选择时会直接报错。
当前仅随附 Linux riscv64 库。
二进制还要求兼容的 libc 和指令集，架构匹配不代表兼容所有 Linux 发行版。

组件测试检查注册、库加载及无硬件的生命周期。
