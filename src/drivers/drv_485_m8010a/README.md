# drv_485_m8010a

M8010-A RS485 驱动通过通用 `motor.h` 使用，消费仓库只保留注册代码与预编译库。

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
`motor_get_paras()` 支持 `feedback_id`（uint8_t）和 `foot_force_raw`（uint16_t）。

## 构建与更新

完整实现及协议回归维护在独立私有仓库 `drv_485_m8010a`，由维护者提供对应架构的构建产物。
库使用对应组件的 `motor.h` 和 `motor_core.h` 编译；接口布局或语义变化时配套更新库。

CMake 自动按目标架构读取 `lib/${CMAKE_SYSTEM_PROCESSOR}/libdrv_485_m8010a.so`，
K3（riscv64）使用 `lib/riscv64/`，无需手动指定库路径。
交叉编译时目标架构由工具链配置决定，其他架构需要自行提供相应二进制。
需要测试替代库时，可使用 `-DMOTOR_M8010A_LIBRARY=/absolute/path/libdrv_485_m8010a.so`
覆盖默认选择，指定的库必须匹配目标架构及运行环境，路径无效会报错。
安装组件时库同时安装到 SDK 的 lib 目录。
未指定驱动列表、自动发现驱动时，缺少当前平台的库会提示并跳过 M8010-A，其他驱动照常选择。
明确选择 M8010-A 但当前平台没有库时（如 x86_64、aarch64），会告警并跳过该驱动；
riscv64 缺失库属于仓库损坏，明确选择时会直接报错。
当前仅随附 Linux riscv64 库；x86_64、ARM64（aarch64）等平台尚未提供产物。
二进制还要求兼容的 libc 和指令集，架构匹配不代表兼容所有 Linux 发行版。

组件测试检查注册、库加载及无硬件的生命周期；完整伪串口协议回归在私有源码仓库执行。
