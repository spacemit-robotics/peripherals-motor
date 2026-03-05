/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef MOTOR_CORE_H
#define MOTOR_CORE_H

#include "../include/motor.h"

/* 1. 参数适配包：用于将特定类型的alloc参数打包成 void* */
struct motor_args_can {
    const char *iface;
    uint32_t can_id;
    void *args;
};

struct motor_args_uart {
    const char *dev_path;
    uint32_t baud;
    uint8_t id;  /* 级联电机ID，用于识别同一总线上的不同电机 */
    void *args;
};

struct motor_args_pwm {
    uint32_t ch;
    void *args;
};

struct motor_args_ecat {
    uint16_t slave_idx;
    void *args;
};



/* 2. 驱动类型枚举：防止类型混用 */
enum driver_type {
    DRV_TYPE_PWM,
    DRV_TYPE_CAN,
    DRV_TYPE_UART,
    DRV_TYPE_ECAT
};

/* 3. 虚函数表 */
struct motor_ops {
    int (*init)(struct motor_dev *dev);
    int (*set_cmd)(struct motor_dev *dev, const struct motor_cmd *cmd);
    int (*get_state)(struct motor_dev *dev, struct motor_state *state);
    void (*free)(struct motor_dev *dev);
    /* Optional: direct register access (for calibration / diagnostics) */
    int (*reg_read_byte)(struct motor_dev *dev, uint8_t addr);
    int (*reg_write_byte)(struct motor_dev *dev, uint8_t addr, uint8_t val);
    int (*reg_read_word)(struct motor_dev *dev, uint8_t addr);
    int (*reg_write_word)(struct motor_dev *dev, uint8_t addr, uint16_t val);
};

/* 4. 电机对象定义 */
struct motor_dev {
    const char *name;
    const struct motor_ops *ops;
    void *priv_data;  // 私有数据
};

/* 5. 通用工厂函数类型 */
typedef struct motor_dev* (*motor_factory_t)(void *args);

/* 6. 注册节点结构 */
struct driver_info {
    const char *name;
    enum driver_type type;
    motor_factory_t factory;
    struct driver_info *next;
};

/* 核心提供的注册API */
void motor_driver_register(struct driver_info *info);

/* 7. 自动注册宏 (GCC/Clang Constructor) */
#define REGISTER_MOTOR_DRIVER(_name, _type, _factory) \
    static struct driver_info __drv_info_##_factory = { \
        .name = _name, \
        .type = _type, \
        .factory = _factory, \
        .next = 0 \
    }; \
    __attribute__((constructor)) \
    static void __auto_reg_##_factory(void) { \
        motor_driver_register(&__drv_info_##_factory); \
    }

#endif
