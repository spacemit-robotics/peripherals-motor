/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <motor.h>
#include <motor_core.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <gpiod.h>
#include <pthread.h>

struct pwm_generic_info {
    uint32_t period; /*us*/
    uint32_t duty_cycle; /*us*/
};

struct pwm_generic_priv {
    uint8_t gpio_index;
    struct pwm_generic_info info;

    /* gpiod objects */
    struct gpiod_chip *chip;

#if defined(LIBGPIOD_V2)
    int offset;
    struct gpiod_line_settings *settings;
    struct gpiod_line_config *lcfg;
    struct gpiod_request_config *rcfg;
    struct gpiod_line_request *req;

#else
    struct gpiod_line *line_gpio;
#endif

    /*thread info*/
    pthread_t *thread_ptr;
    pthread_attr_t *attr;
};

static bool run_flag = false;

// 线程函数包装器
void* pwm_gpio_thread_wrapper(void *args) {
    if (!args) {
        return NULL;
    }

    struct pwm_generic_priv *priv = (struct pwm_generic_priv *)args;
    if (!priv) {
        return NULL;
    }

#if defined(LIBGPIOD_V2)
    if (!priv->req) {
#else
    if (!priv->line_gpio) {
#endif
        return NULL;
    }

    while (run_flag) {
#if defined(LIBGPIOD_V2)
        gpiod_line_request_set_value(priv->req, priv->offset, GPIOD_LINE_VALUE_ACTIVE);
        usleep(priv->info.duty_cycle);
        gpiod_line_request_set_value(priv->req, priv->offset, GPIOD_LINE_VALUE_INACTIVE);
#else
        gpiod_line_set_value(priv->line_gpio, 1);
        usleep(priv->info.duty_cycle);
        gpiod_line_set_value(priv->line_gpio, 0);
#endif
        usleep(priv->info.period - priv->info.duty_cycle);
    }

    return NULL;
}

static int pwm_gpio_init(struct motor_dev *dev)
{
    struct pwm_generic_priv *priv = (struct pwm_generic_priv *)dev->priv_data;
    if (!priv) {
        return -1;
    }

    // 初始化gpio引脚，保存到private里面
#if defined(LIBGPIOD_V2)
    printf("############## gpiod version 2.0, #############\n");

    /* k3用到libgpiod v2.2.1，根据offset 来确认pin脚*/
    char chip_name[20] = {0};
    for (int i = 0; i < 4; i++) {
        if (priv->gpio_index >= i * 32 && priv->gpio_index < (i + 1) * 32) {
            priv->offset = priv->gpio_index - i * 32;
            snprintf(chip_name, sizeof(chip_name), "/dev/gpiochip%d", i);
            break;
        }
    }
    printf("gpio_index=%d, offset=%d, chip_name=%s\n", priv->gpio_index, priv->offset, chip_name);
    struct gpiod_chip *chip = gpiod_chip_open(chip_name);
#else
    printf("#######, gpiod version 1.0,###########\n");
    struct gpiod_chip *chip = gpiod_chip_open_by_name("gpiochip0");
    if (!chip) {
        chip = gpiod_chip_open_by_name("gpiochip1");
    }
#endif

    if (!chip) {
        fprintf(stderr, "Failed to open GPIO chip\n");
        return -1;
    }
    priv->chip = chip;


#if defined(LIBGPIOD_V2)
    priv->settings = gpiod_line_settings_new();
    if (!priv->settings) {
        fprintf(stderr, "Failed to create line settings\n");
        gpiod_chip_close(chip);
        free(priv);
        return -1;
    }
    gpiod_line_settings_set_direction(priv->settings, GPIOD_LINE_DIRECTION_OUTPUT);
    gpiod_line_settings_set_output_value(priv->settings, GPIOD_LINE_VALUE_INACTIVE);

    priv->lcfg = gpiod_line_config_new();
    if (!priv->lcfg) {
        fprintf(stderr, "Failed to create line config\n");
        gpiod_line_settings_free(priv->settings);
        gpiod_chip_close(chip);
        free(priv);
        return -1;
    }

    unsigned int _ofset = priv->offset;
    if (gpiod_line_config_add_line_settings(priv->lcfg, &_ofset, 1, priv->settings) < 0){
        fprintf(stderr, "Failed to add line settings to config\n");
        gpiod_line_config_free(priv->lcfg);
        gpiod_line_settings_free(priv->settings);
        gpiod_chip_close(chip);
        free(priv);
        return -1;
    }
    priv->rcfg = gpiod_request_config_new();
    if (!priv->rcfg) {
        fprintf(stderr, "Failed to create request config\n");
        gpiod_line_config_free(priv->lcfg);
        gpiod_line_settings_free(priv->settings);
        gpiod_chip_close(chip);
        free(priv);
        return -1;
    }

    gpiod_request_config_set_consumer(priv->rcfg, "pwm_gpio_motor");

    priv->req = gpiod_chip_request_lines(chip, priv->rcfg, priv->lcfg);
    if (!priv->req) {
        fprintf(stderr, "Failed to request lines\n");
        gpiod_request_config_free(priv->rcfg);
        gpiod_line_config_free(priv->lcfg);
        gpiod_line_settings_free(priv->settings);
        gpiod_chip_close(chip);
        free(priv);
        return -1;
    }

#else
    priv->line_gpio = gpiod_chip_get_line(chip, priv->gpio_index);
    if (!priv->line_gpio || gpiod_line_request_output(priv->line_gpio, "out", 0) < 0) {
        fprintf(stderr, "无法获取GPIO线 %d\n", priv->gpio_index);
        gpiod_chip_close(chip);
        free(priv);
        return -1;
    }
#endif

    priv->thread_ptr = NULL;
    priv->attr = NULL;

    return 0;
}


static int pwm_gpio_set_cmd(struct motor_dev *dev, const struct motor_cmd *cmd)
{
    struct pwm_generic_priv *priv = (struct pwm_generic_priv *)dev->priv_data;
    if (!priv || !cmd) {
        return -1;
    }
    printf("Setting PWM GPIO motor command: mode=%d vel_des=%.2f\n", cmd->mode, cmd->vel_des);

    if (cmd->vel_des <= 0) {
        priv->info.duty_cycle = 0;
    } else if (cmd->vel_des >= 100) {
        priv->info.duty_cycle = priv->info.period;
    } else {
        priv->info.duty_cycle = (uint32_t)(priv->info.period * (cmd->vel_des / 100.0f));
    }
    if (cmd->mode == MOTOR_MODE_IDLE) {
        priv->info.duty_cycle = 0;
    }

    // 要多线程跑，不然会阻塞
    if (priv->thread_ptr != NULL) {
        run_flag = false;
        usleep(1000);  // 等待线程结束，确保线程函数退出后再释放资源

        free(priv->thread_ptr);
        free(priv->attr);
        priv->thread_ptr = NULL;
        priv->attr = NULL;
    }


    priv->thread_ptr = (pthread_t *)malloc(sizeof(pthread_t));
    priv->attr = (pthread_attr_t *)malloc(sizeof(pthread_attr_t));
    pthread_attr_init(priv->attr);
    pthread_attr_setdetachstate(priv->attr, PTHREAD_CREATE_DETACHED);

    run_flag = true;

    if (pthread_create(priv->thread_ptr, priv->attr, pwm_gpio_thread_wrapper, (void*)priv) != 0) {
        printf("线程创建失败\n");
        free(priv->thread_ptr);
        free(priv->attr);
        priv->thread_ptr = NULL;
        priv->attr = NULL;
        run_flag = false;
        return -1;
    }
    pthread_attr_destroy(priv->attr);
    return 0;
}

static int pwm_gpio_get_state(struct motor_dev *dev, struct motor_state *state)
{
    struct pwm_generic_priv *priv = (struct pwm_generic_priv *)dev->priv_data;
    if (!priv || !state) {
        return -1;
    }

    return 0;
}

static void pwm_gpio_free(struct motor_dev *dev)
{
    if (!dev) {
        return;
    }

    struct pwm_generic_priv *priv = (struct pwm_generic_priv *)dev->priv_data;
    if (!priv) {
        free(dev);
        return;
    }

    if (priv->thread_ptr) {
        run_flag = false;
        usleep(1000);  // 等待线程结束，确保线程函数退出后再释放资源
        free(priv->thread_ptr);
    }
    if (priv->attr) {
        free(priv->attr);
    }

    // 释放gpio, 先拉低电平，避免电机处于运行的状态
#if defined(LIBGPIOD_V2)
    if (priv->req) {
        gpiod_line_request_set_value(priv->req, priv->offset, GPIOD_LINE_VALUE_INACTIVE);
        gpiod_line_request_release(priv->req);
    }
    if (priv->rcfg) {
        gpiod_request_config_free(priv->rcfg);
    }
    if (priv->lcfg) {
        gpiod_line_config_free(priv->lcfg);
    }
    if (priv->settings) {
        gpiod_line_settings_free(priv->settings);
    }
#else
    if (priv->line_gpio) {
        gpiod_line_set_value(priv->line_gpio, 0);
        usleep(1000);  // 确保电平设置成功后再释放
        gpiod_line_release(priv->line_gpio);
    }
#endif
    if (priv->chip) {
        gpiod_chip_close(priv->chip);
    }

    free(dev->priv_data);
    free(dev);
}

static const struct motor_ops g_pwm_gpio_ops = {
    .init = pwm_gpio_init,
    .set_cmd = pwm_gpio_set_cmd,
    .get_state = pwm_gpio_get_state,
    .free = pwm_gpio_free,
};

static struct motor_dev *pwm_gpio_factory(void *args)
{
    struct motor_args_pwm *pwm_args = (struct motor_args_pwm *)args;
    if (!pwm_args) {
        return NULL;
    }

    struct motor_dev *dev = (struct motor_dev *)calloc(1, sizeof(*dev));
    struct pwm_generic_priv *priv = (struct pwm_generic_priv *)calloc(1, sizeof(*priv));
    if (!dev || !priv) {
        free(dev);
        free(priv);
        return NULL;
    }

    if (pwm_args->args) {
        priv->info = *(struct pwm_generic_info *)(pwm_args->args);
    } else {
        struct pwm_generic_info default_info = {100000, 50000};
        priv->info = default_info;
    }
    priv->gpio_index = pwm_args->ch;

    dev->name = "pwm_gpio";
    dev->ops = &g_pwm_gpio_ops;
    dev->priv_data = priv;
    return dev;
}

REGISTER_MOTOR_DRIVER("pwm_gpio", DRV_TYPE_PWM, pwm_gpio_factory)
