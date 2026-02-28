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


struct pwm_RoHS_info {
    uint8_t motor_index;  // 电机索引，用于区分多个步进电机
    uint8_t step_gpio;    // 步进脉冲 GPIO 编号
    uint8_t dir_gpio;     // 方向 GPIO 编号
    uint8_t enable_gpio;  // 使能 GPIO 编号
    uint8_t stop_gpio;    // 限位开关 GPIO 编号 (可选, 没有则填0)

    int current_position;       // 当前步数位置
    int constant_range;         // 固定的电机可转动角度范围，根据实际的角度范围设定
    int gpio_max_steps;         // 最大步数限制，首次会动态获取并写入到配置文件里面

    bool enable_gpio_level;       // 使能电平反转， 如果是false， 表示低电平使能
    bool dir_gpio_left_level;     // 指定向左的电平, 如果是false，表示低电平向左
    bool stop_gpio_active_level;  // 限位开关有效电平, 如果是false，表示低电平有效

    /*没有stop相位开关，则需要配置电机steps步数*/
    int range_steps;
};

struct pwm_RoHS_priv {
    struct pwm_RoHS_info info;
    struct motor_state state;

    /* gpiod objects */
    struct gpiod_chip *chip;

#if defined(LIBGPIOD_V2)
    struct gpiod_chip *chip_step;
    struct gpiod_chip *chip_dir;
    struct gpiod_chip *chip_enable;
    struct gpiod_chip *chip_stop;

    struct gpiod_line_settings *settings_out;
    struct gpiod_line_settings *settings_in;

    struct gpiod_line_config *lcfg_step;
    struct gpiod_line_config *lcfg_dir;
    struct gpiod_line_config *lcfg_enable;
    struct gpiod_line_config *lcfg_stop;

    struct gpiod_request_config *rcfg_step;
    struct gpiod_request_config *rcfg_dir;
    struct gpiod_request_config *rcfg_enable;
    struct gpiod_request_config *rcfg_stop;

    struct gpiod_line_request *req_step;
    struct gpiod_line_request *req_dir;
    struct gpiod_line_request *req_enable;
    struct gpiod_line_request *req_stop;

    unsigned int offset_step;
    unsigned int offset_dir;
    unsigned int offset_enable;
    unsigned int offset_stop;
#else
    struct gpiod_line *line_step;
    struct gpiod_line *line_dir;
    struct gpiod_line *line_enable;
    struct gpiod_line *line_stop;  // 可选
#endif
};

#define BUFFER_SIZE 256

/* pwm sleep time, run speed */
#define STEP_SLEEP_TIME 400
#define STEP_CHECK_TIMES 10
#define STEP_MAX_STEPS 2500
#define STEP_CONTINUE_RUN_TIMES 250

#define STEP_MOTOR_1_MAX_STEPS 900
#define STEP_MOTOR_2_MAX_STEPS 900
#define STEP_MOTOR_3_MAX_STEPS 900
#define STEP_MOTOR_MAX_STEPS(index) STEP_MOTOR_##index##_MAX_STEPS


static inline int read_from_file_return_int(const char* filename) {
    char buffer[BUFFER_SIZE] = {""};

    int fd = open(filename, O_RDONLY);
    if (fd < 0) {
        // printf("打开属性文件失败\n");
        return -1;
    }
    ssize_t bytes = read(fd, buffer, BUFFER_SIZE - 1);
    close(fd);
    if (bytes <= 0) {
        return -1;
    }

    buffer[bytes] = '\0';
    // 去除换行符
    if (buffer[bytes - 1] == '\n') {
        buffer[bytes - 1] = '\0';
    }

    int ret = atoi(buffer);
    return ret;
}

static inline int write_int_to_file(const char* filename, int value) {
    FILE* fp = fopen(filename, "w+");
    if (!fp) {
        printf("Error opening %s: %s\n", filename, strerror(errno));
        return -1;
    }
    int result = fprintf(fp, "%d", value);
    fclose(fp);

    if (result < 0) {
        printf("Error writing to %s: %s\n", filename, strerror(errno));
        return -1;
    }
    return 0;
}

/*
    return actual steps run
*/
int run_step_times(struct pwm_RoHS_priv *priv, int times, int speed, int stop_signal_value)
{
    int get_gpio_value = 0;
    bool continue_run = false;
    int continue_run_times = STEP_CONTINUE_RUN_TIMES;
    int check_steps_time = STEP_CHECK_TIMES;  /* 降低检测gpio次数 */
    int actual_steps = 0;

#if defined(LIBGPIOD_V2)
    if (priv->req_stop != NULL){
        enum gpiod_line_value v = gpiod_line_request_get_value(priv->req_stop, priv->offset_stop);
        get_gpio_value = v == GPIOD_LINE_VALUE_ACTIVE ? 1 : 0;
#else
    if (priv->line_stop != NULL){
        get_gpio_value = gpiod_line_get_value(priv->line_stop);
#endif
        if (get_gpio_value == stop_signal_value){
            continue_run = true;
        }
    }

    if (speed <= 0) {
        speed = 1;  // default speed, incase input speed is 0 or negative, which may cause the motor not run
    }
    if (speed > 10) {
        speed = 10;  // max speed, if speed too high, may cause the motor error
    }

    for (; actual_steps < times; actual_steps++) {
#if defined(LIBGPIOD_V2)
        gpiod_line_request_set_value(priv->req_step, priv->offset_step, GPIOD_LINE_VALUE_ACTIVE);
#else
        gpiod_line_set_value(priv->line_step, 1);
#endif
        usleep(STEP_SLEEP_TIME / speed);
#if defined(LIBGPIOD_V2)
        gpiod_line_request_set_value(priv->req_step, priv->offset_step, GPIOD_LINE_VALUE_INACTIVE);
#else
        gpiod_line_set_value(priv->line_step, 0);
#endif

        usleep(STEP_SLEEP_TIME / speed);
        if (continue_run_times > 0 && continue_run) {
            continue_run_times--;
            continue;
        }

        check_steps_time--;
#if defined(LIBGPIOD_V2)
        if (check_steps_time < 0 && priv->req_stop != NULL) {
            check_steps_time = STEP_CHECK_TIMES;
            enum gpiod_line_value v = gpiod_line_request_get_value(priv->req_stop, priv->offset_stop);
            get_gpio_value = v == GPIOD_LINE_VALUE_ACTIVE ? 1 : 0;
#else
        if (check_steps_time < 0 && priv->line_stop != NULL) {
            check_steps_time = STEP_CHECK_TIMES;
            get_gpio_value = gpiod_line_get_value(priv->line_stop);
#endif
            if (get_gpio_value == stop_signal_value) {
                printf("xxxxxxxxxxxx, detect stop gpio signal xxxxxxxxxxxxxxx\n");
                break;
            }
        }
    }
    return actual_steps;
}

/* turn left until trigger the limit switch, then turn right until trigger the limmit switch */
int init_step_motor_max_steps(struct motor_dev *dev, int speed)
{
    struct pwm_RoHS_priv *priv = (struct pwm_RoHS_priv *)dev->priv_data;
    if (!priv) {
        return -1;
    }
    struct pwm_RoHS_info *motor_info = &priv->info;
    if (!motor_info) {
        return -1;
    }

    /*enable motor*/
#if defined(LIBGPIOD_V2)
    gpiod_line_request_set_value(
        priv->req_enable,
        priv->offset_enable,
        motor_info->enable_gpio_level ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE);
#else
    gpiod_line_set_value(
        priv->line_enable,
        motor_info->enable_gpio_level ? 1 : 0);
#endif
    usleep(1000);
    /* set dir left */
#if defined(LIBGPIOD_V2)
    gpiod_line_request_set_value(priv->req_dir, priv->offset_dir, GPIOD_LINE_VALUE_ACTIVE);
#else
    gpiod_line_set_value(priv->line_dir, 1);
#endif
    usleep(1000);

    // run motor to left
    run_step_times(priv, STEP_MAX_STEPS, speed, motor_info->stop_gpio_active_level ? 1 : 0);
    usleep(100000);

    /* set dir right */
#if defined(LIBGPIOD_V2)
    gpiod_line_request_set_value(priv->req_dir, priv->offset_dir, GPIOD_LINE_VALUE_INACTIVE);
#else
    gpiod_line_set_value(priv->line_dir, 0);
#endif
    usleep(1000);

    // run motor to left
    int max_steps = run_step_times(priv, STEP_MAX_STEPS, speed, motor_info->stop_gpio_active_level ? 1 : 0);
    usleep(100000);
    if (max_steps >= (STEP_MAX_STEPS - 10)) {
        printf("warning: reach max steps, may be the limit switch is not working or motor range exceeds max steps, "
        "please check the motor and limit switch\n");
        /* set max step to default */
        if (motor_info->motor_index == 1) {
            max_steps = STEP_MOTOR_MAX_STEPS(1);
        } else if (motor_info->motor_index == 2) {
            max_steps = STEP_MOTOR_MAX_STEPS(2);
        } else {
            max_steps = STEP_MOTOR_MAX_STEPS(3);
        }
    }


    // save max steps
    char path1[BUFFER_SIZE] = {""};
    snprintf(path1, sizeof(path1), "/root/.motor_%d_max_steps", motor_info->motor_index);
    write_int_to_file(path1, max_steps);
    motor_info->gpio_max_steps = max_steps;

    printf("motor_index:%d, max_steps:%d\n", motor_info->motor_index, max_steps);


    /* set dir left to let robot turn center */
    printf("start to turn to center >>>\n");
#if defined(LIBGPIOD_V2)
    gpiod_line_request_set_value(priv->req_dir, priv->offset_dir, GPIOD_LINE_VALUE_ACTIVE);
#else
    gpiod_line_set_value(priv->line_dir, 1);
#endif
    usleep(1000);
    run_step_times(priv, max_steps/2, speed, motor_info->stop_gpio_active_level ? 1 : 0);
    usleep(10000);

    /* disable motor */
#if defined(LIBGPIOD_V2)
    gpiod_line_request_set_value(
        priv->req_enable,
        priv->offset_enable,
        motor_info->enable_gpio_level ? GPIOD_LINE_VALUE_INACTIVE : GPIOD_LINE_VALUE_ACTIVE);
#else
    gpiod_line_set_value(
        priv->line_enable,
        motor_info->enable_gpio_level ? 0 : 1);
#endif
    usleep(1000);

    // save current angle to 90
    char path2[BUFFER_SIZE] = {""};
    snprintf(path2, sizeof(path2), "/root/.motor_%d_cur_angle", motor_info->motor_index);
    write_int_to_file(path2, 90);
    return 0;
}


int step_motor_rotating_angle_control(struct motor_dev *dev, int angle, int speed)
{
    struct pwm_RoHS_priv *priv = (struct pwm_RoHS_priv *)dev->priv_data;
    if (!priv) {
        return -1;
    }
    struct pwm_RoHS_info *motor_info = &priv->info;
    if (!motor_info) {
        return -1;
    }

    /* limit switch check */
    bool has_limit_switch = false;
#if defined(LIBGPIOD_V2)
    has_limit_switch = (priv->req_stop != NULL);
#else
    has_limit_switch = (priv->line_stop != NULL);
#endif
    if (!has_limit_switch) {
        printf("limit switch does not exist, should set default max steps\n");
        if (motor_info->range_steps == 0) {
            if (motor_info->motor_index == 1) {
                motor_info->gpio_max_steps = STEP_MOTOR_MAX_STEPS(1);
            } else if (motor_info->motor_index == 2) {
                motor_info->gpio_max_steps = STEP_MOTOR_MAX_STEPS(2);
            } else {
                motor_info->gpio_max_steps = STEP_MOTOR_MAX_STEPS(3);
            }
        } else {
            motor_info->gpio_max_steps = motor_info->range_steps;
        }
    } else {
        printf("limit switch exists, turn with limit switch\n");
    }

    /* init motor */
    int max_steps = motor_info->gpio_max_steps;
    if (max_steps <= 0) {
        printf("init motor max steps, please waiting\n");
        init_step_motor_max_steps(dev, speed);
        max_steps = motor_info->gpio_max_steps;
    }

    /* enable motor */
#if defined(LIBGPIOD_V2)
    gpiod_line_request_set_value(
        priv->req_enable,
        priv->offset_enable,
        motor_info->enable_gpio_level ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE);
#else
    gpiod_line_set_value(
        priv->line_enable,
        motor_info->enable_gpio_level ? 1 : 0);
#endif
    usleep(1000);

    /* get mapping equation */
    char cur_position_path[BUFFER_SIZE] = {""};
    snprintf(cur_position_path, sizeof(cur_position_path), "/root/.motor_%d_cur_angle", motor_info->motor_index);
    int current_angle = read_from_file_return_int(cur_position_path);

    /* 获取每度对应的步数 */
    int diff = max_steps / motor_info->constant_range;

    /* avoid input angle exceeds max angle */
    if (angle > (90 + motor_info->constant_range / 2)) {
        angle = 90 + motor_info->constant_range / 2;
    } else if (angle < (90 - motor_info->constant_range / 2)) {
        angle = 90 - motor_info->constant_range / 2;
    }

    int target_steps = abs(current_angle - angle) * diff;

    if (angle < current_angle) {
        /* turn left */
#if defined(LIBGPIOD_V2)
        gpiod_line_request_set_value(
            priv->req_dir,
            priv->offset_dir,
            motor_info->dir_gpio_left_level ? GPIOD_LINE_VALUE_INACTIVE
                                            : GPIOD_LINE_VALUE_ACTIVE);
#else
        gpiod_line_set_value(priv->line_dir, motor_info->dir_gpio_left_level ? 0 : 1);
#endif

        usleep(1000);
        run_step_times(priv, target_steps, speed, motor_info->stop_gpio_active_level ? 1 : 0);
        usleep(1000);
    } else if (angle > current_angle) {
        /* turn right */
#if defined(LIBGPIOD_V2)
        gpiod_line_request_set_value(
            priv->req_dir,
            priv->offset_dir,
            motor_info->dir_gpio_left_level ? GPIOD_LINE_VALUE_ACTIVE
                                            : GPIOD_LINE_VALUE_INACTIVE);
#else
        gpiod_line_set_value(priv->line_dir, motor_info->dir_gpio_left_level ? 1 : 0);
#endif

        usleep(1000);
        run_step_times(priv, target_steps, speed, motor_info->stop_gpio_active_level ? 1 : 0);
    } else {
        printf("is current angle, do nothing\n");
    }

    /* update current angle */
    write_int_to_file(cur_position_path, angle);
    printf("save current angle:%d\n", read_from_file_return_int(cur_position_path));

    /* disable motor */
#if defined(LIBGPIOD_V2)
    gpiod_line_request_set_value(
        priv->req_enable,
        priv->offset_enable,
        motor_info->enable_gpio_level ? GPIOD_LINE_VALUE_INACTIVE : GPIOD_LINE_VALUE_ACTIVE);
#else
    gpiod_line_set_value(
        priv->line_enable,
        motor_info->enable_gpio_level ? 0 : 1);
#endif

    return 0;
}

#if defined(LIBGPIOD_V2)
static struct gpiod_chip *open_chip_by_offset(int gpio_index, unsigned int *offset) {
    char chip_name[20] = {0};
    for (int i = 0; i < 4; i++) {
        if (gpio_index >= i * 32 && gpio_index < (i + 1) * 32) {
            *offset = gpio_index - i * 32;
            snprintf(chip_name, sizeof(chip_name), "/dev/gpiochip%d", i);
            break;
        }
    }
    printf("gpio_index=%d, offset=%d, chip_name=%s\n", gpio_index, *offset, chip_name);
    return gpiod_chip_open(chip_name);
}
#endif

static int step_motor_init(struct motor_dev *dev)
{
    struct pwm_RoHS_priv *priv = (struct pwm_RoHS_priv *)dev->priv_data;
    if (!priv) {
        return -1;
    }
    memset(&priv->state, 0, sizeof(priv->state));

    // 初始化gpio引脚，保存到private里面
#if defined(LIBGPIOD_V2)
    printf("###############, gpiod version 2.0,###############\n");
    unsigned offset = 0;
    priv->chip_step = open_chip_by_offset(priv->info.step_gpio, &offset);
    priv->offset_step = offset;
    if (!priv->chip_step) {
        fprintf(stderr, "Failed to open GPIO chip for step\n");
        free(priv);
        return -1;
    }
    priv->chip_dir = open_chip_by_offset(priv->info.dir_gpio, &offset);
    priv->offset_dir = offset;
    if (!priv->chip_dir) {
        fprintf(stderr, "Failed to open GPIO chip for dir\n");
        gpiod_chip_close(priv->chip_step);
        free(priv);
        return -1;
    }
    priv->chip_enable = open_chip_by_offset(priv->info.enable_gpio, &offset);
    priv->offset_enable = offset;
    if (!priv->chip_enable) {
        fprintf(stderr, "Failed to open GPIO chip for enable\n");
        gpiod_chip_close(priv->chip_step);
        gpiod_chip_close(priv->chip_dir);
        free(priv);
        return -1;
    }
    if (priv->info.stop_gpio != 0) {
        priv->chip_stop = open_chip_by_offset(priv->info.stop_gpio, &offset);
        priv->offset_stop = offset;
        if (!priv->chip_stop) {
            fprintf(stderr, "Failed to open GPIO chip for stop\n");
            gpiod_chip_close(priv->chip_step);
            gpiod_chip_close(priv->chip_dir);
            gpiod_chip_close(priv->chip_enable);
            free(priv);
            return -1;
        }
    }
#else
    printf("###############, gpiod version 1.0,###############\n");
    struct gpiod_chip *chip = gpiod_chip_open_by_name("gpiochip0");
    if (!chip) {
        chip = gpiod_chip_open_by_name("gpiochip1");
    }
    if (!chip) {
        fprintf(stderr, "Failed to open GPIO chip\n");
        return -1;
    }
    priv->chip = chip;
#endif



#if defined(LIBGPIOD_V2)

    priv->settings_out = gpiod_line_settings_new();
    if (!priv->settings_out) {
        fprintf(stderr, "Failed to create line settings\n");
        gpiod_chip_close(priv->chip_step);
        gpiod_chip_close(priv->chip_dir);
        gpiod_chip_close(priv->chip_enable);
        if (priv->chip_stop) {
            gpiod_chip_close(priv->chip_stop);
        }
        free(priv);
        return -1;
    }
    gpiod_line_settings_set_direction(priv->settings_out, GPIOD_LINE_DIRECTION_OUTPUT);
    gpiod_line_settings_set_output_value(priv->settings_out, GPIOD_LINE_VALUE_INACTIVE);

    priv->settings_in = gpiod_line_settings_new();
    if (!priv->settings_in) {
        fprintf(stderr, "Failed to create line settings\n");
        gpiod_chip_close(priv->chip_step);
        gpiod_chip_close(priv->chip_dir);
        gpiod_chip_close(priv->chip_enable);
        if (priv->chip_stop) {
            gpiod_chip_close(priv->chip_stop);
        }
        free(priv);
        return -1;
    }
    gpiod_line_settings_set_direction(priv->settings_in, GPIOD_LINE_DIRECTION_INPUT);
    priv->lcfg_step = gpiod_line_config_new();
    priv->lcfg_dir = gpiod_line_config_new();
    priv->lcfg_enable = gpiod_line_config_new();
    if (priv->chip_stop) {
        priv->lcfg_stop = gpiod_line_config_new();
    }

    offset = priv->offset_step;
    gpiod_line_config_add_line_settings(priv->lcfg_step, &offset, 1, priv->settings_out);
    offset = priv->offset_dir;
    gpiod_line_config_add_line_settings(priv->lcfg_dir, &offset, 1, priv->settings_out);
    offset = priv->offset_enable;
    gpiod_line_config_add_line_settings(priv->lcfg_enable, &offset, 1, priv->settings_out);
    if (priv->chip_stop) {
        offset = priv->offset_stop;
        gpiod_line_config_add_line_settings(priv->lcfg_stop, &offset, 1, priv->settings_in);
    }

    priv->rcfg_step = gpiod_request_config_new();
    priv->rcfg_dir = gpiod_request_config_new();
    priv->rcfg_enable = gpiod_request_config_new();
    if (priv->chip_stop) {
        priv->rcfg_stop = gpiod_request_config_new();
    }
    gpiod_request_config_set_consumer(priv->rcfg_step, "pwm_gpio_motor_step");
    gpiod_request_config_set_consumer(priv->rcfg_dir, "pwm_gpio_motor_dir");
    gpiod_request_config_set_consumer(priv->rcfg_enable, "pwm_gpio_motor_enable");
    if (priv->chip_stop) {
        gpiod_request_config_set_consumer(priv->rcfg_stop, "pwm_gpio_motor_stop");
    }

    priv->req_step = gpiod_chip_request_lines(priv->chip_step, priv->rcfg_step, priv->lcfg_step);
    priv->req_dir = gpiod_chip_request_lines(priv->chip_dir, priv->rcfg_dir, priv->lcfg_dir);
    priv->req_enable = gpiod_chip_request_lines(priv->chip_enable, priv->rcfg_enable, priv->lcfg_enable);
    if (priv->chip_stop) {
        priv->req_stop = gpiod_chip_request_lines(priv->chip_stop, priv->rcfg_stop, priv->lcfg_stop);
    }
    if (!priv->req_step || !priv->req_dir || !priv->req_enable || (priv->chip_stop && !priv->req_stop)) {
        fprintf(stderr, "Failed to request lines\n");
        gpiod_request_config_free(priv->rcfg_step);
        gpiod_request_config_free(priv->rcfg_dir);
        gpiod_request_config_free(priv->rcfg_enable);
        if (priv->chip_stop) {
            gpiod_request_config_free(priv->rcfg_stop);
        }
        gpiod_line_config_free(priv->lcfg_step);
        gpiod_line_config_free(priv->lcfg_dir);
        gpiod_line_config_free(priv->lcfg_enable);
        if (priv->chip_stop) {
            gpiod_line_config_free(priv->lcfg_stop);
        }
        gpiod_line_settings_free(priv->settings_out);
        gpiod_line_settings_free(priv->settings_in);
        gpiod_chip_close(priv->chip_step);
        gpiod_chip_close(priv->chip_dir);
        gpiod_chip_close(priv->chip_enable);
        if (priv->chip_stop) {
            gpiod_chip_close(priv->chip_stop);
        }
        free(priv);
        return -1;
    }

#else
    priv->line_step = gpiod_chip_get_line(chip, priv->info.step_gpio);
    if (!priv->line_step || gpiod_line_request_output(priv->line_step, "out", 0) < 0) {
        fprintf(stderr, "无法获取GPIO线 %d\n", priv->info.step_gpio);
        gpiod_chip_close(chip);
        free(priv);
        return -1;
    }

    priv->line_dir = gpiod_chip_get_line(chip, priv->info.dir_gpio);
    if (!priv->line_dir || gpiod_line_request_output(priv->line_dir, "out", 0) < 0) {
        gpiod_line_release(priv->line_step);
        gpiod_chip_close(chip);
        fprintf(stderr, "无法获取GPIO线 %d\n", priv->info.dir_gpio);
        free(priv);
        return -1;
    }

    priv->line_enable = gpiod_chip_get_line(chip, priv->info.enable_gpio);
    if (!priv->line_enable || gpiod_line_request_output(priv->line_enable, "out", 0) < 0) {
        gpiod_line_release(priv->line_step);
        gpiod_line_release(priv->line_dir);
        gpiod_chip_close(chip);
        fprintf(stderr, "无法获取GPIO线 %d\n", priv->info.enable_gpio);
        free(priv);
        return -1;
    }
    if (priv->info.stop_gpio != 0) {
        priv->line_stop = gpiod_chip_get_line(chip, priv->info.stop_gpio);
        if (!priv->line_stop || gpiod_line_request_input(priv->line_stop, "in") < 0) {
            fprintf(stderr, "无法获取GPIO线 %d\n", priv->info.stop_gpio);
            priv->line_stop = NULL;
        }

        // 初始化max_steps
        char max_steps_path[256];
        snprintf(max_steps_path, sizeof(max_steps_path), "/root/.motor_%d_max_steps", priv->info.motor_index);
        int max_steps = read_from_file_return_int(max_steps_path);
        priv->info.gpio_max_steps = max_steps;
    } else {
        priv->line_stop = NULL;
    }
#endif

#if defined(LIBGPIOD_V2)
    if (priv->info.stop_gpio != 0) {
        char max_steps_path[256];
        snprintf(max_steps_path, sizeof(max_steps_path), "/root/.motor_%d_max_steps", priv->info.motor_index);
        int max_steps = read_from_file_return_int(max_steps_path);
        priv->info.gpio_max_steps = max_steps;
    }
#endif

    // 从配置文件读取当前步数位置
    char cur_position_path[256];
    snprintf(cur_position_path, sizeof(cur_position_path), "/root/.motor_%d_cur_angle", priv->info.motor_index);
    int current_position = read_from_file_return_int(cur_position_path);
    if (current_position < 0) {
        current_position = 0;
        write_int_to_file(cur_position_path, current_position);
    }
    priv->info.current_position = current_position;

    return 0;
}

static int step_motor_set_cmd(struct motor_dev *dev, const struct motor_cmd *cmd)
{
    struct pwm_RoHS_priv *priv = (struct pwm_RoHS_priv *)dev->priv_data;
    if (!priv || !cmd) {
        return -1;
    }

    // printf("%s, pos_des=%f, vel_des=%f, priv->info.step_gpio=%d\n", __func__, cmd->pos_des, cmd->vel_des,
    //        priv->info.step_gpio);
    step_motor_rotating_angle_control(dev, (int)cmd->pos_des, (int)cmd->vel_des);

    /* Demo: mirror command into state */
    priv->state.pos = cmd->pos_des;
    priv->state.vel = cmd->vel_des;
    priv->state.trq = cmd->trq_des;

    return 0;
}

static int step_motor_get_state(struct motor_dev *dev, struct motor_state *state)
{
    struct pwm_RoHS_priv *priv = (struct pwm_RoHS_priv *)dev->priv_data;
    if (!priv || !state) {
        return -1;
    }
    *state = priv->state;
    return 0;
}

static void step_motor_free(struct motor_dev *dev) {
    if (!dev) {
        return;
    }

    struct pwm_RoHS_priv *priv = (struct pwm_RoHS_priv *)dev->priv_data;
    if (!priv) {
        free(dev);
        return;
    }

    // 释放gpio
#if defined(LIBGPIOD_V2)
    if (priv->req_step) {
        gpiod_line_request_release(priv->req_step);
    }
    if (priv->req_dir) {
        gpiod_line_request_release(priv->req_dir);
    }
    if (priv->req_enable) {
        gpiod_line_request_release(priv->req_enable);
    }
    if (priv->req_stop) {
        gpiod_line_request_release(priv->req_stop);
    }

    if (priv->rcfg_step) {
        gpiod_request_config_free(priv->rcfg_step);
    }
    if (priv->rcfg_dir) {
        gpiod_request_config_free(priv->rcfg_dir);
    }
    if (priv->rcfg_enable) {
        gpiod_request_config_free(priv->rcfg_enable);
    }
    if (priv->rcfg_stop) {
        gpiod_request_config_free(priv->rcfg_stop);
    }

    if (priv->lcfg_step) {
        gpiod_line_config_free(priv->lcfg_step);
    }
    if (priv->lcfg_dir) {
        gpiod_line_config_free(priv->lcfg_dir);
    }
    if (priv->lcfg_enable) {
        gpiod_line_config_free(priv->lcfg_enable);
    }
    if (priv->lcfg_stop) {
        gpiod_line_config_free(priv->lcfg_stop);
    }
    if (priv->settings_out) {
        gpiod_line_settings_free(priv->settings_out);
    }
    if (priv->settings_in) {
        gpiod_line_settings_free(priv->settings_in);
    }
    if (priv->chip_step) {
        gpiod_chip_close(priv->chip_step);
    }
    if (priv->chip_dir) {
        gpiod_chip_close(priv->chip_dir);
    }
    if (priv->chip_enable) {
        gpiod_chip_close(priv->chip_enable);
    }
    if (priv->chip_stop) {
        gpiod_chip_close(priv->chip_stop);
    }
#else
    if (priv->line_step) {
        gpiod_line_release(priv->line_step);
    }
    if (priv->line_dir) {
        gpiod_line_release(priv->line_dir);
    }
    if (priv->line_enable) {
        gpiod_line_release(priv->line_enable);
    }
    if (priv->line_stop) {
        gpiod_line_release(priv->line_stop);
    }
    if (priv->chip) {
        gpiod_chip_close(priv->chip);
    }
#endif

    free(dev->priv_data);
    free(dev);
}

static const struct motor_ops g_step_motor_ops = {
    .init = step_motor_init,
    .set_cmd = step_motor_set_cmd,
    .get_state = step_motor_get_state,
    .free = step_motor_free,
};

static struct motor_dev *pwm_RoHS_factory(void *args) {
    struct motor_args_pwm *pwm_args = (struct motor_args_pwm *)args;
    if (!pwm_args) {
        return NULL;
    }

    struct motor_dev *dev = (struct motor_dev *)calloc(1, sizeof(*dev));
    struct pwm_RoHS_priv *priv = (struct pwm_RoHS_priv *)calloc(1, sizeof(*priv));
    if (!dev || !priv) {
        free(dev);
        free(priv);
        return NULL;
    }

    if (pwm_args->args) {
        priv->info = *(struct pwm_RoHS_info *)(pwm_args->args);
    } else {
        memset(&priv->info, 0, sizeof(priv->info));
    }
    dev->name = "pwm_RoHS";
    dev->ops = &g_step_motor_ops;
    dev->priv_data = priv;
    return dev;
}

REGISTER_MOTOR_DRIVER("pwm_RoHS", DRV_TYPE_PWM, pwm_RoHS_factory)
