#include "motor_core.h"
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

/* CRC8 MAXIM: x8 + x5 + x4 + 1 */
static uint8_t crc8_maxim(const uint8_t *data, uint32_t len) {
    uint8_t crc = 0x00;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x01) {
                crc = (crc >> 1) ^ 0x8C;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

static inline void pack_i16_be(uint8_t *buf, int16_t val) {
    buf[0] = (val >> 8) & 0xFF;
    buf[1] = val & 0xFF;
}

static inline void pack_u16_be(uint8_t *buf, uint16_t val) {
    buf[0] = (val >> 8) & 0xFF;
    buf[1] = val & 0xFF;
}

static inline int16_t unpack_i16_be(const uint8_t *buf) {
    return (int16_t)((buf[0] << 8) | buf[1]);
}

static inline uint16_t unpack_u16_be(const uint8_t *buf) {
    return (uint16_t)((buf[0] << 8) | buf[1]);
}

struct m0603c_priv {
    int fd;
    uint8_t id;
    uint8_t current_mode;

    uint8_t rx_74_buf[10];
    bool has_74;
    int32_t turns; // Store raw turns
    float turns_float; // Store float turns
    uint16_t pos_raw; // Store latest DATA[6..7] from 0x74 as raw uint16

    // Cached state
    struct motor_state state;
};

static int m0603c_uart_transfer(int fd, const uint8_t *tx_buf, uint8_t *rx_buf) {
    int wlen = write(fd, tx_buf, 10);
    if (wlen != 10) return -1;

    int rlen = 0;
    int retries = 50; // max 50ms wait
    while (rlen < 10 && retries > 0) {
        int r = read(fd, rx_buf + rlen, 10 - rlen);
        if (r > 0) {
            rlen += r;
        } else if (r < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            tcflush(fd, TCIFLUSH);
            return -1;
        }
        if (rlen < 10) {
            usleep(1000);
            retries--;
        }
    }

    if (rlen != 10) {
        tcflush(fd, TCIFLUSH);
        return -1;
    }
    return 0;
}

static int m0603c_set_mode(struct motor_dev *dev, uint8_t mode) {
    struct m0603c_priv *priv = (struct m0603c_priv *)dev->priv_data;
    uint8_t tx_buf[10] = {0};
    uint8_t rx_buf[10] = {0};

    tx_buf[0] = priv->id;
    tx_buf[1] = 0xA0;
    tx_buf[2] = mode;
    tx_buf[9] = crc8_maxim(tx_buf, 9);

    if (m0603c_uart_transfer(priv->fd, tx_buf, rx_buf) == 0) {
        if (rx_buf[1] == 0xA0 && crc8_maxim(rx_buf, 9) == rx_buf[9]) {
            priv->current_mode = rx_buf[2];
            return (rx_buf[2] == mode) ? 0 : -1;
        }
    }

    // Return error if mode switch failed or no valid response.
    // Do NOT update current_mode so driver will try again next time.
    return -1;
}

static int m0603c_init(struct motor_dev *dev) {
    struct m0603c_priv *priv = (struct m0603c_priv *)dev->priv_data;

    struct termios tty;
    if (tcgetattr(priv->fd, &tty) != 0) return -1;

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_oflag &= ~OPOST;

    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(priv->fd, TCSANOW, &tty) != 0) return -1;

    m0603c_set_mode(dev, 0); // Disable (Open loop with 0)

    return 0;
}

static int m0603c_set_cmd(struct motor_dev *dev, const struct motor_cmd *cmd) {
    struct m0603c_priv *priv = (struct m0603c_priv *)dev->priv_data;
    uint8_t target_mode = 0;

    switch (cmd->mode) {
        case MOTOR_MODE_IDLE: target_mode = 0; break; // Open loop for idle
        case MOTOR_MODE_TRQ:
        case MOTOR_MODE_CST:  target_mode = 0; break; // Open loop/Current: 0x00
        case MOTOR_MODE_VEL:
        case MOTOR_MODE_CSV:  target_mode = 2; break; // Velocity: 0x02
        case MOTOR_MODE_POS:
        case MOTOR_MODE_CSP:  target_mode = 3; break; // Position: 0x03
        default: target_mode = 0; break;
    }

    if (priv->current_mode != target_mode) {
        if (m0603c_set_mode(dev, target_mode) != 0) {
            // If mode switch fails, do not send the 0x64 command because
            // the motor will misinterpret the payload (e.g. torque as velocity)
            return -1;
        }
    }

    uint8_t tx_buf[10] = {0};
    tx_buf[0] = priv->id;
    tx_buf[1] = 0x64;

    if (target_mode == 0) {
        // Current: -8A to 8A -> -32767 to 32767
        int16_t current = 0;
        if (cmd->mode != MOTOR_MODE_IDLE) {
            current = (int16_t)(cmd->trq_des * 32767.0f / 8.0f);
        }
        pack_i16_be(&tx_buf[2], current);
    } else if (target_mode == 2) {
        // Velocity: -210 to 210 rpm -> -2100 to 2100
        float rpm = cmd->vel_des * 60.0f / (2.0f * (float)M_PI);
        pack_i16_be(&tx_buf[2], (int16_t)(rpm * 10.0f));
    } else if (target_mode == 3) {
        // Position: 0 to 360 deg -> 0 to 32767
        float pos_rad = fmodf(cmd->pos_des, 2.0f * (float)M_PI);
        if (pos_rad < 0) pos_rad += 2.0f * (float)M_PI;
        uint16_t pos_val = (uint16_t)(pos_rad * 32767.0f / (2.0f * (float)M_PI));
        pack_u16_be(&tx_buf[2], pos_val);
    }

    tx_buf[9] = crc8_maxim(tx_buf, 9);

    uint8_t rx_buf[10];
    if (m0603c_uart_transfer(priv->fd, tx_buf, rx_buf) == 0) {
        if (crc8_maxim(rx_buf, 9) == rx_buf[9] && rx_buf[1] == 0x64) {
            int16_t vel_val = unpack_i16_be(&rx_buf[2]);
            int16_t cur_val = unpack_i16_be(&rx_buf[4]);

            // Current -8A~8A
            priv->state.trq = ((float)cur_val / 32767.0f) * 8.0f;

            if (priv->current_mode == 3) {
                // In position mode, DATA[2], DATA[3] returns position (0~32767), not velocity.
                // Velocity is unobservable from 0x64 in this mode.
                priv->state.vel = 0.0f;
            } else {
                priv->state.vel = ((float)vel_val / 10.0f) * (2.0f * (float)M_PI) / 60.0f;
            }
            priv->state.temp = (float)rx_buf[7];
            priv->state.err = rx_buf[8];
        }
    }

    // 同步下发 0x74 帧查询里程和位置
    uint8_t tx_74[10] = {0};
    tx_74[0] = priv->id;
    tx_74[1] = 0x74;
    tx_74[9] = crc8_maxim(tx_74, 9);

    if (m0603c_uart_transfer(priv->fd, tx_74, priv->rx_74_buf) == 0) {
        priv->has_74 = true;
    } else {
        priv->has_74 = false;
    }

    return 0;
}

static int m0603c_get_state(struct motor_dev *dev, struct motor_state *state) {
    struct m0603c_priv *priv = (struct m0603c_priv *)dev->priv_data;

    if (priv->has_74) {
        if (crc8_maxim(priv->rx_74_buf, 9) == priv->rx_74_buf[9] && priv->rx_74_buf[1] == 0x74) {
            int32_t turns = (int32_t)((priv->rx_74_buf[2] << 24) | (priv->rx_74_buf[3] << 16) |
                (priv->rx_74_buf[4] << 8) | priv->rx_74_buf[5]);
            uint16_t pos_raw = (uint16_t)((priv->rx_74_buf[6] << 8) | priv->rx_74_buf[7]);

            priv->turns = turns; // Save decoded turns
            priv->pos_raw = pos_raw; // Save raw pos for diagnostics

            float frac = (float)pos_raw / 32767.0f;
            priv->turns_float = (float)turns + frac;

            priv->state.pos = priv->turns_float * 2.0f * (float)M_PI;
            // Fault code is also in 0x74, keep it updated
            priv->state.err = priv->rx_74_buf[8];
        }
        priv->has_74 = false;
    }

    if (state) *state = priv->state;
    return 0;
}

static void m0603c_free(struct motor_dev *dev) {
    if (dev) {
        struct m0603c_priv *priv = (struct m0603c_priv *)dev->priv_data;
        if (priv) {
            if (priv->fd >= 0) close(priv->fd);
            free(priv);
        }
        free(dev);
    }
}

static int m0603c_set_paras(struct motor_dev *dev, const void *address, const void *data, uint32_t data_len) {
    struct m0603c_priv *priv = (struct m0603c_priv *)dev->priv_data;
    if (!address || !data) return -1;

    uint8_t param_type;
    if ((uintptr_t)address < 0x1000) {
        param_type = (uint8_t)(uintptr_t)address;
    } else {
        param_type = *(const uint8_t *)address;
    }

    uint8_t tx_buf[10] = {0};
    uint8_t rx_buf[10] = {0};

    if (param_type == 0x75) { // Get Mode
        tx_buf[0] = priv->id;
        tx_buf[1] = 0x75;
        tx_buf[9] = crc8_maxim(tx_buf, 9);
        if (m0603c_uart_transfer(priv->fd, tx_buf, rx_buf) == 0) {
            if (rx_buf[1] == 0x75 && crc8_maxim(rx_buf, 9) == rx_buf[9]) {
                *(uint8_t *)((void *)data) = rx_buf[2];
                return 0;
            }
        }
    } else if (param_type == 0xC8) { // Get ID
        tx_buf[0] = 0xC8;
        tx_buf[1] = 0x64;
        tx_buf[9] = 0xDE;
        if (m0603c_uart_transfer(priv->fd, tx_buf, rx_buf) == 0) {
            *(uint8_t *)((void *)data) = rx_buf[0];
            return 0;
        }
    } else if (param_type == 0xAA) { // Set ID
        tx_buf[0] = 0xAA;
        tx_buf[1] = 0x55;
        tx_buf[2] = 0x53;
        tx_buf[3] = *(const uint8_t *)data;
        tx_buf[9] = crc8_maxim(tx_buf, 9);
        write(priv->fd, tx_buf, 10);
        priv->id = tx_buf[3];
        return 0;
    }

    return -1;
}

static int m0603c_get_paras(struct motor_dev *dev, const void *address, void *out_data, uint32_t data_len) {
    struct m0603c_priv *priv = (struct m0603c_priv *)dev->priv_data;
    if (!address || !out_data) return -1;

    uint8_t param_type;
    if ((uintptr_t)address < 0x1000) {
        param_type = (uint8_t)(uintptr_t)address;
    } else {
        param_type = *(const uint8_t *)address;
    }

    if (param_type == 0x74 && data_len == sizeof(float)) { // Get float turns
        *(float *)out_data = priv->turns_float;
        return 0;
    }
    if (param_type == 0x74 && data_len == sizeof(uint16_t)) { // Get raw pos value (DATA[6..7])
        *(uint16_t *)out_data = priv->pos_raw;
        return 0;
    }

    return -1;
}

static const struct motor_ops m0603c_ops = {
    .init = m0603c_init,
    .set_cmd = m0603c_set_cmd,
    .get_state = m0603c_get_state,
    .free = m0603c_free,
    .set_paras = m0603c_set_paras,
    .get_paras = m0603c_get_paras,
};

static struct motor_dev *m0603c_factory(void *args) {
    struct motor_args_uart *uart_args = (struct motor_args_uart *)args;
    if (!uart_args || !uart_args->dev_path) return NULL;

    int fd = open(uart_args->dev_path, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0) return NULL;
    fcntl(fd, F_SETFL, 0);

    struct m0603c_priv *priv = (struct m0603c_priv *)calloc(1, sizeof(*priv));
    if (!priv) {
        close(fd);
        return NULL;
    }

    priv->fd = fd;
    priv->id = uart_args->id;
    priv->current_mode = 0xFF;

    struct motor_dev *dev = (struct motor_dev *)calloc(1, sizeof(*dev));
    if (!dev) {
        free(priv);
        close(fd);
        return NULL;
    }

    dev->name = "drv_uart_ddt_m603c111";
    dev->ops = &m0603c_ops;
    dev->priv_data = priv;

    return dev;
}

REGISTER_MOTOR_DRIVER("drv_uart_ddt_m603c111", DRV_TYPE_UART, m0603c_factory);
