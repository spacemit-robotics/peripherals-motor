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

struct m0601c_priv {
    int fd;
    uint8_t id;
    uint8_t current_mode;
    uint16_t pos_raw;

    // Cached state
    struct motor_state state;
};

/* Send and receive a 10-byte frame */
static int m0601c_uart_transfer(int fd, const uint8_t *tx_buf, uint8_t *rx_buf) {
    // Write
    int wlen = write(fd, tx_buf, 10);
    if (wlen != 10) {
        return -1;
    }

    // Read response
    int rlen = 0;
    int retries = 50; // max 50ms wait
    while (rlen < 10 && retries > 0) {
        int r = read(fd, rx_buf + rlen, 10 - rlen);
        if (r > 0) {
            rlen += r;
        } else if (r < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            return -1; // serial port error
        }
        if (rlen < 10) {
            usleep(1000); // 1ms
            retries--;
        }
    }

    if (rlen != 10) {
        return -1; // timeout
    }
    return 0;
}

static int m0601c_set_mode(struct motor_dev *dev, uint8_t mode) {
    struct m0601c_priv *priv = (struct m0601c_priv *)dev->priv_data;
    uint8_t tx_buf[10] = {0};

    tx_buf[0] = priv->id;
    tx_buf[1] = 0xA0;
    tx_buf[9] = mode; // Mode is at DATA[9]
    // 0xA0 command does not seem to have a CRC according to specs, or maybe it does?
    // Let's rely on the documentation: DATA[9] is mode. No CRC.

    write(priv->fd, tx_buf, 10);
    usleep(2000); // Wait a bit for mode change
    priv->current_mode = mode;
    return 0;
}

static int m0601c_init(struct motor_dev *dev) {
    struct m0601c_priv *priv = (struct m0601c_priv *)dev->priv_data;

    // Configure termios
    struct termios tty;
    if (tcgetattr(priv->fd, &tty) != 0) {
        perror("tcgetattr");
        return -1;
    }

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag |= (CLOCAL | CREAD); // Enable receiver, ignore modem control lines
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;              // 8-bit characters
    tty.c_cflag &= ~PARENB;          // No parity
    tty.c_cflag &= ~CSTOPB;          // 1 stop bit
    tty.c_cflag &= ~CRTSCTS;         // No hardware flow control

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Disable software flow control
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Raw input

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Raw input
    tty.c_oflag &= ~OPOST; // Raw output

    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 1; // 100ms timeout

    if (tcsetattr(priv->fd, TCSANOW, &tty) != 0) {
        perror("tcsetattr");
        return -1;
    }

    // Set initial mode to disabled (0)
    m0601c_set_mode(dev, 0);

    return 0;
}

static int m0601c_set_cmd(struct motor_dev *dev, const struct motor_cmd *cmd) {
    struct m0601c_priv *priv = (struct m0601c_priv *)dev->priv_data;
    uint8_t target_mode = 0;
    float send_trq = 0.0f;
    float send_vel = 0.0f;
    float send_pos = 0.0f;

    switch (cmd->mode) {
        case MOTOR_MODE_IDLE:
            target_mode = 1; // 强制使用电流环
            send_trq = 0.0f;
            break;
        case MOTOR_MODE_TRQ:
        case MOTOR_MODE_CST:
            target_mode = 1;
            send_trq = cmd->trq_des;
            break; // 电流环
        case MOTOR_MODE_VEL:
        case MOTOR_MODE_CSV:
            target_mode = 2;
            send_vel = cmd->vel_des;
            break; // 速度环
        case MOTOR_MODE_POS:
        case MOTOR_MODE_CSP:
            target_mode = 3;
            send_pos = cmd->pos_des;
            break; // 位置环
        default:
            target_mode = 1;
            send_trq = 0.0f;
            break;
    }

    if (priv->current_mode != target_mode) {
        m0601c_set_mode(dev, target_mode);
    }

    uint8_t tx_buf[10] = {0};
    tx_buf[0] = priv->id;
    tx_buf[1] = 0x64;

    if (target_mode == 1) {
        // Current: -8A to 8A -> -32767 to 32767
        int16_t current = (int16_t)(send_trq * 32767.0f / 8.0f);
        pack_i16_be(&tx_buf[2], current);
    } else if (target_mode == 2) {
        // Velocity: -330 to 330 rpm
        float rpm = send_vel * 60.0f / (2.0f * (float)M_PI);
        pack_i16_be(&tx_buf[2], (int16_t)rpm);
    } else if (target_mode == 3) {
        // Position: 0 to 360 deg -> 0 to 32767
        // Keep in 0~2PI
        float pos_rad = fmodf(send_pos, 2.0f * (float)M_PI);
        if (pos_rad < 0) pos_rad += 2.0f * (float)M_PI;
        uint16_t pos_val = (uint16_t)(pos_rad * 32767.0f / (2.0f * (float)M_PI));
        pack_u16_be(&tx_buf[2], pos_val);
    }

    tx_buf[9] = crc8_maxim(tx_buf, 9);

    uint8_t rx_buf[10];
    if (m0601c_uart_transfer(priv->fd, tx_buf, rx_buf) == 0) {
        // We no longer strictly check rx_buf[1] == target_mode to avoid "data empty" issues
        // when the motor hasn't switched modes yet. As long as CRC is valid, we parse state.
        if (crc8_maxim(rx_buf, 9) == rx_buf[9]) {
            priv->current_mode = rx_buf[1]; // Update our tracked mode
            int16_t cur_val = unpack_i16_be(&rx_buf[2]);
            int16_t vel_val = unpack_i16_be(&rx_buf[4]);

            // 截取共时转换前的 pos_raw 值，借用 state.temp 向上传递
            uint16_t pos_raw_unmasked = unpack_u16_be(&rx_buf[6]);
            priv->state.temp = (float)pos_raw_unmasked;

            // Mask with 0x7FFF to fix negative values wrapping to 0xFFFF at 0° crossing
            uint16_t pos_val = pos_raw_unmasked & 0x7FFF;
            priv->pos_raw = pos_val;

            // 8A range for current -> 32767
            priv->state.trq = ((float)cur_val / 32767.0f) * 8.0f;
            // 330 rpm range for velocity
            priv->state.vel = ((float)vel_val) * (2.0f * (float)M_PI) / 60.0f;
            // 360 deg range for position -> 32767
            priv->state.pos = ((float)pos_val / 32767.0f) * (2.0f * (float)M_PI);
            priv->state.err = rx_buf[8];
        }
    }

    return 0;
}

static int m0601c_get_state(struct motor_dev *dev, struct motor_state *state) {
    struct m0601c_priv *priv = (struct m0601c_priv *)dev->priv_data;
    if (state) {
        *state = priv->state;
    }
    return 0;
}

static void m0601c_free(struct motor_dev *dev) {
    if (dev) {
        struct m0601c_priv *priv = (struct m0601c_priv *)dev->priv_data;
        if (priv) {
            if (priv->fd >= 0) {
                close(priv->fd);
            }
            free(priv);
        }
        free(dev);
    }
}

static int m0601c_get_paras(struct motor_dev *dev, const void *address, void *out_data, uint32_t data_len) {
    struct m0601c_priv *priv = (struct m0601c_priv *)dev->priv_data;
    uintptr_t addr = (uintptr_t)address;

    if (addr == 0x74 && data_len == sizeof(uint16_t)) {
        *(uint16_t *)out_data = priv->pos_raw;
        return 0;
    }

    if (addr == 0x74 && data_len == 1) {
        uint8_t tx_buf[10] = {0};
        tx_buf[0] = priv->id;
        tx_buf[1] = 0x74;
        tx_buf[9] = crc8_maxim(tx_buf, 9);

        uint8_t rx_buf[10];
        if (m0601c_uart_transfer(priv->fd, tx_buf, rx_buf) == 0) {
            if (crc8_maxim(rx_buf, 9) == rx_buf[9]) {
                *(uint8_t *)out_data = rx_buf[7]; // Return U8 position
                return 0;
            }
        }
    }
    return -1;
}

static const struct motor_ops m0601c_ops = {
    .init = m0601c_init,
    .set_cmd = m0601c_set_cmd,
    .get_state = m0601c_get_state,
    .free = m0601c_free,
    .set_paras = NULL,
    .get_paras = m0601c_get_paras,
};

static struct motor_dev *m0601c_factory(void *args) {
    struct motor_args_uart *uart_args = (struct motor_args_uart *)args;
    if (!uart_args || !uart_args->dev_path) {
        return NULL;
    }

    int fd = open(uart_args->dev_path, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0) {
        perror("open uart");
        return NULL;
    }
    fcntl(fd, F_SETFL, 0); // Blocking read

    struct m0601c_priv *priv = (struct m0601c_priv *)calloc(1, sizeof(*priv));
    if (!priv) {
        close(fd);
        return NULL;
    }

    priv->fd = fd;
    priv->id = uart_args->id;
    priv->current_mode = 0xFF; // Uninitialized

    struct motor_dev *dev = (struct motor_dev *)calloc(1, sizeof(*dev));
    if (!dev) {
        free(priv);
        close(fd);
        return NULL;
    }

    dev->name = "drv_485_ddt_m601c111";
    dev->ops = &m0601c_ops;
    dev->priv_data = priv;

    return dev;
}

REGISTER_MOTOR_DRIVER("drv_485_ddt_m601c111", DRV_TYPE_UART, m0601c_factory);
