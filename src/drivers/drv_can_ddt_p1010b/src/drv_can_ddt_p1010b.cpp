/*
 * Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <map>
#include <string>
#include <mutex>

extern "C" {
#include "../../../../include/motor.h"
#include "../../../motor_core.h"
}

namespace {

constexpr float k_pi = 3.14159265358979323846f;

constexpr uint32_t kIdStateCtrl = 0x38;
constexpr uint32_t kIdDrive1_4 = 0x32;
constexpr uint32_t kIdDrive5_8 = 0x33;
constexpr uint32_t kIdParamSet = 0x36;
constexpr uint32_t kIdFeedbackBase = 0x50;

constexpr uint8_t kModeVoltage = 0x00;
constexpr uint8_t kModeMIT = 0x01;
constexpr uint8_t kModeCurrent = 0x02;
constexpr uint8_t kModeVelocity = 0x03;
constexpr uint8_t kModePosition = 0x04;

struct p1010b_priv {
    char bus_name[16];
    char dev_name[48];
    int can_fd;
    uint8_t can_id;      /* 1..8 */
    uint8_t active_mode;
    bool enabled;
    bool initialized;
    struct motor_state last_state;
    bool have_last_state;
    uint16_t last_pos_raw;
    bool have_last_pos_raw;
    int32_t pos_turns;
    bool print_raw;
};

struct BusState {
    int16_t setpoints[8];
    uint8_t configured_mask;
    uint8_t update_mask;
    BusState() : configured_mask(0), update_mask(0) {
        for (int i = 0; i < 8; ++i) setpoints[i] = 0;
    }
};

static std::map<std::string, BusState> g_bus_states;
static std::mutex g_bus_mutex;

static int32_t clamp_i32(int32_t value, int32_t lo, int32_t hi) {
    if (value < lo) return lo;
    if (value > hi) return hi;
    return value;
}

static int16_t encode_position_raw(float rad) {
    // 100 raw = 1 turn = 2 * PI rad
    float turns = rad / (2.0f * k_pi);
    int32_t raw = static_cast<int32_t>(std::lround(turns * 100.0f));
    return static_cast<int16_t>(clamp_i32(raw, -5000, 5000));
}

static int16_t encode_velocity_raw(float rad_s) {
    // 10 raw = 1 RPM. RPM = rad_s / (2 * PI) * 60
    float rpm = rad_s / (2.0f * k_pi) * 60.0f;
    int32_t raw = static_cast<int32_t>(std::lround(rpm * 10.0f));
    return static_cast<int16_t>(clamp_i32(raw, -16000, 16000));
}

static int16_t encode_current_raw(float current_a) {
    int32_t raw = static_cast<int32_t>(std::lround(current_a * 100.0f));
    return static_cast<int16_t>(clamp_i32(raw, -7500, 7500));
}

static int16_t encode_voltage_raw(float voltage_v) {
    int32_t raw = static_cast<int32_t>(std::lround(voltage_v * 100.0f));
    return static_cast<int16_t>(clamp_i32(raw, -4800, 4800));
}

static void put_be16(uint8_t* dst, int16_t value) {
    dst[0] = static_cast<uint8_t>((static_cast<uint16_t>(value) >> 8) & 0xFF);
    dst[1] = static_cast<uint8_t>(static_cast<uint16_t>(value) & 0xFF);
}

static int16_t get_be16(const uint8_t* src) {
    uint16_t hi = static_cast<uint16_t>(src[0]);
    uint16_t lo = static_cast<uint16_t>(src[1]);
    return static_cast<int16_t>((hi << 8) | lo);
}

static int p1010b_open_socket(const char* iface, int* fd_out) {
    if (!iface || !fd_out) return -1;

    int fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd < 0) return -1;

    struct ifreq ifr;
    std::memset(&ifr, 0, sizeof(ifr));
    std::snprintf(ifr.ifr_name, sizeof(ifr.ifr_name), "%s", iface);
    if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
        close(fd);
        return -1;
    }

    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(fd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        close(fd);
        return -1;
    }

    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 20000;  // 20 ms read timeout
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    *fd_out = fd;
    return 0;
}

static int p1010b_send_frame(int fd, uint32_t can_id, const uint8_t* data, uint8_t dlc) {
    if (fd < 0 || !data || dlc > 8) return -1;

    struct can_frame frame;
    std::memset(&frame, 0, sizeof(frame));
    frame.can_id = can_id;
    frame.can_dlc = dlc;
    std::memcpy(frame.data, data, dlc);
    ssize_t n = write(fd, &frame, sizeof(frame));
    return (n == static_cast<ssize_t>(sizeof(frame))) ? 0 : -1;
}

static int p1010b_recv_id(struct p1010b_priv* priv, uint32_t want_id, struct can_frame* out) {
    struct can_frame frame;
    uint32_t alt_id = (want_id == (kIdFeedbackBase + priv->can_id)) ? (0x70 + priv->can_id) : 0;
    for (int guard = 0; guard < 64; ++guard) {
        ssize_t n = read(priv->can_fd, &frame, sizeof(frame));
        if (n < 0) return -1;
        if (n != static_cast<ssize_t>(sizeof(frame))) continue;
        uint32_t rx_id = frame.can_id & CAN_EFF_MASK;
        if (want_id == 0 || rx_id == want_id || (alt_id != 0 && rx_id == alt_id)) {
            if (out) *out = frame;
            return 0;
        }
    }
    return -1;
}

static int p1010b_set_enable(struct p1010b_priv* priv, bool enable) {
    uint8_t data[8] = {0};
    data[priv->can_id - 1] = enable ? 0x02 : 0x01;
    return p1010b_send_frame(priv->can_fd, kIdStateCtrl, data, 8);
}

static int p1010b_set_mode(struct p1010b_priv* priv, uint8_t mode) {
    uint8_t data[8] = {0};
    data[0] = priv->can_id;
    data[1] = 0x1C;
    data[2] = mode;
    return p1010b_send_frame(priv->can_fd, kIdParamSet, data, 8);
}

static int p1010b_set_active_report(struct p1010b_priv* priv, bool enable) {
    uint8_t data[8] = {0};
    data[0] = priv->can_id;
    data[1] = enable ? 0x01 : 0x00; // 0x01 Active Report, 0x00 Query
    data[2] = 10;   // 10ms interval
    data[3] = 0x01; // Speed * 10
    data[4] = 0x03; // IQ * 100
    data[5] = 0x0D; // Absolute Pos (0-32768)
    data[6] = 0x0A; // System Voltage * 10
    data[7] = 0x00; // Reserved
    return p1010b_send_frame(priv->can_fd, 0x34, data, 8);
}

static int p1010b_send_setpoint(struct p1010b_priv* priv, int16_t raw_value) {
    std::lock_guard<std::mutex> lock(g_bus_mutex);

    auto& bus = g_bus_states[priv->bus_name];
    bus.setpoints[priv->can_id - 1] = raw_value;
    bus.update_mask |= (1 << (priv->can_id - 1));

    int group = (priv->can_id - 1) / 4;
    uint8_t group_mask = (group == 0) ? 0x0F : 0xF0;
    uint8_t target_mask = bus.configured_mask & group_mask;

    // Send only when all configured motors in this group have been updated
    if ((bus.update_mask & group_mask) == target_mask && target_mask != 0) {
        uint8_t data[8] = {0};
        uint32_t frame_id = (group == 0) ? kIdDrive1_4 : kIdDrive5_8;
        int base_idx = group * 4;

        for (int i = 0; i < 4; ++i) {
            put_be16(&data[i * 2], bus.setpoints[base_idx + i]);
        }

        int ret = p1010b_send_frame(priv->can_fd, frame_id, data, 8);

        // Clear the update mask for this group
        bus.update_mask &= ~group_mask;
        return ret;
    }

    return 0; // Buffered successfully
}

static void decode_feedback(struct p1010b_priv* priv, const struct can_frame& frame, struct motor_state* state) {
    int16_t vel_raw = get_be16(&frame.data[0]);
    int16_t cur_raw = get_be16(&frame.data[2]);
    uint16_t pos_raw_u16 = static_cast<uint16_t>(get_be16(&frame.data[4]));
    int16_t vol_raw = get_be16(&frame.data[6]);
    float voltage = static_cast<float>(vol_raw) / 10.0f;

    pos_raw_u16 &= 0x7FFF; // 15-bit resolution (0-32767)

    if (priv->print_raw) {
        printf("\nDEBUG: p1010b decode_feedback: pos_raw=%u, vol=%.1f V\n", pos_raw_u16, voltage);
    }

    if (priv->have_last_pos_raw) {
        int32_t delta = static_cast<int32_t>(pos_raw_u16) - static_cast<int32_t>(priv->last_pos_raw);
        if (delta > 16384) {
            priv->pos_turns -= 1;
        } else if (delta < -16384) {
            priv->pos_turns += 1;
        }
    }
    priv->last_pos_raw = pos_raw_u16;
    priv->have_last_pos_raw = true;

    // vel_raw is 10.0 * RPM
    float rpm = static_cast<float>(vel_raw) / 10.0f;
    state->vel = rpm / 60.0f * (2.0f * k_pi);
    state->trq = static_cast<float>(cur_raw) / 100.0f; // Simplified as A
    state->pos = (static_cast<float>(pos_raw_u16) / 32768.0f + static_cast<float>(priv->pos_turns)) * (2.0f * k_pi);
    state->temp = 0.0f;
    state->err = 0;

    priv->last_state = *state;
    priv->have_last_state = true;
}

static int p1010b_init(struct motor_dev* dev) {
    if (!dev || !dev->priv_data) return -1;
    struct p1010b_priv* priv = static_cast<struct p1010b_priv*>(dev->priv_data);
    if (priv->initialized) return 0;

    if (p1010b_open_socket(priv->bus_name, &priv->can_fd) < 0) return -1;

    priv->initialized = true;
    priv->enabled = false;
    priv->active_mode = kModeVoltage;
    priv->have_last_state = false;
    // Sync multi-turn absolute position
    uint8_t query_data[8] = {0x0B, 0x0D, 0, 0, 0, 0, 0, 0};
    p1010b_send_frame(priv->can_fd, 0x35, query_data, 8);

    bool synced = false;
    struct can_frame frame;
    if (p1010b_recv_id(priv, 0x70 + priv->can_id, &frame) == 0) {
        int16_t multi_turn_raw = get_be16(&frame.data[0]);
        uint16_t single_turn_raw = static_cast<uint16_t>(get_be16(&frame.data[2])) & 0x7FFF;

        float absolute_turns = static_cast<float>(multi_turn_raw) / 100.0f;
        float single_turn_frac = static_cast<float>(single_turn_raw) / 32768.0f;

        priv->pos_turns = static_cast<int32_t>(std::round(absolute_turns - single_turn_frac));
        priv->last_pos_raw = single_turn_raw;
        priv->have_last_pos_raw = true;

        printf("DEBUG: P1010B init ID %u: abs_turns=%.3f, single_frac=%.3f, pos_turns=%d\n",
                priv->can_id, absolute_turns, single_turn_frac, priv->pos_turns);

        synced = true;
    }

    if (!synced) {
        printf("DEBUG: P1010B init ID %u: Failed to sync multi-turn absolute position\n", priv->can_id);
        priv->pos_turns = 0;
        priv->have_last_pos_raw = false;
    }

    // Must be disabled to change mode
    p1010b_set_enable(priv, false);
    usleep(10000);
    p1010b_set_mode(priv, kModePosition);
    priv->active_mode = kModePosition;
    usleep(10000);
    p1010b_set_enable(priv, true);
    priv->enabled = true;

    return 0;
}

static int p1010b_set_cmd(struct motor_dev* dev, const struct motor_cmd* cmd) {
    if (!dev || !dev->priv_data || !cmd) return -1;
    struct p1010b_priv* priv = static_cast<struct p1010b_priv*>(dev->priv_data);
    if (!priv->initialized) {
        if (p1010b_init(dev) < 0) return -1;
    }

    uint8_t mode_value;
    int16_t raw_value = 0;
    bool enable = true;

    switch (cmd->mode) {
        case MOTOR_MODE_IDLE:
            enable = false;
            break;
        case MOTOR_MODE_POS:
        case MOTOR_MODE_CSP:
            mode_value = kModePosition;
            raw_value = encode_position_raw(cmd->pos_des);
            break;
        case MOTOR_MODE_VEL:
        case MOTOR_MODE_CSV:
            mode_value = kModeVelocity;
            raw_value = encode_velocity_raw(cmd->vel_des);
            break;
        case MOTOR_MODE_TRQ:
        case MOTOR_MODE_CST:
            mode_value = kModeCurrent;
            raw_value = encode_current_raw(cmd->trq_des);
            break;
        case MOTOR_MODE_HYBRID:
            mode_value = kModeMIT;
            raw_value = 0; // 硬件暂未实现
            break;
        case MOTOR_MODE_OPEN:
            mode_value = kModeVoltage;
            raw_value = encode_voltage_raw(cmd->vel_des); // open-loop voltage is passed via vel_des
            break;
        default:
            mode_value = kModePosition;
            raw_value = encode_position_raw(cmd->pos_des);
            break;
    }

    if (!enable) {
        if (priv->enabled) {
            p1010b_set_enable(priv, false);
            priv->enabled = false;
        }
        return 0;
    }

    if (mode_value != priv->active_mode) {
        p1010b_set_enable(priv, false);
        usleep(10000);
        p1010b_set_mode(priv, mode_value);
        if (mode_value == kModeVoltage) {
            usleep(10000);
            p1010b_set_active_report(priv, true);
        } else if (priv->active_mode == kModeVoltage) {
            usleep(10000);
            p1010b_set_active_report(priv, false);
        }
        priv->active_mode = mode_value;
        usleep(10000);
        p1010b_set_enable(priv, true);
        priv->enabled = true;
    } else if (!priv->enabled) {
        p1010b_set_enable(priv, true);
        priv->enabled = true;
    }

    return p1010b_send_setpoint(priv, raw_value);
}

static int p1010b_get_state(struct motor_dev* dev, struct motor_state* state) {
    if (!dev || !dev->priv_data || !state) return -1;
    struct p1010b_priv* priv = static_cast<struct p1010b_priv*>(dev->priv_data);
    if (!priv->initialized) {
        if (p1010b_init(dev) < 0) return -1;
    }

    // Active query by sending 0x35 (requesting Speed=0x01, IQ=0x03, AbsPos=0x0D, Voltage=0x0A)
    uint8_t query_data[8] = {0x01, 0x03, 0x0D, 0x0A, 0, 0, 0, 0};
    p1010b_send_frame(priv->can_fd, 0x35, query_data, 8);

    struct can_frame frame;
    // Only accept the 0x70 response to our 0x35 query, avoiding 0x50 auto-reports
    if (p1010b_recv_id(priv, 0x70 + priv->can_id, &frame) == 0) {
        decode_feedback(priv, frame, state);
        return 0;
    }
    if (priv->have_last_state) {
        *state = priv->last_state;
        return 0;
    }
    return -1;
}

static void p1010b_free(struct motor_dev* dev) {
    if (!dev) return;
    struct p1010b_priv* priv = static_cast<struct p1010b_priv*>(dev->priv_data);
    if (priv) {
        {
            std::lock_guard<std::mutex> lock(g_bus_mutex);
            g_bus_states[priv->bus_name].configured_mask &= ~(1 << (priv->can_id - 1));
            g_bus_states[priv->bus_name].update_mask &= ~(1 << (priv->can_id - 1));
        }
        if (priv->can_fd >= 0) {
            p1010b_set_enable(priv, false);
            close(priv->can_fd);
        }
        free(priv);
    }
    free(dev);
}

static int p1010b_set_paras(struct motor_dev *dev, const void *address, const void *data, uint32_t data_len) {
    if (!dev || !dev->priv_data || !address || !data || data_len != 4) return -1;
    struct p1010b_priv* priv = static_cast<struct p1010b_priv*>(dev->priv_data);

    uint8_t param_idx = *static_cast<const uint8_t*>(address);
    uint32_t val = *static_cast<const uint32_t*>(data);

    uint8_t frame_data[8] = {0};
    frame_data[0] = priv->can_id;
    frame_data[1] = param_idx;
    frame_data[2] = static_cast<uint8_t>(val & 0xFF);
    frame_data[3] = static_cast<uint8_t>((val >> 8) & 0xFF);
    frame_data[4] = static_cast<uint8_t>((val >> 16) & 0xFF);
    frame_data[5] = static_cast<uint8_t>((val >> 24) & 0xFF);
    frame_data[6] = 0;
    frame_data[7] = 0;

    return p1010b_send_frame(priv->can_fd, kIdParamSet, frame_data, 8);
}

static int p1010b_get_paras(struct motor_dev* dev, const void* address, void* out_data, uint32_t out_len) {
    if (!dev || !dev->priv_data || !address || !out_data || out_len != 2) return -1;
    struct p1010b_priv* priv = static_cast<struct p1010b_priv*>(dev->priv_data);

    uint8_t code = *static_cast<const uint8_t*>(address);
    uint8_t query_data[8] = {code, 0, 0, 0, 0, 0, 0, 0};

    if (p1010b_send_frame(priv->can_fd, 0x35, query_data, 8) < 0) return -1;

    struct can_frame frame;
    for (int i = 0; i < 10; ++i) {
        if (p1010b_recv_id(priv, 0x70 + priv->can_id, &frame) == 0) {
            uint16_t val = get_be16(&frame.data[0]);
            *static_cast<uint16_t*>(out_data) = val;
            return 0;
        }
    }
    return -1;
}

static const struct motor_ops p1010b_ops = {
    .init = p1010b_init,
    .set_cmd = p1010b_set_cmd,
    .get_state = p1010b_get_state,
    .free = p1010b_free,
    .set_paras = p1010b_set_paras,
    .get_paras = p1010b_get_paras,
};

static struct motor_dev* p1010b_probe(void* args) {
    struct motor_args_can* params = static_cast<struct motor_args_can*>(args);
    if (!params) return nullptr;

    struct motor_dev* dev = static_cast<struct motor_dev*>(calloc(1, sizeof(struct motor_dev)));
    if (!dev) return nullptr;

    struct p1010b_priv* priv = static_cast<struct p1010b_priv*>(calloc(1, sizeof(struct p1010b_priv)));
    if (!priv) {
        free(dev);
        return nullptr;
    }

    dev->ops = &p1010b_ops;
    dev->priv_data = priv;
    priv->can_fd = -1;
    priv->initialized = false;
    priv->enabled = false;
    priv->active_mode = kModeVoltage;
    priv->print_raw = false;

    if (params->args) {
        priv->print_raw = *static_cast<bool*>(params->args);
    }

    uint32_t id = params->can_id;
    if (id < 1 || id > 8) id = 1;
    priv->can_id = static_cast<uint8_t>(id);

    if (params->iface) {
        std::snprintf(priv->bus_name, sizeof(priv->bus_name), "%s", params->iface);
    } else {
        std::snprintf(priv->bus_name, sizeof(priv->bus_name), "can0");
    }
    std::snprintf(priv->dev_name, sizeof(priv->dev_name), "drv_can_ddt_p1010b_%s_id%u", priv->bus_name, priv->can_id);
    dev->name = priv->dev_name;

    {
        std::lock_guard<std::mutex> lock(g_bus_mutex);
        g_bus_states[priv->bus_name].configured_mask |= (1 << (priv->can_id - 1));
    }

    return dev;
}

}  // namespace

extern "C" {
struct motor_dev* drv_can_ddt_p1010b_probe(void* args) {
    return p1010b_probe(args);
}
}

REGISTER_MOTOR_DRIVER("drv_can_ddt_p1010b", DRV_TYPE_CAN, drv_can_ddt_p1010b_probe);
