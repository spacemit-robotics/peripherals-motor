/*
 * Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Driver for the Benmo (本末) M15-2D-133 integrated servo motor over CAN.
 *
 * Bus: standard CAN frame, 1000 kbps, 8-byte payload.
 *
 * Control command frames (up to 8 motors share one broadcast frame):
 *   - 0x105  set mode      data[id-1] = mode value
 *                          (0x00 open, 0x01 current, 0x02 velocity,
 *                           0x03 position, 0x09 disable, 0x0A enable)
 *   - 0x106  set feedback  data[id-1] = feedback config
 *                          (bit7: 1 = query mode / 0 = auto report;
 *                           low7: report period in ms, 1..127)
 *   - 0x32   setpoint for motor id 1..4 (2 bytes each, big-endian)
 *   - 0x33   setpoint for motor id 5..8 (2 bytes each, big-endian)
 *   - 0x107  query request (in query feedback mode)
 *
 * Setpoint ranges per loop:
 *   - open    : -32767..32767  (min..max speed, reverse..forward)
 *   - current : -16383..16383  -> -55A..55A
 *   - velocity: -21000..21000  -> -210rpm..210rpm
 *   - position: 0..32767       -> 0deg..360deg
 *
 * Feedback frame 0x96 + id (auto report or query response):
 *   data[0,1] velocity   (-21000..21000 -> -210..210 rpm)
 *   data[2,3] current    (-32767..32767 -> -55..55 A)
 *   data[4,5] position   (0..65535 -> 0..360 deg, or 0..32767 after zero cal)
 *   data[6]   fault code
 *   data[7]   current mode (or winding/MOS temperature, see last-byte setting)
 *
 * Configuration frames each get a dedicated response identifier which makes
 * read-back reliable (see drv_can_ddt_m152d133.h for the exposed parameters).
 */

#include <algorithm>
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

extern "C" {
#include "../../../../include/motor.h"
#include "../../../motor_core.h"
}

#include "../include/drv_can_ddt_m152d133.h"

namespace {

constexpr float k_pi = 3.14159265358979323846f;
constexpr float k_torque_constant = 0.88f;  // Nm/A, from the datasheet

/* --- CAN identifiers --- */
constexpr uint32_t kIdSetMode = 0x105;
constexpr uint32_t kIdSetFeedback = 0x106;
constexpr uint32_t kIdQuery = 0x107;
constexpr uint32_t kIdSetMotorId = 0x108;
constexpr uint32_t kIdSetTermRes = 0x109;
constexpr uint32_t kIdQueryFwVer = 0x10A;
constexpr uint32_t kIdCommTimeout = 0x10B;
constexpr uint32_t kIdPiParam = 0x10C;
constexpr uint32_t kIdCurrentFilter = 0x10D;
constexpr uint32_t kIdLastByte = 0x10E;
constexpr uint32_t kIdMechZero = 0x112;

constexpr uint32_t kIdFeedbackBase = 0x96;  /* + motor id */
constexpr uint32_t kIdCommTimeoutRespBase = 0x32C;
constexpr uint32_t kIdFwVerRespBase = 0x2C8;

/* --- mode values on the 0x105 frame --- */
constexpr uint8_t kModeCurrent = 0x01;
constexpr uint8_t kModeVelocity = 0x02;
constexpr uint8_t kModePosition = 0x03;
constexpr uint8_t kModeDisable = 0x09;
constexpr uint8_t kModeEnable = 0x0A;
constexpr uint8_t kModeOpen = 0x00;

/* --- query target contents on the 0x107 frame --- */
constexpr uint8_t kQueryVelocity = 0x01;
constexpr uint8_t kQueryBusCurrent = 0x02;
constexpr uint8_t kQueryWindingTemp = 0x03;
constexpr uint8_t kQueryPosition = 0x04;
constexpr uint8_t kQueryFault = 0x05;
constexpr uint8_t kQueryMode = 0x06;

struct bm_priv {
    char bus_name[16];
    char dev_name[48];
    int can_fd;
    uint8_t can_id;      /* 1..8 */
    uint8_t active_mode; /* last mode value pushed on 0x105 */
    bool enabled;
    bool initialized;
    struct motor_state last_state;
    bool have_last_state;
    /* Multi-turn position tracking */
    uint16_t last_pos_raw;    /* Previous raw position for wrap detection */
    int32_t pos_turns;        /* Accumulated full turns (positive or negative) */
    bool have_last_pos_raw;   /* Whether last_pos_raw is valid */
};

static int32_t clamp_i32(int32_t value, int32_t lo, int32_t hi) {
    if (value < lo) return lo;
    if (value > hi) return hi;
    return value;
}

static int16_t encode_position_raw(float rad) {
    float deg = std::fmod(rad * 180.0f / k_pi, 360.0f);
    if (deg < 0.0f) {
        deg += 360.0f;
    }
    // 华为蓝军自检发现：固件严格拒绝 >32767 的位置指令。必须回退到 32767。
    int32_t raw = static_cast<int32_t>(std::lround(deg * 32768.0f / 360.0f));
    return static_cast<int16_t>(clamp_i32(raw, 0, 32767));
}

static int16_t encode_velocity_raw(float rad_s) {
    float rpm = rad_s * 60.0f / (2.0f * k_pi);
    int32_t raw = static_cast<int32_t>(std::lround(rpm * 100.0f));
    return static_cast<int16_t>(clamp_i32(raw, -21000, 21000));
}

/* Torque (Nm) -> current setpoint raw (-16383..16383 maps to -55..55A). */
static int16_t encode_torque_raw(float torque_nm) {
    float amps = torque_nm / k_torque_constant;
    int32_t raw = static_cast<int32_t>(std::lround(amps * 16383.0f / 55.0f));
    return static_cast<int16_t>(clamp_i32(raw, -16383, 16383));
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

static uint16_t get_be16u(const uint8_t* src) {
    uint16_t hi = static_cast<uint16_t>(src[0]);
    uint16_t lo = static_cast<uint16_t>(src[1]);
    return static_cast<uint16_t>((hi << 8) | lo);
}

static int bm_open_socket(const char* iface, int* fd_out) {
    if (!iface || !fd_out) {
        return -1;
    }

    int fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd < 0) {
        std::perror("socket(PF_CAN)");
        return -1;
    }

    struct ifreq ifr;
    std::memset(&ifr, 0, sizeof(ifr));
    std::snprintf(ifr.ifr_name, sizeof(ifr.ifr_name), "%s", iface);
    if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
        std::perror("ioctl(SIOCGIFINDEX)");
        close(fd);
        return -1;
    }

    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(fd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::perror("bind(can)");
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

static int bm_send_frame(int fd, uint32_t can_id, const uint8_t* data, uint8_t dlc) {
    if (fd < 0 || !data || dlc > 8) {
        return -1;
    }

    struct can_frame frame;
    std::memset(&frame, 0, sizeof(frame));
    frame.can_id = can_id;
    frame.can_dlc = dlc;
    std::memcpy(frame.data, data, dlc);
    ssize_t n = write(fd, &frame, sizeof(frame));
    return (n == static_cast<ssize_t>(sizeof(frame))) ? 0 : -1;
}

/*
 * Read one CAN frame matching want_id. When want_id == 0 any frame is accepted.
 * Returns 0 on success (frame copied to out), -1 on timeout/error.
 */
static int bm_recv_id(struct bm_priv* priv, uint32_t want_id, struct can_frame* out) {
    struct can_frame frame;
    for (int guard = 0; guard < 64; ++guard) {
        ssize_t n = read(priv->can_fd, &frame, sizeof(frame));
        if (n < 0) {
            return -1;  // timeout / error
        }
        if (n != static_cast<ssize_t>(sizeof(frame))) {
            continue;
        }
        if (want_id == 0 || (frame.can_id & CAN_EFF_MASK) == want_id) {
            if (out) {
                *out = frame;
            }
            return 0;
        }
    }
    return -1;
}

/* --- control frames --- */

static int bm_send_mode(struct bm_priv* priv, uint8_t mode_value) {
    uint8_t data[8] = {0};
    data[priv->can_id - 1] = mode_value;
    return bm_send_frame(priv->can_fd, kIdSetMode, data, 8);
}

static int bm_send_setpoint(struct bm_priv* priv, int16_t raw_value) {
    uint8_t data[8] = {0};
    uint32_t slot = static_cast<uint32_t>((priv->can_id - 1) % 4);
    put_be16(&data[slot * 2], raw_value);
    uint32_t frame_id = (priv->can_id <= 4) ? 0x32u : 0x33u;
    return bm_send_frame(priv->can_fd, frame_id, data, 8);
}

static void decode_feedback(struct bm_priv* priv, const struct can_frame& frame,
                            struct motor_state* state) {
    int16_t vel_raw = get_be16(&frame.data[0]);
    int16_t cur_raw = get_be16(&frame.data[2]);
    uint16_t pos_raw = get_be16u(&frame.data[4]);
    uint8_t fault = frame.data[6];

    /* Position feedback always has a 15-bit resolution per revolution (0-32767, span 32768).
     * The MSB (bit 15) is an uncalibrated flag used by the firmware. We must mask it out. */
    pos_raw &= 0x7FFF;
    float pos_span = 32768.0f;
    uint16_t span_int = 32768;

    /* Multi-turn wrap detection: if position jumps by more than half a turn,
     * assume it wrapped around rather than moved that far in one sample. */
    if (priv->have_last_pos_raw) {
        int32_t delta = static_cast<int32_t>(pos_raw) - static_cast<int32_t>(priv->last_pos_raw);
        int32_t half_span = span_int / 2;
        if (delta > half_span) {
            /* Jumped from low to high -> actually moved backward across zero */
            priv->pos_turns -= 1;
        } else if (delta < -half_span) {
            /* Jumped from high to low -> actually moved forward across max */
            priv->pos_turns += 1;
        }
    }
    priv->last_pos_raw = pos_raw;
    priv->have_last_pos_raw = true;

    /* Continuous position = (full turns * span + current raw) mapped to radians */
    float continuous_raw = static_cast<float>(priv->pos_turns) * pos_span +
                            static_cast<float>(pos_raw);
    state->pos = continuous_raw * 2.0f * k_pi / pos_span;
    state->vel = static_cast<float>(vel_raw) / 100.0f * (2.0f * k_pi / 60.0f);
    float amps = static_cast<float>(cur_raw) * 55.0f / 32767.0f;
    state->trq = amps * k_torque_constant;
    state->temp = 0.0f;  /* only available when the last byte is set to temperature */
    state->err = static_cast<uint32_t>(fault);

    if (std::getenv("BM_DEBUG_RAW") != nullptr) {
        std::printf("[RAW] pos_raw=%u vel_raw=%d cur_raw=%d -> pos=%.3f vel=%.3f\n",
                    static_cast<unsigned>(pos_raw), static_cast<int>(vel_raw),
                    static_cast<int>(cur_raw), state->pos, state->vel);
    }

    priv->last_state = *state;
    priv->have_last_state = true;
}

static int bm_receive_feedback(struct bm_priv* priv, struct motor_state* state) {
    struct can_frame frame;
    if (bm_recv_id(priv, kIdFeedbackBase + priv->can_id, &frame) == 0) {
        decode_feedback(priv, frame, state);
        return 0;
    }
    if (priv->have_last_state) {
        *state = priv->last_state;
        return 0;
    }
    return -1;
}

/* --- lifecycle --- */

static int bm_init(struct motor_dev* dev) {
    if (!dev || !dev->priv_data) {
        return -1;
    }

    struct bm_priv* priv = static_cast<struct bm_priv*>(dev->priv_data);
    if (priv->initialized) {
        return 0;
    }

    if (bm_open_socket(priv->bus_name, &priv->can_fd) < 0) {
        return -1;
    }

    priv->initialized = true;
    priv->enabled = false;
    priv->active_mode = kModeDisable;
    priv->have_last_state = false;
    std::memset(&priv->last_state, 0, sizeof(priv->last_state));

    /* Start from a known disabled state. */
    (void)bm_send_mode(priv, kModeDisable);
    priv->active_mode = kModeDisable;
    priv->enabled = false;

    /* Completely drain the socket queue of any stale frames to prevent time-machine twitching. */
    struct can_frame drain_frame;
    while (recv(priv->can_fd, &drain_frame, sizeof(drain_frame), MSG_DONTWAIT) > 0) {
        // discard stale frames
    }

    return 0;
}

static int bm_ensure_init(struct motor_dev* dev, struct bm_priv* priv) {
    if (!priv->initialized) {
        return bm_init(dev);
    }
    return 0;
}

static int bm_set_cmd(struct motor_dev* dev, const struct motor_cmd* cmd) {
    if (!dev || !dev->priv_data || !cmd) {
        return -1;
    }

    struct bm_priv* priv = static_cast<struct bm_priv*>(dev->priv_data);
    if (bm_ensure_init(dev, priv) < 0) {
        return -1;
    }

    uint8_t mode_value;
    int16_t raw_value = 0;
    bool has_setpoint = true;

    switch (cmd->mode) {
        case MOTOR_MODE_IDLE:
            mode_value = kModeDisable;
            has_setpoint = false;
            break;
        case MOTOR_MODE_POS:
        case MOTOR_MODE_CSP:
        case MOTOR_MODE_HM:
            mode_value = kModePosition;
            raw_value = encode_position_raw(cmd->pos_des);
            break;
        case MOTOR_MODE_OPEN:
            mode_value = kModeOpen;
            /* For open loop, map the abstract velocity/duty desire to the native open loop command range */
            raw_value = encode_velocity_raw(cmd->vel_des);
            break;
        case MOTOR_MODE_VEL:
        case MOTOR_MODE_CSV:
            mode_value = kModeVelocity;
            raw_value = encode_velocity_raw(cmd->vel_des);
            break;
        case MOTOR_MODE_TRQ:
        case MOTOR_MODE_CST:
            mode_value = kModeCurrent;
            raw_value = encode_torque_raw(cmd->trq_des);
            break;
        case MOTOR_MODE_HYBRID:
            /* No native impedance mode; approximate with position loop. */
            mode_value = kModePosition;
            raw_value = encode_position_raw(cmd->pos_des);
            break;
        default:
            mode_value = kModePosition;
            raw_value = encode_position_raw(cmd->pos_des);
            break;
    }

    /* Only push a mode frame when the loop actually changes or we need to enable/disable. */
    if (mode_value != priv->active_mode || mode_value == kModeDisable || !priv->enabled) {

        if (mode_value != kModeDisable && priv->active_mode != kModeDisable && mode_value != priv->active_mode) {
            /* Switching between different active loops (e.g. Vel -> Pos) requires a disable first. */
            (void)bm_send_mode(priv, kModeDisable);
            usleep(50000);
        }

        if (has_setpoint && mode_value != kModeDisable) {
            /* Preload setpoint */
            (void)bm_send_setpoint(priv, raw_value);
            usleep(2000);
        }

        if (mode_value == kModeDisable) {
            if (bm_send_mode(priv, kModeDisable) < 0) return -1;
            priv->enabled = false;
        } else {
            /* If we are just enabling into the SAME mode we were already in,
             * send 0x0A (Enable) which might perform a smooth zero-twitch enable
             * in the firmware, rather than 0x03 which hard-resets the PID target. */
            if (priv->active_mode == mode_value && !priv->enabled) {
                if (bm_send_mode(priv, kModeEnable) < 0) return -1;
            } else {
                if (bm_send_mode(priv, mode_value) < 0) return -1;
            }
            priv->active_mode = mode_value;
            priv->enabled = true;
        }
    }

    if (!has_setpoint) {
        return 0;
    }
    return bm_send_setpoint(priv, raw_value);
}

static int bm_get_state(struct motor_dev* dev, struct motor_state* state) {
    if (!dev || !dev->priv_data || !state) {
        return -1;
    }
    struct bm_priv* priv = static_cast<struct bm_priv*>(dev->priv_data);
    if (bm_ensure_init(dev, priv) < 0) {
        return -1;
    }
    return bm_receive_feedback(priv, state);
}

static void bm_free(struct motor_dev* dev) {
    if (!dev) {
        return;
    }

    struct bm_priv* priv = static_cast<struct bm_priv*>(dev->priv_data);
    if (priv) {
        if (priv->can_fd >= 0) {
            (void)bm_send_mode(priv, kModeDisable);
            usleep(50000); /* 50ms delay to ensure the CAN frame is transmitted before socket closes */
            close(priv->can_fd);
            priv->can_fd = -1;
        }
    }
    dev->name = nullptr;
    if (dev->priv_data) {
        free(dev->priv_data);
        dev->priv_data = nullptr;
    }
    free(dev);
}

/* --- parameter helpers --- */

/* Query a single-target value through the 0x107 request / 0x96 response cycle. */
static int bm_query_scalar(struct bm_priv* priv, uint8_t target, int16_t* raw_out) {
    uint8_t req[8] = {0};
    req[0] = target;        /* DATA[0]: query target content */
    req[1] = priv->can_id;  /* DATA[1]: motor id */
    req[2] = 0x00;          /* DATA[2]: custom tag to disambiguate the reply */
    if (bm_send_frame(priv->can_fd, kIdQuery, req, 8) < 0) {
        return -1;
    }

    struct can_frame frame;
    if (bm_recv_id(priv, kIdFeedbackBase + priv->can_id, &frame) < 0) {
        return -1;
    }
    if (raw_out) {
        *raw_out = get_be16(&frame.data[0]);
    }
    return 0;
}

static int bm_set_feedback(struct bm_priv* priv, uint8_t config) {
    uint8_t data[8] = {0};
    data[priv->can_id - 1] = config;
    return bm_send_frame(priv->can_fd, kIdSetFeedback, data, 8);
}

static int bm_set_motor_id(struct bm_priv* priv, uint8_t new_id) {
    uint8_t data[8] = {0};
    data[0] = new_id;
    return bm_send_frame(priv->can_fd, kIdSetMotorId, data, 8);
}

static int bm_set_term_res(struct bm_priv* priv, uint8_t on) {
    uint8_t data[8] = {0};
    data[priv->can_id - 1] = on ? 1 : 0;
    return bm_send_frame(priv->can_fd, kIdSetTermRes, data, 8);
}

static int bm_set_comm_timeout(struct bm_priv* priv, uint16_t timeout_ms) {
    uint8_t data[8] = {0};
    data[0] = priv->can_id;
    data[1] = 0x10;  /* set */
    data[2] = 0x01;  /* write */
    data[3] = static_cast<uint8_t>((timeout_ms >> 8) & 0xFF);
    data[4] = static_cast<uint8_t>(timeout_ms & 0xFF);
    return bm_send_frame(priv->can_fd, kIdCommTimeout, data, 8);
}

static int bm_get_comm_timeout(struct bm_priv* priv, uint16_t* out_ms) {
    uint8_t data[8] = {0};
    data[0] = priv->can_id;
    data[1] = 0x10;  /* set flag field, ignored on read */
    data[2] = 0x00;  /* read */
    if (bm_send_frame(priv->can_fd, kIdCommTimeout, data, 8) < 0) {
        return -1;
    }
    struct can_frame frame;
    if (bm_recv_id(priv, kIdCommTimeoutRespBase + priv->can_id, &frame) < 0) {
        return -1;
    }
    if (out_ms) {
        uint16_t hi = static_cast<uint16_t>(frame.data[3]);
        uint16_t lo = static_cast<uint16_t>(frame.data[4]);
        *out_ms = static_cast<uint16_t>((hi << 8) | lo);
    }
    return 0;
}

static int bm_set_current_filter(struct bm_priv* priv, uint16_t value) {
    uint8_t data[8] = {0};
    data[0] = priv->can_id;
    data[1] = 0x20;  /* set */
    data[2] = 0x01;  /* write */
    data[3] = static_cast<uint8_t>((value >> 8) & 0xFF);
    data[4] = static_cast<uint8_t>(value & 0xFF);
    return bm_send_frame(priv->can_fd, kIdCurrentFilter, data, 8);
}

static int bm_set_last_byte(struct bm_priv* priv, uint8_t mode) {
    uint8_t data[8] = {0};
    data[0] = priv->can_id;
    data[1] = 0x30;  /* set */
    data[2] = 0x01;  /* write */
    data[3] = mode;
    return bm_send_frame(priv->can_fd, kIdLastByte, data, 8);
}

static int bm_set_pi(struct bm_priv* priv, const struct bm_m152d133_pi_param* pi) {
    uint8_t data[8] = {0};
    data[0] = priv->can_id;
    data[1] = pi->mode;
    data[2] = static_cast<uint8_t>((pi->p_num >> 8) & 0xFF);
    data[3] = static_cast<uint8_t>(pi->p_num & 0xFF);
    data[4] = pi->p_den;
    data[5] = static_cast<uint8_t>((pi->i_num >> 8) & 0xFF);
    data[6] = static_cast<uint8_t>(pi->i_num & 0xFF);
    data[7] = pi->i_den;
    return bm_send_frame(priv->can_fd, kIdPiParam, data, 8);
}

static int bm_save_params(struct bm_priv* priv) {
    /* Parameter save requires the motor to be disabled first. */
    (void)bm_send_mode(priv, kModeDisable);
    priv->enabled = false;
    priv->active_mode = kModeDisable;

    uint8_t data[8] = {0};
    data[0] = priv->can_id;
    data[1] = 0xFE;  /* save command shares the PI frame id */
    return bm_send_frame(priv->can_fd, kIdPiParam, data, 8);
}

static int bm_calibrate_mech_zero(struct bm_priv* priv) {
    uint8_t data[8] = {0};
    return bm_send_frame(priv->can_fd, kIdMechZero, data, 8);
}

static int bm_query_fw_version(struct bm_priv* priv, uint32_t* out_version) {
    uint8_t data[8] = {0};
    if (bm_send_frame(priv->can_fd, kIdQueryFwVer, data, 8) < 0) {
        return -1;
    }
    struct can_frame frame;
    if (bm_recv_id(priv, kIdFwVerRespBase + priv->can_id, &frame) < 0) {
        return -1;
    }
    if (out_version) {
        uint32_t sw_major = static_cast<uint32_t>(frame.data[1]);
        uint32_t sw_minor = static_cast<uint32_t>(frame.data[2]);
        uint32_t hw_major = static_cast<uint32_t>(frame.data[3]);
        uint32_t hw_minor = static_cast<uint32_t>(frame.data[4]);
        *out_version = (sw_major << 24) | (sw_minor << 16) | (hw_major << 8) | hw_minor;
    }
    return 0;
}

static int bm_set_paras(struct motor_dev* dev, const void* address, const void* data,
                        uint32_t data_len) {
    if (!dev || !dev->priv_data || !address) {
        return -1;
    }
    struct bm_priv* priv = static_cast<struct bm_priv*>(dev->priv_data);
    if (bm_ensure_init(dev, priv) < 0) {
        return -1;
    }

    uint32_t param = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(address));

    switch (param) {
        case BM_M152D133_PARAM_ENABLE: {
            if (!data || data_len < sizeof(uint8_t)) return -1;
            uint8_t on = *static_cast<const uint8_t*>(data);
            uint8_t mode = on ? kModeEnable : kModeDisable;
            if (bm_send_mode(priv, mode) < 0) return -1;
            priv->enabled = (on != 0);
            priv->active_mode = mode;
            return 0;
        }
        case BM_M152D133_PARAM_MODE: {
            if (!data || data_len < sizeof(uint8_t)) return -1;
            uint8_t mode = *static_cast<const uint8_t*>(data);
            /* Switching loops requires the drive to be idle first. */
            if (mode != kModeDisable && priv->active_mode != kModeDisable &&
                mode != priv->active_mode) {
                (void)bm_send_mode(priv, kModeDisable);
                usleep(50000); /* 50ms delay to let the firmware process the disable command */
            }
            if (bm_send_mode(priv, mode) < 0) return -1;
            priv->active_mode = mode;
            priv->enabled = (mode != kModeDisable);
            return 0;
        }
        case BM_M152D133_PARAM_FEEDBACK: {
            if (!data || data_len < sizeof(uint8_t)) return -1;
            return bm_set_feedback(priv, *static_cast<const uint8_t*>(data));
        }
        case BM_M152D133_PARAM_MOTOR_ID: {
            if (!data || data_len < sizeof(uint8_t)) return -1;
            return bm_set_motor_id(priv, *static_cast<const uint8_t*>(data));
        }
        case BM_M152D133_PARAM_TERM_RES: {
            if (!data || data_len < sizeof(uint8_t)) return -1;
            return bm_set_term_res(priv, *static_cast<const uint8_t*>(data));
        }
        case BM_M152D133_PARAM_COMM_TIMEOUT: {
            if (!data || data_len < sizeof(uint16_t)) return -1;
            return bm_set_comm_timeout(priv, *static_cast<const uint16_t*>(data));
        }
        case BM_M152D133_PARAM_CURRENT_FILTER: {
            if (!data || data_len < sizeof(uint16_t)) return -1;
            return bm_set_current_filter(priv, *static_cast<const uint16_t*>(data));
        }
        case BM_M152D133_PARAM_LAST_BYTE: {
            if (!data || data_len < sizeof(uint8_t)) return -1;
            return bm_set_last_byte(priv, *static_cast<const uint8_t*>(data));
        }
        case BM_M152D133_PARAM_PI: {
            if (!data || data_len < sizeof(struct bm_m152d133_pi_param)) return -1;
            return bm_set_pi(priv, static_cast<const struct bm_m152d133_pi_param*>(data));
        }
        case BM_M152D133_PARAM_SAVE:
            return bm_save_params(priv);
        case BM_M152D133_PARAM_MECH_ZERO:
            return bm_calibrate_mech_zero(priv);
        default:
            return -1;
    }
}

static int bm_get_paras(struct motor_dev* dev, const void* address, void* out_data,
                        uint32_t data_len) {
    if (!dev || !dev->priv_data || !address || !out_data) {
        return -1;
    }
    struct bm_priv* priv = static_cast<struct bm_priv*>(dev->priv_data);
    if (bm_ensure_init(dev, priv) < 0) {
        return -1;
    }

    uint32_t param = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(address));

    switch (param) {
        case BM_M152D133_PARAM_FW_VERSION: {
            if (data_len < sizeof(uint32_t)) return -1;
            return bm_query_fw_version(priv, static_cast<uint32_t*>(out_data));
        }
        case BM_M152D133_PARAM_COMM_TIMEOUT: {
            if (data_len < sizeof(uint16_t)) return -1;
            return bm_get_comm_timeout(priv, static_cast<uint16_t*>(out_data));
        }
        case BM_M152D133_PARAM_VELOCITY: {
            if (data_len < sizeof(float)) return -1;
            int16_t raw;
            if (bm_query_scalar(priv, kQueryVelocity, &raw) < 0) return -1;
            *static_cast<float*>(out_data) =
                static_cast<float>(raw) / 100.0f * (2.0f * k_pi / 60.0f);
            return 0;
        }
        case BM_M152D133_PARAM_POSITION: {
            if (data_len < sizeof(float)) return -1;
            int16_t raw;
            if (bm_query_scalar(priv, kQueryPosition, &raw) < 0) return -1;
            uint16_t pos_raw = static_cast<uint16_t>(raw) & 0x7FFF;
            *static_cast<float*>(out_data) =
                static_cast<float>(pos_raw) * 2.0f * k_pi / 32768.0f;
            return 0;
        }
        case BM_M152D133_PARAM_CURRENT: {
            if (data_len < sizeof(float)) return -1;
            int16_t raw;
            if (bm_query_scalar(priv, kQueryBusCurrent, &raw) < 0) return -1;
            *static_cast<float*>(out_data) = static_cast<float>(raw) * 55.0f / 32767.0f;
            return 0;
        }
        case BM_M152D133_PARAM_TORQUE: {
            if (data_len < sizeof(float)) return -1;
            int16_t raw;
            if (bm_query_scalar(priv, kQueryBusCurrent, &raw) < 0) return -1;
            float amps = static_cast<float>(raw) * 55.0f / 32767.0f;
            *static_cast<float*>(out_data) = amps * k_torque_constant;
            return 0;
        }
        case BM_M152D133_PARAM_TEMPERATURE: {
            if (data_len < sizeof(float)) return -1;
            int16_t raw;
            if (bm_query_scalar(priv, kQueryWindingTemp, &raw) < 0) return -1;
            *static_cast<float*>(out_data) = static_cast<float>(raw);
            return 0;
        }
        case BM_M152D133_PARAM_FAULT: {
            if (data_len < sizeof(uint32_t)) return -1;
            int16_t raw;
            if (bm_query_scalar(priv, kQueryFault, &raw) < 0) return -1;
            *static_cast<uint32_t*>(out_data) = static_cast<uint32_t>(raw) & 0xFFFFu;
            return 0;
        }
        case BM_M152D133_PARAM_CUR_MODE: {
            if (data_len < sizeof(uint8_t)) return -1;
            /* The current loop is reported in DATA[7] of the feedback frame
             * (0x96 + id), which the motor auto-reports by default. */
            struct can_frame frame;
            if (bm_recv_id(priv, kIdFeedbackBase + priv->can_id, &frame) < 0) {
                return -1;
            }
            *static_cast<uint8_t*>(out_data) = frame.data[7];
            return 0;
        }
        default:
            return -1;
    }
}

static const struct motor_ops bm_ops = {
    .init = bm_init,
    .set_cmd = bm_set_cmd,
    .get_state = bm_get_state,
    .free = bm_free,
    .set_paras = bm_set_paras,
    .get_paras = bm_get_paras,
};

static struct motor_dev* bm_probe(void* args) {
    struct motor_args_can* params = static_cast<struct motor_args_can*>(args);
    if (!params) {
        return nullptr;
    }

    struct motor_dev* dev = static_cast<struct motor_dev*>(calloc(1, sizeof(struct motor_dev)));
    if (!dev) {
        return nullptr;
    }

    struct bm_priv* priv = static_cast<struct bm_priv*>(calloc(1, sizeof(struct bm_priv)));
    if (!priv) {
        free(dev);
        return nullptr;
    }

    dev->ops = &bm_ops;
    dev->priv_data = priv;
    priv->can_fd = -1;
    priv->initialized = false;
    priv->enabled = false;
    priv->active_mode = kModeDisable;
    priv->last_pos_raw = 0;
    priv->pos_turns = 0;
    priv->have_last_pos_raw = false;

    uint32_t id = params->can_id;
    if (id < 1 || id > 8) {
        id = 1;
    }
    priv->can_id = static_cast<uint8_t>(id);

    if (params->iface) {
        std::snprintf(priv->bus_name, sizeof(priv->bus_name), "%s", params->iface);
    } else {
        std::snprintf(priv->bus_name, sizeof(priv->bus_name), "can0");
    }
    unsigned int id_num = static_cast<unsigned int>(priv->can_id);
    std::snprintf(priv->dev_name, sizeof(priv->dev_name),
        "bm_m15_2d_133_%s_id%u", priv->bus_name, id_num);
    dev->name = priv->dev_name;
    return dev;
}

}  // namespace

extern "C" {
struct motor_dev* drv_can_ddt_m152d133_probe(void* args) {
    return bm_probe(args);
}
}

REGISTER_MOTOR_DRIVER("drv_can_ddt_m152d133", DRV_TYPE_CAN, drv_can_ddt_m152d133_probe);
