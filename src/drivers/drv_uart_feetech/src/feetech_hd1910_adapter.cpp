/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * HD1910-specific motor adapter with echo-tolerant serial transactions.
 */
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <poll.h>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <vector>
#include "feetech_hd1910_adapter.h"
extern "C" {
#include "../../../motor_core.h"
}

namespace {
constexpr uint8_t kTorque = 40, kGoalPosition = 42;
constexpr uint8_t kTorqueLimit = 48, kP = 50, kD = 51, kFeedback = 56;
constexpr uint8_t kFeedbackLength = 15;
constexpr uint16_t kLimit = 1000;
constexpr float kTau = 6.2831853071795864769f;
constexpr auto kReplyTimeout = std::chrono::milliseconds(80);

class Hd1910Bus {
public:
    explicit Hd1910Bus(const char* path) {
        fd_ = open(path, O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0) return;
        struct termios options{};
        if (tcgetattr(fd_, &options) != 0) return;
        cfmakeraw(&options);
        options.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);
        options.c_cflag |= CS8 | CREAD | CLOCAL;
        options.c_iflag &= ~(IXON | IXOFF | IXANY);
        if (cfsetispeed(&options, B1000000) != 0 ||
            cfsetospeed(&options, B1000000) != 0 ||
            tcsetattr(fd_, TCSANOW, &options) != 0 ||
            tcflush(fd_, TCIOFLUSH) != 0) return;
        ready_ = true;
    }
    ~Hd1910Bus() { if (fd_ >= 0) close(fd_); }
    bool ready() const { return ready_; }

    int read(uint8_t id, uint8_t address, uint8_t* out, uint8_t n) {
        if (!out || !n || n > 253 || unsigned(address) + n > 256) return -1;
        return exchange(id, 2, {address, n}, out, n);
    }
    int write(uint8_t id, uint8_t address, const uint8_t* data, uint8_t n) {
        if (!data || !n || n > 252 || unsigned(address) + n > 256) return -1;
        std::vector<uint8_t> params{address};
        params.insert(params.end(), data, data + n);
        return exchange(id, 3, params, nullptr, 0);
    }
    int sync_write(uint8_t id, uint8_t address, const uint8_t* data, uint8_t n) {
        if (!data || !n || n > 250 || unsigned(address) + n > 256) return -1;
        std::vector<uint8_t> params{address, n, id};
        params.insert(params.end(), data, data + n);
        if (tcflush(fd_, TCIFLUSH) != 0) return -1;
        return send(packet(0xfe, 0x83, params));
    }


    int sync_targets(const std::vector<uint8_t>& rows) {
        if (rows.empty() || rows.size() % 7 || rows.size() > 245) return -1;
        std::vector<uint8_t> params{kGoalPosition, 6};
        params.insert(params.end(), rows.begin(), rows.end());
        if (tcflush(fd_, TCIFLUSH) != 0) return -1;
        return send(packet(0xfe, 0x83, params));
    }
    int sync_feedback(const std::vector<uint8_t>& ids,
        std::vector<std::vector<uint8_t>>& results) {
        if (ids.empty() || ids.size() > 251) return -1;
        std::vector<uint8_t> params{kFeedback, kFeedbackLength};
        params.insert(params.end(), ids.begin(), ids.end());
        const auto request = packet(0xfe, 0x82, params);
        if (tcflush(fd_, TCIFLUSH) != 0 || send(request) != 0) return -1;
        results.assign(ids.size(), {});
        size_t received = 0;
        std::vector<uint8_t> bytes;
        auto deadline = std::chrono::steady_clock::now() + kReplyTimeout;
        while (std::chrono::steady_clock::now() < deadline) {
            struct pollfd pfd{fd_, POLLIN, 0};
            int ready = poll(&pfd, 1, remaining_ms(deadline));
            if (ready < 0 && errno == EINTR) continue;
            if (ready <= 0 || (pfd.revents & (POLLERR | POLLHUP | POLLNVAL))) break;
            uint8_t chunk[256];
            ssize_t n = ::read(fd_, chunk, sizeof(chunk));
            if (n < 0 && (errno == EINTR || errno == EAGAIN)) continue;
            if (n <= 0) break;
            bytes.insert(bytes.end(), chunk, chunk + n);
            while (bytes.size() >= 4) {
                if (bytes[0] != 0xff || bytes[1] != 0xff || bytes[3] < 2) {
                    bytes.erase(bytes.begin()); continue;
                }
                size_t size = static_cast<size_t>(bytes[3]) + 4;
                if (bytes.size() < size) break;
                uint8_t sum = 0;
                for (size_t i = 2; i < size; ++i) sum += bytes[i];
                if (sum != 0xff) { bytes.erase(bytes.begin()); continue; }
                std::vector<uint8_t> frame(bytes.begin(), bytes.begin() + size);
                bytes.erase(bytes.begin(), bytes.begin() + size);
                if (frame == request) continue;
                for (size_t i = 0; i < ids.size(); ++i) {
                    if (frame[2] != ids[i]) continue;
                    if (frame[4] != 0 || frame[3] != kFeedbackLength + 2 || !results[i].empty()) return -1;
                    results[i].assign(frame.begin() + 5, frame.end() - 1);
                    ++received;
                    break;
                }
                if (received == ids.size()) return 0;
            }
        }
        return -1;
    }

private:
    int fd_ = -1;
    bool ready_ = false;

    static std::vector<uint8_t> packet(uint8_t id, uint8_t instruction,
        const std::vector<uint8_t>& params) {
        std::vector<uint8_t> frame{0xff, 0xff, id,
            static_cast<uint8_t>(params.size() + 2), instruction};
        frame.insert(frame.end(), params.begin(), params.end());
        uint8_t sum = 0;
        for (size_t i = 2; i < frame.size(); ++i) sum += frame[i];
        frame.push_back(static_cast<uint8_t>(~sum));
        return frame;
    }

    static int remaining_ms(std::chrono::steady_clock::time_point deadline) {
        auto left = std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - std::chrono::steady_clock::now()).count();
        return left > 0 ? static_cast<int>(left) : 0;
    }

    int send(const std::vector<uint8_t>& frame) {
        auto deadline = std::chrono::steady_clock::now() + kReplyTimeout;
        size_t offset = 0;
        while (offset < frame.size()) {
            ssize_t n = ::write(fd_, frame.data() + offset, frame.size() - offset);
            if (n > 0) { offset += static_cast<size_t>(n); continue; }
            if (n < 0 && errno == EINTR) continue;
            if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) return -1;
            struct pollfd pfd{fd_, POLLOUT, 0};
            if (poll(&pfd, 1, remaining_ms(deadline)) <= 0 ||
                (pfd.revents & (POLLERR | POLLHUP | POLLNVAL))) return -1;
        }
        return 0;
    }

    int exchange(uint8_t id, uint8_t instruction, const std::vector<uint8_t>& params,
        uint8_t* out, uint8_t expected) {
        if (!ready_ || id >= 0xfe) return -1;
        const auto request = packet(id, instruction, params);
        if (tcflush(fd_, TCIFLUSH) != 0 || send(request) != 0) return -1;
        auto deadline = std::chrono::steady_clock::now() + kReplyTimeout;
        std::vector<uint8_t> bytes;
        while (std::chrono::steady_clock::now() < deadline) {
            struct pollfd pfd{fd_, POLLIN, 0};
            int polled = poll(&pfd, 1, remaining_ms(deadline));
            if (polled < 0 && errno == EINTR) continue;
            if (polled <= 0 || (pfd.revents & (POLLERR | POLLHUP | POLLNVAL))) break;
            uint8_t chunk[256];
            ssize_t n = ::read(fd_, chunk, sizeof(chunk));
            if (n < 0 && (errno == EINTR || errno == EAGAIN)) continue;
            if (n <= 0) break;
            bytes.insert(bytes.end(), chunk, chunk + n);
            while (bytes.size() >= 4) {
                if (bytes[0] != 0xff || bytes[1] != 0xff ||
                    bytes[2] >= 0xfe || bytes[3] < 2) {
                    bytes.erase(bytes.begin());
                    continue;
                }
                const size_t size = static_cast<size_t>(bytes[3]) + 4;
                if (bytes.size() < size) break;
                uint8_t sum = 0;
                for (size_t i = 2; i < size; ++i) sum += bytes[i];
                if (sum != 0xff) { bytes.erase(bytes.begin()); continue; }
                std::vector<uint8_t> frame(bytes.begin(), bytes.begin() + size);
                bytes.erase(bytes.begin(), bytes.begin() + size);
                if (frame == request || frame[2] != id) continue;
                if (frame[4] != 0) {
                    std::cerr << "[HD1910] ID " << unsigned(id) << " status error 0x"
                        << std::hex << unsigned(frame[4]) << std::dec << '\n';
                    return -1;
                }
                if (frame[3] != static_cast<unsigned>(expected) + 2) continue;
                if (expected) std::memcpy(out, frame.data() + 5, expected);
                return 0;
            }
        }
        return -1;
    }
};

struct Device {
    uint8_t id = 0;
    uint8_t firmware_p = 5;
    int32_t zero_offset = 2048;
    int32_t direction = 1;
    Hd1910Bus* bus = nullptr;
    bool armed = false;
    bool restore_p = false;
    uint8_t saved_p = 0;
};
std::map<std::string, std::unique_ptr<Hd1910Bus>> buses;
std::map<uint8_t, Device*> devices;
std::mutex bus_mutex;

Hd1910Bus* get_bus(const char* path) {
    if (!path) return nullptr;
    auto it = buses.find(path);
    if (it != buses.end()) return it->second.get();
    std::unique_ptr<Hd1910Bus> bus(new Hd1910Bus(path));
    if (!bus->ready()) return nullptr;
    auto* result = bus.get();
    buses.emplace(path, std::move(bus));
    return result;
}

int read_bytes(Device* d, uint8_t address, uint8_t* out, uint8_t n) {
    return d && d->bus ? d->bus->read(d->id, address, out, n) : -1;
}
int write_bytes(Device* d, uint8_t address, const uint8_t* data, uint8_t n) {
    return d && d->bus ? d->bus->write(d->id, address, data, n) : -1;
}
int write_byte(Device* d, uint8_t address, uint8_t value) {
    return write_bytes(d, address, &value, 1);
}
int write_word(Device* d, uint8_t address, uint16_t value) {
    uint8_t b[2] = {static_cast<uint8_t>(value), static_cast<uint8_t>(value >> 8)};
    return write_bytes(d, address, b, 2);
}
int verify_byte(Device* d, uint8_t address, uint8_t value) {
    uint8_t got = 0;
    return read_bytes(d, address, &got, 1) == 0 && got == value ? 0 : -1;
}
int verify_word(Device* d, uint8_t address, uint16_t value) {
    uint8_t got[2]{};
    return read_bytes(d, address, got, 2) == 0 &&
        got[0] == static_cast<uint8_t>(value) && got[1] == static_cast<uint8_t>(value >> 8) ? 0 : -1;
}
int torque(Device* d, bool enabled) {
    const uint8_t val = enabled ? 1 : 0;
    return write_byte(d, kTorque, val) == 0 && verify_byte(d, kTorque, val) == 0 ? 0 : -1;
}
int write_target(Device* d, uint16_t pos) {
    /* HD1910 native payload: goal position, goal current=0, goal speed=0.
     * Address 41 (acceleration) is intentionally left untouched. */
    uint8_t row[6] = {static_cast<uint8_t>(pos), static_cast<uint8_t>(pos >> 8), 0, 0, 0, 0};
    return d->bus->sync_write(d->id, kGoalPosition, row, sizeof(row));
}
int position_raw(Device* d, float rad, uint16_t* out) {
    if (!std::isfinite(rad) || !out) return -1;
    const double ticks = static_cast<double>(rad) * 4096.0 / kTau * d->direction + d->zero_offset;
    if (ticks < 0.0 || ticks > 4095.0) return -1;
    *out = static_cast<uint16_t>(std::lround(ticks));
    return 0;
}
int disarm(Device* d);

int arm(Device* d) {
    if (d->armed) return 0;
    if (d->restore_p) return -1;  // finish the prior rollback through IDLE first
    uint8_t torque_state = 0, p = 0;
    if (read_bytes(d, kTorque, &torque_state, 1) || read_bytes(d, kP, &p, 1)) return -1;
    const bool was_off = torque_state == 0;
    if (torque_state != 0 && torque_state != 1) return -1;
    if (!was_off && p != d->firmware_p) {
        std::cerr << "[HD1910] torque is on and P gain differs from deployment contract\n";
        return -1;
    }
    auto fail = [d, was_off]() {
        if (was_off && d->restore_p && disarm(d)) {
            std::cerr << "[HD1910] ID " << unsigned(d->id) << " rollback failed; retry IDLE\n";
        }
        return -1;
    };
    if (was_off) {
        d->saved_p = p;
        d->restore_p = true;
        if (write_byte(d, kP, d->firmware_p) || verify_byte(d, kP, d->firmware_p)) return fail();
    }
    if (write_byte(d, kD, 0) || verify_byte(d, kD, 0) ||
        write_word(d, kTorqueLimit, kLimit) || verify_word(d, kTorqueLimit, kLimit)) return fail();
    if (was_off) {
        uint8_t pos[2]{};
        if (read_bytes(d, kFeedback, pos, 2)) return fail();
        const uint16_t current = static_cast<uint16_t>(pos[0] | (pos[1] << 8));
        if (write_target(d, current) || verify_word(d, kGoalPosition, current) || torque(d, true)) return fail();
    }
    d->armed = true;
    return 0;
}
int disarm(Device* d) {
    if ((d->armed || d->restore_p) && torque(d, false)) return -1;
    d->armed = false;
    if (d->restore_p) {
        if (write_byte(d, kP, d->saved_p) || verify_byte(d, kP, d->saved_p)) return -1;
        d->restore_p = false;
    }
    return 0;
}
int init(struct motor_dev* dev) {
    if (!dev || !dev->priv_data) return -1;
    auto* d = static_cast<Device*>(dev->priv_data);
    std::lock_guard<std::mutex> lock(bus_mutex);
    uint8_t ping = 0;
    return read_bytes(d, kTorque, &ping, 1);
}
int set_cmd(struct motor_dev* dev, const struct motor_cmd* cmd) {
    if (!dev || !dev->priv_data || !cmd) return -1;
    auto* d = static_cast<Device*>(dev->priv_data);
    std::lock_guard<std::mutex> lock(bus_mutex);
    if (cmd->mode == MOTOR_MODE_IDLE) return disarm(d);
    if (cmd->mode != MOTOR_MODE_POS) return -1;
    uint16_t pos = 0;
    if (position_raw(d, cmd->pos_des, &pos)) return -1;
    if (arm(d)) return -1;
    return write_target(d, pos);
}
void decode_state(Device* d, const uint8_t* raw, struct motor_state* state) {
    const uint16_t posword = static_cast<uint16_t>(raw[0] | (raw[1] << 8));
    int32_t ticks = posword & 0x7fff;
    if (posword & 0x8000) ticks = -ticks;
    int32_t delta = ticks - d->zero_offset;
    while (delta > 2048) delta -= 4096;
    while (delta < -2048) delta += 4096;
    state->pos = static_cast<float>(delta) * kTau / 4096.0f * d->direction;
    uint16_t speedword = static_cast<uint16_t>(raw[2] | (raw[3] << 8));
    int32_t speed = speedword & 0x7fff;
    if (speedword & 0x8000) speed = -speed;
    state->vel = static_cast<float>(speed) * 50.0f * kTau / 4096.0f * d->direction;
    const uint16_t loadword = static_cast<uint16_t>(raw[4] | (raw[5] << 8));
    int32_t load = loadword & 0x03ff;
    if (loadword & 0x0400) load = -load;
    state->trq = static_cast<float>(load);
    state->temp = raw[7];
    state->err = 0;  // A nonzero status byte is rejected by Hd1910Bus::exchange.
}
int get_state(struct motor_dev* dev, struct motor_state* state) {
    if (!dev || !dev->priv_data || !state) return -1;
    auto* d = static_cast<Device*>(dev->priv_data);
    std::lock_guard<std::mutex> lock(bus_mutex);
    uint8_t raw[kFeedbackLength]{};
    if (read_bytes(d, kFeedback, raw, kFeedbackLength)) return -1;
    decode_state(d, raw, state);
    return 0;
}
void release(struct motor_dev* dev) {
    if (!dev) return;
    auto* d = static_cast<Device*>(dev->priv_data);
    if (d) {
        std::lock_guard<std::mutex> lock(bus_mutex);
        if ((d->armed || d->restore_p) && disarm(d)) {
            std::cerr << "[HD1910] ID " << unsigned(d->id)
                << " release failed to confirm torque off and restore P\n";
        }
        devices.erase(d->id);
        delete d;
    }
    free(dev);
}
int get_paras(struct motor_dev* dev, const void* address, void* out, uint32_t n) {
    if (!dev || !dev->priv_data || !address || !out || !n || n > 255) return -1;
    auto* d = static_cast<Device*>(dev->priv_data);
    std::lock_guard<std::mutex> lock(bus_mutex);
    return read_bytes(d, *static_cast<const uint8_t*>(address), static_cast<uint8_t*>(out), static_cast<uint8_t>(n));
}
int set_paras(struct motor_dev* dev, const void* address, const void* data, uint32_t n) {
    if (!dev || !dev->priv_data || !address || !data || !n || n > 255) return -1;
    auto* d = static_cast<Device*>(dev->priv_data);
    std::lock_guard<std::mutex> lock(bus_mutex);
    return write_bytes(d, *static_cast<const uint8_t*>(address), static_cast<const uint8_t*>(data), static_cast<uint8_t>(n));
}
const struct motor_ops ops = {init, set_cmd, get_state, release, set_paras, get_paras, nullptr, nullptr, nullptr};
struct motor_dev* factory(void* args) {
    if (!args) return nullptr;
    auto* ua = static_cast<motor_args_uart*>(args);
    if (!ua->dev_path || ua->baud != 1000000 || ua->id >= 0xfe) return nullptr;
    auto* dev = static_cast<motor_dev*>(std::calloc(1, sizeof(motor_dev)));
    auto* d = new Device();
    if (!dev || !d) { std::free(dev); delete d; return nullptr; }
    d->id = ua->id;
    if (ua->args) {
        auto* cfg = static_cast<const feetech_hd1910_config*>(ua->args);
        d->zero_offset = cfg->zero_offset_ticks;
        d->direction = cfg->direction;
        d->firmware_p = cfg->firmware_p ? cfg->firmware_p : 5;
    }
    if (d->zero_offset < 0 || d->zero_offset > 4095 || (d->direction != 1 && d->direction != -1)) {
        std::free(dev); delete d; return nullptr;
    }
    std::lock_guard<std::mutex> lock(bus_mutex);
    d->bus = get_bus(ua->dev_path);
    if (!d->bus || devices.count(d->id)) { std::free(dev); delete d; return nullptr; }
    devices[d->id] = d;
    dev->name = "drv_uart_feetech_hd1910";
    dev->ops = &ops;
    dev->priv_data = d;
    return dev;
}
} // namespace

extern "C" int feetech_hd1910_set_cmds_sync(struct motor_dev **devs,
    const struct motor_cmd *cmds, uint32_t count) {
    if (!devs || !cmds || !count || count > 35) return -1;
    std::lock_guard<std::mutex> lock(bus_mutex);
    Hd1910Bus* bus = nullptr;
    std::vector<uint8_t> rows;
    std::vector<Device*> batch;
    for (uint32_t i = 0; i < count; ++i) {
        if (!devs[i] || devs[i]->ops != &ops || !devs[i]->priv_data || cmds[i].mode != MOTOR_MODE_POS) return -1;
        auto* d = static_cast<Device*>(devs[i]->priv_data);
        if (bus && bus != d->bus) return -1;
        for (auto* prior : batch) if (prior->id == d->id) return -1;
        bus = d->bus;
        uint16_t pos;
        if (position_raw(d, cmds[i].pos_des, &pos)) return -1;
        batch.push_back(d);
        rows.insert(rows.end(), {d->id, static_cast<uint8_t>(pos), static_cast<uint8_t>(pos >> 8), 0, 0, 0, 0});
    }
    std::vector<Device*> newly_armed;
    for (auto* d : batch) {
        bool was_armed = d->armed;
        if (arm(d)) {
            for (auto* prior : newly_armed) {
                if (disarm(prior)) {
                    std::cerr << "[HD1910] ID " << unsigned(prior->id) << " batch rollback failed; retry IDLE\n";
                }
            }
            return -1;
        }
        if (!was_armed && d->restore_p) newly_armed.push_back(d);
    }
    return bus->sync_targets(rows);
}

extern "C" int feetech_hd1910_get_states_sync(struct motor_dev **devs,
    struct motor_state *states, uint32_t count) {
    if (!devs || !states || !count || count > 251) return -1;
    std::lock_guard<std::mutex> lock(bus_mutex);
    Hd1910Bus* bus = nullptr;
    std::vector<uint8_t> ids;
    for (uint32_t i = 0; i < count; ++i) {
        if (!devs[i] || devs[i]->ops != &ops || !devs[i]->priv_data) return -1;
        auto* d = static_cast<Device*>(devs[i]->priv_data);
        if (bus && bus != d->bus) return -1;
        for (auto id : ids) if (id == d->id) return -1;
        bus = d->bus;
        ids.push_back(d->id);
    }
    std::vector<std::vector<uint8_t>> results;
    if (bus->sync_feedback(ids, results)) return -1;
    for (uint32_t i = 0; i < count; ++i) {
        decode_state(static_cast<Device*>(devs[i]->priv_data), results[i].data(), &states[i]);
        devs[i]->feedback_timestamp_us = 0;
    }
    return 0;
}
REGISTER_MOTOR_DRIVER("drv_uart_feetech_hd1910", DRV_TYPE_UART, factory)
