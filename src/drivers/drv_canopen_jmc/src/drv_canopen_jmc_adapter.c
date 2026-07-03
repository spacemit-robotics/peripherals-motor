/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>
#include <motor.h>
#include <motor_core.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sched.h>
#include <string.h>
#include <unistd.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#define MAX_ADAPTER_MOTORS 16

/* Motor mechanics parameters */
#define PULSES_PER_REV 10000.0f
#define RAD_PER_REV (2.0f * (float)M_PI)
#define PULSE_TO_RAD (RAD_PER_REV / PULSES_PER_REV)
#define RAD_TO_PULSE (PULSES_PER_REV / RAD_PER_REV)

typedef struct {
    uint32_t cycle_ms;
} motor_config_canopen_jmc_t;

typedef struct {
    int id;
    int socket;
    struct motor_state state;
    struct motor_cmd cmd;
    int pdo_ready;
    uint32_t profile_vel;
    uint32_t profile_acc;
    uint32_t profile_dec;
    int32_t start_pos;
    int32_t app_start_pos;
    int start_pos_set;
    uint16_t status_word;
    int operation_enabled;
    int pp_state;
    int current_mode;
    int32_t last_sdo_vel;
    int homing_state;
    int sdo_resp_ready;
    uint8_t last_sdo_resp[8];
} motor_ctx_t;

typedef struct {
    uint32_t index;
    uint32_t subindex;
    uint32_t size;
} sdo_addr_t;

static motor_ctx_t g_motors[MAX_ADAPTER_MOTORS];
static int g_configured_motors = 0;
static pthread_mutex_t g_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_t g_thread;
static volatile int g_running = 0;
static int g_sync_socket = -1;
static uint32_t g_requested_cycle_ms = 10;
static const char* g_iface = "can0";

static void send_can_frame(int sock, uint32_t can_id, const uint8_t* data, uint8_t len) {
    struct can_frame frame;
    memset(&frame, 0, sizeof(frame));
    frame.can_id = can_id;
    frame.can_dlc = len;
    if (len > 0 && data) {
        memcpy(frame.data, data, len);
    }
    write(sock, &frame, sizeof(frame));
}

static int sdo_write(int sock, int node_id, uint16_t index, uint8_t subindex, uint32_t data, uint8_t size) {
    uint8_t cmd = 0;
    if (size == 1) cmd = 0x2F;
    else if (size == 2) cmd = 0x2B;
    else if (size == 4) cmd = 0x23;
    else return -1;

    uint8_t payload[8] = {
        cmd,
        (uint8_t)(index & 0xFF), (uint8_t)(index >> 8),
        subindex,
        (uint8_t)(data & 0xFF), (uint8_t)((data >> 8) & 0xFF),
        (uint8_t)((data >> 16) & 0xFF), (uint8_t)((data >> 24) & 0xFF)
    };
    send_can_frame(sock, 0x600 + node_id, payload, 8);
    usleep(5000);
    return 0;
}

static int sdo_write_sync(int sock, int node_id, int motor_idx, uint16_t index, uint8_t subindex, uint32_t data, uint8_t size) {
    uint8_t cmd = 0;
    if (size == 1) cmd = 0x2F;
    else if (size == 2) cmd = 0x2B;
    else if (size == 4) cmd = 0x23;
    else return -1;

    uint8_t payload[8] = {
        cmd,
        (uint8_t)(index & 0xFF), (uint8_t)(index >> 8),
        subindex,
        (uint8_t)(data & 0xFF), (uint8_t)((data >> 8) & 0xFF),
        (uint8_t)((data >> 16) & 0xFF), (uint8_t)((data >> 24) & 0xFF)
    };
    g_motors[motor_idx].sdo_resp_ready = 0;
    send_can_frame(sock, 0x600 + node_id, payload, 8);

    int timeout = 50;
    while (timeout > 0 && g_running) {
        if (g_motors[motor_idx].sdo_resp_ready) {
            uint8_t* resp = g_motors[motor_idx].last_sdo_resp;
            if ((resp[0] & 0xE0) == 0x60) {
                return 0; // Success
            } else if (resp[0] == 0x80) {
                fprintf(stderr, "SDO Write Aborted. Index: %04X, Sub: %02X, Code: %02X%02X%02X%02X\n",
                        index, subindex, resp[7], resp[6], resp[5], resp[4]);
                return -1; // Abort
            }
        }
        usleep(1000);
        timeout--;
    }
    return -1; // Timeout
}

static int sdo_write_async(int sock, int node_id, uint16_t index, uint8_t subindex, uint32_t data, uint8_t size) {
    uint8_t cmd = 0;
    if (size == 1) cmd = 0x2F;
    else if (size == 2) cmd = 0x2B;
    else if (size == 4) cmd = 0x23;
    else return -1;

    uint8_t payload[8] = {
        cmd,
        (uint8_t)(index & 0xFF), (uint8_t)(index >> 8),
        subindex,
        (uint8_t)(data & 0xFF), (uint8_t)((data >> 8) & 0xFF),
        (uint8_t)((data >> 16) & 0xFF), (uint8_t)((data >> 24) & 0xFF)
    };
    send_can_frame(sock, 0x600 + node_id, payload, 8);
    return 0;
}

static int sdo_read(int sock, int node_id, int motor_idx, uint16_t index, uint8_t subindex, uint32_t* out_data, uint8_t* out_size) {
    uint8_t payload[8] = {
        0x40,
        (uint8_t)(index & 0xFF), (uint8_t)(index >> 8),
        subindex,
        0, 0, 0, 0
    };
    g_motors[motor_idx].sdo_resp_ready = 0;
    send_can_frame(sock, 0x600 + node_id, payload, 8);

    int timeout = 50; // 50ms
    while (timeout > 0 && g_running) {
        if (g_motors[motor_idx].sdo_resp_ready) {
            uint8_t* resp = g_motors[motor_idx].last_sdo_resp;
            if ((resp[0] & 0xE0) == 0x40) {
                if (out_data) {
                    *out_data = resp[4] | (resp[5] << 8) | (resp[6] << 16) | (resp[7] << 24);
                }
                if (out_size) {
                    if (resp[0] == 0x4F) *out_size = 1;
                    else if (resp[0] == 0x4B) *out_size = 2;
                    else if (resp[0] == 0x43) *out_size = 4;
                    else *out_size = 4;
                }
                return 0;
            } else if (resp[0] == 0x80) {
                return -1;
            }
        }
        usleep(1000);
        timeout--;
    }
    return -1;
}

static void send_nmt(int sock, uint8_t command, uint8_t node_id) {
    uint8_t data[2] = {command, node_id};
    send_can_frame(sock, 0x000, data, 2);
    usleep(5000);
}

static int init_can_socket(const char* iface) {
    int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock < 0) return -1;

    struct ifreq ifr;
    strcpy(ifr.ifr_name, iface);
    ioctl(sock, SIOCGIFINDEX, &ifr);

    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        close(sock);
        return -1;
    }

    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 1000; // 1ms timeout for non-blocking like read
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(tv));

    return sock;
}

static void* canopen_background_thread(void* arg) {
    // Send NMT start to all nodes
    send_nmt(g_sync_socket, 0x01, 0x00);

    uint64_t cycle_us = g_requested_cycle_ms * 1000;

    while (g_running) {
        pthread_mutex_lock(&g_mutex);

        // Read incoming TPDOs
        for (int i = 0; i < g_configured_motors; i++) {
            struct can_frame frame;
            while (read(g_motors[i].socket, &frame, sizeof(frame)) == sizeof(frame)) {
                if (frame.can_id == (uint32_t)(0x280 + g_motors[i].id)) {
                    if (frame.can_dlc >= 6) {
                        uint16_t status = frame.data[0] | (frame.data[1] << 8);
                        int32_t pos = frame.data[2] | (frame.data[3] << 8) | (frame.data[4] << 16) | (frame.data[5] << 24);
                        g_motors[i].status_word = status;
                        g_motors[i].state.pos = pos * PULSE_TO_RAD;
                        g_motors[i].state.err = status;

                        if ((status & 0x6F) == 0x27) {
                            g_motors[i].operation_enabled = 1;
                        } else {
                            g_motors[i].operation_enabled = 0;
                        }
                    }
                } else if (frame.can_id == (uint32_t)(0x580 + g_motors[i].id)) {
                    if ((frame.data[0] & 0xE0) == 0x40 || (frame.data[0] & 0xE0) == 0x60 || frame.data[0] == 0x80) {
                        memcpy(g_motors[i].last_sdo_resp, frame.data, 8);
                        g_motors[i].sdo_resp_ready = 1;
                    }
                }
            }
        }

        // Send SYNC
        send_can_frame(g_sync_socket, 0x080, NULL, 0);

        // Send RPDOs
        for (int i = 0; i < g_configured_motors; i++) {
            if (1) {
                if (g_motors[i].cmd.mode == MOTOR_MODE_POS || g_motors[i].cmd.mode == MOTOR_MODE_CSP ||
                    g_motors[i].cmd.mode == MOTOR_MODE_VEL || g_motors[i].cmd.mode == MOTOR_MODE_CSV ||
                    g_motors[i].cmd.mode == MOTOR_MODE_HM) {
                    uint16_t ctrl = 0x0006; // Default to Shutdown to push state machine
                    int32_t pos_pulses = (g_motors[i].start_pos_set == 2) ? g_motors[i].start_pos : (int32_t)(g_motors[i].state.pos * RAD_TO_PULSE);

                    if (g_motors[i].status_word & 0x0008) {
                        // Fault state
                        static int fault_count = 0;
                        if (fault_count++ < 10) {
                            ctrl = 0x0000; // Drop edge
                        } else {
                            ctrl = 0x0080; // Rising edge Fault reset
                        }
                        if (fault_count > 20) fault_count = 0;
                        g_motors[i].pp_state = 0;
                    } else if ((g_motors[i].status_word & 0x004F) == 0x0040) {
                        // Switch on disabled
                        ctrl = 0x0006;
                    } else if ((g_motors[i].status_word & 0x006F) == 0x0021) {
                        // Ready to switch on
                        ctrl = 0x0007;
                    } else if ((g_motors[i].status_word & 0x006F) == 0x0023) {
                        // Switched on
                        ctrl = 0x000F;
                    } else if ((g_motors[i].status_word & 0x006F) == 0x0027) {
                        // Operation enabled
                        ctrl = 0x000F;
                        int id = g_motors[i].id;
                        int sock = g_motors[i].socket;

                        if (g_motors[i].cmd.mode == MOTOR_MODE_POS || g_motors[i].cmd.mode == MOTOR_MODE_CSP) {
                            if (g_motors[i].current_mode != 1) {
                                sdo_write_async(sock, id, 0x6060, 0, 0x01, 1);
                                g_motors[i].current_mode = 1;
                            }
                            if (g_motors[i].start_pos_set == 2) {
                                int32_t target = (int32_t)(g_motors[i].cmd.pos_des * RAD_TO_PULSE);
                                pos_pulses = (target - g_motors[i].app_start_pos) + g_motors[i].start_pos;
                            }

                            // PP mode bit toggling for new setpoint
                            if (g_motors[i].pp_state == 0) {
                                ctrl |= 0x003F; // New setpoint
                                g_motors[i].pp_state = 1;
                            } else {
                                ctrl &= ~0x0010; // Clear new setpoint
                                g_motors[i].pp_state = 0;
                            }
                        } else if (g_motors[i].cmd.mode == MOTOR_MODE_VEL || g_motors[i].cmd.mode == MOTOR_MODE_CSV) {
                            if (g_motors[i].current_mode != 3) {
                                sdo_write_async(sock, id, 0x6060, 0, 0x03, 1);
                                sdo_write_async(sock, id, 0x6083, 0, 1000, 2);
                                sdo_write_async(sock, id, 0x6084, 0, 1000, 2);
                                g_motors[i].current_mode = 3;
                            }
                            int32_t vel_des = (int32_t)(g_motors[i].cmd.vel_des / (2.0f * (float)M_PI) * 10.0f);
                            if (vel_des != g_motors[i].last_sdo_vel) {
                                sdo_write_async(sock, id, 0x6081, 0, vel_des, 4);
                                g_motors[i].last_sdo_vel = vel_des;
                            }
                        } else if (g_motors[i].cmd.mode == MOTOR_MODE_HM) {
                            if (g_motors[i].current_mode != 6) {
                                sdo_write_async(sock, id, 0x6060, 0, 0x06, 1);
                                sdo_write_async(sock, id, 0x6098, 0, 0x01, 1);
                                sdo_write_async(sock, id, 0x609A, 0, 1000, 2);
                                sdo_write_async(sock, id, 0x6099, 1, 10, 4);
                                sdo_write_async(sock, id, 0x6099, 2, 5, 4);
                                g_motors[i].current_mode = 6;
                                g_motors[i].homing_state = 0;
                            }
                            if (g_motors[i].homing_state == 0) {
                                ctrl |= 0x0010; // bit 4 start homing
                            }
                        }
                    }

                    uint8_t data[6];
                    data[0] = ctrl & 0xFF;
                    data[1] = (ctrl >> 8) & 0xFF;
                    data[2] = pos_pulses & 0xFF;
                    data[3] = (pos_pulses >> 8) & 0xFF;
                    data[4] = (pos_pulses >> 16) & 0xFF;
                    data[5] = (pos_pulses >> 24) & 0xFF;

                    send_can_frame(g_motors[i].socket, 0x400 + g_motors[i].id, data, 6);
                }
            }
        }

        pthread_mutex_unlock(&g_mutex);
        usleep(cycle_us);
    }
    return NULL;
}

static int adapter_init(struct motor_dev* dev) {
    pthread_mutex_lock(&g_mutex);

    if (!g_running) {
        g_sync_socket = init_can_socket(g_iface);
        if (g_sync_socket < 0) {
            pthread_mutex_unlock(&g_mutex);
            return -1;
        }

        for (int i = 0; i < g_configured_motors; i++) {
            int sock = init_can_socket(g_iface);
            g_motors[i].socket = sock;
            int id = g_motors[i].id;

            // Set RPDO params
            sdo_write(sock, id, 0x1402, 1, 0x400 + id, 4);
            sdo_write(sock, id, 0x1402, 2, 0x01, 1);
            sdo_write(sock, id, 0x1602, 0, 0x00, 1);
            sdo_write(sock, id, 0x1602, 1, 0x60400010, 4);
            sdo_write(sock, id, 0x1602, 2, 0x607A0020, 4);
            sdo_write(sock, id, 0x1602, 0, 0x02, 1);

            // Set TPDO params
            sdo_write(sock, id, 0x1801, 1, 0x280 + id, 4);
            sdo_write(sock, id, 0x1801, 2, 0x01, 1);
            sdo_write(sock, id, 0x1A01, 0, 0x00, 1);
            sdo_write(sock, id, 0x1A01, 1, 0x60410010, 4);
            sdo_write(sock, id, 0x1A01, 2, 0x60640020, 4);
            sdo_write(sock, id, 0x1A01, 0, 0x02, 1);

            // Set Mode
            sdo_write(sock, id, 0x6060, 0, 0x01, 1); // PP mode
            g_motors[i].current_mode = 1;
            g_motors[i].last_sdo_vel = 0;

            // Set Profile params
            sdo_write(sock, id, 0x6081, 0, 10000, 4); // Vel
            sdo_write(sock, id, 0x6083, 0, 2000, 2); // Accel
            sdo_write(sock, id, 0x6084, 0, 2000, 2); // Decel

            g_motors[i].pdo_ready = 1;
        }

        g_running = 1;
        if (pthread_create(&g_thread, NULL, canopen_background_thread, NULL) != 0) {
            g_running = 0;
            pthread_mutex_unlock(&g_mutex);
            return -1;
        }
    }

    pthread_mutex_unlock(&g_mutex);
    return 0;
}

static int adapter_set_cmd(struct motor_dev* dev, const struct motor_cmd* cmd) {
    if (!dev || !cmd || !g_running) return -1;
    int motor_idx = (int)(intptr_t)dev->priv_data;

    pthread_mutex_lock(&g_mutex);

    motor_ctx_t* m = &g_motors[motor_idx];

    if (m->operation_enabled && m->start_pos_set == 0) {
        m->start_pos = (int32_t)(m->state.pos * RAD_TO_PULSE);
        m->start_pos_set = 1;
    }
    if (m->operation_enabled && m->start_pos_set == 1) {
        m->app_start_pos = (int32_t)(cmd->pos_des * RAD_TO_PULSE);
        m->start_pos_set = 2;
    }
    if (!m->operation_enabled) {
        m->start_pos_set = 0;
    }

    m->cmd = *cmd;

    pthread_mutex_unlock(&g_mutex);
    return 0;
}

static int adapter_get_state(struct motor_dev* dev, struct motor_state* state) {
    if (!dev || !state || !g_running) return -1;
    int motor_idx = (int)(intptr_t)dev->priv_data;

    pthread_mutex_lock(&g_mutex);
    *state = g_motors[motor_idx].state;
    pthread_mutex_unlock(&g_mutex);
    return 0;
}

static int adapter_set_paras(struct motor_dev* dev, const void* address, const void* data, uint32_t data_len) {
    if (!dev || !address || !data) return -1;
    int motor_idx = (int)(intptr_t)dev->priv_data;

    const sdo_addr_t* addr = (const sdo_addr_t*)address;

    uint32_t value = 0;
    if (data_len == 1) value = *(uint8_t*)data;
    else if (data_len == 2) value = *(uint16_t*)data;
    else if (data_len == 4) value = *(uint32_t*)data;

    return sdo_write_sync(g_motors[motor_idx].socket, g_motors[motor_idx].id, motor_idx, (uint16_t)addr->index, (uint8_t)addr->subindex, value, (uint8_t)addr->size);
}

static int adapter_get_paras(struct motor_dev* dev, const void* address, void* out_data, uint32_t data_len) {
    if (!dev || !address || !out_data) return -1;
    int motor_idx = (int)(intptr_t)dev->priv_data;

    const sdo_addr_t* addr = (const sdo_addr_t*)address;
    uint32_t val = 0;
    uint8_t sz = 0;

    if (sdo_read(g_motors[motor_idx].socket, g_motors[motor_idx].id, motor_idx, (uint16_t)addr->index, (uint8_t)addr->subindex, &val, &sz) == 0) {
        if (data_len == 1) *(uint8_t*)out_data = (uint8_t)val;
        else if (data_len == 2) *(uint16_t*)out_data = (uint16_t)val;
        else if (data_len == 4) *(uint32_t*)out_data = val;
        return 0;
    }
    return -1;
}

static void adapter_free(struct motor_dev* dev) {
    if (!dev) return;

    if (g_running) {
        g_running = 0;
        pthread_join(g_thread, NULL);

        // Send NMT Pre-operational to all nodes to stop autonomous motion (like Homing)
        send_nmt(g_sync_socket, 0x80, 0x00);

        // Close sockets
        for (int i = 0; i < g_configured_motors; i++) {
            if (g_motors[i].socket > 0) {
                close(g_motors[i].socket);
                g_motors[i].socket = -1;
            }
        }
        if (g_sync_socket > 0) {
            close(g_sync_socket);
            g_sync_socket = -1;
        }
    }

    free(dev);
}

static const struct motor_ops g_adapter_ops = {
    .init = adapter_init,
    .set_cmd = adapter_set_cmd,
    .get_state = adapter_get_state,
    .free = adapter_free,
    .set_paras = adapter_set_paras,
    .get_paras = adapter_get_paras
};

static struct motor_dev* adapter_factory(void* args) {
    if (!args) return NULL;
    struct motor_args_can* can_args = (struct motor_args_can*)args;

    if (g_configured_motors >= MAX_ADAPTER_MOTORS) return NULL;

    if (can_args->iface) {
        g_iface = can_args->iface;
    }

    struct motor_dev* dev = calloc(1, sizeof(struct motor_dev));
    if (!dev) return NULL;

    dev->name = "drv_canopen_jmc";
    dev->ops = &g_adapter_ops;
    dev->priv_data = (void*)(intptr_t)g_configured_motors;

    memset(&g_motors[g_configured_motors], 0, sizeof(motor_ctx_t));
    g_motors[g_configured_motors].id = can_args->can_id;

    g_configured_motors++;
    return dev;
}

__attribute__((used)) struct driver_info __drv_info_adapter_factory_canopen = {
    .name = "drv_canopen_jmc", .type = DRV_TYPE_CAN, .factory = adapter_factory, .next = 0};

__attribute__((used, constructor)) void __auto_reg_adapter_factory_canopen(void) {
    motor_driver_register(&__drv_info_adapter_factory_canopen);
}
