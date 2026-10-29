#!/usr/bin/env python3
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""Offline HD1910 rollback tests using a simulated FT-SCS serial device."""

import ctypes
import os
import pty
import select
import sys
import threading
import unittest

LIBRARY_PATH = sys.argv.pop(1)
LIB = ctypes.CDLL(LIBRARY_PATH)
LIB.motor_alloc_uart.argtypes = [ctypes.c_char_p, ctypes.c_char_p, ctypes.c_uint32,
                                 ctypes.c_uint8, ctypes.c_void_p]
LIB.motor_alloc_uart.restype = ctypes.c_void_p
LIB.motor_set_cmds.argtypes = [ctypes.POINTER(ctypes.c_void_p), ctypes.c_void_p, ctypes.c_uint32]
LIB.motor_set_cmds.restype = ctypes.c_int
LIB.motor_free.argtypes = [ctypes.POINTER(ctypes.c_void_p), ctypes.c_uint32]


class Command(ctypes.Structure):
    _fields_ = [("mode", ctypes.c_uint32), ("pos_des", ctypes.c_float),
                ("vel_des", ctypes.c_float), ("trq_des", ctypes.c_float),
                ("kp", ctypes.c_float), ("kd", ctypes.c_float)]


class Servo:
    def __init__(self):
        self.master, self.slave = pty.openpty()
        self.path = os.ttyname(self.slave).encode()
        self.reg = bytearray(256)
        self.reg[50] = 9
        self.reg[56:58] = b"\x00\x08"
        self.regs = {31: self.reg, 32: bytearray(self.reg)}
        self.writes = []
        self.drop_enable_id = 31
        self.drop_torque_read = False
        self.torque_read_pending = False
        self.block_off = False
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self.serve, daemon=True)
        self.thread.start()

    def serve(self):
        data = bytearray()
        while not self.stop.is_set():
            if not select.select([self.master], [], [], 0.05)[0]:
                continue
            try:
                data.extend(os.read(self.master, 512))
            except OSError:
                break
            while len(data) >= 4:
                if data[:2] != b"\xff\xff":
                    del data[0]
                    continue
                length = data[3] + 4
                if len(data) < length:
                    break
                frame = bytes(data[:length])
                del data[:length]
                ident, instruction = frame[2], frame[4]
                params = frame[5:-1]
                if ident == 0xfe:
                    if instruction == 0x83 and len(params) >= 3:
                        address, width, ident = params[:3]
                        self.regs[ident][address:address + width] = params[3:3 + width]
                    continue
                reg = self.regs[ident]
                if instruction == 2:
                    address, size = params
                    if address == 40 and self.torque_read_pending:
                        self.torque_read_pending = False
                        continue
                    payload = bytes(reg[address:address + size])
                elif instruction == 3:
                    address, value = params[0], params[1:]
                    self.writes.append((address, bytes(value)))
                    if address == 40 and value == b"\x00" and self.block_off:
                        continue
                    reg[address:address + len(value)] = value
                    if address == 40 and value == b"\x01" and self.drop_torque_read:
                        self.torque_read_pending = True
                    if address == 40 and value == b"\x01" and self.drop_enable_id == ident:
                        self.drop_enable_id = None
                        continue
                    payload = b""
                else:
                    continue
                reply = bytearray([0xff, 0xff, ident, len(payload) + 2, 0])
                reply.extend(payload)
                reply.append((~sum(reply[2:])) & 0xff)
                os.write(self.master, reply)

    def close(self):
        self.stop.set()
        self.thread.join(timeout=1)
        os.close(self.master)
        os.close(self.slave)


class RollbackTest(unittest.TestCase):
    def setUp(self):
        self.servo = Servo()
        motor = LIB.motor_alloc_uart(b"drv_uart_feetech_hd1910", self.servo.path, 1000000, 31, None)
        self.assertTrue(motor)
        self.motors = (ctypes.c_void_p * 1)(motor)

    def tearDown(self):
        if self.motors[0]:
            LIB.motor_free(self.motors, 1)
        self.servo.close()

    def command(self, mode):
        command = Command(mode=mode)
        return LIB.motor_set_cmds(self.motors, ctypes.byref(command), 1)

    def test_enable_ack_lost_rolls_back_before_return(self):
        self.assertNotEqual(self.command(1), 0)
        self.assertEqual(self.servo.reg[40], 0)
        self.assertEqual(self.servo.reg[50], 9)
        self.assertLess(self.servo.writes.index((40, b"\x00")),
                        self.servo.writes.index((50, b"\x09")))

    def test_enable_readback_timeout_rolls_back(self):
        self.servo.drop_enable_id = None
        self.servo.drop_torque_read = True
        self.assertNotEqual(self.command(1), 0)
        self.assertEqual(self.servo.reg[40], 0)
        self.assertEqual(self.servo.reg[50], 9)

    def test_original_zero_p_is_restored(self):
        self.servo.reg[50] = 0
        self.assertNotEqual(self.command(1), 0)
        self.assertEqual(self.servo.reg[40], 0)
        self.assertEqual(self.servo.reg[50], 0)
        self.assertIn((50, b"\x00"), self.servo.writes)

    def test_release_retries_failed_rollback(self):
        self.servo.block_off = True
        self.assertNotEqual(self.command(1), 0)
        self.assertEqual(self.servo.reg[40], 1)
        self.assertEqual(self.servo.reg[50], 5)
        self.assertNotIn((50, b"\x09"), self.servo.writes)
        self.servo.block_off = False
        LIB.motor_free(self.motors, 1)
        self.motors[0] = None
        self.assertEqual(self.servo.reg[40], 0)
        self.assertEqual(self.servo.reg[50], 9)

    def test_batch_failure_rolls_back_earlier_motor(self):
        self.servo.drop_enable_id = 32
        second = LIB.motor_alloc_uart(b"drv_uart_feetech_hd1910", self.servo.path, 1000000, 32, None)
        self.assertTrue(second)
        motors = (ctypes.c_void_p * 2)(self.motors[0], second)
        commands = (Command * 2)(Command(mode=1), Command(mode=1))
        LIB.feetech_hd1910_set_cmds_sync.argtypes = [ctypes.POINTER(ctypes.c_void_p),
                                                     ctypes.POINTER(Command), ctypes.c_uint32]
        try:
            self.assertNotEqual(LIB.feetech_hd1910_set_cmds_sync(motors, commands, 2), 0)
            for reg in self.servo.regs.values():
                self.assertEqual(reg[40], 0)
                self.assertEqual(reg[50], 9)
        finally:
            LIB.motor_free(motors, 2)
            self.motors[0] = None

    def test_idle_retries_failed_rollback(self):
        self.servo.block_off = True
        self.assertNotEqual(self.command(1), 0)
        self.assertNotEqual(self.command(0), 0)
        self.assertEqual(self.servo.reg[50], 5)
        self.servo.block_off = False
        self.assertEqual(self.command(0), 0)
        self.assertEqual(self.servo.reg[40], 0)
        self.assertEqual(self.servo.reg[50], 9)


if __name__ == "__main__":
    unittest.main()
