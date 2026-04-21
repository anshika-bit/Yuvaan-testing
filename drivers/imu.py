"""
YUVAAN IMU Driver — drivers/imu.py (Renamed from imu_sensor.py)
Hardware: MPU9250 / MPU6500 @ I2C 0x68 on Bus 1

Core reusable driver for orientation sensing.
"""

import smbus2 as smbus
import time
import math


# ---- MPU Register Map ----
MPU_ADDR     = 0x68
PWR_MGMT_1   = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H  = 0x43
GYRO_CONFIG  = 0x1B
ACCEL_CONFIG = 0x1C

GYRO_SCALE  = 131.0
ACCEL_SCALE = 16384.0
ALPHA       = 0.98


class IMUSensor:
    def __init__(self, bus: int = 1, addr: int = MPU_ADDR):
        self.bus  = smbus.SMBus(bus)
        self.addr = addr
        self._roll  = 0.0
        self._pitch = 0.0
        self._yaw   = 0.0
        self._last_time = time.time()
        self._gx_bias = 0.0
        self._gy_bias = 0.0
        self._gz_bias = 0.0
        self._init()

    def _init(self):
        self.bus.write_byte_data(self.addr, PWR_MGMT_1, 0x00)
        time.sleep(0.1)
        self.bus.write_byte_data(self.addr, GYRO_CONFIG, 0)
        self.bus.write_byte_data(self.addr, ACCEL_CONFIG, 0)
        time.sleep(0.05)

    def _read_raw(self, reg: int) -> int:
        high = self.bus.read_byte_data(self.addr, reg)
        low  = self.bus.read_byte_data(self.addr, reg + 1)
        val  = (high << 8) | low
        return val - 65536 if val > 32768 else val

    def calibrate(self, samples: int = 150) -> dict:
        print(f"[IMU] Calibrating ({samples} samples) — keep rover still...")
        bx = by = bz = 0.0
        for _ in range(samples):
            bx += self._read_raw(GYRO_XOUT_H)     / GYRO_SCALE
            by += self._read_raw(GYRO_XOUT_H + 2) / GYRO_SCALE
            bz += self._read_raw(GYRO_XOUT_H + 4) / GYRO_SCALE
            time.sleep(0.01)
        self._gx_bias = bx / samples
        self._gy_bias = by / samples
        self._gz_bias = bz / samples
        return {"gx": self._gx_bias, "gy": self._gy_bias, "gz": self._gz_bias}

    def update(self) -> dict:
        now = time.time()
        dt  = min(now - self._last_time, 0.5)
        self._last_time = now

        # Read all data
        ax = self._read_raw(ACCEL_XOUT_H)     / ACCEL_SCALE
        ay = self._read_raw(ACCEL_XOUT_H + 2) / ACCEL_SCALE
        az = self._read_raw(ACCEL_XOUT_H + 4) / ACCEL_SCALE
        gx = (self._read_raw(GYRO_XOUT_H)     / GYRO_SCALE) - self._gx_bias
        gy = (self._read_raw(GYRO_XOUT_H + 2) / GYRO_SCALE) - self._gy_bias
        gz = (self._read_raw(GYRO_XOUT_H + 4) / GYRO_SCALE) - self._gz_bias

        accel_roll  = math.degrees(math.atan2(ay, az))
        accel_pitch = math.degrees(math.atan2(-ax, math.sqrt(ay**2 + az**2)))

        self._roll  = ALPHA * (self._roll  + gx * dt) + (1 - ALPHA) * accel_roll
        self._pitch = ALPHA * (self._pitch + gy * dt) + (1 - ALPHA) * accel_pitch

        if abs(gz) > 0.25:
            self._yaw += gz * dt
        self._yaw = self._yaw % 360.0

        return {
            "pitch": round(self._pitch, 2),
            "roll":  round(self._roll,  2),
            "yaw":   round(self._yaw,   2),
        }
