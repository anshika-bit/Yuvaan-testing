"""
YUVAAN IMU Driver — imu_sensor.py
Hardware: MPU9250 / MPU6500 @ I2C 0x68 on Bus 1

This is the core reusable driver. Import it from other scripts instead of
duplicating smbus calls everywhere.

Usage:
    from drivers.imu_sensor import IMUSensor
    imu = IMUSensor()
    data = imu.read()
    print(data)  # {'pitch': X, 'roll': Y, 'yaw': Z}
"""

import smbus2 as smbus
import time
import math


# ---- MPU Register Map ----
MPU_ADDR     = 0x68
PWR_MGMT_1   = 0x6B
ACCEL_XOUT_H = 0x3B  # AX high byte (AY=0x3D, AZ=0x3F)
GYRO_XOUT_H  = 0x43  # GX high byte (GY=0x45, GZ=0x47)
GYRO_CONFIG  = 0x1B
ACCEL_CONFIG = 0x1C

# Full-scale selection
# Gyro:  0→±250°/s, 1→±500°/s, 2→±1000°/s, 3→±2000°/s
# Accel: 0→±2g,     1→±4g,     2→±8g,       3→±16g
GYRO_FS_SEL  = 0   # 250°/s = 131.0 LSB/(°/s)  ← most sensitive
ACCEL_FS_SEL = 0   # ±2g    = 16384.0 LSB/g     ← most sensitive

GYRO_SCALE  = 131.0   # LSB per °/s at FS_SEL=0
ACCEL_SCALE = 16384.0 # LSB per g  at AFS_SEL=0

ALPHA = 0.98   # Complementary filter coefficient


class IMUSensor:
    """Low-level MPU9250/MPU6500 driver using direct I2C register reads."""

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
        """Wake the sensor and set full-scale ranges."""
        # Exit sleep mode
        self.bus.write_byte_data(self.addr, PWR_MGMT_1, 0x00)
        time.sleep(0.1)
        # Set gyro full-scale
        self.bus.write_byte_data(self.addr, GYRO_CONFIG, GYRO_FS_SEL << 3)
        # Set accel full-scale
        self.bus.write_byte_data(self.addr, ACCEL_CONFIG, ACCEL_FS_SEL << 3)
        time.sleep(0.05)

    def _read_raw(self, reg: int) -> int:
        """Read a signed 16-bit value from two consecutive registers."""
        high = self.bus.read_byte_data(self.addr, reg)
        low  = self.bus.read_byte_data(self.addr, reg + 1)
        val  = (high << 8) | low
        return val - 65536 if val > 32768 else val

    def calibrate(self, samples: int = 150) -> dict:
        """
        Compute gyro bias by averaging stationary readings.
        Returns bias dict for logging/storage.
        """
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
        print(f"[IMU] Bias → Gx:{self._gx_bias:.3f}  Gy:{self._gy_bias:.3f}  Gz:{self._gz_bias:.3f}")
        return {"gx": self._gx_bias, "gy": self._gy_bias, "gz": self._gz_bias}

    def read_raw_all(self) -> dict:
        """Read raw scaled accel and gyro values."""
        ax = self._read_raw(ACCEL_XOUT_H)     / ACCEL_SCALE
        ay = self._read_raw(ACCEL_XOUT_H + 2) / ACCEL_SCALE
        az = self._read_raw(ACCEL_XOUT_H + 4) / ACCEL_SCALE
        gx = self._read_raw(GYRO_XOUT_H)      / GYRO_SCALE
        gy = self._read_raw(GYRO_XOUT_H + 2)  / GYRO_SCALE
        gz = self._read_raw(GYRO_XOUT_H + 4)  / GYRO_SCALE
        return {"ax": ax, "ay": ay, "az": az, "gx": gx, "gy": gy, "gz": gz}

    def update(self) -> dict:
        """
        Run one complementary-filter cycle and return Pitch/Roll/Yaw.
        Call this in a tight loop.
        """
        now = time.time()
        dt  = now - self._last_time
        self._last_time = now

        # Clamp dt to avoid integration blowup after a stall
        dt = min(dt, 0.5)

        raw = self.read_raw_all()
        ax, ay, az = raw["ax"], raw["ay"], raw["az"]
        gx = raw["gx"] - self._gx_bias
        gy = raw["gy"] - self._gy_bias
        gz = raw["gz"] - self._gz_bias

        # Accelerometer absolute angles
        accel_roll  = math.degrees(math.atan2(ay, az))
        accel_pitch = math.degrees(math.atan2(-ax, math.sqrt(ay**2 + az**2)))

        # Complementary filter
        self._roll  = ALPHA * (self._roll  + gx * dt) + (1 - ALPHA) * accel_roll
        self._pitch = ALPHA * (self._pitch + gy * dt) + (1 - ALPHA) * accel_pitch

        # Yaw gyro integration (noise-gated)
        if abs(gz) > 0.25:
            self._yaw += gz * dt
        self._yaw = self._yaw % 360.0

        return {
            "pitch": round(self._pitch, 2),
            "roll":  round(self._roll,  2),
            "yaw":   round(self._yaw,   2),
        }
