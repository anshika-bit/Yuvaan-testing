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
        self.bus_id = bus
        self.addr = addr
        self.connected = False
        self.bus = None
        
        self._roll  = 0.0
        self._pitch = 0.0
        self._yaw   = 0.0
        self._last_time = time.time()
        self._gx_bias = 0.0
        self._gy_bias = 0.0
        self._gz_bias = 0.0
        
        try:
            self.bus = smbus.SMBus(bus)
            self._init()
            self.connected = True
        except Exception as e:
            print(f"[IMU] Failed to initialize at {hex(addr)}: {e}")
            self.connected = False

    def _init(self):
        if not self.bus: return
        try:
            self.bus.write_byte_data(self.addr, PWR_MGMT_1, 0x00)
            time.sleep(0.1)
            self.bus.write_byte_data(self.addr, GYRO_CONFIG, 0)
            self.bus.write_byte_data(self.addr, ACCEL_CONFIG, 0)
            time.sleep(0.05)
        except Exception as e:
            print(f"[IMU] Init Error: {e}")
            self.connected = False

    def _read_raw(self, reg: int) -> int:
        if not self.bus:
            return 0
        try:
            high = self.bus.read_byte_data(self.addr, reg)
            low  = self.bus.read_byte_data(self.addr, reg + 1)
            val  = (high << 8) | low
            self.connected = True # Successfully read, so it's connected
            return val - 65536 if val > 32768 else val
        except Exception:
            # Don't permanently set connected=False here, just return 0 for this sample
            return 0

    def calibrate(self, samples: int = 150) -> dict:
        # Try to re-init if not connected
        if not self.connected:
            self._init()
            
        if not self.connected:
            print("[IMU] Calibration skipped: Sensor not connected.")
            return {"gx": 0, "gy": 0, "gz": 0}
            
        print(f"[IMU] Calibrating ({samples} samples) — keep rover still...")
        bx = by = bz = 0.0
        success_samples = 0
        
        for _ in range(samples + 50): # Extra samples in case of bus glitches
            val_x = self._read_raw(GYRO_XOUT_H)
            val_y = self._read_raw(GYRO_XOUT_H + 2)
            val_z = self._read_raw(GYRO_XOUT_H + 4)
            
            if val_x != 0 or val_y != 0 or val_z != 0:
                bx += val_x / GYRO_SCALE
                by += val_y / GYRO_SCALE
                bz += val_z / GYRO_SCALE
                success_samples += 1
            
            if success_samples >= samples:
                break
            time.sleep(0.01)
            
        if success_samples > 0:
            self._gx_bias = bx / success_samples
            self._gy_bias = by / success_samples
            self._gz_bias = bz / success_samples
            self.connected = True
            
        return {"gx": self._gx_bias, "gy": self._gy_bias, "gz": self._gz_bias}

    def update(self) -> dict:
        if not self.connected:
            # Try to re-init once
            self._init()
            if not self.connected:
                return {"pitch": 0.0, "roll": 0.0, "yaw": 0.0, "yaw_rate": 0.0, "connected": False}

        now = time.time()
        dt  = min(now - self._last_time, 0.5)
        self._last_time = now

        # Read all data
        try:
            ax_raw = self._read_raw(ACCEL_XOUT_H)
            ay_raw = self._read_raw(ACCEL_XOUT_H + 2)
            az_raw = self._read_raw(ACCEL_XOUT_H + 4)
            
            gx_raw = self._read_raw(GYRO_XOUT_H)
            gy_raw = self._read_raw(GYRO_XOUT_H + 2)
            gz_raw = self._read_raw(GYRO_XOUT_H + 4)

            # Check if we actually got data (all zeros usually means a read fail)
            if all(v == 0 for v in [ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw]):
                # Failed to read, return last known good data
                return {
                    "pitch": round(self._pitch, 2),
                    "roll":  round(self._roll,  2),
                    "yaw":   round(self._yaw,   2),
                    "yaw_rate": 0.0,
                    "connected": False
                }

            ax = ax_raw / ACCEL_SCALE
            ay = ay_raw / ACCEL_SCALE
            az = az_raw / ACCEL_SCALE
            
            gx = (gx_raw / GYRO_SCALE) - self._gx_bias
            gy = (gy_raw / GYRO_SCALE) - self._gy_bias
            gz = (gz_raw / GYRO_SCALE) - self._gz_bias

            accel_roll  = math.degrees(math.atan2(ay, az))
            accel_pitch = math.degrees(math.atan2(-ax, math.sqrt(ay**2 + az**2)))

            # --- Complementary Filter with Shortest Path Logic ---
            def get_shortest_error(target, current):
                err = target - current
                return (err + 180) % 360 - 180

            # Update Roll/Pitch/Yaw
            roll_err = get_shortest_error(accel_roll, self._roll)
            self._roll += (gx * dt) + (1 - ALPHA) * roll_err
            self._roll = (self._roll + 180) % 360 - 180 # Stay in -180..180

            pitch_err = get_shortest_error(accel_pitch, self._pitch)
            self._pitch += (gy * dt) + (1 - ALPHA) * pitch_err
            self._pitch = (self._pitch + 180) % 360 - 180

            if abs(gz) > 0.25:
                # Flip sign to match Compass (Clockwise Positive)
                self._yaw -= gz * dt
            self._yaw = self._yaw % 360.0
            self.connected = True
            
        except Exception:
            # Temporary error, will retry on next update()
            return {
                "pitch": round(self._pitch, 2),
                "roll":  round(self._roll,  2),
                "yaw":   round(self._yaw,   2),
                "yaw_rate": 0.0,
                "connected": False
            }

        return {
            "pitch": round(self._pitch, 2),
            "roll":  round(self._roll,  2),
            "yaw":   round(self._yaw,   2),
            "yaw_rate": round(gz, 3),
            "connected": True
        }
