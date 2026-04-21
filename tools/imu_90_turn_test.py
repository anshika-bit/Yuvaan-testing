"""
YUVAAN — IMU 90° Turn Test Tool

Standalone script to validate the gyro Z-axis integration for turning.
Run on the Pi directly. NOT part of the core driver package.

Usage:
    python tools/imu_90_turn_test.py
"""

import smbus
import time

# MPU6500/9250 Registers
MPU_ADDR = 0x68
PWR_MGMT_1 = 0x6b
GYRO_ZOUT_H = 0x47

bus = smbus.SMBus(1)

def init_mpu():
    bus.write_byte_data(MPU_ADDR, PWR_MGMT_1, 0)
    print("MPU Sensor Initialized.")

def read_raw_data(addr):
    high = bus.read_byte_data(MPU_ADDR, addr)
    low = bus.read_byte_data(MPU_ADDR, addr+1)
    value = ((high << 8) | low)
    if value > 32768:
        value = value - 65536
    return value

init_mpu()

print("--- Project Yuvaan: 90-Degree Turn Test ---")
print("Calibrating... DO NOT MOVE ROVER")

bias = 0
for _ in range(100):
    bias += read_raw_data(GYRO_ZOUT_H)
    time.sleep(0.01)
bias /= 100

print(f"Calibration Done. Bias: {bias:.2f}")

heading = 0.0
last_time = time.time()

try:
    while True:
        curr_time = time.time()
        dt = curr_time - last_time
        last_time = curr_time

        raw_z = read_raw_data(GYRO_ZOUT_H) - bias
        gyro_z = raw_z / 131.0

        if abs(gyro_z) > 0.1:
            heading += gyro_z * dt

        print(f"Heading: {heading:7.2f}° | Z-Vel: {gyro_z:6.2f} deg/s", end="\r")
        time.sleep(0.02)

except KeyboardInterrupt:
    print(f"\nFinal Heading: {heading:.2f}°")
