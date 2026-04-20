import smbus
import time
import math

# MPU6500/9250 Registers
MPU_ADDR = 0x68
PWR_MGMT_1 = 0x6b
GYRO_ZOUT_H = 0x47

bus = smbus.SMBus(1)

def init_mpu():
    # Wake up the sensor
    bus.write_byte_data(MPU_ADDR, PWR_MGMT_1, 0)
    print("MPU Sensor Initialized.")

def read_raw_data(addr):
    # Accel and Gyro data are 16-bit (two 8-bit registers)
    high = bus.read_byte_data(MPU_ADDR, addr)
    low = bus.read_byte_data(MPU_ADDR, addr+1)
    # Combine high and low for signed 16-bit value
    value = ((high << 8) | low)
    if value > 32768:
        value = value - 65536
    return value

init_mpu()

print("--- Project Yuvaan: Direct Gyro Test ---")
print("Calibrating... DO NOT MOVE ROVER")

# Calibration: Average 100 readings to find drift
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

        # Read Z-axis Gyroscope
        # FS_SEL 0 (default) = 131 LSB per deg/s
        raw_z = read_raw_data(GYRO_ZOUT_H) - bias
        gyro_z = raw_z / 131.0

        # Integration: Change in Angle = Velocity * Time
        if abs(gyro_z) > 0.1: # Noise gate
            heading += gyro_z * dt

        print(f"Current Heading: {heading:7.2f}° | Z-Velocity: {gyro_z:6.2f} deg/s", end="\r")
        time.sleep(0.02)

except Exception as e:
    print(f"\n[ERROR] Bus Error: {e}")
except KeyboardInterrupt:
    print(f"\nFinal Heading: {heading:.2f}°")