import smbus
import time

# I2C setup
BUS = smbus.SMBus(1)
ADDR = 0x68

def read_raw_16(addr):
    try:
        high = BUS.read_byte_data(ADDR, addr)
        low = BUS.read_byte_data(ADDR, addr + 1)
        val = (high << 8) | low
        return val - 65536 if val > 32768 else val
    except:
        return None

# Wake up MPU
BUS.write_byte_data(ADDR, 0x6B, 0)

print(f"{'--- YUVAAN IMU DIAGNOSTICS ---':^50}")
print(f"{'ACCEL (G)':^25} | {'GYRO (deg/s)':^25}")
print("-" * 55)

try:
    while True:
        ax = read_raw_16(0x3B)
        ay = read_raw_16(0x3D)
        az = read_raw_16(0x3F)
        gx = read_raw_16(0x43)
        gy = read_raw_16(0x45)
        gz = read_raw_16(0x47)

        if None not in [ax, ay, az, gx, gy, gz]:
            print(f"X:{ax/16384.0:+5.2f} Y:{ay/16384.0:+5.2f} Z:{az/16384.0:+5.2f} | "
                  f"X:{gx/131.0:+7.1f} Y:{gy/131.0:+7.1f} Z:{gz/131.0:+7.1f}", end="\r")
        time.sleep(0.1)
except KeyboardInterrupt:
    print("\nCheck Complete.")
