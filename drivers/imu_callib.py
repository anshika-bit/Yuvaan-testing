import smbus
import time

# MPU-9250/6500 Direct Register Access
BUS = smbus.SMBus(1)
ADDR = 0x68
GYRO_Z_REG = 0x47

def read_raw_z():
    try:
        high = BUS.read_byte_data(ADDR, GYRO_Z_REG)
        low = BUS.read_byte_data(ADDR, GYRO_Z_REG + 1)
        val = (high << 8) | low
        return val - 65536 if val > 32768 else val
    except:
        return None

# Wake up sensor
BUS.write_byte_data(ADDR, 0x6B, 0)

print("--- PROJECT YUVAAN: 90° TURN CALIBRATION ---")
print("1. Keep the rover perfectly still for calibration.")

# Step 1: Find the Bias (Drift)
bias = 0
samples = 200
for i in range(samples):
    val = read_raw_z()
    if val is not None:
        bias += val
    time.sleep(0.01)
bias /= samples

print(f"Calibration Complete! Bias (Drift Offset): {bias:.2f}")
print("2. Now, manually turn the rover 90 degrees.")
print("--- Press CTRL+C when finished to see the final error ---")

heading = 0.0
last_time = time.time()

try:
    while True:
        curr_time = time.time()
        dt = curr_time - last_time
        
        raw_z = read_raw_z()
        if raw_z is not None:
            # Scale factor: 131.0 for +/- 250 deg/s range
            actual_z = (raw_z - bias) / 131.0
            
            # Noise Gate: Ignore very tiny movements
            if abs(actual_z) > 0.15:
                heading += actual_z * dt
            
            print(f"Current Heading: {heading:7.2f}° | Speed: {actual_z:6.2f} d/s", end="\r")
        
        last_time = curr_time
        time.sleep(0.02) # 50Hz Sampling

except KeyboardInterrupt:
    print(f"\n\nFINAL RESULT: {heading:.2f}°")
    error = abs(90.0 - abs(heading))
    print(f"Error from 90°: {error:.2f}°")