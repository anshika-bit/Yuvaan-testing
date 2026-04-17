from mpu9250_jmdev.mpu_9250 import MPU9250
import time

# Initialize MPU9250
# address_mpu_master=0x68 matches your i2cdetect result
mpu = MPU9250(
    address_ak=0x0C, 
    address_mpu_master=0x68, 
    bus=1, 
    gfs=1000, 
    afs=8, 
    mfs=1
)
mpu.configure()

print("--- MPU9250 Live Test ---")
print("Rotate the rover to see Yaw (Z-axis) changes.")

try:
    while True:
        # Read Gyroscope (Degrees per second)
        gyro = mpu.readGyroscopeMaster()
        # Read Accelerometer (G-force)
        accel = mpu.readAccelerometerMaster()
        
        # We focus on gyro[2] which is the Z-axis (Yaw/Turning)
        print(f"Z-Rotation: {gyro[2]:.2f} deg/s | X-Accel: {accel[0]:.2f}", end="\r")
        
        time.sleep(0.1)
except KeyboardInterrupt:
    print("\nTest Stopped.")
