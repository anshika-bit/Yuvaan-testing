from mpu9250_jmdev.mpu_9250 import MPU9250
import time

# Initialize MPU9250
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
        gyro = mpu.readGyroscopeMaster()
        accel = mpu.readAccelerometerMaster()
        print(f"Z-Rotation: {gyro[2]:.2f} deg/s | X-Accel: {accel[0]:.2f}", end="\r")
        time.sleep(0.1)
except KeyboardInterrupt:
    print("\nTest Stopped.")
