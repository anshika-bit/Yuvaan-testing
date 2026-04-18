import requests
import time
import math
from mpu9250_jmdev.mpu_9250 import MPU9250

# Configuration
# NOTE: 127.0.0.1 works if this script AND telemetry/app.py run on the same Pi.
PI_IP = "127.0.0.1" 
URL = f"http://{PI_IP}:5000/api/imu/update"

print("--- YUVAAN ROVER: REAL IMU FEED INITIATED ---")

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

# Filter Constants
ALPHA = 0.98  # Weight for gyro (higher = smoother, lower = faster response)

def calibrate_sensors():
    print("Calibrating Gyroscope... DO NOT MOVE THE ROVER.")
    samples = 100
    bias_z = 0
    for _ in range(samples):
        gyro = mpu.readGyroscopeMaster()
        bias_z += gyro[2]
        time.sleep(0.01)
    return bias_z / samples

def main():
    gyro_bias_z = calibrate_sensors()
    print(f"Calibration Complete. Z-Bias: {gyro_bias_z:.4f}")
    
    roll = 0.0
    pitch = 0.0
    yaw = 0.0
    last_time = time.time()

    try:
        while True:
            curr_time = time.time()
            dt = curr_time - last_time
            last_time = curr_time
            
            # Read Raw Measurements
            accel = mpu.readAccelerometerMaster()  # [ax, ay, az] in Gs
            gyro = mpu.readGyroscopeMaster()       # [gx, gy, gz] in deg/s
            
            # 1. Calculate Angles from Accelerometer (Raw Tilt)
            # Roll: Tilt around X-axis
            accel_roll = math.degrees(math.atan2(accel[1], accel[2]))
            # Pitch: Tilt around Y-axis
            accel_pitch = math.degrees(math.atan2(-accel[0], math.sqrt(accel[1]**2 + accel[2]**2)))
            
            # 2. Complementary Filter (Merge Gyro and Accel)
            # Roll/Pitch use Accel for long-term stability and Gyro for short-term precision
            roll = ALPHA * (roll + gyro[0] * dt) + (1.0 - ALPHA) * accel_roll
            pitch = ALPHA * (pitch + gyro[1] * dt) + (1.0 - ALPHA) * accel_pitch
            
            # 3. Yaw Integration (Z-Axis Rotation)
            # Yaw cannot be corrected by accel, so it relies on gyro integration (drift is normal)
            gz_corrected = gyro[2] - gyro_bias_z
            if abs(gz_corrected) > 0.3:  # Noise threshold
                yaw += gz_corrected * dt
            
            yaw = yaw % 360 # Keep within 0-360 range
            
            # Prepare Payload
            payload = {
                "pitch": round(pitch, 2),
                "roll": round(roll, 2),
                "yaw": round(yaw, 2)
            }
            
            # Send POST request to backend
            try:
                response = requests.post(URL, json=payload, timeout=0.1)
                print(f"STREAMING: P:{payload['pitch']:6.1f} | R:{payload['roll']:6.1f} | Y:{payload['yaw']:6.1f}", end="\r")
            except Exception as e:
                # Silently wait for backend to come online
                pass
                
            time.sleep(0.05) # Send at 20Hz for high-fidelity GCS visuals
            
    except KeyboardInterrupt:
        print("\nStopping real IMU telemetry stream.")

if __name__ == "__main__":
    main()
