import time
import sys
import os

# Add project root to sys.path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    from drivers.imu_sensor import IMUSensor
except ImportError:
    print("[ERROR] Could not import IMUSensor from drivers. Ensure you are running from project root.")
    sys.exit(1)

def test_sensor():
    print("--- Yuvaan IMU Driver Test ---")
    try:
        imu = IMUSensor(bus=1, addr=0x68)
        print("[OK] MPU9250 Initialized.")
        
        print("\nCalibrating (3 seconds)...")
        imu.calibrate(samples=100)
        
        print("\nReading live data (Ctrl+C to stop)...")
        while True:
            data = imu.update()
            print(f"P: {data['pitch']:+7.2f} | R: {data['roll']:+7.2f} | Y: {data['yaw']:7.2f}", end="\r")
            time.sleep(0.05)
    except Exception as e:
        print(f"\n[FAIL] {e}")

if __name__ == "__main__":
    test_sensor()
