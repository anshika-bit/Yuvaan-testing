"""
YUVAAN ROVER — Real IMU Feed Script
Hardware: MPU9250 @ I2C Address 0x68, Bus 1 (Raspberry Pi 5)

This script reads real Pitch/Roll/Yaw from the MPU9250 using
a Complementary Filter, then streams it to the telemetry backend.

Run:
    python3 test_imu_feed.py

Requirements:
    pip3 install mpu9250-jmdev requests
"""

import requests
import time
import math
import sys
import os

# ---- Clear terminal on start ----
os.system('clear' if os.name == 'posix' else 'cls')

# ---- Try importing hardware library ----
try:
    from mpu9250_jmdev.mpu_9250 import MPU9250
except ImportError:
    print("[FATAL] mpu9250-jmdev not installed. Run: pip3 install mpu9250-jmdev")
    sys.exit(1)

# ---- Configuration ----
PI_IP   = "127.0.0.1"  # Same device (change if running remotely)
URL     = f"http://{PI_IP}:5000/api/imu/update"
ALPHA   = 0.98          # Complementary filter weight (higher = trust gyro more)
HZ      = 20            # Target loop frequency
DT_STEP = 1.0 / HZ

print("=" * 55)
print("  YUVAAN IMU FEED — MPU9250")
print("  Target: " + URL)
print("=" * 55)

# ---- Initialize Hardware ----
try:
    mpu = MPU9250(
        address_ak=0x0C,          # Magnetometer address (on same bus)
        address_mpu_master=0x68,  # Confirmed on i2cdetect
        bus=1,                    # I2C Bus 1 on Pi 5
        gfs=0,                    # Gyro Full Scale: 0=±250°/s (most accurate for slow motion)
        afs=0,                    # Accel Full Scale: 0=±2g  (most accurate)
        mfs=1,                    # Mag Full Scale: 100uT
    )
    mpu.configure()
    print("[OK] MPU9250 initialised at I2C 0x68")
except Exception as e:
    print(f"[FATAL] Cannot initialise MPU9250: {e}")
    print("  → Check: sudo i2cdetect -y 1  (should show 68)")
    print("  → Check: sudo raspi-config → Interfaces → I2C Enabled")
    sys.exit(1)


# ---- Gyroscope Bias Calibration ----
def calibrate_gyro(samples: int = 150) -> tuple[float, float, float]:
    """
    Collects stationary gyro readings to compute drift bias.
    DO NOT move the rover during this phase.
    """
    print(f"\n[CAL] Calibrating gyroscope ({samples} samples) — KEEP ROVER STILL...")
    bx, by, bz = 0.0, 0.0, 0.0
    for i in range(samples):
        g = mpu.readGyroscopeMaster()
        bx += g[0]; by += g[1]; bz += g[2]
        # progress bar
        filled = int((i / samples) * 30)
        bar = "█" * filled + "░" * (30 - filled)
        print(f"  [{bar}] {int(i/samples*100)}%", end="\r")
        time.sleep(0.01)
    print(f"\n[OK] Calibration done.  Bias → Gx:{bx/samples:.3f}  Gy:{by/samples:.3f}  Gz:{bz/samples:.3f}")
    return bx / samples, by / samples, bz / samples


def stream():
    gx_bias, gy_bias, gz_bias = calibrate_gyro()

    roll  = 0.0
    pitch = 0.0
    yaw   = 0.0
    last_time = time.time()
    error_count = 0

    print("\n[STREAM] Broadcasting IMU at 20 Hz → Press Ctrl+C to stop\n")

    try:
        while True:
            loop_start = time.time()
            dt = loop_start - last_time
            last_time = loop_start

            # Guard against huge dt spikes (e.g. after a stall)
            if dt > 0.5:
                dt = DT_STEP

            # ---- Read Hardware ----
            try:
                accel = mpu.readAccelerometerMaster()  # [ax, ay, az] in g
                gyro  = mpu.readGyroscopeMaster()      # [gx, gy, gz] in °/s
            except Exception as e:
                error_count += 1
                print(f"\n[WARN] I2C read failed ({error_count}x): {e}", end="\r")
                time.sleep(0.05)
                continue

            # ---- Remove Bias ----
            gx = gyro[0] - gx_bias
            gy = gyro[1] - gy_bias
            gz = gyro[2] - gz_bias

            # ---- Accelerometer Angle (absolute reference) ----
            ax, ay, az = accel[0], accel[1], accel[2]
            accel_roll  = math.degrees(math.atan2(ay, az))
            accel_pitch = math.degrees(math.atan2(-ax, math.sqrt(ay**2 + az**2)))

            # ---- Complementary Filter (fuse gyro + accel) ----
            roll  = ALPHA * (roll  + gx * dt) + (1.0 - ALPHA) * accel_roll
            pitch = ALPHA * (pitch + gy * dt) + (1.0 - ALPHA) * accel_pitch

            # ---- Yaw: Gyro-only integration with noise gate ----
            if abs(gz) > 0.25:       # 0.25°/s noise gate
                yaw += gz * dt
            yaw = yaw % 360.0        # Keep in 0–360° range

            payload = {
                "pitch": round(pitch, 2),
                "roll":  round(roll,  2),
                "yaw":   round(yaw,   2)
            }

            # ---- POST to backend ----
            try:
                requests.post(URL, json=payload, timeout=0.08)
                error_count = 0  # reset on success
            except requests.exceptions.ConnectionError:
                print("[WAIT] Backend not reachable — is app.py running?", end="\r")
            except Exception:
                pass

            # ---- Console readout ----
            print(
                f"  Pitch:{pitch:+7.2f}°  Roll:{roll:+7.2f}°  Yaw:{yaw:6.1f}°   ",
                end="\r"
            )

            # ---- Rate-limit to HZ ----
            elapsed = time.time() - loop_start
            sleep_for = DT_STEP - elapsed
            if sleep_for > 0:
                time.sleep(sleep_for)

    except KeyboardInterrupt:
        print("\n\n[STOP] IMU stream stopped.")
        print(f"  Final → Pitch:{pitch:+.2f}°  Roll:{roll:+.2f}°  Yaw:{yaw:.1f}°")


if __name__ == "__main__":
    stream()
