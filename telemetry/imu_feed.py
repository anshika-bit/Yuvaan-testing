"""
YUVAAN ROVER — Production IMU Feed Script
Hardware : MPU9250 / MPU6500 @ I2C 0x68, Bus 1 (Raspberry Pi)
Driver   : drivers/imu_sensor.py  (smbus2 — no third-party sensor lib)

HOW TO RUN (on the Pi — two terminals):
    Terminal 1:  python3 telemetry/app.py
    Terminal 2:  python3 telemetry/imu_feed.py

The script:
  1. Imports our own IMUSensor driver (complementary filter + bias cal)
  2. Runs gyro bias calibration (150 samples, ~1.5 s) — keep rover still
  3. Reads real Pitch / Roll / Yaw at 20 Hz
  4. POSTs JSON payload to the FastAPI backend on localhost
  5. FastAPI pushes it over WebSocket to all connected GCS browsers

Troubleshooting:
  - sudo i2cdetect -y 1   →  must show "68" in the table
  - sudo raspi-config     →  Interfaces → I2C  → Enable
  - Check wiring: SDA→GPIO2, SCL→GPIO3, VCC→3.3V, GND→GND
"""

import sys
import os
import time
import signal
import requests

# ── Make sure 'drivers/' and the project root are on the path ──────────────────
SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))          # telemetry/
PROJECT_ROOT = os.path.dirname(SCRIPT_DIR)                         # Yuvaan-testing/
sys.path.insert(0, PROJECT_ROOT)

# ── Clear terminal ─────────────────────────────────────────────────────────────
os.system('clear' if os.name == 'posix' else 'cls')

print("=" * 58)
print("  YUVAAN ROVER — REAL IMU FEED  (smbus2 / MPU9250)")
print("=" * 58)

# ── Import our driver ─────────────────────────────────────────────────────────
try:
    from drivers.imu_sensor import IMUSensor
except ImportError as e:
    print(f"[FATAL] Cannot import IMUSensor driver: {e}")
    print("  Ensure you are running from the project root, or")
    print("  that drivers/imu_sensor.py exists.")
    sys.exit(1)

# ── Configuration ──────────────────────────────────────────────────────────────
BACKEND_URL  = "http://127.0.0.1:5000/api/imu/update"  # FastAPI on same Pi
RATE_HZ      = 20                                        # 20 readings per second
DT_STEP      = 1.0 / RATE_HZ
CAL_SAMPLES  = 150                                       # ~1.5 s calibration

print(f"  Target  : {BACKEND_URL}")
print(f"  Rate    : {RATE_HZ} Hz  (dt = {DT_STEP*1000:.0f} ms)")
print(f"  Cal     : {CAL_SAMPLES} samples  (~{CAL_SAMPLES*0.01:.1f} s)")
print("=" * 58 + "\n")


# ── Initialise sensor ──────────────────────────────────────────────────────────
try:
    imu = IMUSensor(bus=1, addr=0x68)
    print("[OK] MPU9250 / MPU6500 initialised at I2C Bus 1, Addr 0x68")
except Exception as e:
    print(f"[FATAL] Cannot open I2C device: {e}")
    print("  → sudo i2cdetect -y 1  (should show 68)")
    print("  → sudo raspi-config → Interfaces → I2C → Enable")
    sys.exit(1)


# ── Calibrate gyro bias ────────────────────────────────────────────────────────
print("\n[CAL] Calibrating gyroscope — KEEP ROVER COMPLETELY STILL ...")
bias = imu.calibrate(samples=CAL_SAMPLES)    # returns {gx, gy, gz}
print(
    f"[OK] Bias zeroed → "
    f"Gx:{bias['gx']:+.4f}  Gy:{bias['gy']:+.4f}  Gz:{bias['gz']:+.4f}  °/s"
)


# ── Graceful shutdown on Ctrl-C / SIGTERM ─────────────────────────────────────
_running = True

def _stop(signum, frame):
    global _running
    _running = False
    print("\n\n[STOP] Shutdown signal received — stopping IMU stream.")

signal.signal(signal.SIGINT,  _stop)
signal.signal(signal.SIGTERM, _stop)


# ── Main streaming loop ────────────────────────────────────────────────────────
print(f"\n[STREAM] Broadcasting real Pitch / Roll / Yaw at {RATE_HZ} Hz")
print("  Press Ctrl-C to stop.\n")

error_count  = 0
packet_count = 0
last_status  = time.time()

while _running:
    loop_start = time.time()

    # 1. Read + filter
    try:
        data = imu.update()          # {'pitch': X, 'roll': Y, 'yaw': Z}
    except Exception as e:
        error_count += 1
        print(f"[WARN] I2C read error ({error_count}x): {e}", end="\r")
        time.sleep(0.05)
        continue

    # 2. POST to FastAPI
    try:
        resp = requests.post(BACKEND_URL, json=data, timeout=0.08)
        error_count = 0   # reset on clean POST
        packet_count += 1
    except requests.exceptions.ConnectionError:
        print("[WAIT] Backend unreachable — is telemetry/app.py running?   ", end="\r")
    except requests.exceptions.Timeout:
        pass   # Backend is slow, keep going
    except Exception as e:
        print(f"[POST ERR] {e}   ", end="\r")

    # 3. Console readout every 0.5 s (not every packet, avoids flicker)
    now = time.time()
    if now - last_status >= 0.5:
        print(
            f"  Pitch: {data['pitch']:+7.2f}°   "
            f"Roll: {data['roll']:+7.2f}°   "
            f"Yaw: {data['yaw']:6.1f}°   "
            f"[{packet_count} pkts sent]   ",
            end="\r"
        )
        last_status = now

    # 4. Maintain loop rate
    elapsed   = time.time() - loop_start
    sleep_for = DT_STEP - elapsed
    if sleep_for > 0:
        time.sleep(sleep_for)


print(f"\n[DONE] IMU feed stopped after {packet_count} packets.")
