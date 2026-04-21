import time
import requests
import logging
from drivers.imu import IMUSensor

log = logging.getLogger("YUVAAN.IMU")

class IMUService:
    def __init__(self, backend_url="http://127.0.0.1:5000/api/imu/update", rate_hz=20):
        self.backend_url = backend_url
        self.rate_hz = rate_hz
        self.dt_step = 1.0 / rate_hz
        self.imu = IMUSensor(bus=1, addr=0x68)
        self.is_running = False

    def calibrate(self, samples=150):
        log.info(f"Calibrating IMU ({samples} samples)...")
        return self.imu.calibrate(samples=samples)

    def run(self):
        log.info(f"Starting IMU stream to {self.backend_url} at {self.rate_hz}Hz")
        self.is_running = True
        packet_count = 0
        
        while self.is_running:
            loop_start = time.time()
            try:
                data = self.imu.update()
                # POST to FastAPI
                try:
                    requests.post(self.backend_url, json=data, timeout=0.08)
                    packet_count += 1
                except Exception:
                    pass # Silent failure for telemetry bursts
                
            except Exception as e:
                log.error(f"IMU Service Error: {e}")
                time.sleep(0.1)
                
            # Maintain loop rate
            elapsed = time.time() - loop_start
            sleep_for = self.dt_step - elapsed
            if sleep_for > 0:
                time.sleep(sleep_for)

    def stop(self):
        self.is_running = False
        log.info("IMU Service Stopped.")
