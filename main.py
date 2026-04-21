import multiprocessing
import time
import logging
import signal
import sys
import os
import uvicorn

# Project Imports
from telemetry.app import app
from telemetry.imu_service import IMUService
from perception.camera_engine import get_camera
from drivers.gps_module import get_gps_driver

# Setup Logging
logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] %(levelname)s | %(name)s | %(message)s",
    datefmt="%H:%M:%S"
)
log = logging.getLogger("YUVAAN.LAUNCH")

def run_api():
    """Start the FastAPI Telemetry Server."""
    log.info("Starting Telemetry API on port 5000...")
    uvicorn.run(app, host="0.0.0.0", port=5000, log_level="warning")

def run_imu():
    """Initialize and run the IMU service."""
    service = IMUService()
    # 1. Calibration phase
    log.info("Starting IMU Calibration (Keep rover still)...")
    service.calibrate(samples=150)
    # 2. Feed phase
    service.run()

def run_perception():
    """Warm up the camera engine."""
    log.info("Initializing Camera Engine...")
    cam = get_camera()
    # The camera engine starts its own thread internally upon get_camera()
    while True:
        time.sleep(10)

def run_gps():
    """Initialize and run the GPS service."""
    log.info("Initializing GPS Service...")
    gps = get_gps_driver()
    while True:
        data = gps.read()
        if data['fix']:
            log.info(f"GPS Fix: {data['lat']}, {data['lng']}")
        time.sleep(5)

def main():
    print("=" * 60)
    print("  YUVAAN ROVER SYSTEM — MASTER LAUNCH")
    print("=" * 60)
    
    # 1. Start components in background processes
    api_process = multiprocessing.Process(target=run_api, name="API_Server")
    imu_process = multiprocessing.Process(target=run_imu, name="IMU_Stream")
    cam_process = multiprocessing.Process(target=run_perception, name="Perception")
    gps_process = multiprocessing.Process(target=run_gps, name="GPS_Stream")

    # Handle termination gracefully
    def signal_handler(sig, frame):
        log.info("Shutdown signal received. Killing sub-processes...")
        api_process.terminate()
        imu_process.terminate()
        cam_process.terminate()
        gps_process.terminate()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    log.info("Launching System Nodes...")
    api_process.start()
    time.sleep(1) # Give API a second to bind
    imu_process.start()
    cam_process.start()
    gps_process.start()

    log.info("All nodes active. System operational.")
    
    # Keep main alive
    api_process.join()
    imu_process.join()
    cam_process.join()

if __name__ == "__main__":
    main()
