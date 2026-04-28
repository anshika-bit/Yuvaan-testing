"""
YUVAAN Rover — Master Launch Entry Point
Run this on the Raspberry Pi to start the full system.

Usage:
    python main.py

Starts:
  1. API_Server  — FastAPI WebSocket + HTTP telemetry server (port 5000)
  2. IMU_Stream  — MPU9250 calibration + 20Hz IMU feed to API
  3. Perception  — IMX500 camera capture + AI inference + frame/detection relay
  4. GPS_Stream  — (optional) NMEA GPS reader on /dev/ttyUSB0

Press Ctrl+C to stop all processes cleanly.
"""

import multiprocessing
import time
import logging
import signal
import sys
import os
import uvicorn

# ---- Project Imports ----
# NOTE: Hardware-dependent imports (IMUService, get_camera, get_gps_driver, CompassDriver)
# are done LOCALLY inside each multiprocessing.Process function to avoid ImportError
# at module load time when system libraries (smbus2, picamera2, etc.) aren't available
# in the current Python path. Each process imports what it needs.
import requests

# ---- Logging ----
logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] %(levelname)s | %(message)s",
    datefmt="%H:%M:%S"
)
log = logging.getLogger("YUVAAN.LAUNCH")


# ==========================================================================
# Process Functions — each runs in an isolated multiprocessing.Process
# ==========================================================================

def run_api():
    """Process 1: FastAPI Telemetry Server (WebSocket + MJPEG endpoint)."""
    log.info("Starting Telemetry API on port 5000...")
    # Use string-based loading to avoid pickling issues with 'app' object in multiprocessing
    import uvicorn
    uvicorn.run("telemetry.app:app", host="0.0.0.0", port=5000, log_level="warning")


def run_sensors():
    """Consolidated Process: Handles IMU (20Hz) and GPS/Fusion (2Hz) to avoid I2C contention."""
    from telemetry.imu_service import IMUService
    from drivers.sensor_fusion import get_fusion_engine
    from drivers.gps_module import get_gps_driver
    from drivers.compass import CompassDriver
    from core.bridge import get_bridge
    from core.navigation import get_nav_engine
    
    # 1. Initialize all drivers in this single process
    imu_service = IMUService() 
    fusion = get_fusion_engine()
    gps = get_gps_driver()
    compass = CompassDriver()
    bridge = get_bridge()
    nav = get_nav_engine()
    
    log.info("Starting Sensors Calibration (keep rover still)...")
    imu_service.calibrate(samples=100)
    log.info("Sensors Node initialized and calibrated.")

    GPS_API_URL = "http://localhost:5000/api/gps/update"
    IMU_API_URL = "http://localhost:5000/api/imu/update"
    BRIDGE_API_URL = "http://localhost:5000/api/bridge/update"
    NAV_API_URL = "http://localhost:5000/api/navigation/update"
    
    API_URL = "http://127.0.0.1:5000/api"
    last_gps_update = 0
    last_telemetry_time = 0
    imu_dt = 1.0 / 20.0 # 20Hz
    gps_dt = 0.5        # 2Hz
    
    tick = 0
    nav_status = {"state": "IDLE"}
    gps_raw = {"lat": 0, "lng": 0, "fix": False}
    mag_heading = 0
    imu_data = {}
    bridge_data = {"connected": False}
    remote_state = {}

    while True:
        loop_start = time.time()
        
        try:
            tick += 1
            
            # 1. Sync with API (5Hz throttled) to get GCS manual commands
            try:
                # Update local driver states (20Hz)
                bridge_data = bridge.get_telemetry()
                imu_data = imu_service.imu.update()

                if tick % 4 == 0:
                    state_resp = requests.get(f"http://127.0.0.1:5000/api/navigation/state", timeout=0.08)
                    if state_resp.ok:
                        remote_state = state_resp.json()
                        
                        # --- REMOTE TOGGLE & GCS SYNC ---
                        hw_mode = bridge_data.get('mode', 'MAN')
                        gcs_active = remote_state.get('active', False)
                        
                        # Only trigger hardware-forced changes on actual SWITCH TRANSITIONS
                        # to avoid fighting with the GCS button.
                        if not hasattr(nav, 'last_hw_mode'): nav.last_hw_mode = hw_mode
                        
                        hw_changed = hw_mode != nav.last_hw_mode
                        
                        if hw_changed:
                            if hw_mode == 'AUT' and not nav.active:
                                log.info("REMOTE: Hardware switch toggled to AUTO. Engaging Nav Engine.")
                                nav.start()
                                requests.post("http://127.0.0.1:5000/api/navigation/toggle?active=true", timeout=0.05)
                            elif hw_mode == 'MAN' and nav.active:
                                log.warning("REMOTE: Hardware switch toggled to MANUAL. Halting Nav Engine.")
                                nav.stop()
                                requests.post("http://127.0.0.1:5000/api/navigation/toggle?active=false", timeout=0.05)
                        else:
                            # If hardware hasn't changed, follow the GCS button
                            if gcs_active and not nav.active and not nav.emergency_locked:
                                nav.start()
                            elif not gcs_active and nav.active:
                                nav.stop()
                        
                        nav.last_hw_mode = hw_mode

                        # Only sync waypoints every 0.5s (10 ticks)
                        if tick % 10 == 0:
                            remote_wps = remote_state.get('waypoints', [])
                            if not hasattr(nav, 'last_wps') or remote_wps != nav.last_wps:
                                if len(remote_wps) > 0:
                                    log.info(f"NAV: Waypoints synced ({len(remote_wps)} points)")
                                    nav.set_waypoints(remote_wps)
                                    nav.last_wps = remote_wps.copy()
                            
                            if not remote_state.get('locked') and nav.emergency_locked:
                                nav.unlock()
                    else:
                        if tick % 100 == 0: log.warning(f"SYNC: GCS State unreachable (HTTP {state_resp.status_code})")
            except Exception as e:
                if tick % 100 == 0: log.debug(f"SYNC: GCS State polling failed: {e}")
                pass 

            # --- GPS & Fusion Update (2Hz) ---
            if time.time() - last_gps_update >= 0.5:
                try:
                    gps_raw = gps.read()
                    mag_heading = compass.read_heading()
                    last_gps_update = time.time()
                except Exception as e:
                    log.error(f"GPS Read Failure: {e}")

            # 2. Re-fuse using latest data
            fused_data = fusion.update(gps_raw, imu_data, mag_heading, bridge_data)
            
            # 3. BULK SYNC (Throttle to 10Hz to save CPU)
            if time.time() - last_telemetry_time > 0.1:
                bulk_data = {
                    "imu": imu_data,
                    "gps": fused_data,
                    "bridge": bridge_data,
                    "navigation": nav_status
                }
                try:
                    requests.post(f"{API_URL}/telemetry/sync", json=bulk_data, timeout=0.08)
                    last_telemetry_time = time.time()
                except: pass
            
            # 4. Arbitration: Manual WASD overrides Autonomous Navigation
            manual_cmd = remote_state.get('manual_cmd')
            # Only consider manual if it's recent AND it's not a dummy STOP command
            is_manual = manual_cmd and (time.time() - manual_cmd.get('ts', 0)) < 1.0 and manual_cmd['type'] != 'STOP'
            
            if is_manual:
                cmd_type = manual_cmd['type']
                val = manual_cmd['val']
                
                if nav.active:
                    nav.stop()
                    log.info("MANUAL OVERRIDE: Active command received. Halting navigation engine.")

                # RE-MAPPED CONTROLS (Matched to your wiring)
                if cmd_type == 'MOVE': 
                    l_speed = -val
                    r_speed = val
                    bridge.send_command("DRIVE", int(l_speed), int(r_speed))
                elif cmd_type == 'TURN':
                    speed = -val
                    bridge.send_command("DRIVE", int(speed), int(speed))
                
                nav_status['state'] = "MANUAL_CONTROL"
            else:
                # Run Autonomous Navigation Engine
                if nav.active and not nav.emergency_locked:
                    prev_state = nav.state
                    nav_status = nav.update(fused_data)
                    if nav.state != prev_state:
                        log.info(f"NAV: State transition {prev_state} -> {nav.state}")
                else:
                    nav_status['state'] = "LOCKED" if nav.emergency_locked else "IDLE"
            
            # Handle ESTOP request from GCS
            if remote_state.get('estop_requested') and not nav.emergency_locked:
                nav.stop(lock=True)
                bridge.send_command("STOP")
                log.critical("!!! ESTOP TRIGGERED: SYSTEM PERMANENTLY LOCKED !!!")
                requests.post("http://127.0.0.1:5000/api/navigation/update", json={"estop_requested": False, "active": False, "locked": True}, timeout=0.1)


            # 5. Dashboard HUD (Update terminal every 0.5s)
            if tick % 10 == 0:
                last_rx_ago = time.time() - bridge_data.get('last_update', 0)
                rx_status = f"{last_rx_ago:.1f}s" if bridge_data.get('last_update', 0) > 0 else "NEVER"
                
                print(f"\n{'='*60}")
                print(f" YUVAAN MISSION CONTROL | T+{tick/20:.1f}s | {time.strftime('%H:%M:%S')}")
                print(f"{'='*60}")
                
                # SECTION 1: HARDWARE & COMMS
                mode_str = bridge_data.get('mode','UNKNOWN')
                link_str = "ONLINE" if bridge_data['connected'] else "OFFLINE"
                print(f" [CONTROL]  MODE: {mode_str:<10} | LINK: {link_str:<10} | RX: {rx_status}")
                
                # SECTION 2: POWER & MOTORS
                batt_pct = bridge_data.get('battery',{}).get('percent',0)
                batt_v = bridge_data.get('battery',{}).get('voltage',0.0)
                vel = bridge_data.get('motors',{}).get('velocity',0.0)
                last_cmd = bridge_data.get('last_cmd', 'NONE')
                print(f" [POWER]    BATT: {batt_pct}% ({batt_v}V) | VELOCITY: {vel:.2f} m/s | CMD: {last_cmd}")
                
                # SECTION 3: NAVIGATION & SENSORS
                gps_fix = "FIXED" if gps_raw['fix'] else "SEARCHING"
                sats = gps_raw.get('sats', 0)
                print(f" [GPS]      FIX:  {gps_fix:<10} | SATS: {sats:<10} | HDG: {fused_data['heading']:.1f}°")
                
                # SECTION 4: MISSION STATUS
                if nav.active:
                    wp_progress = f"{nav.current_wp_index + 1}/{len(nav.waypoints)}"
                    dist_to_wp = f"{nav_status.get('distance_to_wp', 0.0):.1f}m"
                    print(f" [MISSION]  STATE: {nav_status['state']:<9} | WP: {wp_progress:<12} | DIST: {dist_to_wp}")
                else:
                    status_str = "LOCKED (E-STOP)" if nav.emergency_locked else "IDLE (WAITING)"
                    print(f" [MISSION]  STATUS: {status_str}")
                
                print(f"{'-'*60}")
                if last_rx_ago > 5.0 and bridge_data['connected']:
                    print(" [WARNING] BRIDGE TIMEOUT: No data received from STM32 for >5s!")

                print(" (Ctrl+C to Shutdown)")

            # Maintain 20Hz loop rate
            elapsed = time.time() - loop_start
            sleep_for = imu_dt - elapsed
            if sleep_for > 0:
                time.sleep(sleep_for)

        except Exception as e:
            log.error(f"Nav Loop Critical Error: {e}")
            time.sleep(0.05)



def run_perception():
    """Process 2: Camera Engine (IMX500 via picamera2)."""
    try:
        log.info("Initializing Camera Engine...")
        cam = get_camera()
        cam.run()
    except Exception as e:
        log.error(f"Perception Node failed: {e}")
        while True: time.sleep(10)


# ==========================================================================
# Main
# ==========================================================================

def main():
    print("=" * 60)
    print("  YUVAAN ROVER SYSTEM — MASTER LAUNCH")
    print("=" * 60)

    # Build process list
    api_process = multiprocessing.Process(target=run_api,        name="API_Server",  daemon=False)
    cam_process = multiprocessing.Process(target=run_perception, name="Perception",  daemon=True)
    sns_process = multiprocessing.Process(target=run_sensors,    name="Sensors_Node", daemon=True)

    all_processes = [api_process, cam_process, sns_process]

    # Store parent PID to ensure only the parent handles process termination
    PARENT_PID = os.getpid()

    def signal_handler(sig, frame):
        if os.getpid() != PARENT_PID:
            # Children should just exit silently upon receiving signal
            sys.exit(0)
            
        log.info("Shutdown signal received. Stopping all nodes...")
        for p in all_processes:
            try:
                if p.is_alive():
                    p.terminate()
                    p.join(timeout=1)
            except Exception:
                pass
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # Start processes — API first, then wait for it to bind
    log.info("Launching System Nodes...")
    api_process.start()
    
    # Wait for API to be responsive before starting sensors
    log.info("Waiting for Telemetry API to bind...")
    max_retries = 15
    while max_retries > 0:
        try:
            requests.get("http://127.0.0.1:5000/health", timeout=1)
            log.info("API Server is ONLINE.")
            break
        except:
            max_retries -= 1
            time.sleep(1)
    
    if max_retries == 0:
        log.error("CRITICAL: API Server failed to start. Aborting.")
        api_process.terminate()
        return

    sns_process.start()
    cam_process.start()

    log.info("All nodes active. System operational.")
    log.info("  → Dashboard: http://<Pi_IP>:5000/health")
    log.info("  → Video:     http://<Pi_IP>:5000/api/video_feed")
    log.info("  → Telemetry: ws://<Pi_IP>:5000/ws/telemetry")

    # Keep main alive until API process exits
    api_process.join()


if __name__ == "__main__":
    main()
