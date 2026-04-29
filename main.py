"""
YUVAAN Rover - master launch entry point.

Run this on the Raspberry Pi to start the full rover stack:
  1. API_Server   - FastAPI telemetry and control server
  2. Sensors_Node - IMU, GPS, fusion, bridge, and navigation loop
  3. Perception   - IMX500 camera relay and detections
"""

import logging
import multiprocessing
import os
import signal
import sys
import time


# ---- Project Root Path ----
_PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))


def _ensure_project_path():
    """Ensure the project root is available in every process."""
    if _PROJECT_ROOT not in sys.path:
        sys.path.insert(0, _PROJECT_ROOT)
    os.chdir(_PROJECT_ROOT)


_ensure_project_path()

import requests
from runtime_logging import (
    build_mission_control_screen,
    configure_runtime_logging,
    get_recent_runtime_logs,
    reset_runtime_logs,
)


configure_runtime_logging(console=True)
log = logging.getLogger("YUVAAN.LAUNCH")


def run_api():
    """Process 1: FastAPI telemetry server."""
    _ensure_project_path()
    configure_runtime_logging(console=False)
    log.info("Starting Telemetry API on port 5000...")

    import uvicorn

    uvicorn.run("telemetry.app:app", host="0.0.0.0", port=5000, log_level="warning")


def run_sensors():
    """Process 2: IMU, GPS, fusion, bridge, and navigation loop."""
    _ensure_project_path()
    configure_runtime_logging(console=False)

    from core.bridge import get_bridge
    from core.navigation import get_nav_engine
    from drivers.compass import CompassDriver
    from drivers.gps_module import get_gps_driver
    from drivers.sensor_fusion import get_fusion_engine
    from telemetry.imu_service import IMUService

    imu_service = IMUService()
    fusion = get_fusion_engine()
    gps = get_gps_driver()
    compass = CompassDriver()
    bridge = get_bridge()
    nav = get_nav_engine()

    # Set home position for geofence checks
    import os as _os
    _home_lat = float(_os.environ.get("YUVAAN_BASE_LAT", "18.4485"))
    _home_lng = float(_os.environ.get("YUVAAN_BASE_LNG", "77.4562"))
    nav.set_home({"lat": _home_lat, "lng": _home_lng})

    log.info("Starting Sensors Calibration (keep rover still)...")
    imu_service.calibrate(samples=100)
    log.info("Sensors Node initialized and calibrated.")

    api_url = "http://127.0.0.1:5000/api"
    last_gps_update = 0.0
    last_telemetry_time = 0.0
    imu_dt = 1.0 / 20.0

    tick = 0
    nav_status = {"state": "IDLE"}
    gps_raw = {"lat": 0, "lng": 0, "fix": False}
    mag_heading = 0.0
    imu_data = {}
    bridge_data = {"connected": False}
    remote_state = {}

    while True:
        loop_start = time.time()

        try:
            tick += 1

            try:
                bridge_data = bridge.get_telemetry()
                imu_data = imu_service.imu.update()

                if tick % 4 == 0:
                    state_resp = requests.get(f"{api_url}/navigation/state", timeout=0.08)
                    if state_resp.ok:
                        remote_state = state_resp.json()

                        hw_mode = bridge_data.get("mode", "MAN")
                        gcs_active = remote_state.get("active", False)

                        if not hasattr(nav, "last_hw_mode"):
                            nav.last_hw_mode = hw_mode
                        hw_changed = hw_mode != nav.last_hw_mode

                        # ── 1. Sync waypoints FIRST — before any start/stop ──
                        remote_wps = remote_state.get("waypoints", [])
                        if not hasattr(nav, "last_wps") or remote_wps != nav.last_wps:
                            log.info(f"NAV: Waypoints synced ({len(remote_wps)} points)")
                            nav.set_waypoints(remote_wps)
                            nav.last_wps = remote_wps.copy()

                        if tick % 40 == 0:
                            log.info(
                                f"SYNC_DEBUG: hw={hw_mode} gcs_active={gcs_active} "
                                f"sensor_active={nav.active} locked={nav.emergency_locked} "
                                f"wps={len(remote_wps)}"
                            )

                        # ── 2. Hardware switch change has highest priority ──
                        if hw_changed:
                            if hw_mode == "AUT" and not nav.active:
                                log.info("REMOTE: Hardware switch toggled to AUTO. Engaging Nav Engine.")
                                nav.start()
                                requests.post(f"{api_url}/navigation/toggle?active=true", timeout=0.05)
                            elif hw_mode == "MAN" and nav.active:
                                log.warning("REMOTE: Hardware switch toggled to MANUAL. Halting Nav Engine.")
                                nav.stop()
                                requests.post(f"{api_url}/navigation/toggle?active=false", timeout=0.05)
                        else:
                            # ── 3. GCS active request (only if HW didn't override) ──
                            if gcs_active and not nav.active and not nav.emergency_locked:
                                nav.start()
                            elif not gcs_active and nav.active:
                                nav.stop()

                        nav.last_hw_mode = hw_mode

                        if tick % 10 == 0:
                            if not remote_state.get("locked") and nav.emergency_locked:
                                nav.unlock()

                            # Sync home position from GCS for geofence
                            remote_home = remote_state.get("home_position")
                            if (
                                remote_home
                                and remote_home.get("lat") is not None
                                and remote_home.get("lng") is not None
                            ):
                                nav.set_home(remote_home)

                            if remote_state.get("calibration_requested"):
                                log.info("COMPASS: Calibration request received from GCS. Starting hardware calibration...")
                                try:
                                    compass.calibrate(duration=30)
                                    log.info("COMPASS: Hardware calibration COMPLETE.")
                                except Exception as cal_err:
                                    log.error(f"COMPASS: Calibration FAILED: {cal_err}")
                                try:
                                    requests.post(
                                        f"{api_url}/navigation/update",
                                        json={"calibration_requested": False},
                                        timeout=0.1,
                                    )
                                except Exception:
                                    pass
                    else:
                        if tick % 100 == 0:
                            log.warning(f"SYNC: GCS State unreachable (HTTP {state_resp.status_code})")
            except Exception as sync_err:
                if tick % 100 == 0:
                    log.debug(f"SYNC: GCS State polling failed: {sync_err}")

            if time.time() - last_gps_update >= 0.5:
                try:
                    gps_raw = gps.read()
                    mag_heading = compass.read_heading()
                    last_gps_update = time.time()
                except Exception as gps_err:
                    log.error(f"GPS Read Failure: {gps_err}")

            fused_data = fusion.update(gps_raw, imu_data, mag_heading, bridge_data)

            manual_cmd = remote_state.get("manual_cmd")
            is_manual = (
                manual_cmd
                and (time.time() - manual_cmd.get("ts", 0)) < 1.0
                and manual_cmd.get("type") != "STOP"
            )

            if is_manual:
                cmd_type = manual_cmd["type"]
                val = manual_cmd["val"]

                if nav.active:
                    nav.stop()
                    log.info("MANUAL OVERRIDE: Active command received. Halting navigation engine.")

                if cmd_type == "MOVE":
                    bridge.send_command("DRIVE", int(val), int(val))
                elif cmd_type == "TURN":
                    bridge.send_command("DRIVE", int(-val), int(val))

                nav_status["state"] = "MANUAL_CONTROL"
                nav_status["active"] = False
            else:
                if nav.active and not nav.emergency_locked:
                    prev_state = nav.state
                    # Pass detections for human-pause safety
                    active_detections = remote_state.get("detections", [])
                    nav_status = nav.update(fused_data, detections=active_detections)
                    if nav.state != prev_state:
                        log.info(f"NAV: State transition {prev_state} -> {nav.state}")
                else:
                    nav_status["state"] = "LOCKED" if nav.emergency_locked else "IDLE"
                    nav_status["active"] = False

            if remote_state.get("estop_requested") and not nav.emergency_locked:
                nav.stop(lock=True)
                bridge.send_command("STOP")
                log.critical("!!! ESTOP TRIGGERED: SYSTEM PERMANENTLY LOCKED !!!")
                requests.post(
                    f"{api_url}/navigation/update",
                    json={"estop_requested": False, "active": False, "locked": True},
                    timeout=0.1,
                )

            if time.time() - last_telemetry_time > 0.1:
                nav_status["active"] = nav.active and not nav.emergency_locked
                bulk_data = {
                    "imu": imu_data,
                    "gps": fused_data,
                    "bridge": bridge_data,
                    "navigation": nav_status,
                }
                try:
                    requests.post(f"{api_url}/telemetry/sync", json=bulk_data, timeout=0.08)
                    last_telemetry_time = time.time()
                except Exception:
                    pass

            if tick % 10 == 0:
                hud = build_mission_control_screen(
                    tick=tick,
                    bridge_data=bridge_data,
                    gps_raw=gps_raw,
                    fused_data=fused_data,
                    nav=nav,
                    nav_status=nav_status,
                    recent_logs=get_recent_runtime_logs(limit=8),
                )
                if sys.stdout.isatty():
                    sys.stdout.write("\x1b[2J\x1b[H" + hud + "\n")
                else:
                    sys.stdout.write(hud + "\n")
                sys.stdout.flush()

            elapsed = time.time() - loop_start
            sleep_for = imu_dt - elapsed
            if sleep_for > 0:
                time.sleep(sleep_for)

        except Exception as loop_err:
            log.error(f"Nav Loop Critical Error: {loop_err}")
            time.sleep(0.05)


def run_perception():
    """Process 3: Camera engine."""
    _ensure_project_path()
    configure_runtime_logging(console=False)
    try:
        from perception.camera_engine import get_camera

        log.info("Initializing Camera Engine...")
        cam = get_camera()
        cam.run()
    except Exception as err:
        log.exception(f"Perception Node failed: {err}")
        while True:
            time.sleep(10)


def main():
    reset_runtime_logs()
    configure_runtime_logging(console=True)

    print("=" * 60)
    print("  YUVAAN ROVER SYSTEM - MASTER LAUNCH")
    print("=" * 60)

    api_process = multiprocessing.Process(target=run_api, name="API_Server", daemon=False)
    cam_process = multiprocessing.Process(target=run_perception, name="Perception", daemon=False)
    sns_process = multiprocessing.Process(target=run_sensors, name="Sensors_Node", daemon=True)

    all_processes = [api_process, cam_process, sns_process]
    parent_pid = os.getpid()

    def signal_handler(sig, frame):
        if os.getpid() != parent_pid:
            sys.exit(0)

        log.info("Shutdown signal received. Stopping all nodes...")
        for process in all_processes:
            try:
                if process.is_alive():
                    process.terminate()
                    process.join(timeout=1)
            except Exception:
                pass
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    log.info("Launching System Nodes...")
    api_process.start()

    log.info("Waiting for Telemetry API to bind...")
    max_retries = 15
    while max_retries > 0:
        try:
            requests.get("http://127.0.0.1:5000/health", timeout=1)
            log.info("API Server is ONLINE.")
            break
        except Exception:
            max_retries -= 1
            time.sleep(1)

    if max_retries == 0:
        log.error("CRITICAL: API Server failed to start. Aborting.")
        api_process.terminate()
        return

    sns_process.start()
    cam_process.start()

    log.info("All nodes active. System operational.")
    log.info("  -> Dashboard: http://<Pi_IP>:5000/health")
    log.info("  -> Video:     http://<Pi_IP>:5000/api/video_feed")
    log.info("  -> Telemetry: ws://<Pi_IP>:5000/ws/telemetry")

    api_process.join()

    for process in (cam_process, sns_process):
        try:
            if process.is_alive():
                process.terminate()
                process.join(timeout=1)
        except Exception:
            pass


if __name__ == "__main__":
    main()
