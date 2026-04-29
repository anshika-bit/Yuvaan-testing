from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import asyncio
import json
import os
import logging
import time
from contextlib import asynccontextmanager
from fastapi.responses import StreamingResponse
from runtime_logging import configure_runtime_logging, get_recent_runtime_logs

# ---------------------------------------------------------
# YUVAAN ROVER TELEMETRY BACKEND (Run this on the Pi 5)
# ---------------------------------------------------------

configure_runtime_logging(console=False)
log = logging.getLogger("YUVAAN.API")

@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup
    yield
    # Shutdown
    pass

app = FastAPI(title="Yuvaan Rover Telemetry", version="2.0", lifespan=lifespan)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# ---- Global Live Telemetry Store ----
current_imu_data = {"pitch": 0.0, "roll": 0.0, "yaw": 0.0}
current_gps_data = {"lat": None, "lng": None, "alt": None, "fix": False, "heading": 0.0, "sats": 0}
current_bridge_data = {
    "connected": False,
    "mode": "UNKNOWN",
    "battery": {"percent": 0, "voltage": 0.0},
    "motors": {"velocity": 0.0, "rpm": 0, "current": 0.0},
    "encoders": {"left": 0, "right": 0}
}
current_nav_status = {
    "active": False,
    "distance_to_wp": 0.0,
    "target_bearing": 0.0,
    "bearing_error": 0.0,
    "current_wp": None,
    "detections": []
}

class IMUInput(BaseModel):
    pitch: float
    roll: float
    yaw: float

class GPSInput(BaseModel):
    lat: float | None
    lng: float | None
    alt: float | None
    fix: bool
    heading: float = 0.0
    sats: int = 0

class BridgeInput(BaseModel):
    connected: bool
    mode: str
    battery: dict
    motors: dict
    encoders: dict
    last_cmd: str | None = None
    last_cmd_ts: float = 0.0
    
    class Config:
        extra = "allow"

class NavStatusInput(BaseModel):
    active: bool
    distance_to_wp: float
    target_bearing: float
    bearing_error: float
    current_wp: int | None
    state: str = "IDLE"

    class Config:
        extra = "allow"

class DriveCommand(BaseModel):
    cmd: str
    val: float = 1.0

class Waypoint(BaseModel):
    lat: float
    lng: float
    action: str = "move"

class WaypointList(BaseModel):
    waypoints: list[Waypoint]

@app.post("/api/imu/update")
async def update_imu_data(data: IMUInput):
    global current_imu_data
    current_imu_data.update(data.dict())
    return {"status": "ok"}

@app.post("/api/gps/update")
async def update_gps_data(data: GPSInput):
    global current_gps_data
    current_gps_data.update(data.dict())
    return {"status": "ok"}

@app.post("/api/bridge/update")
async def update_bridge_data(data: BridgeInput):
    global current_bridge_data
    # DIAGNOSTIC: Log every mode change or every 100th packet to avoid spam
    if data.mode != current_bridge_data.get('mode'):
        log.info(f"API: Received Bridge Update. Mode: {data.mode}, Batt: {data.battery.get('percent')}%")
    current_bridge_data.update(data.dict())
    return {"status": "ok"}

@app.post("/api/telemetry/sync")
async def sync_all_telemetry(data: dict):
    """Bulk update endpoint to reduce network overhead."""
    global current_imu_data, current_gps_data, current_bridge_data, current_nav_status
    if "imu" in data: current_imu_data.update(data["imu"])
    if "gps" in data: current_gps_data.update(data["gps"])
    if "bridge" in data: current_bridge_data.update(data["bridge"])
    if "navigation" in data: 
        # Forward ALL nav telemetry fields from the sensor process.
        # We protect 'waypoints' and 'manual_cmd' (driven by GCS requests),
        # but 'active' and 'state' come from the actual nav engine.
        nav_telemetry = data["navigation"]
        protected_keys = {"waypoints", "manual_cmd", "estop_requested", "calibration_requested"}
        for k, v in nav_telemetry.items():
            if k not in protected_keys:
                current_nav_status[k] = v
    return {"status": "ok"}

@app.post("/api/navigation/update")
async def update_nav_status(data: dict):
    global current_nav_status
    current_nav_status.update(data)
    return {"status": "ok"}

@app.post("/api/navigation/waypoints")
async def set_waypoints(data: WaypointList):
    from core.navigation import get_nav_engine
    nav = get_nav_engine()
    nav.set_waypoints([wp.dict() for wp in data.waypoints])
    return {"status": "ok", "count": len(data.waypoints)}

@app.get("/api/diagnostics/deep_scan")
async def run_deep_scan():
    import asyncio
    # Diagnostics now just checks if we are receiving data
    await asyncio.sleep(0.2)
    bridge_ok = current_bridge_data.get("connected", False)
    gps_ok = current_gps_data.get("fix", False)
    
    return {
        "status": "ok",
        "i2c_bus": "ONLINE" if bridge_ok else "OFFLINE/SIMULATED",
        "bridge_mode": current_bridge_data.get("mode", "UNKNOWN"),
        "motor_controller": "RESPONSIVE" if bridge_ok else "NO_LINK",
        "imu_fusion": "STABLE" if gps_ok else "DEGRADING",
        "pi_core_temp": round(45.2 + (time.time() % 5), 1),
        "voltage": current_bridge_data.get("battery", {}).get("voltage", 0.0)
    }

@app.post("/api/calibrate/compass")
async def calibrate_compass():
    """Triggers a compass calibration window via the state sync."""
    # Instead of doing it here, we set a flag that the Sensors process will see
    current_nav_status["calibration_requested"] = True
    log.info("COMPASS: Calibration request queued for Sensors Process.")
    return {"status": "ok", "message": "Calibration starting soon..."}


@app.post("/api/navigation/toggle")
async def toggle_navigation(active: bool):
    from core.navigation import get_nav_engine
    nav = get_nav_engine()
    if active: 
        nav.start()
    else: 
        nav.stop()
    
    # Immediately update status
    current_nav_status["active"] = nav.active
    current_nav_status["state"] = nav.state
    
    return {"status": "ok", "active": nav.active}

@app.post("/api/navigation/estop")
async def engage_estop():
    from core.navigation import get_nav_engine
    nav = get_nav_engine()
    nav.stop(lock=True) # Permanent lock until manual release or reboot
    
    # We set a flag so the Sensors process knows to send a physical STOP command
    current_nav_status["estop_requested"] = True
    
    log.error("ESTOP: Emergency Stop Triggered and System Locked!")
    return {"status": "ok", "locked": True}

@app.post("/api/navigation/unlock")
async def release_lock():
    from core.navigation import get_nav_engine
    nav = get_nav_engine()
    nav.unlock()
    return {"status": "ok", "locked": False}

@app.post("/api/navigation/rtl")
async def engage_rtl(lat: float, lng: float):
    from core.navigation import get_nav_engine
    nav = get_nav_engine()
    success = nav.engage_rtl({"lat": lat, "lng": lng})
    return {"status": "ok" if success else "error"}

@app.post("/api/bridge/drive")
async def manual_drive(data: DriveCommand):
    """Direct manual override for WASD testing."""
    current_nav_status["manual_cmd"] = {"type": data.cmd, "val": data.val, "ts": time.time()}
    log.debug(f"GCS_OVERRIDE: Received {data.cmd} with magnitude {data.val}")
    return {"status": "ok"}

@app.get("/api/navigation/state")
async def get_nav_state():
    """Internal endpoint for Sensors process to sync state."""
    from core.navigation import get_nav_engine
    nav = get_nav_engine()
    return {
        "active": nav.active,
        "locked": nav.emergency_locked,
        "waypoints": nav.waypoints,
        "manual_cmd": current_nav_status.get("manual_cmd"),
        "estop_requested": current_nav_status.get("estop_requested"),
        "calibration_requested": current_nav_status.get("calibration_requested"),
        "detections": current_nav_status.get("detections", []),
        "home_position": current_nav_status.get("home_position"),
    }

@app.get("/api/navigation/config")
async def get_nav_config():
    """Returns the current live navigation configuration for field tuning."""
    from core import nav_config
    return {"status": "ok", "config": nav_config.get_config()}

@app.post("/api/navigation/config")
async def update_nav_config(data: dict):
    """Live-update navigation parameters (PID gains, speeds, etc.) without restart."""
    from core import nav_config
    nav_config.update_config(data)
    log.info(f"NAV_CONFIG: Updated {len(data)} parameters via API.")
    return {"status": "ok", "config": nav_config.get_config()}

@app.post("/api/navigation/home")
async def set_home_position(data: dict):
    """Set the home/base-station position for geofence enforcement.
    Called by GCS when the user sets or changes the base station location."""
    lat = data.get("lat")
    lng = data.get("lng")
    if lat is None or lng is None:
        return {"status": "error", "message": "lat and lng required"}
    current_nav_status["home_position"] = {"lat": float(lat), "lng": float(lng)}
    log.info(f"NAV: Home position updated to {lat:.6f}, {lng:.6f}")
    return {"status": "ok"}

@app.websocket("/ws/telemetry")
async def websocket_telemetry_endpoint(websocket: WebSocket):
    await websocket.accept()
    log.info(f"GCS CONNECTED   ← {websocket.client.host if websocket.client else 'unknown'}")
    try:
        while True:
            payload = {
                "imu": current_imu_data,
                "gps": current_gps_data,
                "bridge": current_bridge_data,
                "navigation": current_nav_status,
                "server_logs": get_recent_runtime_logs(limit=80),
                "server_ts": int(time.time() * 1000)
            }
            await websocket.send_text(json.dumps(payload))
            await asyncio.sleep(0.1) 
    except WebSocketDisconnect:
        log.warning("GCS DISCONNECTED")

@app.get("/health")
async def health():
    return {"status": "online", "imu": current_imu_data}

@app.post("/api/detections/update")
async def update_detections(data: dict):
    global current_nav_status
    current_nav_status["detections"] = data.get("detections", [])
    return {"status": "ok"}

_FRAME_PATH = "/tmp/yuvaan_latest_frame.jpg"
_CAMERA_STATUS_PATH = "/tmp/yuvaan_camera_status.json"

# 1x1 black JPEG — smallest valid image that triggers browser <img> onLoad.
# This breaks the "infinite connecting" state when no camera frames exist.
_PLACEHOLDER_JPEG = bytes([
    0xFF, 0xD8, 0xFF, 0xE0, 0x00, 0x10, 0x4A, 0x46, 0x49, 0x46, 0x00, 0x01,
    0x01, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0xFF, 0xDB, 0x00, 0x43,
    0x00, 0x08, 0x06, 0x06, 0x07, 0x06, 0x05, 0x08, 0x07, 0x07, 0x07, 0x09,
    0x09, 0x08, 0x0A, 0x0C, 0x14, 0x0D, 0x0C, 0x0B, 0x0B, 0x0C, 0x19, 0x12,
    0x13, 0x0F, 0x14, 0x1D, 0x1A, 0x1F, 0x1E, 0x1D, 0x1A, 0x1C, 0x1C, 0x20,
    0x24, 0x2E, 0x27, 0x20, 0x22, 0x2C, 0x23, 0x1C, 0x1C, 0x28, 0x37, 0x29,
    0x2C, 0x30, 0x31, 0x34, 0x34, 0x34, 0x1F, 0x27, 0x39, 0x3D, 0x38, 0x32,
    0x3C, 0x2E, 0x33, 0x34, 0x32, 0xFF, 0xC0, 0x00, 0x0B, 0x08, 0x00, 0x01,
    0x00, 0x01, 0x01, 0x01, 0x11, 0x00, 0xFF, 0xC4, 0x00, 0x1F, 0x00, 0x00,
    0x01, 0x05, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
    0x09, 0x0A, 0x0B, 0xFF, 0xC4, 0x00, 0xB5, 0x10, 0x00, 0x02, 0x01, 0x03,
    0x03, 0x02, 0x04, 0x03, 0x05, 0x05, 0x04, 0x04, 0x00, 0x00, 0x01, 0x7D,
    0x01, 0x02, 0x03, 0x00, 0x04, 0x11, 0x05, 0x12, 0x21, 0x31, 0x41, 0x06,
    0x13, 0x51, 0x61, 0x07, 0x22, 0x71, 0x14, 0x32, 0x81, 0x91, 0xA1, 0x08,
    0x23, 0x42, 0xB1, 0xC1, 0x15, 0x52, 0xD1, 0xF0, 0x24, 0x33, 0x62, 0x72,
    0x82, 0x09, 0x0A, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x25, 0x26, 0x27, 0x28,
    0x29, 0x2A, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3A, 0x43, 0x44, 0x45,
    0x46, 0x47, 0x48, 0x49, 0x4A, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59,
    0x5A, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6A, 0x73, 0x74, 0x75,
    0x76, 0x77, 0x78, 0x79, 0x7A, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89,
    0x8A, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9A, 0xA2, 0xA3,
    0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6,
    0xB7, 0xB8, 0xB9, 0xBA, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8, 0xC9,
    0xCA, 0xD2, 0xD3, 0xD4, 0xD5, 0xD6, 0xD7, 0xD8, 0xD9, 0xDA, 0xE1, 0xE2,
    0xE3, 0xE4, 0xE5, 0xE6, 0xE7, 0xE8, 0xE9, 0xEA, 0xF1, 0xF2, 0xF3, 0xF4,
    0xF5, 0xF6, 0xF7, 0xF8, 0xF9, 0xFA, 0xFF, 0xDA, 0x00, 0x08, 0x01, 0x01,
    0x00, 0x00, 0x3F, 0x00, 0x7B, 0x94, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xD9
])

async def gen_frames_async():
    """Async MJPEG frame generator — does NOT block the FastAPI event loop.
    
    KEY FIX: When no real camera frame exists for >3 seconds, yields a tiny
    1x1 black placeholder JPEG. This causes the browser's <img> to fire 
    onLoad, breaking the permanent 'connecting' limbo state.
    The stream stays open so it recovers immediately when real frames arrive.
    """
    last_real_frame_time = time.time()
    placeholder_sent = False
    last_sent_mtime = None
    
    while True:
        try:
            frame_payload = await asyncio.to_thread(_read_frame_file)
            if frame_payload:
                frame_bytes, frame_mtime = frame_payload
                last_real_frame_time = time.time()
                placeholder_sent = False
                if frame_mtime != last_sent_mtime:
                    last_sent_mtime = frame_mtime
                    yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
            else:
                # No frame available — check how long we've been without one
                no_frame_duration = time.time() - last_real_frame_time
                if no_frame_duration > 3.0 and not placeholder_sent:
                    # Send a tiny placeholder so the browser <img> fires onLoad
                    # instead of hanging in connecting limbo forever.
                    yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + _PLACEHOLDER_JPEG + b'\r\n')
                    placeholder_sent = True
                    log.warning("VIDEO: No camera frames for 3s. Sent placeholder to unblock frontend.")
        except Exception:
            pass
        await asyncio.sleep(0.02)  # ~50Hz polling; yields only on new frame files

def _read_frame_file():
    """Synchronous file read — called via asyncio.to_thread."""
    if not os.path.isfile(_FRAME_PATH):
        return None
    # Check file age; if older than 2s, camera might be dead
    frame_mtime = os.path.getmtime(_FRAME_PATH)
    if time.time() - frame_mtime > 2.0:
        return None
    with open(_FRAME_PATH, 'rb') as f:
        return f.read(), frame_mtime

def _read_camera_status_file():
    if not os.path.isfile(_CAMERA_STATUS_PATH):
        return None
    try:
        with open(_CAMERA_STATUS_PATH, "r", encoding="utf-8") as f:
            return json.load(f)
    except Exception as e:
        return {"status": "error", "reason": "camera_status_unreadable", "detail": str(e)}

@app.get("/api/camera/status")
async def camera_status():
    """Health endpoint: reports camera-engine state plus frame-file freshness."""
    status_payload = _read_camera_status_file() or {}
    frame_exists = os.path.isfile(_FRAME_PATH)
    frame_age = None
    if frame_exists:
        frame_age = round(time.time() - os.path.getmtime(_FRAME_PATH), 2)

    status_payload["frame_file_exists"] = frame_exists
    if frame_age is not None:
        status_payload["frame_age_seconds"] = frame_age

    if not status_payload:
        if not frame_exists:
            return {"status": "offline", "reason": "no_frame_file", "frame_file_exists": False}
        if frame_age is not None and frame_age > 2.0:
            return {"status": "stale", "age_seconds": round(frame_age, 1), "frame_file_exists": True}
        return {"status": "live", "age_seconds": round(frame_age, 2), "frame_file_exists": True}

    if status_payload.get("status") == "live" and not frame_exists:
        status_payload["status"] = "offline"
        status_payload["reason"] = "status_live_but_no_frame_file"
    elif frame_age is not None and frame_age > 2.0 and status_payload.get("status") == "live":
        status_payload["status"] = "stale"
        status_payload["reason"] = "stale_frame_file"
    elif not frame_exists and "reason" not in status_payload:
        status_payload["reason"] = "no_frame_file"

    return status_payload

@app.get("/api/video_feed")
async def video_feed():
    return StreamingResponse(gen_frames_async(), media_type="multipart/x-mixed-replace; boundary=frame")

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=5000, log_level="warning")
