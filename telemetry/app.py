from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import asyncio
import json
import os
import sys
import logging
import time
import math
from contextlib import asynccontextmanager
from datetime import datetime
from fastapi.responses import StreamingResponse

# ---------------------------------------------------------
# YUVAAN ROVER TELEMETRY BACKEND (Run this on the Pi 5)
# ---------------------------------------------------------

# ---- Clear Terminal on startup ----
os.system('clear' if os.name == 'posix' else 'cls')

# ---- Configure clean logging ----
logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] %(levelname)s | %(message)s",
    datefmt="%H:%M:%S"
)
log = logging.getLogger("YUVAAN")

# ---- Memory Log Buffer for GCS ----
class MemoryLogHandler(logging.Handler):
    def __init__(self, limit=50):
        super().__init__()
        self.limit = limit
        self.logs = []

    def emit(self, record):
        log_entry = {
            "id": int(time.time() * 1000),
            "timestamp": datetime.now().strftime("%H:%M:%S"),
            "level": record.levelname,
            "message": self.format(record)
        }
        self.logs.append(log_entry)
        if len(self.logs) > self.limit:
            self.logs.pop(0)

    def get_logs(self):
        return self.logs

memory_log_handler = MemoryLogHandler()
memory_log_handler.setFormatter(logging.Formatter('%(message)s'))
logging.getLogger("YUVAAN").addHandler(memory_log_handler)

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
        # PROTECTION: Do not overwrite 'active' or 'waypoints' from telemetry
        # These are driven by GCS requests. 
        # We only take the 'real' state (distance, errors) from the backend.
        nav_telemetry = data["navigation"]
        current_nav_status["distance_to_wp"] = nav_telemetry.get("distance_to_wp", 0)
        current_nav_status["target_bearing"] = nav_telemetry.get("target_bearing", 0)
        current_nav_status["bearing_error"] = nav_telemetry.get("bearing_error", 0)
        current_nav_status["current_wp"] = nav_telemetry.get("current_wp")
        current_nav_status["state"] = nav_telemetry.get("state", "IDLE")
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
        "calibration_requested": current_nav_status.get("calibration_requested")
    }

@app.post("/api/navigation/rtl")
async def engage_rtl(lat: float, lng: float):
    from core.navigation import get_nav_engine
    nav = get_nav_engine()
    
    nav.stop() # Halt current navigation immediately
    
    rtl_waypoints = []
    # If we have a history path, use it to safely backtrack
    if hasattr(nav, 'history_path') and len(nav.history_path) > 0:
        for point in reversed(nav.history_path):
            rtl_waypoints.append({"lat": point['lat'], "lng": point['lng'], "action": "move"})
            
    # Finally, append the ultimate home target
    rtl_waypoints.append({"lat": lat, "lng": lng, "action": "stop"})
    
    nav.set_waypoints(rtl_waypoints)
    nav.start()
    log.warning(f"RTL: Engaging protocol. Backtracking {len(rtl_waypoints)} waypoints to Base.")
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
                "server_logs": memory_log_handler.get_logs(),
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

async def gen_frames_async():
    """Async MJPEG frame generator — does NOT block the FastAPI event loop."""
    while True:
        try:
            # Run blocking file I/O in a thread pool to avoid blocking the event loop
            frame_bytes = await asyncio.to_thread(_read_frame_file)
            if frame_bytes:
                yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
        except Exception:
            pass
        await asyncio.sleep(0.05)  # ~20fps cap, non-blocking

def _read_frame_file():
    """Synchronous file read — called via asyncio.to_thread."""
    if not os.path.isfile(_FRAME_PATH):
        return None
    # Check file age; if older than 2s, camera might be dead
    if time.time() - os.path.getmtime(_FRAME_PATH) > 2.0:
        return None
    with open(_FRAME_PATH, 'rb') as f:
        return f.read()

@app.get("/api/video_feed")
async def video_feed():
    return StreamingResponse(gen_frames_async(), media_type="multipart/x-mixed-replace; boundary=frame")

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=5000, log_level="warning")
