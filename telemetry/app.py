from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import asyncio
import json
import os
import sys
import logging
import time
from datetime import datetime

# ---------------------------------------------------------
# YUVAAN ROVER TELEMETRY BACKEND (Run this on the Pi 5)
# ---------------------------------------------------------

# ---- Clear Terminal on startup ----
os.system('clear' if os.name == 'posix' else 'cls')

# ---- Configure clean logging (no duplicate uvicorn noise) ----
logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] %(levelname)s | %(message)s",
    datefmt="%H:%M:%S"
)
log = logging.getLogger("YUVAAN")

# Suppress verbose uvicorn access logs
logging.getLogger("uvicorn.access").setLevel(logging.WARNING)

print("=" * 55)
print("  YUVAAN ROVER — TELEMETRY BACKEND")
print(f"  Started at: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
print("=" * 55)
print("  WebSocket → ws://<PI_IP>:5000/ws/imu")
print("  IMU Feed  → POST /api/imu/update")
print("  Calibrate → POST /api/calibrate")
print("=" * 55)
print()

app = FastAPI(title="Yuvaan Rover Telemetry", version="2.0")

# Allow connections from the Windows GCS React App over LAN / Tailscale
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# ---- Global Live Telemetry Store ----
current_imu_data = {
    "pitch": 0.0,
    "roll": 0.0,
    "yaw": 0.0
}

# Track connected clients for logging
connected_clients: list[WebSocket] = []


class IMUInput(BaseModel):
    pitch: float
    roll: float
    yaw: float


@app.post("/api/imu/update")
async def update_imu_data(data: IMUInput):
    """IMU feed script calls this to push real Roll/Pitch/Yaw values."""
    global current_imu_data
    current_imu_data["pitch"] = round(data.pitch, 2)
    current_imu_data["roll"]  = round(data.roll,  2)
    current_imu_data["yaw"]   = round(data.yaw,   2)
    return {"status": "ok"}


@app.websocket("/ws/imu")
async def websocket_imu_endpoint(websocket: WebSocket):
    await websocket.accept()
    connected_clients.append(websocket)
    client_host = websocket.client.host if websocket.client else "unknown"
    log.info(f"GCS CONNECTED   ← {client_host}  (clients: {len(connected_clients)})")
    try:
        while True:
            # Push live data at 10 Hz — include server timestamp for latency calc
            payload = {
                **current_imu_data,
                "server_ts": int(time.time() * 1000)  # Unix ms
            }
            await websocket.send_text(json.dumps(payload))
            await asyncio.sleep(0.1)
    except WebSocketDisconnect:
        connected_clients.remove(websocket)
        log.warning(f"GCS DISCONNECTED ← {client_host}  (clients: {len(connected_clients)})")
    except Exception as e:
        if websocket in connected_clients:
            connected_clients.remove(websocket)
        log.error(f"Socket error: {e}")


@app.post("/api/calibrate")
async def calibrate_imu():
    """GCS Settings → Calibration triggers this to zero IMU offsets."""
    global current_imu_data
    current_imu_data = {"pitch": 0.0, "roll": 0.0, "yaw": 0.0}
    log.info("IMU CALIBRATED — offsets zeroed by GCS request.")
    return {"status": "ok", "message": "IMU offsets zeroed."}


@app.get("/health")
async def health():
    """Quick health check for the GCS or monitoring tool."""
    return {
        "status": "online",
        "clients": len(connected_clients),
        "imu": current_imu_data
    }


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        app,
        host="0.0.0.0",
        port=5000,
        log_level="warning"   # suppress raw uvicorn HTTP logs — we use our own
    )
