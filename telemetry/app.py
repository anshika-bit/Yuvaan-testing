from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import asyncio
import json

# ---------------------------------------------------------
# YUVAAN ROVER TELEMTRY BACKEND (Run this on the Pi 5)
# ---------------------------------------------------------

app = FastAPI()

# Allow connections from your Windows GCS React App over the local LAN
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Global dictionary to hold the latest IMU values
current_imu_data = {
    "pitch": 0.0,
    "roll": 0.0,
    "yaw": 0.0
}

class IMUInput(BaseModel):
    pitch: float
    roll: float
    yaw: float

@app.post("/api/imu/update")
async def update_imu_data(data: IMUInput):
    """
    Endpoint for your real IMU script to feed the actual roll, pitch, yaw
    """
    global current_imu_data
    current_imu_data["pitch"] = data.pitch
    current_imu_data["roll"] = data.roll
    current_imu_data["yaw"] = data.yaw
    return {"status": "success", "message": "IMU data updated"}

@app.websocket("/ws/imu")
async def websocket_imu_endpoint(websocket: WebSocket):
    await websocket.accept()
    print("Windows GCS Client connected to live IMU stream!")
    try:
        while True:
            # Broadcast the real data continuously at 10Hz
            await websocket.send_text(json.dumps(current_imu_data))
            await asyncio.sleep(0.1) 
            
    except WebSocketDisconnect:
        print("Windows GCS Client disconnected from IMU stream.")

@app.post("/api/calibrate")
async def calibrate_imu():
    print("Hardware Calibration Override Triggered by Windows GCS!")
    # Optional: zero the IMU state on calibrate
    global current_imu_data
    current_imu_data = {"pitch": 0.0, "roll": 0.0, "yaw": 0.0}
    return {"status": "success", "message": "IMU Offsets Zeroed."}

if __name__ == "__main__":
    import uvicorn
    print("Rover Backend Link Initiated.")
    uvicorn.run(app, host="0.0.0.0", port=5000)
