import time
import logging
import os
import cv2
import numpy as np
import requests
from drivers.camera import get_camera_driver

log = logging.getLogger("YUVAAN.CameraEngine")

_FRAME_PATH = "/tmp/yuvaan_latest_frame.jpg"
_FRAME_TMP  = "/tmp/yuvaan_latest_frame.tmp.jpg"

class CameraEngine:
    """
    Relays IMX500 frames and hardware detections to the GCS.
    Draws bounding boxes for visual feedback.
    """
    def __init__(self, api_base="http://127.0.0.1:5000"):
        self.api_base = api_base
        self.driver = get_camera_driver()
        self.is_running = True
        
        # Ensure tmp directory exists
        os.makedirs("/tmp", exist_ok=True)

    def _write_frame(self, jpeg_bytes):
        try:
            with open(_FRAME_TMP, 'wb') as f:
                f.write(jpeg_bytes)
            os.replace(_FRAME_TMP, _FRAME_PATH)
        except Exception: pass

    def run(self):
        log.info("Camera Engine: Active (IMX500 Hardware Relay)")
        
        while self.is_running:
            frame, detections = self.driver.get_data()
            
            if frame is None:
                time.sleep(0.1)
                continue
            
            # --- DRAW OVERLAYS (ONLY HUMANS) ---
            for det in detections:
                x, y, w, h = det['box']
                # Draw Blue Bounding Box (Mars Rover Theme)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 100, 0), 2)
                # Label
                cv2.putText(frame, f"HUMAN {int(det['conf']*100)}%", (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 100, 0), 2)

            # --- HEARTBEAT INDICATOR ---
            cv2.circle(frame, (15, 15), 5, (0, 255, 0), -1)

            # Encode and Relay
            _, encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            self._write_frame(encoded.tobytes())
            
            # Update GCS detections list (throttled)
            try:
                requests.post(f"{self.api_base}/api/detections/update", 
                              json={"detections": detections}, timeout=0.05)
            except: pass
            
            time.sleep(0.033) # ~30fps

    def stop(self):
        self.is_running = False

_engine = None
def get_camera():
    global _engine
    if _engine is None:
        _engine = CameraEngine()
    return _engine
