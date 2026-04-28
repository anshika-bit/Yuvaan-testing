import time
import logging
import os
import json
import cv2
import numpy as np
import requests
from drivers.camera import get_camera_driver

log = logging.getLogger("YUVAAN.CameraEngine")

_FRAME_PATH = "/tmp/yuvaan_latest_frame.jpg"
_FRAME_TMP  = "/tmp/yuvaan_latest_frame.tmp.jpg"
_STATUS_PATH = "/tmp/yuvaan_camera_status.json"
_STATUS_TMP = "/tmp/yuvaan_camera_status.tmp.json"

class CameraEngine:
    """
    Relays IMX500 frames and hardware detections to the GCS.
    Draws bounding boxes for visual feedback.
    """
    def __init__(self, api_base="http://127.0.0.1:5000"):
        self.api_base = api_base
        self.driver = get_camera_driver()
        self.is_running = True
        self._last_status = None
        
        # Ensure tmp directory exists
        os.makedirs("/tmp", exist_ok=True)
        self._write_status("starting", reason="engine_initialized")

    def _write_frame(self, jpeg_bytes):
        try:
            with open(_FRAME_TMP, 'wb') as f:
                f.write(jpeg_bytes)
            os.replace(_FRAME_TMP, _FRAME_PATH)
            return True
        except Exception as e:
            log.error(f"Camera Engine: Frame write failed: {e}")
            self._write_status("error", reason="frame_write_failed", detail=str(e))
            return False

    def _write_status(self, status, **extra):
        payload = {"status": status, "updated_at": time.time(), **extra}
        if payload == self._last_status:
            return
        self._last_status = payload.copy()
        try:
            with open(_STATUS_TMP, "w", encoding="utf-8") as f:
                json.dump(payload, f)
            os.replace(_STATUS_TMP, _STATUS_PATH)
        except Exception as e:
            log.warning(f"Camera Engine: Status write failed: {e}")

    def run(self):
        log.info("Camera Engine: Active (IMX500 Hardware Relay)")
        wait_log_at = 0.0
        
        while self.is_running:
            try:
                frame, detections = self.driver.get_data()
                
                if frame is None:
                    driver_error = getattr(self.driver, "last_error", None)
                    if driver_error:
                        self._write_status("error", reason="camera_driver_error", detail=driver_error)
                    else:
                        self._write_status("starting", reason="waiting_for_first_frame")
                        if time.time() - wait_log_at > 5.0:
                            log.info("Camera Engine: Waiting for first frame from IMX500...")
                            wait_log_at = time.time()
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
                ok, encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                if not ok:
                    log.error("Camera Engine: JPEG encode failed")
                    self._write_status("error", reason="jpeg_encode_failed")
                    time.sleep(0.1)
                    continue
                if self._write_frame(encoded.tobytes()):
                    self._write_status("live", detections=len(detections), frame_path=_FRAME_PATH)
                
                # Update GCS detections list (throttled)
                try:
                    requests.post(f"{self.api_base}/api/detections/update", 
                                  json={"detections": detections}, timeout=0.05)
                except Exception:
                    pass
            except Exception as e:
                log.exception(f"Camera Engine loop failed: {e}")
                self._write_status("error", reason="camera_engine_exception", detail=str(e))
                time.sleep(0.25)
            
            time.sleep(0.033) # ~30fps

    def stop(self):
        self.is_running = False
        self._write_status("stopped", reason="engine_stopped")

_engine = None
def get_camera():
    global _engine
    if _engine is None:
        _engine = CameraEngine()
    return _engine
