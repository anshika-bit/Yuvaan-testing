"""
YUVAAN Camera Driver — drivers/camera.py
Hardware: Raspberry Pi Camera (libcamera / USB fallback)

Low-level camera access. Provides raw JPEG frames.
Higher-level processing (object detection, AI) lives in perception/camera_engine.py.

Usage:
    from drivers.camera import CameraDriver
    cam = CameraDriver()
    frame = cam.get_frame()  # Returns JPEG bytes or None
"""

import cv2
import threading
import time


class CameraDriver:
    """
    Low-level camera access driver.
    Opens a video source, captures raw frames on a background thread.
    """

    def __init__(self, source: int = 0, width: int = 640, height: int = 480):
        self.source = source
        self._cap = cv2.VideoCapture(source)
        if not self._cap.isOpened():
            print(f"[WARN] CameraDriver: Could not open video source {source}.")

        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

        self._latest_frame = None
        self._lock = threading.Lock()
        self._running = True

        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()

    def _capture_loop(self):
        while self._running:
            ret, frame = self._cap.read()
            if ret:
                with self._lock:
                    self._latest_frame = frame
            time.sleep(0.01)  # ~100 fps cap to avoid busy-loop

    def get_frame(self) -> bytes | None:
        """Returns the latest frame encoded as JPEG bytes, or None if unavailable."""
        with self._lock:
            if self._latest_frame is None:
                return None
            ret, jpeg = cv2.imencode('.jpg', self._latest_frame)
            return jpeg.tobytes() if ret else None

    def get_raw_frame(self):
        """Returns the latest raw numpy BGR frame, for use in AI pipelines."""
        with self._lock:
            return self._latest_frame.copy() if self._latest_frame is not None else None

    def stop(self):
        self._running = False
        self._cap.release()


# Singleton accessor
_driver: CameraDriver | None = None


def get_camera_driver() -> CameraDriver:
    global _driver
    if _driver is None:
        _driver = CameraDriver()
    return _driver
