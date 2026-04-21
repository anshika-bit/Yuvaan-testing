import threading
import time
from drivers.camera import get_camera_driver
from perception.object_detect import get_detector

class CameraEngine:
    def __init__(self):
        self.driver = get_camera_driver()
        self.latest_frame = None
        self.lock = threading.Lock()
        self.is_running = True
        self.detection_results = []
        
        # Start capture thread
        self.thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.thread.start()

    def _capture_loop(self):
        detector = get_detector()
        while self.is_running:
            # Use driver to get raw frame for AI
            frame = self.driver.get_raw_frame()
            if frame is not None:
                # Run AI Inference
                detections = detector.detect(frame)
                
                with self.lock:
                    self.latest_frame = frame
                    self.detection_results = detections
            time.sleep(0.01)

    def get_frame(self):
        # Driver provides encoded JPEG directly
        return self.driver.get_frame()

    def stop(self):
        self.is_running = False
        self.driver.stop()

# Global engine instance
camera_engine = None

def get_camera():
    global camera_engine
    if camera_engine is None:
        camera_engine = CameraEngine()
    return camera_engine
