"""
YUVAAN Object Detector — perception/object_detect.py
Focus: Native IMX500 Hardware Detection.
This file remains as a placeholder for non-hardware specific detection logic.
"""

class ObjectDetector:
    """
    Hardware-first detector.
    Actual inference is done on the IMX500 chip via picamera2.
    """
    def __init__(self):
        pass

    def detect(self, frame):
        # Detections are now handled natively in drivers/camera.py
        # and passed through perception/camera_engine.py
        return []

def get_detector():
    return ObjectDetector()
