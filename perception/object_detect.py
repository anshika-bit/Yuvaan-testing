import cv2
import numpy as np

class ObjectDetector:
    def __init__(self):
        # Placeholder for AI Camera Models (e.g., IMX500 TFLite models)
        # Using a standard OpenCV Haar Cascade for immediate testing
        self.face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcas_frontalface_default.xml')
        
    def detect(self, frame):
        """
        Processes a frame and returns a list of detected objects with coordinates.
        Format: [{'label': 'person', 'box': [x, y, w, h], 'conf': 0.9}]
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detections = []
        
        # Simple Face Detection as a placeholder for AI features
        faces = self.face_cascade.detectMultiScale(gray, 1.1, 4)
        for (x, y, w, h) in faces:
            detections.append({
                'label': 'HUMAN_DETECTED',
                'box': [int(x), int(y), int(w), int(h)],
                'conf': 0.95
            })
            
        # Here we would add advanced IMX500 logic 
        # (parsing output of rpicam-apps post-processing)
        
        return detections

# Global detector instance
_detector = None

def get_detector():
    global _detector
    if _detector is None:
        _detector = ObjectDetector()
    return _detector
