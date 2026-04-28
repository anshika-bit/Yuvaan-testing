"""
YUVAAN Rover — IMX500 AI Camera Driver (drivers/camera.py)

Native Raspberry Pi AI Camera (Sony IMX500) driver using the REAL picamera2 API.
Uses `picamera2.devices.IMX500` for hardware-accelerated inference.

Model: imx500_network_ssd_mobilenetv2_fpnlite_320x320_pp.rpk
       (pre-packaged with `sudo apt install imx500-all`)

Provides:
  - get_data() -> (numpy_frame, detections_list)
  - Detections are filtered to COCO category 0 (person/human) only.
"""

import threading
import time
import logging
import os
import numpy as np

log = logging.getLogger("YUVAAN.Camera")

# COCO label index for "person"
COCO_PERSON_ID = 0
DETECTION_THRESHOLD = 0.55

# Default model path (installed by `sudo apt install imx500-all`)
DEFAULT_MODEL_PATH = "/usr/share/imx500-models/imx500_network_ssd_mobilenetv2_fpnlite_320x320_pp.rpk"


class CameraDriver:
    """
    Native Raspberry Pi AI Camera (IMX500) Driver.
    Uses picamera2.devices.IMX500 for hardware-accelerated object detection.
    """

    def __init__(self, model_path=None, width=640, height=480):
        self.width = width
        self.height = height
        self.model_path = model_path or DEFAULT_MODEL_PATH
        self._latest_frame = None
        self._latest_dets = []
        self._lock = threading.Lock()
        self._running = True
        self._picam = None
        self._imx500 = None
        self._intrinsics = None

        self._init_hardware()

        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()

    def _init_hardware(self):
        """Initialize the IMX500 sensor and picamera2 using the REAL API."""
        try:
            from picamera2 import Picamera2
            from picamera2.devices import IMX500
            from picamera2.devices.imx500 import NetworkIntrinsics

            if not os.path.exists(self.model_path):
                log.error(f"IMX500 model not found at {self.model_path}!")
                log.error("Install with: sudo apt install imx500-all")
                raise FileNotFoundError(f"IMX500 Model Missing: {self.model_path}")

            # 1. Create IMX500 device handle (loads firmware to the NPU)
            #    This MUST be done BEFORE creating the Picamera2 instance.
            self._imx500 = IMX500(self.model_path)
            self._intrinsics = self._imx500.network_intrinsics
            if not self._intrinsics:
                self._intrinsics = NetworkIntrinsics()
                self._intrinsics.task = "object detection"
            self._intrinsics.update_with_defaults()

            # 2. Create and configure the camera
            self._picam = Picamera2(self._imx500.camera_num)
            config = self._picam.create_preview_configuration(
                main={"size": (self.width, self.height), "format": "BGR888"},
                controls={"FrameRate": 30},
                buffer_count=12
            )

            self._imx500.show_network_fw_progress_bar()
            self._picam.start(config)

            log.info("IMX500 Hardware AI Camera: INITIALIZED & ACTIVE")
            log.info(f"  Model: {os.path.basename(self.model_path)}")
            log.info(f"  Resolution: {self.width}x{self.height} @ 30fps")

        except ImportError as e:
            log.error(f"IMX500 Import Failed (not on Pi?): {e}")
            self._picam = None
        except Exception as e:
            log.error(f"IMX500 Initialization Failed: {e}")
            self._picam = None

    def _parse_detections(self, metadata):
        """
        Parse raw NPU output tensors into detection objects.
        Follows the official picamera2 IMX500 example pattern.
        """
        if self._imx500 is None:
            return []

        try:
            np_outputs = self._imx500.get_outputs(metadata, add_batch=True)
            if np_outputs is None:
                return []

            input_w, input_h = self._imx500.get_input_size()

            # Standard SSD MobileNet output format: [boxes, scores, classes]
            intrinsics = self._intrinsics

            if hasattr(intrinsics, 'postprocess') and intrinsics.postprocess == "nanodet":
                from picamera2.devices.imx500.postprocess import (
                    postprocess_nanodet_detection, scale_boxes
                )
                boxes, scores, classes = postprocess_nanodet_detection(
                    outputs=np_outputs[0],
                    conf=DETECTION_THRESHOLD,
                    iou_thres=0.65,
                    max_out_dets=10
                )[0]
                boxes = scale_boxes(boxes, 1, 1, input_h, input_w, False, False)
            else:
                boxes, scores, classes = np_outputs[0][0], np_outputs[1][0], np_outputs[2][0]
                if intrinsics.bbox_normalization:
                    boxes = boxes / input_h
                if intrinsics.bbox_order == "xy":
                    boxes = boxes[:, [1, 0, 3, 2]]

            # Filter for humans (COCO person = category 0) above threshold
            human_dets = []
            for box, score, category in zip(boxes, scores, classes):
                if int(category) == COCO_PERSON_ID and score > DETECTION_THRESHOLD:
                    # Convert inference coords to pixel coords on the main stream
                    pixel_box = self._imx500.convert_inference_coords(
                        box, metadata, self._picam
                    )
                    x, y, w, h = [int(c) for c in pixel_box]
                    human_dets.append({
                        'label': 'HUMAN',
                        'box': [x, y, w, h],
                        'conf': float(score)
                    })

            return human_dets

        except Exception as e:
            log.debug(f"Detection parse error: {e}")
            return []

    def _capture_loop(self):
        """Continuous capture thread — grabs frames + detections at ~30fps."""
        while self._running:
            if self._picam is None:
                time.sleep(1)
                continue

            try:
                # Capture the frame (BGR numpy array)
                frame = self._picam.capture_array()

                # Capture metadata (contains NPU inference outputs)
                metadata = self._picam.capture_metadata()

                # Parse detections from metadata
                detections = self._parse_detections(metadata)

                with self._lock:
                    self._latest_frame = frame
                    self._latest_dets = detections

            except Exception as e:
                log.debug(f"Camera Loop Error: {e}")
                time.sleep(0.01)

    def get_data(self):
        """Returns (frame, detections) — thread-safe snapshot."""
        with self._lock:
            if self._latest_frame is None:
                return None, []
            return self._latest_frame.copy(), self._latest_dets.copy()

    def stop(self):
        """Shutdown the camera driver cleanly."""
        self._running = False
        if self._picam:
            try:
                self._picam.stop()
                self._picam.close()
            except Exception:
                pass


# ---- Singleton ----
_driver = None

def get_camera_driver():
    global _driver
    if _driver is None:
        _driver = CameraDriver()
    return _driver
