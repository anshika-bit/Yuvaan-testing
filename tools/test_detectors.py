import sys
import os
import cv2
import logging

# Add project root to path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

print(f"--- RUNNING DIAGNOSTIC FROM: {os.path.abspath(__file__)} ---")

from perception.object_detect import get_detector
from drivers.camera import get_camera_driver

logging.basicConfig(level=logging.INFO)
log = logging.getLogger("YUVAAN.Test")

def test_detectors():
    print("\n" + "="*50)
    print("  YUVAAN PERCEPTION DIAGNOSTIC")
    print("="*50)

    # 1. Check Hardware AI
    print("\n[1/3] Testing Hardware AI (IMX500)...")
    try:
        cam = get_camera_driver()
        local_model = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "assets", "models", "imx500_mobilenet_v2_ssd_10_256_256_1.rpk")
        if cam._initialized:
            print("  [V] Camera Initialized.")
            if cam._ai_model or os.path.exists(local_model):
                print(f"  [V] AI Model Found (Local or System).")
            else:
                print("  [!] WARNING: AI Model NOT Found. Run tools/download_assets.py")
        else:
            print("  [!] ERROR: Camera Driver not initialized.")
    except Exception as e:
        print(f"  [!] ERROR: {e}")

    # 2. Check Software Detectors
    print("\n[2/3] Testing Software Detectors (HOG/HAAR)...")
    try:
        det = get_detector()
        if det.hog:
            print("  [V] HOG Descriptor: Ready.")
        loaded_count = len(det.cascades)
        if loaded_count > 0:
            print(f"  [V] Haar Cascades: {loaded_count} loaded.")
        else:
            print("  [!] ERROR: No Haar Cascades loaded. Run tools/download_assets.py")
    except Exception as e:
        print(f"  [!] ERROR: {e}")

    # 3. Test OpenCV Data Path
    print("\n[3/3] System Environment...")
    print(f"  Python Version: {sys.version.split()[0]}")
    print(f"  OpenCV Version: {cv2.__version__}")
    if hasattr(cv2, 'data'):
        print(f"  OpenCV Data: {cv2.data.haarcascades}")
    else:
        print("  OpenCV Data: [!] Attribute 'cv2.data' missing.")

    print("\n" + "="*50)
    print("  DIAGNOSTIC COMPLETE")
    print("="*50 + "\n")

if __name__ == "__main__":
    test_detectors()
