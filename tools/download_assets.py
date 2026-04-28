import os
import urllib.request
import logging

logging.basicConfig(level=logging.INFO)
log = logging.getLogger("YUVAAN.Downloader")

def download_assets():
    # 1. Prepare directories
    base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    assets_dir = os.path.join(base_dir, "assets", "cascades")
    models_dir = os.path.join(base_dir, "assets", "models")
    
    os.makedirs(assets_dir, exist_ok=True)
    os.makedirs(models_dir, exist_ok=True)

    # 2. Haar Cascades from OpenCV GitHub
    xml_url_base = "https://raw.githubusercontent.com/opencv/opencv/master/data/haarcascades/"
    xmls = [
        "haarcascade_fullbody.xml",
        "haarcascade_upperbody.xml",
        "haarcascade_frontalface_default.xml",
        "haarcascade_profileface.xml"
    ]

    print("\n[1/2] Downloading Haar Cascades...")
    for xml in xmls:
        target = os.path.join(assets_dir, xml)
        if not os.path.exists(target):
            print(f"  → Downloading {xml}...")
            try:
                urllib.request.urlretrieve(xml_url_base + xml, target)
                print(f"  [V] Saved to {target}")
            except Exception as e:
                print(f"  [!] Failed to download {xml}: {e}")
        else:
            print(f"  [V] {xml} already exists.")

    # 3. IMX500 Model (RPK)
    rpk_url = "https://github.com/raspberrypi/rpi-camera-assets/raw/master/models/imx500_mobilenet_v2_ssd_10_256_256_1.rpk"
    json_url = "https://github.com/raspberrypi/rpi-camera-assets/raw/master/models/imx500_mobilenet_v2_ssd_10_256_256_1.json"
    
    print("\n[2/2] Downloading IMX500 AI Models...")
    for url, ext in [(rpk_url, ".rpk"), (json_url, ".json")]:
        name = os.path.basename(url)
        target = os.path.join(models_dir, name)
        if not os.path.exists(target):
            print(f"  → Downloading {name}...")
            try:
                urllib.request.urlretrieve(url, target)
                print(f"  [V] Saved to {target}")
            except Exception as e:
                print(f"  [!] Failed to download {name}: {e}")
        else:
            print(f"  [V] {name} already exists.")

    print("\n" + "="*50)
    print("  DOWNLOAD COMPLETE")
    print("  The rover now has its assets locally in /assets/")
    print("="*50 + "\n")

if __name__ == "__main__":
    download_assets()
