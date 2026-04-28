#!/bin/bash
# ============================================================
#  YUVAAN ROVER — Raspberry Pi Setup Script
#  Run ONCE on a fresh Pi to prepare the rover backend.
#  Compatible with Raspberry Pi 5 + AI Camera (IMX500)
# ============================================================

set -e   # exit immediately on error

echo "============================================================"
echo "  YUVAAN ROVER — Pi Environment Setup"
echo "  $(date)"
echo "============================================================"

# Helper: wait for apt lock to be freed (prevents 'Unable to acquire lock')
wait_for_apt() {
    echo "Checking for package manager lock..."
    while fuser /var/lib/dpkg/lock-frontend >/dev/null 2>&1 ; do
        echo "  → Another package manager is running. Waiting 5s..."
        sleep 5
    done
}

# ---- [1/6] System Packages ----
echo ""
echo "[1/6] Updating system packages..."
wait_for_apt
sudo apt-get update -y
wait_for_apt
sudo apt-get install -y \
    python3-venv \
    python3-pip \
    i2c-tools \
    python3-libcamera \
    python3-kms++ \
    libcap-dev

# ---- [2/6] Camera & OpenCV via APT (required for IMX500 / picamera2) ----
echo ""
echo "[2/6] Installing picamera2, OpenCV, and IMX500 AI models..."
echo "      (These MUST be installed via apt, not pip)"
wait_for_apt
sudo apt-get install -y \
    python3-picamera2 \
    python3-opencv \
    imx500-models

# ---- [3/6] Enable I2C ----
echo ""
echo "[3/6] Enabling I2C interface..."
sudo raspi-config nonint do_i2c 0
echo "      I2C enabled. Verify with: sudo i2cdetect -y 1"

# ---- [4/6] Enable Camera (libcamera stack) ----
echo ""
echo "[4/6] Ensuring libcamera is enabled..."
# Ensure camera auto-detect is ON (Raspberry Pi OS Bookworm default)
if ! grep -q "camera_auto_detect=1" /boot/firmware/config.txt 2>/dev/null; then
    echo "camera_auto_detect=1" | sudo tee -a /boot/firmware/config.txt
    echo "      Written camera_auto_detect=1 to /boot/firmware/config.txt"
else
    echo "      camera_auto_detect already set."
fi

# ---- [5/6] Python Virtual Environment ----
echo ""
echo "[5/6] Creating Python virtual environment at ~/rover_env..."
# --system-site-packages lets the venv access apt-installed picamera2 & opencv
python3 -m venv --system-site-packages ~/rover_env
source ~/rover_env/bin/activate

echo "      Installing pip dependencies..."
pip install --upgrade pip
pip install -r "$(dirname "$0")/requirements.txt"

# ---- [6/6] Permissions ----
echo ""
echo "[6/6] Adding user to i2c and video groups..."
sudo usermod -aG i2c,video "$USER"

# ---- [7/7] Auto-Start Service ----
echo ""
echo "[7/7] Installing Yuvaan Rover auto-start service..."
SERVICE_FILE="yuvaan.service"
if [ -f "$SERVICE_FILE" ]; then
    # Update the service file with the current user and directory
    sed -i "s|User=pi|User=$USER|g" "$SERVICE_FILE"
    sed -i "s|/home/pi/Yuvaan-testing|$(pwd)|g" "$SERVICE_FILE"
    
    sudo cp "$SERVICE_FILE" /etc/systemd/system/yuvaan.service
    sudo systemctl daemon-reload
    sudo systemctl enable yuvaan.service
    echo "      Service installed and enabled. It will start on next boot."
else
    echo "      WARNING: yuvaan.service not found in $(pwd). Skipping auto-start setup."
fi

echo ""
echo "============================================================"
echo "  SETUP COMPLETE!"
echo "============================================================"
echo ""
echo "  HOW TO START THE ROVER BACKEND:"
echo ""
echo "  Activate venv first:"
echo "    source ~/rover_env/bin/activate"
echo ""
echo "  Then launch everything with:"
echo "    python3 main.py"
echo ""
echo "  Endpoints that will be live:"
echo "    Health:  http://$(hostname -I | awk '{print $1}'):5000/health"
echo "    Video:   http://$(hostname -I | awk '{print $1}'):5000/api/video_feed"
echo "    WS IMU:  ws://$(hostname -I | awk '{print $1}'):5000/ws/imu"
echo ""
echo "  GCS Settings → Network → IP: $(hostname -I | awk '{print $1}')  Port: 5000"
echo "============================================================"
echo ""
echo "  NOTE: Reboot once before first run so camera and I2C"
echo "        group permissions take effect."
echo "    sudo reboot"
echo "============================================================"
