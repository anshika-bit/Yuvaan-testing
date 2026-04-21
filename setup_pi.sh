#!/bin/bash
# ============================================================
#  YUVAAN ROVER — Raspberry Pi Setup Script
#  Run ONCE on a fresh Pi to prepare the rover backend.
# ============================================================

set -e   # exit immediately on error

echo "============================================================"
echo "  YUVAAN ROVER — Pi Environment Setup"
echo "  $(date)"
echo "============================================================"

# 1. System packages
echo ""
echo "[1/5] Updating system packages..."
sudo apt-get update -y
sudo apt-get install -y python3-venv python3-pip i2c-tools

# 2. Enable I2C (non-interactive)
echo ""
echo "[2/5] Enabling I2C interface..."
sudo raspi-config nonint do_i2c 0
echo "      I2C enabled. Verify with: sudo i2cdetect -y 1"

# 3. Create virtual environment
echo ""
echo "[3/5] Creating Python virtual environment at ~/rover_env..."
python3 -m venv ~/rover_env
source ~/rover_env/bin/activate

# 4. Install Python dependencies
echo ""
echo "[4/5] Installing project Python dependencies..."
pip install --upgrade pip
pip install -r "$(dirname "$0")/requirements.txt"

# 5. Permissions for I2C
echo ""
echo "[5/5] Adding user to i2c group (for hardware access without sudo)..."
sudo usermod -aG i2c "$USER"

echo ""
echo "============================================================"
echo "  SETUP COMPLETE!"
echo "============================================================"
echo ""
echo "  HOW TO START THE ROVER BACKEND:"
echo ""
echo "  Terminal 1 — FastAPI WebSocket Server:"
echo "    source ~/rover_env/bin/activate"
echo "    python3 telemetry/app.py"
echo ""
echo "  Terminal 2 — Real IMU Feed (MPU9250 → FastAPI):"
echo "    source ~/rover_env/bin/activate"
echo "    python3 telemetry/imu_feed.py"
echo ""
echo "  GCS URL (on Windows machine):"
echo "    Settings → Network → IP: $(hostname -I | awk '{print $1}')  Port: 5000"
echo "============================================================"
