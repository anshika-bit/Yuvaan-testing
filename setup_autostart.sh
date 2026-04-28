#!/bin/bash

# YUVAAN ROVER — Auto-start Setup Script
# This script creates a systemd service to run the rover backend on boot.

SERVICE_FILE="/etc/systemd/system/yuvaan-rover.service"
USER_NAME=$(whoami)
WORKING_DIR=$(pwd)

echo "Creating systemd service at $SERVICE_FILE..."

sudo bash -c "cat <<EOF > $SERVICE_FILE
[Unit]
Description=Yuvaan Rover Master Service
After=network.target

[Service]
ExecStart=/usr/bin/python3 $WORKING_DIR/main.py
WorkingDirectory=$WORKING_DIR
StandardOutput=inherit
StandardError=inherit
Restart=always
User=$USER_NAME

[Install]
WantedBy=multi-user.target
EOF"

echo "Reloading systemd daemon..."
sudo systemctl daemon-reload

echo "Enabling Yuvaan Rover service..."
sudo systemctl enable yuvaan-rover.service

echo "------------------------------------------------"
echo "SETUP COMPLETE!"
echo "The rover backend will now start automatically on boot from $WORKING_DIR"
echo "To start it now: sudo systemctl start yuvaan-rover"
echo "To check status: sudo systemctl status yuvaan-rover"
echo "To stop: sudo systemctl stop yuvaan-rover"
echo "------------------------------------------------"
