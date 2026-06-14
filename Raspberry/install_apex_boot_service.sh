#!/usr/bin/env bash
set -e

# Run this script from the same folder that contains main_startup.py.
# Example:
#   cd /home/apex-rover/ApexRover_Repo/Raspberry
#   bash install_apex_boot_service.sh

SERVICE_NAME="apex-rover.service"
SERVICE_USER="${SUDO_USER:-$(whoami)}"
WORKDIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_BIN="$(command -v python3)"

if [ ! -f "$WORKDIR/main_startup.py" ]; then
  echo "ERROR: main_startup.py was not found in: $WORKDIR"
  echo "Put this script inside the Raspberry folder and run it again."
  exit 1
fi

if [ ! -f "$WORKDIR/raspberry_voice_auto_service.py" ]; then
  echo "ERROR: raspberry_voice_auto_service.py was not found in: $WORKDIR"
  exit 1
fi

echo "Installing required packages..."
sudo apt update
sudo apt install -y python3-flask python3-websocket espeak-ng ffmpeg mpg123

echo "Creating systemd service for user: $SERVICE_USER"
sudo tee "/etc/systemd/system/$SERVICE_NAME" >/dev/null <<SERVICE
[Unit]
Description=Apex Rover Raspberry Startup Manager
After=network-online.target sound.target
Wants=network-online.target sound.target

[Service]
Type=simple
User=$SERVICE_USER
WorkingDirectory=$WORKDIR
ExecStart=$PYTHON_BIN -u $WORKDIR/main_startup.py
Restart=always
RestartSec=5
Environment=PYTHONUNBUFFERED=1
Environment=APEX_IDLE_MUSIC=ASSAULT.mp4
Environment=APEX_SPEAK_ON_STARTUP=1
Environment=APEX_TTS_VOICE=en-us
Environment=APEX_TTS_SPEED=150

[Install]
WantedBy=multi-user.target
SERVICE

sudo systemctl daemon-reload
sudo systemctl enable "$SERVICE_NAME"
sudo systemctl restart "$SERVICE_NAME"

echo "Done. Service status:"
systemctl --no-pager status "$SERVICE_NAME" || true

echo ""
echo "Useful checks:"
echo "  systemctl status $SERVICE_NAME"
echo "  journalctl -u $SERVICE_NAME -f"
echo "  cat /tmp/apex_rover_logs/voice_auto_service.log"
echo "  curl http://127.0.0.1:5050/status"
