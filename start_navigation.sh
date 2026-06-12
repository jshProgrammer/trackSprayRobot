#!/bin/bash
#
# Phase 2 – Kalibrierung + Navigation starten (Motor + IMU + Navigation + Teleop).
# Wird MANUELL vom Frontend gestartet, NACHDEM der Nutzer die Ausrichtung bestätigt hat.
# Liest die vom Frontend geschriebenen JSON-Files
#   /home/ubuntu/tracksprayer/waypoints.json
#   /home/ubuntu/tracksprayer/obstacles.json
# implizit über die Nodes ein.
#
# Bewusst KEIN pkill, KEIN catkin_make (Build läuft schon, Localization läuft weiter).

set -e

TARGET_DIR="$HOME/trackSprayRobot/robot"

echo "========================================="
echo "   TrackSprayRobot – Phase 2: Navigation"
echo "========================================="

# ROS-Umgebung laden
source /opt/ros/noetic/setup.bash

cd "$TARGET_DIR"

if [ ! -f "devel/setup.bash" ]; then
    echo "[FEHLER] devel/setup.bash fehlt – einmalig 'catkin_make' im Workspace ausführen."
    exit 1
fi
source devel/setup.bash

# pigpiod für die Motoransteuerung (idempotent: startet nur, falls nicht aktiv)
echo "--> Stelle pigpiod sicher..."
if ! pgrep -x pigpiod > /dev/null; then
    sudo pigpiod && echo "[OK] pigpiod gestartet." || echo "[WARNUNG] pigpiod konnte nicht gestartet werden."
else
    echo "[OK] pigpiod läuft bereits."
fi

echo "--> Starte Navigation (motor + imu + navigation + teleop)..."
roslaunch bringup navigation_only.launch
