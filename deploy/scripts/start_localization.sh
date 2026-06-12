#!/bin/bash
#
# Phase 1 – Localization starten (GPS + NTRIP).
# Wird MANUELL vom Frontend gestartet (z.B. via SSH), wenn der Roboter READY gehen soll.
# Voraussetzung: roscore + rosbridge laufen bereits (deploy/rosbridge.service via systemd).
#
# Bewusst KEIN pkill, KEIN catkin_make, KEIN eigenes roscore.

set -e

TARGET_DIR="$HOME/trackSprayRobot/robot"

echo "========================================="
echo "   TrackSprayRobot – Phase 1: Localization"
echo "========================================="

# ROS-Umgebung laden (systemd/SSH lädt kein interaktives Profil)
source /opt/ros/noetic/setup.bash

cd "$TARGET_DIR"

if [ ! -f "devel/setup.bash" ]; then
    echo "[FEHLER] devel/setup.bash fehlt – einmalig 'catkin_make' im Workspace ausführen."
    exit 1
fi
source devel/setup.bash

echo "--> Starte localization (gps_node + ntrip_client)..."
roslaunch bringup localization_only.launch
