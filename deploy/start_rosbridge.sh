#!/bin/bash
#
# Wrapper für den rosbridge-systemd-Service.
# Stellt roscore sicher und startet dann den rosbridge-Websocket auf Port 9090.
# So ist die Bridge ab Boot erreichbar, bevor das Frontend die Phasen-Skripte triggert.

set -e

TARGET_DIR="$HOME/trackSprayRobot/robot"

# ROS-Umgebung laden (systemd lädt kein interaktives Profil)
source /opt/ros/noetic/setup.bash

cd "$TARGET_DIR"
if [ ! -f "devel/setup.bash" ]; then
    echo "[FEHLER] devel/setup.bash fehlt – einmalig 'catkin_make' im Workspace ausführen." >&2
    exit 1
fi
source devel/setup.bash

# roscore sicherstellen: starten, falls kein Master erreichbar ist.
if ! rostopic list > /dev/null 2>&1; then
    echo "--> Kein roscore erreichbar, starte roscore..."
    roscore &
    # Warten bis der Master oben ist
    until rostopic list > /dev/null 2>&1; do
        sleep 0.5
    done
    echo "[OK] roscore läuft."
else
    echo "[OK] roscore läuft bereits."
fi

echo "--> Starte rosbridge (Port 9090)..."
exec roslaunch robot_web_bridge start.launch
