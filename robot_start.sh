#!/bin/bash

# --- KONFIGURATION ---
# Falls der Ordner nicht direkt im Home-Verzeichnis liegt, den Pfad hier anpassen:
TARGET_DIR="$HOME/trackSprayRobot/robot"
# ---------------------

echo "========================================="
echo "   TrackSprayRobot Start-Sequenz"
echo "========================================="

# 1. In den Ordner wechseln
if cd "$TARGET_DIR"; then
    echo "[OK] In Workspace gewechselt: $TARGET_DIR"
else
    echo "[FEHLER] Ordner $TARGET_DIR wurde nicht gefunden!"
    exit 1
fi

# 2. pigpiod mit sudo starten
echo "--> Starte pigpiod Daemon (Sudo-Passwort erforderlich)..."
sudo pigpiod
if [ $? -eq 0 ]; then
    echo "[OK] pigpiod läuft."
else
    echo "[WARNUNG] pigpiod konnte nicht gestartet werden (läuft vielleicht schon?)."
fi

# 3. catkin_make ausführen
echo "--> Führe catkin_make aus..."
if catkin_make; then
    echo "[OK] Build erfolgreich!"
else
    echo "[FEHLER] catkin_make ist fehlgeschlagen! Start abgebrochen."
    exit 1
fi

# 4. Workspace sourcen
echo "--> Source devel/setup.bash..."
if [ -f "devel/setup.bash" ]; then
    source devel/setup.bash
    echo "[OK] Umgebung geladen."
else
    echo "[FEHLER] devel/setup.bash nicht gefunden! Wurde catkin_make jemals erfolgreich ausgeführt?"
    exit 1
fi

# 5. ROS Launchfile starten
echo "--> Starte ROS Bringup..."
echo "========================================="
roslaunch bringup robot.launch