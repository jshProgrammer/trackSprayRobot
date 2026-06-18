# Deploy – TrackSprayRobot Raspberry Pi

Reihenfolge und Verantwortlichkeiten der Start-Mechanismen:

| Was | Wie gestartet | Inhalt |
|---|---|---|
| roscore + rosbridge (Port 9090) | **systemd ab Boot** (`rosbridge.service`) | ROS-Master + Websocket fürs Frontend |
| Localization (GPS + NTRIP) | **manuell vom Frontend** (`start_localization.sh`) | Phase 1 – Roboter wird READY |
| Navigation (Motor + IMU + Nav + Teleop) | **manuell vom Frontend** (`start_navigation.sh`) | Phase 2 – Kalibrierung + Fahren |

Frontend und Robot tauschen Laufzeitdaten über den Shared-Folder aus. Standardpfade:
- `/home/ubuntu/trackSprayRobot/shared_files/waypoints.json` – `[{"lat":..,"lon":..}, ...]` oder `[[lat, lon], ...]`
- `/home/ubuntu/trackSprayRobot/shared_files/obstacles.json` – `[{"lat_min":..,"lon_min":..,"lat_max":..,"lon_max":..}, ...]`
- `/home/ubuntu/trackSprayRobot/shared_files/dosenstand.txt` – Plain-Text-Grammwert, wird beim automatischen Spray aktualisiert

Die Pfade können vom Backend über `TRACKSPRAYER_WAYPOINTS_FILE` und
`TRACKSPRAYER_OBSTACLES_FILE` überschrieben werden. `navigation.launch` reicht
diese Werte an die ROS-Nodes weiter. Der Dosenstand kann über `DOSENSTAND_FILE`
überschrieben werden; falls noch `/home/ubuntu/dosenstand.txt` existiert, wird dieser
alte Pfad einmalig als Lesefallback verwendet und danach in `shared_files` geschrieben.

## Einmaliges Setup auf dem Pi

```bash
# 1. Workspace einmalig bauen
cd ~/trackSprayRobot/robot
catkin_make

# 2. RTK/NTRIP-Konfiguration lokal anlegen und ausfüllen
cp src/localization/config/rtk_credentials.example.yaml \
   src/localization/config/rtk_credentials.yaml
nano src/localization/config/rtk_credentials.yaml

# 3. Skripte ausführbar machen
chmod +x ~/trackSprayRobot/deploy/scripts/start_localization.sh \
         ~/trackSprayRobot/deploy/scripts/start_navigation.sh \
         ~/trackSprayRobot/deploy/start_rosbridge.sh

# 4. Verzeichnis für Frontend/Robot-Shared-State
mkdir -p /home/ubuntu/trackSprayRobot/shared_files

# 5. systemd-Service installieren (roscore + rosbridge ab Boot)
sudo cp ~/trackSprayRobot/deploy/rosbridge.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now rosbridge

# Status prüfen
systemctl status rosbridge
```

## Betrieb

- Port 9090 ist nach dem Boot offen (`ss -tlnp | grep 9090`).
- Frontend triggert `start_localization.sh`, dann (nach Ausrichtung) `start_navigation.sh`.
- **Not-Aus:** Topic `/emergency_reset` (`teleop/EmergencyReset`, `is_soft:false`) → killt
  die Fahr-/Navigationsprozesse per `pkill`; roscore + rosbridge + localization laufen
  weiter, das Frontend bleibt verbunden.

## Hinweis zu pigpiod

`start_navigation.sh` startet `pigpiod` per `sudo` (für die Motoransteuerung), idempotent.
Damit das ohne Passwort-Prompt läuft, ggf. eine sudoers-Regel für `ubuntu` ergänzen:

```
ubuntu ALL=(ALL) NOPASSWD: /usr/bin/pigpiod
```
