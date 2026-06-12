# Deploy – TrackSprayRobot Raspberry Pi

Reihenfolge und Verantwortlichkeiten der Start-Mechanismen:

| Was | Wie gestartet | Inhalt |
|---|---|---|
| roscore + rosbridge (Port 9090) | **systemd ab Boot** (`rosbridge.service`) | ROS-Master + Websocket fürs Frontend |
| Localization (GPS + NTRIP) | **manuell vom Frontend** (`start_localization.sh`) | Phase 1 – Roboter wird READY |
| Navigation (Motor + IMU + Nav + Teleop) | **manuell vom Frontend** (`start_navigation.sh`) | Phase 2 – Kalibrierung + Fahren |

Das Frontend schreibt vorher die JSON-Files direkt auf den Pi:
- `/home/ubuntu/tracksprayer/waypoints.json` – `[[lat, lon], ...]`
- `/home/ubuntu/tracksprayer/obstacles.json` – `[{"lat_min":..,"lon_min":..,"lat_max":..,"lon_max":..}, ...]`

Der Sprühdosenstand bleibt unter `/home/ubuntu/dosenstand.txt` (Frontend liest direkt).

## Einmaliges Setup auf dem Pi

```bash
# 1. Workspace einmalig bauen
cd ~/trackSprayRobot/robot
catkin_make

# 2. Skripte ausführbar machen
chmod +x ~/trackSprayRobot/start_localization.sh \
         ~/trackSprayRobot/start_navigation.sh \
         ~/trackSprayRobot/deploy/start_rosbridge.sh

# 3. Verzeichnis für die Frontend-JSON-Files
mkdir -p /home/ubuntu/tracksprayer

# 4. systemd-Service installieren (roscore + rosbridge ab Boot)
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
