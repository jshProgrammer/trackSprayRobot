# ROS Bridge API

**Verbindung:** `ws://<ROBOT_IP>:9090`

---

## Topics

> **Hinweis:** Waypoints, Hindernisse und Dosenstand werden **nicht** über rosbridge übertragen,
> sondern als Dateien im Shared-Folder ausgetauscht
> (`/home/ubuntu/trackSprayRobot/shared_files/waypoints.json`, `obstacles.json`,
> `dosenstand.txt`, siehe
> `deploy/README.md`). Diese Doku beschreibt nur die rosbridge-Live-Datenflüsse.

## Shared Files

Diese Dateien liegen standardmäßig unter `/home/ubuntu/trackSprayRobot/shared_files/`.

| Datei | Richtung | Format | Bedeutung |
|---|---|---|---|
| `waypoints.json` | Client → Robot | JSON | Ziel-Waypoints, z.B. `[{"lat":..,"lon":..}]` oder `[[lat, lon], ...]` |
| `obstacles.json` | Client → Robot | JSON | Hindernis-Rechtecke, z.B. `[{"lat_min":..,"lon_min":..,"lat_max":..,"lon_max":..}]` |
| `dosenstand.txt` | Robot → Client | Text | Restinhalt der Sprühdose in Gramm, z.B. `249.75` |

Falls `dosenstand.txt` fehlt, startet der Zähler intern mit `250.0` g. Pro
automatischem Waypoint-Spray zieht die Navigation `0.25` g ab. Der Pfad kann über
die Umgebungsvariable `DOSENSTAND_FILE` überschrieben werden; der alte Pfad
`/home/ubuntu/dosenstand.txt` wird nur noch als Lesefallback unterstützt.

### `gps/fix`
**Typ:** `sensor_msgs/NavSatFix` · **Richtung:** Robot → Client

| Feld | Typ | Einheit |
|---|---|---|
| `header.frame_id` | `string` | `gps_link` |
| `latitude` | `float` | Grad |
| `longitude` | `float` | Grad |
| `altitude` | `float` | m |
| `status.status` | `int` | ROS-`NavSatStatus` (`STATUS_NO_FIX=-1` · `STATUS_FIX=0` · `STATUS_SBAS_FIX=1` · `STATUS_GBAS_FIX=2`). Hier publiziert: `0`=Fix (RTK Fix/DGPS/GPS) · `1`=RTK Float. Fürs Frontend vermutlich nicht relevant – die RTK-Qualität ist hier nicht eindeutig ablesbar (RTK Fix, DGPS und GPS sind alle `0`); stattdessen `gps/quality` verwenden. |

---

### `gps/quality`
**Typ:** `std_msgs/UInt8` · **Richtung:** Robot → Client

Roher GGA-Qualitätsindikator des GPS-Empfängers (ergänzend zu `gps/fix.status.status`).

| Feld | Typ | Bedeutung |
|---|---|---|
| `data` | `uint8` | `4`=RTK Fix (cm) · `5`=RTK Float · `2`=DGPS · `1`=GPS · `0`=kein Fix |

---


### `/navigation_state`
**Typ:** `robot_msgs/NavigationState` · **Richtung:** Robot → Client

**Aktueller Navigations-Lebenszustand** (nicht zu verwechseln mit `/robot_status`, das
diskrete Events/Fehler liefert). Publiziert bei **Zustandswechsel** und bei
**Wegpunktwechsel**, **latched** (ein spät verbundener Client bekommt sofort den
aktuellen Zustand).

| Feld | Typ | Bedeutung |
|---|---|---|
| `stamp` | `time` | Zeitpunkt des Zustands |
| `state` | `uint8` | `0`=IDLE · `1`=CALIBRATING · `2`=NAVIGATING · `3`=GOAL_REACHED · `4`=PAUSED |
| `state_name` | `string` | Klartext-Label (`IDLE` \| `CALIBRATING` \| `NAVIGATING` \| `GOAL_REACHED` \| `PAUSED`) |
| `waypoint_index` | `uint16` | aktueller Ziel-Wegpunkt, **1-basiert** (N von total); `0` wenn n/a |
| `waypoint_total` | `uint16` | Anzahl Ziel-Wegpunkte |
| `target_lat` | `float64` | Zielkoordinate – in `NAVIGATING` und `PAUSED` gesetzt, sonst `0.0` |
| `target_lon` | `float64` | s.o. |

**Zustände:** `IDLE` = wartet auf stabilen RTK-FIXED · `CALIBRATING` = fährt 3 m
geradeaus zur Heading-Kalibrierung · `NAVIGATING` = fährt zum Ziel-Wegpunkt
(`waypoint_index`/`target_*`) · `GOAL_REACHED` = alle Wegpunkte abgefahren ·
`PAUSED` = autonome Navigation pausiert; der Controller kann manuell fahren.

> `target_lat/lon` sind nur die **echten** Ziel-Wegpunkte – Hindernis-Umfahrungspunkte
> erscheinen hier nicht (`NAVIGATING`/`PAUSED` zeigen weiter das eigentliche Ziel).

---

### `/navigation_pause`
**Typ:** `std_msgs/Bool` · **Richtung:** Client → Robot

Pausiert oder setzt die autonome Navigation fort, ohne die Navigation-Node zu beenden.
Während `PAUSED` publiziert die Navigation keine Fahrbefehle auf `/cmd_vel_controll`,
damit der Controller manuell steuern kann.

| Feld | Typ | Bedeutung |
|---|---|---|
| `data` | `bool` | `true` = pausieren · `false` = fortsetzen |

Beim Fortsetzen bleibt der aktuelle Ziel-Wegpunkt erhalten. Leitpunkte und
Hindernis-Umfahrungspunkte werden verworfen und von der aktuellen Roboterposition aus
neu berechnet; der Roboter fährt also nicht zur Position vor der Pause zurück.

---

### `/robot_status`
**Typ:** `robot_msgs/RobotStatus` · **Richtung:** Robot → Client

Elementare Status-/Fehler-Events, **edge-triggered** (nur bei Zustandswechsel) und
**latched** (ein spät verbundener Client bekommt sofort den letzten Status). Der Client
abonniert nur dieses eine Topic und filtert über `source` / `type` / `code`.

| Feld | Typ | Bedeutung |
|---|---|---|
| `stamp` | `time` | Zeitpunkt des Events |
| `type` | `string` | `info` \| `warn` \| `error` |
| `source` | `string` | meldende Node (`navigation`, `gps_node`, `ntrip_client`, `pwm_drive`) |
| `code` | `string` | maschinenlesbare Event-ID (siehe unten) |
| `message` | `string` | menschenlesbarer Klartext (DE) |

**`code`-Werte nach `source`:**

| source | code | type | Bedeutung |
|---|---|---|---|
| navigation | `RTK_FIX_INITIALIZED` | info | RTK stabil – Navigation freigegeben |
| navigation | `RTK_UNSTABLE` | warn | RTK-FIXED unterbrochen vor Freigabe |
| navigation | `RTK_LOST` | error | Kein frischer RTK-FIXED – Roboter stoppt |
| navigation | `RTK_RECOVERED` | info | RTK-Fix wieder da – Fahrt geht weiter |
| navigation | `NAVIGATION_PAUSED` | info | Autonome Navigation pausiert; manuelle Steuerung möglich |
| navigation | `NAVIGATION_RESUMED` | info | Autonome Navigation fortgesetzt |
| navigation | `GOAL_REACHED` | info | Alle Waypoints erreicht – Roboter stoppt |
| gps_node | `GPS_SERIAL_FAILED` | error | GPS-Sensor nicht erreichbar (Node beendet sich) |
| gps_node | `GPS_SERIAL_READ_ERROR` | warn | Lesefehler auf serieller GPS-Leitung |
| ntrip_client | `NTRIP_CONNECTED` | info | NTRIP verbunden – empfange Korrekturen |
| ntrip_client | `NTRIP_DISCONNECTED` | warn | NTRIP getrennt – Reconnect läuft |
| ntrip_client | `NTRIP_DNS_FAILED` | warn | NTRIP-Server nicht auflösbar (DNS) |
| ntrip_client | `NTRIP_AUTH_FAILED` | error | NTRIP abgelehnt (Auth/Mountpoint, Node beendet sich) |
| ntrip_client | `NTRIP_NO_GGA` | error | Kein GGA empfangen (Node beendet sich) |
| pwm_drive | `MOTOR_PIGPIOD_DOWN` | error | pigpio-Daemon läuft nicht – Roboter kann nicht fahren |
| emergency_kill | `EMERGENCY_STOP` | error | Not-Aus ausgelöst; Fahr-/Navigationsprozesse werden beendet |
| emergency_kill | `SOFT_RESET` | warn | Soft-Reset ausgelöst; Motortreiber wird neu gestartet |

---

### `/emergency_reset`
**Typ:** `teleop/EmergencyReset` · **Richtung:** Client → Robot

| Feld | Typ | Bedeutung |
|---|---|---|
| `is_soft` | `bool` | `true` = Soft-Reset (nur Motortreiber neu); `false` = Hard-Kill aller Fahr-/Navigationsprozesse (localization + rosbridge bleiben am Leben) |
