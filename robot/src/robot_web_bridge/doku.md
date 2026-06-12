# ROS Bridge API

**Verbindung:** `ws://<ROBOT_IP>:9090`

---

## Topics

> **Hinweis:** Waypoints und Hindernisse werden **nicht** über rosbridge gesendet,
> sondern als JSON-Files direkt auf den Pi geschrieben
> (`/home/ubuntu/trackSprayRobot/shared_files/waypoints.json` + `obstacles.json`, siehe
> `deploy/README.md`). Diese Doku beschreibt nur die rosbridge-Live-Datenflüsse.

### `gps/fix`
**Typ:** `sensor_msgs/NavSatFix` · **Richtung:** Robot → Client

| Feld | Typ | Einheit |
|---|---|---|
| `header.frame_id` | `string` | `gps_link` |
| `latitude` | `float` | Grad |
| `longitude` | `float` | Grad |
| `altitude` | `float` | m |
| `status.status` | `int` | `4`=RTK Fix · `5`=RTK Float · `1–3`=GPS · `0`=kein Fix |

---

### `gps/quality`
**Typ:** `std_msgs/UInt8` · **Richtung:** Robot → Client

Roher GGA-Qualitätsindikator des GPS-Empfängers (ergänzend zu `gps/fix.status.status`).

| Feld | Typ | Bedeutung |
|---|---|---|
| `data` | `uint8` | `4`=RTK Fix (cm) · `5`=RTK Float · `2`=DGPS · `1`=GPS · `0`=kein Fix |

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
| navigation | `RTK_READY` | info | RTK stabil – Navigation freigegeben |
| navigation | `RTK_UNSTABLE` | warn | RTK-FIXED unterbrochen vor Freigabe |
| navigation | `RTK_LOST` | error | Kein frischer RTK-FIXED – Roboter stoppt |
| navigation | `RTK_RECOVERED` | info | RTK-Fix wieder da – Fahrt geht weiter |
| navigation | `GOAL_REACHED` | info | Alle Waypoints erreicht – Roboter stoppt |
| gps_node | `GPS_SERIAL_FAILED` | error | GPS-Sensor nicht erreichbar (Node beendet sich) |
| gps_node | `GPS_SERIAL_READ_ERROR` | warn | Lesefehler auf serieller GPS-Leitung |
| ntrip_client | `NTRIP_CONNECTED` | info | NTRIP verbunden – empfange Korrekturen |
| ntrip_client | `NTRIP_DISCONNECTED` | warn | NTRIP getrennt – Reconnect läuft |
| ntrip_client | `NTRIP_DNS_FAILED` | warn | NTRIP-Server nicht auflösbar (DNS) |
| ntrip_client | `NTRIP_AUTH_FAILED` | error | NTRIP abgelehnt (Auth/Mountpoint, Node beendet sich) |
| ntrip_client | `NTRIP_NO_GGA` | error | Kein GGA empfangen (Node beendet sich) |
| pwm_drive | `MOTOR_PIGPIOD_DOWN` | error | pigpio-Daemon läuft nicht – Roboter kann nicht fahren |

---

### `/emergency_reset`
**Typ:** `teleop/EmergencyReset` · **Richtung:** Client → Robot

| Feld | Typ | Bedeutung |
|---|---|---|
| `is_soft` | `bool` | `true` = Soft-Reset (nur Motortreiber neu); `false` = Hard-Kill aller Fahr-/Navigationsprozesse (localization + rosbridge bleiben am Leben) |

---


## Will be there soon 
- Battery Level
- Spray Level 
