# ROS Bridge API

**Verbindung:** `ws://<ROBOT_IP>:9090`

---

## Topics

### `/goal_coordinates`
**Typ:** `geometry_msgs/PoseStamped` · **Richtung:** Client → Robot

| Feld | Typ | Einheit |
|---|---|---|
| `header.frame_id` | `string` | `map` \| `odom` \| `base_link` \| `world` |
| `header.stamp.secs` | `int` | Unix-Sekunden |
| `pose.position.x` | `float` | m |
| `pose.position.y` | `float` | m |
| `pose.position.z` | `float` | m |
| `pose.orientation.z` | `float` | — |
| `pose.orientation.w` | `float` | — |

---

### `/test_data`
**Typ:** `std_msgs/String` · **Richtung:** Robot → Client

| Feld | Typ |
|---|---|
| `data` | `string` |

---

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

### `gps/unfixed`
**Typ:** `sensor_msgs/NavSatFix` · **Richtung:** Robot → Client

Gleiche Felder wie `gps/fix`, ungefiltert.

---

### `gps/nmea_sentence`
**Typ:** `std_msgs/String` · **Richtung:** `gps_node` → `ntrip_client_node`

| Feld | Typ |
|---|---|
| `data` | `string` (NMEA) |

---

### `gps/rtcm`
**Typ:** `std_msgs/ByteMultiArray` · **Richtung:** `ntrip_client_node` → `gps_node`

| Feld | Typ |
|---|---|
| `data` | `byte[]` (RTCM3) |