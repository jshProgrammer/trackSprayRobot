# GPS + NTRIP RTK ROS Integration

This package provides two ROS nodes that together deliver RTK-corrected GPS data — or can be run independently without RTK.

---

## Architecture

```
GPS Hardware (/dev/ttyUSB0)
        │
        ▼
  [gps_node] ◄─────────────────────────────────┐
        │                                        │
        │  exclusive serial access               │ writes RTCM
        │                                        │ to serial
        ├──► gps/fix            (NavSatFix)      │
        ├──► gps/unfixed        (NavSatFix)      │
        └──► gps/nmea_sentence  (String)         │
                        │                        │
                        ▼                        │
              [ntrip_client_node]                │
                        │                        │
                        └──► gps/rtcm ───────────┘
                             (ByteMultiArray)
```

> **Important:** `gps_node` is the sole gatekeeper to the serial port — both for reading and writing. `ntrip_client_node` communicates **exclusively** via ROS topics.

---

## Topics

| Topic | Type | Direction | Description |
|---|---|---|---|
| `gps/fix` | `NavSatFix` | gps_node → | Fixed GPS position (RTK if available) |
| `gps/unfixed` | `NavSatFix` | gps_node → | Raw position without quality filtering |
| `gps/nmea_sentence` | `String` | gps_node → ntrip_client | NMEA sentences (incl. GGA for NTRIP) |
| `gps/rtcm` | `ByteMultiArray` | ntrip_client → gps_node | RTCM correction data from caster |

---

## Nodes

### `gps_node`

Holds exclusive access to the serial port. Parses NMEA sentences and publishes GPS fixes. Receives RTCM data and writes it back to the serial port.

**Parameters:**

| Parameter | Default | Description |
|---|---|---|
| `~port` | `/dev/ttyUSB0` | Serial port of the GPS module |
| `~baud` | `115200` | Baud rate |
| `~frame_id` | `gps_link` | TF frame ID |

**GPS quality levels (NMEA `gps_qual`):**

| Value | Status | Description |
|---|---|---|
| 4 | `STATUS_FIX` | RTK Fix (~cm accuracy) |
| 5 | `STATUS_SBAS_FIX` | RTK Float (~dm accuracy) |
| 1–3 | `STATUS_FIX` | Standard GPS |
| 0 | `STATUS_NO_FIX` | No fix |

---

### `ntrip_client_node`

Connects to an NTRIP caster and receives RTCM correction data. Requires no direct serial access.

**Parameters:**

| Parameter | Default | Description |
|---|---|---|
| `~caster` | `""` | NTRIP caster hostname |
| `~port` | `2101` | Caster port |
| `~mountpoint` | `""` | Mountpoint |
| `~user` | `""` | Username |
| `~password` | `""` | Password |
| `~verbose` | `false` | Enable debug output (unused yet) |

---

## RTK Credentials Config

`launch/localization.launch` loads `config/rtk_credentials.yaml`. Create that
file from the example and keep real credentials local:

```bash
cp robot/src/localization/config/rtk_credentials.example.yaml \
   robot/src/localization/config/rtk_credentials.yaml
```

Expected structure:

```yaml
ntrip_client:
  caster: "sapos-rtk.bayern.de"
  port: 2101
  mountpoint: "VRS_3_4G_BY"
  user: "DEIN_NTRIP_USERNAME"
  password: "DEIN_NTRIP_PASSWORD"
  verbose: false

gps_node:
  port: "/dev/ttyUSB0"
  baud: 115200
  frame_id: "gps_link"
  gps_rate_hz: 10
```

The names matter: `ntrip_client.py` reads `~caster` and `~user`, not `~host`
or `~username`.

---

## Usage

```bash
roslaunch track_spray_robot diff_drive.launch 
```
