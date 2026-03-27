# GPS Sensor Data Receiver

This project provides a simple interface to receive and process GPS sensor data, either directly via Python or through ROS.

---

## Overview

Two modes are available:

- **Without ROS**: Access raw GPS data directly from the sensor  
- **With ROS**: Publish filtered GPS data to a ROS topic (`/gps/fix`)

---

## Requirements

- Python 3.x  
- ROS (e.g. Noetic) with catkin workspace  
- GPS sensor connected (e.g. via serial/USB)  
- Required Python dependencies (e.g. pyserial)

---

## 1. Run Without ROS

Execute the standalone script:

```bash
python3 gps_test_without_ros.py
```

### Description

- Outputs **raw, unfiltered GPS data**
- Includes all incoming sensor messages, also if no satellite connection is established yet

---

## 2. Run With ROS

### Build the workspace

```bash
catkin_make
source devel/setup.bash
```

### Run the GPS node

```bash
python3 src/scripts/gps_driver/src/scripts/gps_node.py
```

### Listen to the topic

Open a new terminal:

```bash
rostopic echo /gps/fix
```

---

## ROS Topic: `/gps/fix`

This topic publishes processed GPS data using the standard ROS message type:

```
sensor_msgs/NavSatFix
```

### Example Output

```yaml
header:
  seq: 4
  stamp:
    secs: 1774608158
    nsecs: 169350862
  frame_id: "gps_link"
status:
  status: 0
  service: 0
latitude: xx.xxxxxxxx
longitude: xx.xxxxxxxx
altitude: 0.0
position_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
position_covariance_type: 0
```

---

## Field Explanation

- **header**  
  Contains timestamp and frame reference (`gps_link`)

- **status**  
  Indicates GPS fix status:
  - -1: No fix  
  - 0: No reliable fix  
  - 1: Valid GPS fix  

- **latitude / longitude**  
  Position in decimal degrees  

- **altitude**  
  Height above sea level (meters)  

- **position_covariance**  
  Estimated uncertainty of the position  

- **position_covariance_type**  
  Defines how covariance is interpreted  


## Troubleshooting

**No data received**
- Check GPS connection (USB/serial port)  
- Verify correct port and baud rate  

**Status = 0**
- No satellite fix yet  
- Wait for GPS initialization (can take several minutes)  

## Possible Improvements

- Filter out invalid GPS fixes before publishing   
- Add launch file for easier ROS startup  
- Add configuration for serial port and baud rate  