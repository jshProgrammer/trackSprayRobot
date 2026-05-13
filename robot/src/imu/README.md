# MPU9250 ROS Noetic Node

ROS Noetic node for the MPU9250 9-DOF IMU sensor (Accelerometer, Gyroscope).

## Features

- ✅ Reads Accelerometer and Gyroscope data
- ✅ Automatic unit conversion to SI units (m/s², rad/s)
- ✅ Configurable sensor ranges
- ✅ Standard ROS sensor messages
- ✅ Error handling and comprehensive logging

## Installation

### System Dependencies

```bash
sudo apt install python3-smbus i2c-tools
pip3 install smbus2 mpu9250-jmdev
```

### Verify I2C Connection

```bash
sudo i2cdetect -y 1
```

You should see the MPU9250 at address `0x68`. The magnetometer would have `0x0c` so it does not seem to be available.

### Build ROS Package

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Usage

### Basic Launch

```bash
roslaunch imu_node imu.launch
```

### With Custom Parameters

```bash
roslaunch imu_node imu.launch \
  publish_rate:=50 \
  gyro_fsr:=500 \,
  accel_fsr:=4
```

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `i2c_bus` | `1` | I2C bus number (usually 1 for Raspberry Pi) |
| `publish_rate` | `20` | Publish frequency in Hz |
| `gyro_fsr` | `250` | Gyroscope Full Scale Range: 250, 500, 1000, 2000 deg/s |
| `accel_fsr` | `2` | Accelerometer Full Scale Range: 2, 4, 8, 16 g |
| `frame_id` | `imu_link` | TF frame ID for the IMU |

## Published Topics

### `/imu/data` (sensor_msgs/Imu)

IMU data including:
- **linear_acceleration**: Acceleration in m/s² (x, y, z)
- **angular_velocity**: Angular velocity in rad/s (x, y, z)
- **orientation**: Quaternion (not available from raw IMU, would need fusion algorithm)

## Viewing Data

```bash
# View IMU data
rostopic echo /imu/data

# Plot data (not working currently)
rqt_plot /imu/data/linear_acceleration
rqt_plot /imu/data/angular_velocity
```

## Troubleshooting

### Sensor not found
```bash
# Check I2C connection
sudo i2cdetect -y 1

# Check permissions
sudo usermod -a -G i2c $USER
```

### Permission denied on /dev/i2c-1
```bash
sudo chmod 666 /dev/i2c-1
# Or add user to i2c group and reboot
```

### Wrong values
- Check sensor orientation
- Verify I2C addresses match your configuration

## Wiring (Raspberry Pi)

| MPU9250 | Raspberry Pi |
|---------|-------------|
| VCC | 3.3V (Pin 1) |
| GND | GND (Pin 6) |
| SCL | GPIO3/SCL (Pin 5) |
| SDA | GPIO2/SDA (Pin 3) |

## References

- [MPU9250 Datasheet](https://invensense.tdk.com/products/motion-tracking/9-axis/mpu-9250/)
- [mpu9250-jmdev Library](https://github.com/jmdev18/MPU9250)
- [ROS Sensor Messages](http://docs.ros.org/en/noetic/api/sensor_msgs/html/index-msg.html)

## License

MIT
