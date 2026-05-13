#!/usr/bin/env python3
"""
ROS Noetic Node for MPU9250 9-DOF IMU Sensor
Publishes accelerometer and gyroscope data

Dependencies:
    sudo apt install python3-smbus i2c-tools
    pip3 install smbus2 mpu9250-jmdev
"""

import rospy
import math
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped, Quaternion
import time

try:
    from mpu9250_jmdev.mpu_9250 import MPU9250
    from mpu9250_jmdev.registers import (
        AK8963_ADDRESS,
        MPU9050_ADDRESS_68,
        GFS_250, GFS_500, GFS_1000, GFS_2000,
        AFS_2G, AFS_4G, AFS_8G, AFS_16G,
        AK8963_BIT_16,
        AK8963_MODE_C8HZ
    )
except ImportError as e:
    rospy.logerr(f"Failed to import MPU9250 library: {e}")
    rospy.logerr("Install with: pip3 install mpu9250-jmdev")
    exit(1)


class MPU9250Node:
    """ROS Node for MPU9250 IMU Sensor"""

    # Gyroscope Full Scale Range (FSR) in deg/s
    GFS_RANGES = {
        250: GFS_250,
        500: GFS_500,
        1000: GFS_1000,
        2000: GFS_2000
    }

    # Accelerometer Full Scale Range (FSR) in g
    AFS_RANGES = {
        2: AFS_2G,
        4: AFS_4G,
        8: AFS_8G,
        16: AFS_16G
    }

    # Conversion factors
    GYRO_SENSITIVITY = {
        250: 131.0,
        500: 65.5,
        1000: 32.8,
        2000: 16.4
    }

    ACCEL_SENSITIVITY = {
        2: 16384.0,
        4: 8192.0,
        8: 4096.0,
        16: 2048.0
    }

    def __init__(self):
        """Initialize ROS node and MPU9250 sensor"""
        rospy.init_node('mpu9250_node', anonymous=False)
        
        # Get parameters
        self.i2c_bus = rospy.get_param('~i2c_bus', 1)
        #TODO: might need higher publish rate!
        self.publish_rate = rospy.get_param('~publish_rate', 20)  # Hz
        self.gyro_fsr = rospy.get_param('~gyro_fsr', 250)  # deg/s
        self.accel_fsr = rospy.get_param('~accel_fsr', 2)  # g
        self.frame_id = rospy.get_param('~frame_id', 'imu_link')
        
        rospy.loginfo(f"MPU9250 Node Parameters:")
        rospy.loginfo(f"  I2C Bus: {self.i2c_bus}")
        rospy.loginfo(f"  Publish Rate: {self.publish_rate} Hz")
        rospy.loginfo(f"  Gyro FSR: {self.gyro_fsr} deg/s")
        rospy.loginfo(f"  Accel FSR: {self.accel_fsr} g")
        rospy.loginfo(f"  Frame ID: {self.frame_id}")
        
        # Validate parameters
        if self.gyro_fsr not in self.GFS_RANGES:
            rospy.logerr(f"Invalid gyro_fsr: {self.gyro_fsr}. Valid values: {list(self.GFS_RANGES.keys())}")
            exit(1)
        if self.accel_fsr not in self.AFS_RANGES:
            rospy.logerr(f"Invalid accel_fsr: {self.accel_fsr}. Valid values: {list(self.AFS_RANGES.keys())}")
            exit(1)
        
        # Initialize sensor
        try:
            self.mpu = MPU9250(
                address_ak=AK8963_ADDRESS,
                address_mpu_master=MPU9050_ADDRESS_68,
                address_mpu_slave=None,
                bus=self.i2c_bus,
                gfs=self.GFS_RANGES[self.gyro_fsr],
                afs=self.AFS_RANGES[self.accel_fsr],
                mfs=AK8963_BIT_16,
                mode=AK8963_MODE_C8HZ
            )
            self.mpu.configureMPU6500(
                self.GFS_RANGES[self.gyro_fsr],
                self.AFS_RANGES[self.accel_fsr]
            )
            rospy.loginfo("MPU9250 sensor initialized successfully")
        except Exception as e:
            rospy.logerr(f"Failed to initialize MPU9250: {e}")
            rospy.logerr("Make sure:")
            rospy.logerr("  1. Sensor is connected to I2C bus")
            rospy.logerr("  2. Run: sudo i2cdetect -y 1")
            exit(1)
        
        # Publisher
        self.imu_pub = rospy.Publisher('imu/data', Imu, queue_size=10)
        
        rospy.loginfo("Publisher created. Topic: imu/data (sensor_msgs/Imu)")
    
    def run(self):
        """Main loop to read and publish sensor data"""
        rate = rospy.Rate(self.publish_rate)
        rospy.loginfo(f"Starting main loop at {self.publish_rate} Hz")
        
        while not rospy.is_shutdown():
            try:
                # Read sensor data
                accel_raw = self.mpu.readAccelerometerMaster()
                gyro_raw = self.mpu.readGyroscopeMaster()
                
                # Convert to SI units
                accel_sensitivity = self.ACCEL_SENSITIVITY[self.accel_fsr]
                gyro_sensitivity = self.GYRO_SENSITIVITY[self.gyro_fsr]
                
                # Accelerometer: g to m/s^2
                accel_x = (accel_raw[0] / accel_sensitivity) * 9.81
                accel_y = (accel_raw[1] / accel_sensitivity) * 9.81
                accel_z = (accel_raw[2] / accel_sensitivity) * 9.81
                
                # Gyroscope: deg/s to rad/s
                gyro_x = (gyro_raw[0] / gyro_sensitivity) * (math.pi / 180.0)
                gyro_y = (gyro_raw[1] / gyro_sensitivity) * (math.pi / 180.0)
                gyro_z = (gyro_raw[2] / gyro_sensitivity) * (math.pi / 180.0)
                
                # Create IMU message
                imu_msg = Imu()
                imu_msg.header.stamp = rospy.Time.now()
                imu_msg.header.frame_id = self.frame_id
                
                # Set accelerometer data
                imu_msg.linear_acceleration.x = accel_x
                imu_msg.linear_acceleration.y = accel_y
                imu_msg.linear_acceleration.z = accel_z
                
                # Set gyroscope data
                imu_msg.angular_velocity.x = gyro_x
                imu_msg.angular_velocity.y = gyro_y
                imu_msg.angular_velocity.z = gyro_z
                
                # Orientation is not available from raw IMU data (would need fusion)
                imu_msg.orientation = Quaternion(x=0, y=0, z=0, w=1)
                
                # Set covariance (estimate)
                imu_msg.linear_acceleration_covariance = [0.01] * 9
                imu_msg.angular_velocity_covariance = [0.01] * 9
                imu_msg.orientation_covariance = [-1] * 9  # -1 means not available
                
                # Publish messages
                self.imu_pub.publish(imu_msg)
                
                rate.sleep()
                
            except Exception as e:
                rospy.logerr(f"Error reading sensor data: {e}")
                rate.sleep()
    
    def shutdown(self):
        """Cleanup on shutdown"""
        rospy.loginfo("MPU9250 node shutting down")


if __name__ == '__main__':
    node = MPU9250Node()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        node.shutdown()
