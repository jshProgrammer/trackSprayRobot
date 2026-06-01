#!/usr/bin/env python3
import rospy
import math
import time
from sensor_msgs.msg import Imu
from mpu9250_jmdev.mpu_9250 import MPU9250
from mpu9250_jmdev.registers import *

def yaw_to_quaternion(yaw_rad):
    """Konvertiert den aufsummierten Yaw-Winkel in ein Quaternion"""
    q = Imu().orientation
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw_rad / 2.0)
    q.w = math.cos(yaw_rad / 2.0)
    return q

def imu_node():
    rospy.init_node('imu_node', anonymous=True)
    imu_pub = rospy.Publisher('imu/data', Imu, queue_size=10)
    
    # MPU Setup (Ohne Magnetometer, nur Beschleunigung & Gyro)
    mpu = MPU9250(
        address_ak=None,
        address_mpu_master=0x68,
        bus=1,
        gfs=GFS_250,
        afs=AFS_2G
    )
    mpu.configureMPU6500(GFS_250, AFS_2G)
    
    rospy.loginfo("--- IMU KNOTEN GESTARTET ---")
    rospy.logwarn("Kalibriere Gyroskop... Roboter fuer 2 Sekunden NICHT BEWEGEN!")
    time.sleep(1)
    
    # Gyro-Bias (Ruhe-Drift) ermitteln, damit der Roboter im Stand nicht virtuell dreht
    gyro_z_bias = 0.0
    samples = 200
    for _ in range(samples):
        gyro_z_bias += mpu.readGyroscopeMaster()[2]
        time.sleep(0.01)
    gyro_z_bias /= samples
    
    rospy.loginfo(f"Kalibrierung fertig! Z-Bias: {gyro_z_bias:.4f} deg/s")
    
    rate_hz = 100
    rate = rospy.Rate(rate_hz)
    
    yaw_rad = 0.0
    last_time = rospy.Time.now().to_sec()
    
    while not rospy.is_shutdown():
        accel = mpu.readAccelerometerMaster()
        gyro = mpu.readGyroscopeMaster()
        
        current_time = rospy.Time.now().to_sec()
        dt = current_time - last_time
        last_time = current_time
        
        # Gyro-Z bereinigen und in rad/s umwandeln
        gyro_z_clean_deg = gyro[2] - gyro_z_bias

        gyro_z_rad = math.radians(gyro_z_clean_deg)
        
        # INTEGRATION: Aktueller Winkel = Alter Winkel + (Drehgeschwindigkeit * Zeit)
        yaw_rad += gyro_z_rad * dt
        
        # ROS Imu Nachricht zusammenbauen
        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.from_sec(current_time)
        imu_msg.header.frame_id = "imu_link"
        
        imu_msg.linear_acceleration.x = accel[0]
        imu_msg.linear_acceleration.y = accel[1]
        imu_msg.linear_acceleration.z = accel[2]
        
        imu_msg.angular_velocity.x = math.radians(gyro[0])
        imu_msg.angular_velocity.y = math.radians(gyro[1])
        imu_msg.angular_velocity.z = gyro_z_rad
        
        # DAS ist der magische Schritt für die Navigation Node:
        # Hier wird der berechnete Winkel (yaw_rad) in das z/w-Quaternion umgewandelt!
        imu_msg.orientation = yaw_to_quaternion(yaw_rad)
        
        imu_pub.publish(imu_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        imu_node()
    except rospy.ROSInterruptException:
        pass