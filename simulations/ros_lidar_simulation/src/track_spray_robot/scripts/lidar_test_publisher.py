#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
import math

rospy.init_node('test_scan_publisher')
pub = rospy.Publisher('/scan', LaserScan, queue_size=10)
rate = rospy.Rate(10)

scan = LaserScan()
scan.header.frame_id = "lidar_link"
scan.angle_min = -math.pi
scan.angle_max = math.pi
scan.angle_increment = math.radians(1)
scan.range_min = 0.05
scan.range_max = 30.0
scan.ranges = [2.0]*360  # 2 meters to all sides
scan.intensities = [1.0]*360

while not rospy.is_shutdown():
    scan.header.stamp = rospy.Time.now()
    pub.publish(scan)
    rate.sleep()