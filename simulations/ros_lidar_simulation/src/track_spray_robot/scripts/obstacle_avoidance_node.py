#!/usr/bin/env python3
"""
Obstacle Avoidance Node for Track Spray Robot
Uses LiDAR sensor data for obstacle recognition
"""

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
import time

class ObstacleAvoidance:
    def __init__(self):
        rospy.init_node('obstacle_avoidance', anonymous=False)
        
        self.obstacle_distance_threshold = rospy.get_param('~obstacle_distance_threshold', 1.0)
        self.forward_velocity = rospy.get_param('~forward_velocity', 0.5)
        self.angular_velocity = rospy.get_param('~angular_velocity', 0.5) 
        
        # Subscriber for LiDAR-data
        self.lidar_sub = rospy.Subscriber('robot/scan', LaserScan, self.lidar_callback)
        
        # Publisher for movement commands
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        
        # State Variables
        self.current_scan = None
        self.robot_stopped = False
        self.last_log_time = time.time()
        self.log_interval = 1.0 
        
        rospy.loginfo("Obstacle Avoidance Node gestartet")
        rospy.loginfo(f"Obstacle Distance Threshold: {self.obstacle_distance_threshold} m")
        
    def lidar_callback(self, msg):
        """
        Callback for LiDAR-sensor data
        Args:
            msg (LaserScan): LiDAR-sensor measurement
        """
        self.current_scan = msg
        self.process_scan()
    
    def process_scan(self):
        """
        Processes LiDAR-sensor data and controlls the robot accordingly
        """
        if self.current_scan is None:
            return
        
        ranges = self.current_scan.ranges
        angle_min = self.current_scan.angle_min
        angle_increment = self.current_scan.angle_increment
        
        # -- Filtering invalid data --
        valid_ranges = []
        valid_angles = []
        
        for i, range_val in enumerate(ranges):
            if math.isfinite(range_val):
                if range_val >= self.current_scan.range_min and range_val <= self.current_scan.range_max:
                    angle = angle_min + i * angle_increment
                    valid_ranges.append(range_val)
                    valid_angles.append(angle)
        
        if not valid_ranges:
            rospy.logwarn("No valid LiDAR-measurements received")
            return
        
        # -- Calculation of minimal distance --
        front_ranges = []
        for i, angle in enumerate(valid_angles):
            # angle range of -30° until +30°
            if abs(angle) < math.radians(30):
                front_ranges.append(valid_ranges[i])
        
        if front_ranges:
            min_front_distance = min(front_ranges)
        else:
            min_front_distance = float('inf')
        
        # -- Logging --
        current_time = time.time()
        if current_time - self.last_log_time >= self.log_interval:
            rospy.loginfo(f"Minimal Distance in front: {min_front_distance:.2f} m")
            self.last_log_time = current_time
        
        # -- Controll logic for movements --
        cmd_vel = Twist()
        
        if min_front_distance > self.obstacle_distance_threshold:
            # drive forward if no obstacle in sight
            cmd_vel.linear.x = self.forward_velocity
            cmd_vel.angular.z = 0.0
            
            if self.robot_stopped:
                rospy.loginfo("Obstacle resolved - Driving forward again")
                self.robot_stopped = False
        else:
            # stop if obstacle in front
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            
            if not self.robot_stopped:
                rospy.logwarn(f"⚠️ OBSTACLE IN FRONT! Distance: {min_front_distance:.2f} m (Threshold: {self.obstacle_distance_threshold} m)")
                rospy.logwarn("🛑 Robot is being stopped")
                self.robot_stopped = True
        
        self.cmd_vel_pub.publish(cmd_vel)
    
    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = ObstacleAvoidance()
        node.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Obstacle Avoidance Node was interrupted")