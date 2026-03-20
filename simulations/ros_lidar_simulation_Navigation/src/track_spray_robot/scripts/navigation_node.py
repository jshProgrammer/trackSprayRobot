#!/usr/bin/env python3
"""
Navigation Node for Track Spray Robot
Navigates to a hardcoded goal point while avoiding obstacles
"""

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
import math
import time

class NavigationNode:
    def __init__(self):
        rospy.init_node('navigation_node', anonymous=False)
        
        # Parameters
        self.obstacle_distance_threshold = rospy.get_param('~obstacle_distance_threshold', 1.0)
        self.forward_velocity = rospy.get_param('~forward_velocity', 0.3)
        self.angular_velocity = rospy.get_param('~angular_velocity', 0.5)
        self.goal_x = rospy.get_param('~goal_x', 5.0)  # Hardcoded goal
        self.goal_y = rospy.get_param('~goal_y', 2.0)
        self.distance_tolerance = 0.1  # Stop when within this distance
        
        # Subscribers
        self.lidar_sub = rospy.Subscriber('robot/scan', LaserScan, self.lidar_callback)
        self.model_states_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback)
        
        # Publisher
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        
        # State
        self.current_scan = None
        self.current_pose = None
        self.robot_stopped = False
        self.at_goal = False
        self.last_log_time = time.time()
        self.log_interval = 1.0
        
        rospy.loginfo("Navigation Node gestartet")
        rospy.loginfo(f"Goal: ({self.goal_x}, {self.goal_y})")
        
    def model_states_callback(self, msg):
        try:
            index = msg.name.index('track_spray_robot')
            self.current_pose = msg.pose[index]
        except ValueError:
            pass
    
    def lidar_callback(self, msg):
        self.current_scan = msg
        self.navigate()
    
    def navigate(self):
        if self.current_pose is None or self.current_scan is None:
            return
        
        # Get current position
        x = self.current_pose.position.x
        y = self.current_pose.position.y
        orientation = self.current_pose.orientation
        # Convert quaternion to yaw
        yaw = math.atan2(2.0*(orientation.w*orientation.z + orientation.x*orientation.y),
                         1.0 - 2.0*(orientation.y*orientation.y + orientation.z*orientation.z))
        
        # Calculate distance and angle to goal
        dx = self.goal_x - x
        dy = self.goal_y - y
        distance = math.sqrt(dx**2 + dy**2)
        angle_to_goal = math.atan2(dy, dx) - yaw
        angle_to_goal = math.atan2(math.sin(angle_to_goal), math.cos(angle_to_goal))  # Normalize to -pi to pi
        
        # Check if at goal
        if distance < self.distance_tolerance:
            if not self.at_goal:
                rospy.loginfo(f"Reached goal! Position: ({x:.2f}, {y:.2f})")
                self.at_goal = True
            cmd_vel = Twist()
            self.cmd_vel_pub.publish(cmd_vel)
            return
        
        self.at_goal = False
        
        # Process scan for obstacles
        ranges = self.current_scan.ranges
        angle_min = self.current_scan.angle_min
        angle_increment = self.current_scan.angle_increment
        
        valid_ranges = []
        valid_angles = []
        for i, range_val in enumerate(ranges):
            if math.isfinite(range_val) and range_val >= self.current_scan.range_min and range_val <= self.current_scan.range_max:
                angle = angle_min + i * angle_increment
                valid_ranges.append(range_val)
                valid_angles.append(angle)
        
        front_ranges = [r for r, a in zip(valid_ranges, valid_angles) if abs(a) < math.radians(30)]
        min_front_distance = min(front_ranges) if front_ranges else float('inf')
        
        # Logging
        current_time = time.time()
        if current_time - self.last_log_time >= self.log_interval:
            rospy.loginfo(f"Position: ({x:.2f}, {y:.2f}), Distance to goal: {distance:.2f}, Angle to goal: {math.degrees(angle_to_goal):.1f}°")
            self.last_log_time = current_time
        
        # Control logic
        cmd_vel = Twist()
        
        if min_front_distance > self.obstacle_distance_threshold:
            # No obstacle, navigate to goal
            if abs(angle_to_goal) > math.radians(5):  # Turn if angle > 5 degrees
                cmd_vel.angular.z = self.angular_velocity if angle_to_goal > 0 else -self.angular_velocity
            else:
                cmd_vel.linear.x = self.forward_velocity
        else:
            # Obstacle, stop
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            if not self.robot_stopped:
                rospy.logwarn(f"Obstacle detected! Distance: {min_front_distance:.2f} m")
                self.robot_stopped = True
        
        self.cmd_vel_pub.publish(cmd_vel)
    
    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = NavigationNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass