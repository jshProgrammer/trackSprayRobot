#!/usr/bin/env python3
"""
Navigation Node für echten Roboter – Dead Reckoning (kein Lidar, kein GPS)
Position wird aus cmd_vel + Zeit integriert.
"""

import rospy
from geometry_msgs.msg import Twist
import math
import time

class NavigationNode:
    def __init__(self):
        rospy.init_node('navigation_node', anonymous=False)

        # ── Physikalische Parameter ──────────────────────────────────────────
        self.wheel_base   = rospy.get_param('~wheel_base')   # Abstand Räder [m]
        self.wheel_radius = rospy.get_param('~wheel_radius')  # Radradius [m]
        self.max_linear   = rospy.get_param('~max_linear')   # wie im MotorDriver
        self.max_angular  = rospy.get_param('~max_angular')   # wie im MotorDriver

        # ── Navigationsparameter ──────────────────────────────────────────── 
        self.distance_tolerance = rospy.get_param('~distance_tolerance') # m
        self.angle_tolerance    = rospy.get_param('~angle_tolerance')    # Grad

        # ── Waypoints ───────────────────────────────────────────────────────
        self.waypoints = rospy.get_param('~waypoints', [[1.0, 1.0]])  # Liste von [x, y]
        self.current_waypoint_index = 0
        self.at_goal = False

        # ── Odometrie-State (Dead Reckoning) ────────────────────────────────
        self.x   = 0.0   # Startposition = Ursprung
        self.y   = 0.0
        self.yaw = 0.0   # Startausrichtung = Blick in +X Richtung
        self.last_time = rospy.Time.now()

        # Letzter gesendeter cmd_vel (für Integration nötig)
        self._last_linear  = 0.0
        self._last_angular = 0.0

        # ── ROS ─────────────────────────────────────────────────────────────
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # Timer: Regelschleife mit 10 Hz
        rospy.Timer(rospy.Duration(0.1), self.control_loop)

        rospy.loginfo("Navigation Node gestartet (Dead Reckoning)")
        rospy.loginfo(f"Waypoints: {self.waypoints}")

    # ════════════════════════════════════════════════════════════════════════
    # ODOMETRIE  –  Position aus letztem cmd_vel hochrechnen
    # ════════════════════════════════════════════════════════════════════════
    def update_odometry(self):
        now = rospy.Time.now()
        dt  = (now - self.last_time).to_sec()
        self.last_time = now

        if dt <= 0.0 or dt > 1.0:   # Sprünge ignorieren
            return

        # Dead Reckoning mit dem zuletzt gesendeten Befehl
        v = self._last_linear
        w = self._last_angular

        # Pose-Integration (Euler-Methode)
        self.x   += v * math.cos(self.yaw) * dt
        self.y   += v * math.sin(self.yaw) * dt
        self.yaw += w * dt
        # Winkel auf [-π, π] normalisieren
        self.yaw  = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

    # ════════════════════════════════════════════════════════════════════════
    # HAUPTREGELSCHLEIFE  (10 Hz Timer)
    # ════════════════════════════════════════════════════════════════════════
    def control_loop(self, event):
        self.update_odometry()

        # Alle Waypoints abgefahren?
        if self.current_waypoint_index >= len(self.waypoints):
            if not self.at_goal:
                rospy.loginfo("Alle Waypoints erreicht! Roboter stoppt.")
                self.at_goal = True
            self._publish(0.0, 0.0)
            return

        goal_x, goal_y = self.waypoints[self.current_waypoint_index]
        dx = goal_x - self.x
        dy = goal_y - self.y
        distance     = math.sqrt(dx**2 + dy**2)
        angle_to_goal = math.atan2(dy, dx) - self.yaw
        angle_to_goal = math.atan2(math.sin(angle_to_goal), math.cos(angle_to_goal))

        rospy.logdebug(
            f"WP {self.current_waypoint_index+1}: "
            f"pos=({self.x:.2f},{self.y:.2f}) "
            f"ziel=({goal_x},{goal_y}) "
            f"dist={distance:.2f}m  angle={math.degrees(angle_to_goal):.1f}°"
        )

        # ── Waypoint erreicht? ───────────────────────────────────────────────
        if distance < self.distance_tolerance:
            rospy.loginfo(f"Waypoint {self.current_waypoint_index+1} erreicht: ({goal_x}, {goal_y})")
            self.current_waypoint_index += 1
            self._publish(0.0, 0.0)
            return

        # ── Regellogik: erst drehen, dann fahren ────────────────────────────
        if abs(math.degrees(angle_to_goal)) > self.angle_tolerance:
            # Drehen auf Richtung
            turn = math.copysign(self.angular_velocity, angle_to_goal)
            self._publish(0.0, turn)
        else:
            # Geradeaus – Geschwindigkeit mit Distanz skalieren damit
            # der Roboter sanft abbremst kurz vor dem Ziel
            speed = min(self.forward_velocity, distance * 1.5)
            speed = max(speed, 0.05)   # Mindestgeschwindigkeit
            self._publish(speed, 0.0)

    # ════════════════════════════════════════════════════════════════════════
    # HILFSMETHODEN
    # ════════════════════════════════════════════════════════════════════════
    def _publish(self, linear: float, angular: float):
        """Speichert den Befehl (für Odometrie) und publisht ihn."""
        self._last_linear  = linear
        self._last_angular = angular
        msg = Twist()
        msg.linear.x  = linear
        msg.angular.z = angular
        self.cmd_vel_pub.publish(msg)

    def spin(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = NavigationNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass