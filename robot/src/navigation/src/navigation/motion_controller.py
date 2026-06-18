"""Proportionaler Lenkungsregler.

``MotionController`` kapselt die Regler-Mathematik, das Logging und das
Publishen auf /cmd_vel_controll – analog zu RTKTracker/HeadingCalibrator
bekommt er Params und Logger im Konstruktor.
"""

import math

import rospy
from geometry_msgs.msg import Twist

from navigation.geo import normalize_angle


class MotionController:
    def __init__(self, params, logger):
        self._p = params
        self._logger = logger
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel_controll', Twist, queue_size=1)

    def publish(self, linear: float, angular: float):
        """Sendet einen Twist-Befehl an /cmd_vel_controll."""
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_vel_pub.publish(msg)

    def stop(self):
        self.publish(0.0, 0.0)

    def steer(self, cur_x: float, cur_y: float, target_x: float, target_y: float,
              true_heading: float, raw_heading: float, label: str = ""):
        """Proportionaler Regler: dreht und fährt den Roboter Richtung (target_x, target_y).

        Berechnet den Lenkbefehl, loggt ihn und publisht ihn.
        """
        dx = target_x - cur_x
        dy = target_y - cur_y
        target_heading = math.atan2(dy, dx)
        angle_error = normalize_angle(target_heading - true_heading)
        distance = math.hypot(dx, dy)

        k_p = 1.0
        angular = max(-self._p.angular_velocity,
                      min(self._p.angular_velocity, k_p * angle_error))

        # TODO: attempt to reduce speed when distance <= 1.5
        if abs(angle_error) > math.radians(45):
            linear = 0.05
        else:
            linear = 0.1

        debug_output = (
            f"{label} | "
            f"Distanz: {distance:.2f}m | "
            f"Robot-Angle: {math.degrees(true_heading):.1f}° | "
            f"IMU-Roh: {math.degrees(raw_heading):.1f}° | "
            f"Target-Angle: {math.degrees(target_heading):.1f}°"
        )
        rospy.loginfo_throttle(1, debug_output)
        self._logger.info(debug_output)

        rospy.loginfo(
            f"Target={math.degrees(target_heading):.1f}° "
            f"Robot={math.degrees(true_heading):.1f}° "
            f"Error={math.degrees(angle_error):.1f}° "
            f"Angular={angular:.2f}"
        )
        self.publish(linear, angular)
