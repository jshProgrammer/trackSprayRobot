#!/usr/bin/env python3
"""
hw_backend.py
=============
Hardware-Implementierungen der drei Interfaces.

Hier an echte HW dann anpassen das der ganze Spass dann auch in echt läuft
"""

import rospy
from geometry_msgs.msg import Point

from interfaces import DriveInterface, ActuatorInterface, MarkerInterface


class HwDrive(DriveInterface):
    """
    Beispiel: Motor-Controller ueber ROS-Topic oder Serial.
    Ersetze den Stub durch deinen echten Treiber-Aufruf.
    """

    def __init__(self):
        # Beispiel: self._serial = serial.Serial('/dev/ttyUSB0', 115200)
        rospy.loginfo("hw_backend: HwDrive bereit (Stub)")

    def set_velocity(self, linear_x: float) -> None:
        # Beispiel: self._serial.write(f"V {linear_x:.3f}\n".encode())
        rospy.loginfo_throttle(1.0, "HwDrive.set_velocity(%.2f)", linear_x)


class HwActuator(ActuatorInterface):
    """
    Beispiel: PWM-Servo oder Pneumatik-Magnetventil ueber GPIO.
    """

    def __init__(self):
        # Beispiel: GPIO.setup(PIN, GPIO.OUT)
        rospy.loginfo("hw_backend: HwActuator bereit (Stub)")

    def move_to(self, position: float, duration: float) -> None:
        # Beispiel: GPIO.output(PIN, position > 0)
        rospy.loginfo("HwActuator.move_to(pos=%.3f, dur=%.2f)", position, duration)


class HwMarker(MarkerInterface):
    """
    Beispiel: GPS-Koordinate in Datenbank schreiben oder
    Leuchtmarkierung physisch ausloesen.
    """

    def __init__(self):
        rospy.loginfo("hw_backend: HwMarker bereit (Stub)")

    def place_marker(self, position: Point, marker_id: int) -> None:
        # Beispiel: db.insert(marker_id, position.x, position.y)
        rospy.loginfo("HwMarker.place_marker(id=%d, x=%.2f, y=%.2f)",
                      marker_id, position.x, position.y)