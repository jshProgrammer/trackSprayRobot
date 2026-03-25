#!/usr/bin/env python3
"""
drive_and_spray_node.py
=======================
Faehrt geradeaus, stoppt alle 2 m (Odometrie), betaetigt Spray-Aktuator,
spawnt orangen Farbfleck in Gazebo.

Rueckwaertskompatibel: python drive_and_spray_node.py laeuft wie bisher.
Backend per ROS-Parameter waehlbar:
  rosparam set /drive_and_spray/backend sim   # Standard
  rosparam set /drive_and_spray/backend hw    # Echte Hardware
"""

import math
import rospy
import os 
import sys
from nav_msgs.msg   import Odometry
from geometry_msgs.msg import Point

# ------------------------------------------------------------------ #
#  Konfiguration (alle Werte auch als ROS-Parameter ueberschreibbar)  #
# ------------------------------------------------------------------ #
def _p(name, default):
    """Liest ROS-Parameter, faellt auf Default zurueck."""
    return rospy.get_param('~' + name, default)

DRIVE_DISTANCE    = _p('drive_distance',    2.0)   # m
FORWARD_VELOCITY  = _p('forward_velocity',  0.5)   # m/s
STROKE_DOWN_M     = _p('stroke_down',       0.10)  # m
STROKE_UP_M       = _p('stroke_up',         0.0)   # m
STROKE_DURATION_S = _p('stroke_duration',   0.6)   # s
SPRAY_DWELL_TIME  = _p('spray_dwell_time',  1.0)   # s
# ------------------------------------------------------------------ #

# Zustandsmaschine
STATE_DRIVING     = 0
STATE_STROKE_DOWN = 1
STATE_DWELL       = 2
STATE_STROKE_UP   = 3


def _load_backends():
    scripts_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)))
    if scripts_dir not in sys.path:
        sys.path.insert(0, scripts_dir)

    backend = rospy.get_param('~backend', 'sim').strip().lower()
    rospy.loginfo("drive_and_spray: Backend = '%s'", backend)

    if backend == 'hw':
        from hw_backend import HwDrive, HwActuator, HwMarker
        return HwDrive(), HwActuator(), HwMarker()
    else:
        from sim_backend import SimDrive, SimActuator, SimMarker
        return SimDrive(), SimActuator(), SimMarker()


class DriveAndSprayNode:

    def __init__(self):
        rospy.init_node('drive_and_spray', anonymous=False)

        # Backends laden (Sim oder HW)
        self._drive, self._actuator, self._marker = _load_backends()

        # --- Subscriber ---
        rospy.Subscriber('/odom', Odometry, self._odom_callback, queue_size=10)

        # --- Odometrie ---
        self.last_x          = None
        self.last_y          = None
        self.dist_since_stop = 0.0

        # --- Zustandsmaschine ---
        self.state           = STATE_DRIVING
        self.state_start     = None
        self.spray_position  = Point()

        # --- Aktuator ---
        self.joint_position  = STROKE_UP_M

        # --- Marker ---
        self.marker_id       = 0

        rospy.loginfo("drive_and_spray: gestartet – stoppe alle %.1f m", DRIVE_DISTANCE)

    # ---------------------------------------------------------------- #
    #  Odometrie-Callback                                               #
    # ---------------------------------------------------------------- #

    def _odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if self.last_x is None:
            self.last_x = x
            self.last_y = y
            return

        if self.state == STATE_DRIVING:
            dx   = x - self.last_x
            dy   = y - self.last_y
            step = math.sqrt(dx * dx + dy * dy)
            self.dist_since_stop += step

            if self.dist_since_stop >= DRIVE_DISTANCE:
                self.dist_since_stop = 0.0
                self.spray_position  = Point(x=x, y=y, z=0.0)
                self._enter_state(STATE_STROKE_DOWN)
                rospy.loginfo("drive_and_spray: %.1f m erreicht – Spray-Zyklus startet",
                              DRIVE_DISTANCE)

        self.last_x = x
        self.last_y = y

    # ---------------------------------------------------------------- #
    #  Zustandsmaschine                                                 #
    # ---------------------------------------------------------------- #

    def _enter_state(self, new_state):
        self.state       = new_state
        self.state_start = rospy.Time.now()

        if new_state == STATE_STROKE_DOWN:
            self._actuator.move_to(STROKE_DOWN_M, STROKE_DURATION_S)
            rospy.loginfo("drive_and_spray: Stab faehrt AUS")

        elif new_state == STATE_DWELL:
            rospy.loginfo("drive_and_spray: Stab unten – Marker setzen")
            self._marker.place_marker(self.spray_position, self.marker_id)
            self.marker_id += 1

        elif new_state == STATE_STROKE_UP:
            self._actuator.move_to(STROKE_UP_M, STROKE_DURATION_S)
            rospy.loginfo("drive_and_spray: Stab faehrt EIN")

        elif new_state == STATE_DRIVING:
            rospy.loginfo("drive_and_spray: Weiterfahren")

    def _update_state(self):
        if self.state == STATE_DRIVING or self.state_start is None:
            return

        elapsed = (rospy.Time.now() - self.state_start).to_sec()

        if self.state == STATE_STROKE_DOWN and elapsed >= STROKE_DURATION_S:
            self._enter_state(STATE_DWELL)

        elif self.state == STATE_DWELL and elapsed >= SPRAY_DWELL_TIME:
            self._enter_state(STATE_STROKE_UP)

        elif self.state == STATE_STROKE_UP and elapsed >= STROKE_DURATION_S:
            self._enter_state(STATE_DRIVING)

    # ---------------------------------------------------------------- #
    #  Haupt-Loop                                                       #
    # ---------------------------------------------------------------- #

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self._update_state()
            self._drive.set_velocity(
                FORWARD_VELOCITY if self.state == STATE_DRIVING else 0.0
            )
            rate.sleep()


if __name__ == '__main__':
    try:
        node = DriveAndSprayNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("drive_and_spray: beendet")