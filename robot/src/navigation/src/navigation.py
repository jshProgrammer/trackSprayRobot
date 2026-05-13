#!/usr/bin/env python3
"""
Navigation Node – GPS-basiert (RTK)
Position kommt direkt von gps/fix (NavSatFix).

Erweiterbar für:
  - IMU  → Heading aus Magnetometer statt aus GPS-Bewegung
  - Odometrie/Encoder → lokale Dead-Reckoning als Fallback
  - EKF  → sensor_msgs/Imu + nav_msgs/Odometry fusionieren
"""

import math
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix, NavSatStatus

# ═══════════════════════════════════════════════════════════════════════
# KONSTANTEN
# ═══════════════════════════════════════════════════════════════════════
EARTH_RADIUS = 6_371_000.0   # m


class NavigationNode:
    def __init__(self):
        self._init_params()
        self._init_state()
        self._init_ros()

        rospy.loginfo("Navigation Node gestartet (GPS-Modus)")
        rospy.loginfo(f"Waypoints: {self.waypoints}")
        rospy.loginfo("Warte auf GPS-Fix...")

    # ════════════════════════════════════════════════════════════════════
    # INIT
    # ════════════════════════════════════════════════════════════════════
    def _init_params(self):
        # ── Vom Motor Driver geerbt (global, kein ~) ─────────────────────
        self.wheel_base  = rospy.get_param('~wheel_base')
        self.max_linear  = rospy.get_param('~max_linear')
        self.max_angular = rospy.get_param('~max_angular')

        # ── Nur Navigation (lokal, mit ~) ────────────────────────────────
        self.forward_velocity   = rospy.get_param('~forward_velocity')
        self.angular_velocity   = rospy.get_param('~angular_velocity')
        self.distance_tolerance = rospy.get_param('~distance_tolerance')
        self.angle_tolerance    = rospy.get_param('~angle_tolerance')
        self.waypoints          = rospy.get_param('~waypoints', [])
        self.min_gps_status     = rospy.get_param('~min_gps_status', 0)

    def _init_state(self):
        # ── GPS-State ────────────────────────────────────────────────────
        self.current_lat  = None
        self.current_lon  = None
        self.has_fix      = False

        # ── Ursprung für lokale XY-Konvertierung ─────────────────────────
        # Wird beim ersten Fix automatisch gesetzt
        self.origin_lat   = None
        self.origin_lon   = None

        # ── Heading-Schätzung aus GPS-Bewegung ───────────────────────────
        # TODO: durch IMU (sensor_msgs/Imu) ersetzen sobald verfügbar
        self.heading      = None   # rad, None = unbekannt
        self.prev_x       = None
        self.prev_y       = None

        # ── Waypoint-State ───────────────────────────────────────────────
        self.current_waypoint_index = 0
        self.at_goal                = False

    def _init_ros(self):
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        rospy.Subscriber('gps/fix', NavSatFix, self._gps_callback)

        # TODO: IMU-Subscriber hier ergänzen, sobald Hardware vorhanden:
        # rospy.Subscriber('imu/data', Imu, self._imu_callback)

        # TODO: Odometrie-Subscriber für Encoder-Feedback:
        # rospy.Subscriber('odom', Odometry, self._odom_callback)

        rospy.Timer(rospy.Duration(0.01), self._control_loop)   # more than 10 Hz

    # ════════════════════════════════════════════════════════════════════
    # GPS CALLBACK
    # ════════════════════════════════════════════════════════════════════
    def _gps_callback(self, msg: NavSatFix):
        if msg.status.status < self.min_gps_status:
            if self.has_fix:
                rospy.logwarn_throttle(5, "GPS-Fix verloren!")
                self.has_fix = False
            return

        self.current_lat = msg.latitude
        self.current_lon = msg.longitude

        # Ursprung beim allerersten Fix setzen
        if self.origin_lat is None:
            self.origin_lat = msg.latitude
            self.origin_lon = msg.longitude
            rospy.loginfo(
                f"Ursprung gesetzt: lat={self.origin_lat:.7f}, lon={self.origin_lon:.7f}"
            )

        self.has_fix = True

    # ════════════════════════════════════════════════════════════════════
    # KOORDINATEN-UMRECHNUNG
    # ════════════════════════════════════════════════════════════════════
    def _gps_to_xy(self, lat: float, lon: float):
        """
        Konvertiert GPS-Koordinaten in lokale XY-Meter relativ zum Ursprung.
        Ausreichend genau für kurze Distanzen (< 1 km).
        """
        dlat = math.radians(lat - self.origin_lat)
        dlon = math.radians(lon - self.origin_lon)
        x = dlon * EARTH_RADIUS * math.cos(math.radians(self.origin_lat))
        y = dlat * EARTH_RADIUS
        return x, y

    # ════════════════════════════════════════════════════════════════════
    # HEADING-SCHÄTZUNG (aus GPS-Bewegung)
    # ════════════════════════════════════════════════════════════════════
    def _update_heading(self, x: float, y: float):
        """
        Schätzt den Heading aus zwei aufeinanderfolgenden GPS-Positionen.
        Nur sinnvoll wenn der Roboter sich bewegt (> ~0.2 m Versatz).

        TODO: Diese Methode entfernen / auskommentieren sobald IMU vorhanden.
              Dann: self.heading = self._imu_heading
        """
        if self.prev_x is None:
            self.prev_x, self.prev_y = x, y
            return

        dx = x - self.prev_x
        dy = y - self.prev_y
        moved = math.sqrt(dx**2 + dy**2)

        if moved > 0.2:   # Mindestbewegung für sinnvollen Heading
            self.heading = math.atan2(dy, dx)
            self.prev_x  = x
            self.prev_y  = y

    # ════════════════════════════════════════════════════════════════════
    # HAUPTREGELSCHLEIFE (10 Hz)
    # ════════════════════════════════════════════════════════════════════
    def _control_loop(self, event):
        # ── Kein Fix? Anhalten ───────────────────────────────────────────
        if not self.has_fix:
            self._publish(0.0, 0.0)
            return

        # ── Alle Waypoints abgefahren? ───────────────────────────────────
        if self.current_waypoint_index >= len(self.waypoints):
            if not self.at_goal:
                rospy.loginfo("Alle Waypoints erreicht! Roboter stoppt.")
                self.at_goal = True
            self._publish(0.0, 0.0)
            return

        # ── Aktuelle Position als XY ─────────────────────────────────────
        cur_x, cur_y = self._gps_to_xy(self.current_lat, self.current_lon)
        self._update_heading(cur_x, cur_y)

        # ── Ziel-Waypoint ────────────────────────────────────────────────
        goal_lat, goal_lon = self.waypoints[self.current_waypoint_index]
        goal_x, goal_y     = self._gps_to_xy(goal_lat, goal_lon)

        dx       = goal_x - cur_x
        dy       = goal_y - cur_y
        distance = math.sqrt(dx**2 + dy**2)

        rospy.logdebug(
            f"WP {self.current_waypoint_index + 1}/{len(self.waypoints)}: "
            f"pos=({cur_x:.2f},{cur_y:.2f}) "
            f"ziel=({goal_x:.2f},{goal_y:.2f}) "
            f"dist={distance:.2f}m"
        )

        # ── Waypoint erreicht? ───────────────────────────────────────────
        if distance < self.distance_tolerance:
            rospy.loginfo(
                f"Waypoint {self.current_waypoint_index + 1} erreicht "
                f"(lat={goal_lat}, lon={goal_lon})"
            )
            self.current_waypoint_index += 1
            self._publish(0.0, 0.0)
            return

        # ── Heading unbekannt → erst geradeaus fahren um ihn zu ermitteln
        if self.heading is None:
            rospy.loginfo_throttle(2, "Heading noch unbekannt – fahre geradeaus zum Kalibrieren")
            self._publish(self.forward_velocity * 0.5, 0.0)
            return

        # ── Winkel zum Ziel ──────────────────────────────────────────────
        angle_to_goal = math.atan2(dy, dx) - self.heading
        angle_to_goal = math.atan2(math.sin(angle_to_goal), math.cos(angle_to_goal))
        angle_deg     = math.degrees(angle_to_goal)

        #rospy.loginfo(f"Angle to goal: {angle_deg:.2f} degrees, distance: {distance:.2f}")

        # ── Regellogik: erst drehen, dann fahren ─────────────────────────
        if abs(angle_deg) > self.angle_tolerance:
            turn = math.copysign(self.angular_velocity, angle_to_goal)
            rospy.loginfo(f"Drehe zum Ziel: turn={turn:.2f} rad/s")
            self._publish(0.0, turn)
        else:
            # Sanftes Abbremsen nahe am Ziel
            speed = min(self.forward_velocity, distance * 1.5)
            speed = max(speed, 0.05)   # Mindestgeschwindigkeit
            #rospy.loginfo(f"Sanftes abbremsen am ende: speed={speed:.2f}")
            self._publish(speed, 0.0)

    # ════════════════════════════════════════════════════════════════════
    # HILFSMETHODEN
    # ════════════════════════════════════════════════════════════════════
    def _publish(self, linear: float, angular: float):
        msg = Twist()
        msg.linear.x  = linear
        msg.angular.z = angular
        #rospy.loginfo(f"Publishing cmd_vel: linear={linear:.2f}, angular={angular:.2f}")
        self.cmd_vel_pub.publish(msg)


# ═══════════════════════════════════════════════════════════════════════
if __name__ == '__main__':
    try:
        rospy.init_node('navigation_node')
        node = NavigationNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass