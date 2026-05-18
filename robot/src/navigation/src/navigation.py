#!/usr/bin/env python3
"""
Navigation Node – GPS-basiert (RTK) + IMU Heading
Position kommt direkt von gps/fix (NavSatFix).
Heading kommt von imu/data (sensor_msgs/Imu) via Quaternion → Yaw.
 
Erweiterbar für:
  - Odometrie/Encoder → lokale Dead-Reckoning als Fallback
  - EKF  → sensor_msgs/Imu + nav_msgs/Odometry fusionieren
"""

import math
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu
import os
import datetime
import logging
from std_msgs.msg import UInt8

# ═══════════════════════════════════════════════════════════════════════
# KONSTANTEN
# ═══════════════════════════════════════════════════════════════════════
EARTH_RADIUS = 6_371_000.0   # m

def quaternion_to_yaw(q) -> float:
    """
    Extrahiert den Yaw-Winkel (Heading) aus einem Quaternion.
    Gibt Winkel in Radiant zurück (ENU-Frame: 0 = Ost, π/2 = Nord).
    """
    # Rotation um Z-Achse: yaw = atan2(2*(w*z + x*y), 1 - 2*(y² + z²))
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class NavigationNode:
    def __init__(self):
        self.init_logging()
        self._init_params()
        self._init_state()
        self._init_ros()
        
        debugOutput = "Navigation Node gestartet (GPS-Modus)"
        rospy.loginfo(debugOutput)
        self.logger.info(debugOutput)

        self.logger.info(f"Waypoints: {self.waypoints}")

        debugOutput = "Warte auf GPS-Fix..."
        rospy.loginfo(debugOutput)
        self.logger.info(debugOutput)

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
        self.had_rtk_fix = False
        self.gps_quality = 0

        # ── Ursprung für lokale XY-Konvertierung ─────────────────────────
        # Wird beim ersten Fix automatisch gesetzt
        self.origin_lat   = None
        self.origin_lon   = None

       # ── Heading aus IMU ───────────────────────────────────────────────
        self.heading      = None   # rad, None = noch kein IMU-Paket empfangen
        self.has_imu      = False

        # ── Waypoint-State ───────────────────────────────────────────────
        self.current_waypoint_index = 0
        self.at_goal                = False

    def _init_ros(self):
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel_controll', Twist, queue_size=1)

        rospy.Subscriber('gps/fix', NavSatFix, self._gps_callback)

        rospy.Subscriber(
            'gps/quality',
            UInt8,
            self._gps_quality_callback
        )

        rospy.Subscriber('imu/data', Imu, self._imu_callback)

        # TODO: Odometrie-Subscriber für Encoder-Feedback:
        # rospy.Subscriber('odom', Odometry, self._odom_callback)

        rospy.Timer(rospy.Duration(0.01), self._control_loop)   # more than 10 Hz

    # =========================
    # Additional Debug logging
    # =========================
    def init_logging(self):
        log_dir = os.path.expanduser(f"~/trackRobotLogs/trackRobot_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}")
        os.makedirs(log_dir, exist_ok=True)

        log_file = os.path.join(
            log_dir,
            "navigation_node.log"
        )

        self.logger = logging.getLogger("navigation_node")
        self.logger.setLevel(logging.INFO)

        file_handler = logging.FileHandler(log_file)
        formatter = logging.Formatter(
            '%(asctime)s [%(levelname)s] %(message)s'
        )

        file_handler.setFormatter(formatter)
        self.logger.addHandler(file_handler)


    # ════════════════════════════════════════════════════════════════════
    # GPS CALLBACK
    # ════════════════════════════════════════════════════════════════════
    def _gps_callback(self, msg: NavSatFix):
        # ─────────────────────────────────────────────────────────────
        # Mindestqualität prüfen
        # ─────────────────────────────────────────────────────────────
        if msg.status.status < self.min_gps_status:
            if self.has_fix:
                debugOutput = (
                    f"GPS-Fix verloren! "
                    f"aktueller status={msg.status.status}"
                )

                rospy.logwarn_throttle(5, debugOutput)
                self.logger.warning(debugOutput)

                self.has_fix = False

            return
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude

        debugOutput = (
            f"GPS Fix: lat={self.current_lat:.7f}, "
            f"lon={self.current_lon:.7f}"
        )
        rospy.loginfo_throttle(1, debugOutput)
        self.logger.info(debugOutput)

        # Ursprung beim allerersten Fix setzen
        if self.origin_lat is None:
            self.origin_lat = msg.latitude
            self.origin_lon = msg.longitude
            debugOutput =  f"Ursprung gesetzt: lat={self.origin_lat:.7f}, lon={self.origin_lon:.7f}"
            rospy.loginfo(debugOutput)
            self.logger.info(debugOutput)

        self.has_fix = True



    def _gps_quality_callback(self, msg):
        self.gps_quality = msg.data

        rospy.loginfo_throttle(
            1,
            f"GPS Quality = {self.gps_quality}"
        )

        # 4 = RTK FIX
        if self.gps_quality >= 4 and not self.had_rtk_fix:
            self.had_rtk_fix = True

            rospy.loginfo(
                "RTK FIX erreicht -> Navigation freigegeben"
            )

    # ════════════════════════════════════════════════════════════════════
    # IMU CALLBACK
    # ════════════════════════════════════════════════════════════════════
    def _imu_callback(self, msg: Imu):
        """
        Liest den Yaw-Winkel aus dem IMU-Quaternion.
 
        Wichtig: Die IMU muss im ENU-Frame kalibriert sein
        (REP-103: X=Ost, Y=Nord, Z=Oben), damit yaw=0 → Ost und
        der berechnete angle_to_goal zum GPS-Bearing passt.
 
        Falls deine IMU im NED-Frame arbeitet (X=Nord, Y=Ost, Z=Unten),
        muss das Quaternion vorher konvertiert werden – oder du verwendest
        ein IMU-Treiber-Package das REP-103 bereits umsetzt (z.B. imu_filter_madgwick).
        """
        q = msg.orientation
        # Rohwerte loggen
        rospy.loginfo_throttle(0.5,
            f"RAW Quaternion: x={q.x:.4f} y={q.y:.4f} z={q.z:.4f} w={q.w:.4f}"
        )
        self.heading = quaternion_to_yaw(q)


        heading_deg = math.degrees(self.heading)

        debugOutput = (
            f"IMU Heading: {heading_deg:.2f}°"
        )
        rospy.loginfo_throttle(1, debugOutput)
        self.logger.info(debugOutput)

        if not self.has_imu:
            self.has_imu = True
            rospy.loginfo(f"IMU aktiv. Erster Heading: {math.degrees(self.heading):.1f}°")
            self.logger.info(f"IMU aktiv. Erster Heading: {math.degrees(self.heading):.1f}°")

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
    # HAUPTREGELSCHLEIFE (100 Hz)
    # ════════════════════════════════════════════════════════════════════
    def _control_loop(self, event):
        # ─────────────────────────────────────────────────────────────
        # Warten bis mindestens einmal RTK-Fix vorhanden war
        # ─────────────────────────────────────────────────────────────
        if not self.had_rtk_fix:
            rospy.logwarn_throttle(
                2,
                "Warte auf ersten RTK-Fix (status=4)..."
            )

            self._publish(0.0, 0.0)
            return

        # ── Kein Fix? Anhalten ───────────────────────────────────────────
        #TODO: remove in future version as robot should be able to briefly drive without gps fix
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
        #self._update_heading(cur_x, cur_y)

        # ── Ziel-Waypoint ────────────────────────────────────────────────
        goal_lat, goal_lon = self.waypoints[self.current_waypoint_index]
        goal_x, goal_y     = self._gps_to_xy(goal_lat, goal_lon)

        dx       = goal_x - cur_x
        dy       = goal_y - cur_y
        distance = math.sqrt(dx**2 + dy**2)

        debugOutput = (
            f"\n"
            f"========== NAVIGATION ==========\n"
            f"Current GPS:\n"
            f"  lat={self.current_lat:.7f}\n"
            f"  lon={self.current_lon:.7f}\n"
            f"\n"
            f"Current XY:\n"
            f"  x={cur_x:.2f} m\n"
            f"  y={cur_y:.2f} m\n"
            f"\n"
            f"Goal GPS:\n"
            f"  lat={goal_lat:.7f}\n"
            f"  lon={goal_lon:.7f}\n"
            f"\n"
            f"Goal XY:\n"
            f"  x={goal_x:.2f} m\n"
            f"  y={goal_y:.2f} m\n"
            f"\n"
            f"Delta:\n"
            f"  dx={dx:.2f}\n"
            f"  dy={dy:.2f}\n"
            f"\n"
            f"Distance:\n"
            f"  {distance:.2f} m\n"
            f"================================"
        )
        rospy.logdebug(debugOutput)
        self.logger.info(debugOutput)

        # ── Waypoint erreicht? ───────────────────────────────────────────
        if distance < self.distance_tolerance:
            debugOutput = (
                f"Waypoint {self.current_waypoint_index + 1} erreicht "
                f"(lat={goal_lat}, lon={goal_lon})"
            )
            rospy.loginfo(debugOutput)
            self.logger.info(debugOutput)

            self.current_waypoint_index += 1
            self._publish(0.0, 0.0)
            return

        # ── Heading unbekannt (noch kein IMU-Paket) → warten ────────────
        if self.heading is None:
            rospy.logwarn_throttle(2, "Warte auf IMU-Daten – Roboter steht still.")
            self._publish(0.0, 0.0)
            return

        # ── Winkel zum Ziel ──────────────────────────────────────────────
        angle_to_goal = math.atan2(dy, dx) - self.heading
        angle_to_goal = math.atan2(math.sin(angle_to_goal), math.cos(angle_to_goal))
        angle_deg     = math.degrees(angle_to_goal)

        target_heading = math.degrees(math.atan2(dy, dx))


        rospy.loginfo(
            f"IMU={math.degrees(self.heading):.1f}° "
            f"TARGET={target_heading:.1f}°"
        )

        debugOutput = (
            f"\n"
            f"===== ANGLE DEBUG =====\n"
            f"Robot Heading: {math.degrees(self.heading):.2f}°\n"
            f"Target Heading: {target_heading:.2f}°\n"
            f"Angle Error: {angle_deg:.2f}°\n"
            f"Tolerance: {self.angle_tolerance:.2f}°\n"
            f"======================="
        )

        rospy.loginfo_throttle(1, debugOutput)
        self.logger.info(debugOutput)

        debugOutput = (
            f"Heading={math.degrees(self.heading):.1f}° "
            f"Zielwinkel={angle_deg:.1f}°"
        )
        rospy.logdebug(debugOutput)
        self.logger.info(debugOutput)

        # ── Regellogik: erst drehen, dann fahren ─────────────────────────
        if abs(angle_deg) > self.angle_tolerance:
            turn = math.copysign(self.angular_velocity, angle_to_goal)
            debugOutput = f"Drehe zum Ziel: turn={turn:.2f} rad/s"
            rospy.loginfo(debugOutput)
            self.logger.info(debugOutput)
            self._publish(0.0, turn)
        else:
            # Sanftes Abbremsen nahe am Ziel
            speed = min(self.forward_velocity, distance * 1.5)
            speed = max(speed, 0.05)   # Mindestgeschwindigkeit
            debugOutput = f"Sanftes abbremsen am ende: speed={speed:.2f}"
            rospy.loginfo(debugOutput)
            self.logger.info(debugOutput)
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

        rospy.loginfo_throttle(
            1,
            f"CMD_VEL -> linear={linear:.2f}, angular={angular:.2f}"
        )


# ═══════════════════════════════════════════════════════════════════════
if __name__ == '__main__':
    try:
        rospy.init_node('navigation_node')
        node = NavigationNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass