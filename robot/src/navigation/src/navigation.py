#!/usr/bin/env python3
"""
Navigation Node – GPS-basiert (RTK) + IMU Heading (Auto-Calibrated)
Position kommt direkt von gps/fix (NavSatFix).
Heading kommt von imu/data (Gyro-Integration) und wird über die ersten 1.5m 
Fahrtstrecke automatisch zur GPS-Weltkarte ausgerichtet (Kinematic Alignment).
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
    Gibt Winkel in Radiant zurück.
    """
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class NavigationNode:
    def __init__(self):
        self.init_logging()
        self._init_params()
        self._init_state()
        self._init_ros()
        
        debugOutput = "Navigation Node gestartet (Auto-Kalibrierungs-Modus)"
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
        self.wheel_base   = rospy.get_param('~wheel_base', 0.5)
        self.max_linear   = rospy.get_param('~max_linear', 1.0)
        self.max_angular  = rospy.get_param('~max_angular', 1.0)

        self.forward_velocity   = rospy.get_param('~forward_velocity', 0.5)
        self.angular_velocity   = rospy.get_param('~angular_velocity', 0.5)
        self.distance_tolerance = rospy.get_param('~distance_tolerance', 0.3)
        self.angle_tolerance    = rospy.get_param('~angle_tolerance', 5.0)
        self.waypoints          = rospy.get_param('~waypoints', [])
        self.min_gps_status     = rospy.get_param('~min_gps_status', 0)

    def _init_state(self):
        # ── GPS-State ────────────────────────────────────────────────────
        self.current_lat  = None
        self.current_lon  = None
        self.has_fix      = False
        self.had_rtk_fix  = False
        self.gps_quality  = 0

        self.origin_lat   = None
        self.origin_lon   = None

        # ── Heading aus IMU ───────────────────────────────────────────────
        self.heading      = None   # rad, roher Wert aus der IMU (beginnt blind bei 0)
        self.has_imu      = False

        # ── Waypoint-State ───────────────────────────────────────────────
        self.current_waypoint_index = 0
        self.at_goal                = False

        # ── Auto-Kalibrierung (Kinematic Alignment) ──────────────────────
        self.nav_state = "WAITING_FOR_FIX"  # WAITING_FOR_FIX -> CALIBRATING -> NAVIGATING
        self.heading_offset = 0.0
        self.calib_start_x = None
        self.calib_start_y = None
        self.calib_start_time = None

    def _init_ros(self):
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel_controll', Twist, queue_size=1)
        rospy.Subscriber('gps/fix', NavSatFix, self._gps_callback)
        rospy.Subscriber('gps/quality', UInt8, self._gps_quality_callback)
        rospy.Subscriber('imu/data', Imu, self._imu_callback)
        rospy.Timer(rospy.Duration(0.01), self._control_loop)   # 100 Hz

    def init_logging(self):
        log_dir = os.path.expanduser(f"~/trackRobotLogs/trackRobot_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}")
        os.makedirs(log_dir, exist_ok=True)
        log_file = os.path.join(log_dir, "navigation_node.log")

        self.logger = logging.getLogger("navigation_node")
        self.logger.setLevel(logging.INFO)
        file_handler = logging.FileHandler(log_file)
        formatter = logging.Formatter('%(asctime)s [%(levelname)s] %(message)s')
        file_handler.setFormatter(formatter)
        self.logger.addHandler(file_handler)

    # ════════════════════════════════════════════════════════════════════
    # CALLBACKS
    # ════════════════════════════════════════════════════════════════════
    def _gps_callback(self, msg: NavSatFix):
        # Wenn Status nicht ausreicht -> Fehler loggen und abbrechen
        if msg.status.status < self.min_gps_status:
            if self.has_fix:
                debugOutput = f"GPS-Fix verloren! aktueller status={msg.status.status}"
                rospy.logwarn_throttle(5, debugOutput)
                self.logger.warning(debugOutput)
                self.has_fix = False
            return
            
        self.has_fix = True # Fix wiederhergestellt / aktiv
        
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude

        rospy.loginfo_throttle(5, f"GPS Fix: lat={self.current_lat:.7f}, lon={self.current_lon:.7f}")

        # Ursprung beim allerersten guten Fix setzen
        if self.origin_lat is None:
            self.origin_lat = msg.latitude
            self.origin_lon = msg.longitude
            rospy.loginfo(f"Ursprung gesetzt: lat={self.origin_lat:.7f}, lon={self.origin_lon:.7f}")

    def _gps_quality_callback(self, msg):
        self.gps_quality = msg.data
        rospy.loginfo_throttle(5, f"GPS Quality = {self.gps_quality}")

        if self.gps_quality >= 4 and not self.had_rtk_fix:
            self.had_rtk_fix = True
            rospy.loginfo("RTK FIX erreicht -> Navigation freigegeben")

    def _imu_callback(self, msg: Imu):
        q = msg.orientation
        self.heading = quaternion_to_yaw(q)
        
        if not self.has_imu:
            self.has_imu = True
            rospy.loginfo(f"IMU aktiv. Erster roher Heading: {math.degrees(self.heading):.1f}°")

    # ════════════════════════════════════════════════════════════════════
    # KOORDINATEN-UMRECHNUNG
    # ════════════════════════════════════════════════════════════════════
    def _gps_to_xy(self, lat: float, lon: float):
        dlat = math.radians(lat - self.origin_lat)
        dlon = math.radians(lon - self.origin_lon)
        x = dlon * EARTH_RADIUS * math.cos(math.radians(self.origin_lat))
        y = dlat * EARTH_RADIUS
        return x, y

    # ════════════════════════════════════════════════════════════════════
    # HAUPTREGELSCHLEIFE (100 Hz)
    # ════════════════════════════════════════════════════════════════════
    def _control_loop(self, event):
        # ── Start-Bedingungen prüfen ──────────────────────────────────────
        if not self.had_rtk_fix:
            rospy.logwarn_throttle(2, "Warte auf ersten RTK-Fix (status=4)...")
            self._publish(0.0, 0.0)
            return

        if not self.has_fix:
            self._publish(0.0, 0.0)
            return
            
        if self.current_waypoint_index >= len(self.waypoints):
            if not self.at_goal:
                rospy.loginfo("Alle Waypoints erreicht! Roboter stoppt.")
                self.at_goal = True
            self._publish(0.0, 0.0)
            return

        # ── Aktuelle Position ────────────────────────────────────────────
        if self.origin_lat is None or self.origin_lon is None:
            rospy.logwarn_throttle(2, "Warte auf GPS-Ursprung...")
            self._publish(0.0, 0.0)
            return

        # ── Aktuelle Position ────────────────────────────────────────────
        cur_x, cur_y = self._gps_to_xy(self.current_lat, self.current_lon)

        # ═════════════════════════════════════════════════════════════════
        # PHASE 1: AUTOMATISCHE KALIBRIERUNG (Kinematic Alignment)
        # ═════════════════════════════════════════════════════════════════
        if self.nav_state == "WAITING_FOR_FIX":
            self.calib_start_x = cur_x
            self.calib_start_y = cur_y
            self.calib_start_time = rospy.Time.now()

            self.nav_state = "CALIBRATING"
            rospy.loginfo("Starte Auto-Kalibrierung: Fahre 1.5m geradeaus, um den GPS-Vektor zu messen...")
            return

        if self.nav_state == "CALIBRATING":
            if self.heading is None:
                self._publish(0.0, 0.0)
                rospy.logwarn_throttle(2, "Warte auf IMU Daten für Kalibrierung...")
                return

            calib_dx = cur_x - self.calib_start_x
            calib_dy = cur_y - self.calib_start_y
            distance_driven = math.sqrt(calib_dx**2 + calib_dy**2)

            # Drive straight at half speed during calibration
            calib_speed = max(0.1, self.forward_velocity * 0.5)
            self._publish(calib_speed, 0.0)

            rospy.loginfo_throttle(0.5, f"Calibrating... Distance: {distance_driven:.2f}m")

            calib_time = (rospy.Time.now() - self.calib_start_time).to_sec()

            # Sanity check: detect GPS jumps (e.g., >1 m/s unrealistic)
            max_reasonable_distance = calib_time * 1.0
            if distance_driven > max_reasonable_distance:
                rospy.logwarn("GPS jump detected -> ignoring calibration")
                return

            # Calibration trigger: Need BOTH conditions for robust heading estimation:
            # 1. At least 0.5m distance: filters GPS noise, needs spatial separation
            # 2. At least 2.0s elapsed: RTK GPS has ~200-500ms latency, need 4-5 measurements
            # This strategy is robust across different robot speeds (slow vs fast)
            if distance_driven >= 0.5 and calib_time >= 2.0:
                self._publish(0.0, 0.0)  # Stop briefly

                if abs(calib_dx) < 0.1 and abs(calib_dy) < 0.1:
                    rospy.logwarn("GPS movement too small for reliable heading")
                    return

                # Calculate true world heading from GPS trajectory
                true_gps_heading = math.atan2(calib_dy, calib_dx)

                # Compute IMU offset: how much does IMU heading differ from GPS truth?
                self.heading_offset = true_gps_heading - self.heading

                self.nav_state = "NAVIGATING"
                rospy.loginfo("="*50)
                rospy.loginfo(f"CALIBRATION SUCCESSFUL!")
                rospy.loginfo(f"GPS world heading: {math.degrees(true_gps_heading):.1f}°")
                rospy.loginfo(f"Raw IMU heading: {math.degrees(self.heading):.1f}°")
                rospy.loginfo(f"Computed offset: {math.degrees(self.heading_offset):.1f}°")
                rospy.loginfo("="*50)
            return

        # ═════════════════════════════════════════════════════════════════
        # PHASE 2: NORMALE NAVIGATION ZUM ZIEL
        # ═════════════════════════════════════════════════════════════════
        if self.nav_state == "NAVIGATING":
            
            # 1. Den echten Kompass-Winkel berechnen (Roh + Offset)
            true_robot_heading = self.heading + self.heading_offset
            # Winkel auf -Pi bis +Pi normalisieren
            true_robot_heading = math.atan2(math.sin(true_robot_heading), math.cos(true_robot_heading))

            # 2. Ziel-Wegpunkt berechnen
            goal_lat, goal_lon = self.waypoints[self.current_waypoint_index]
            goal_x, goal_y     = self._gps_to_xy(goal_lat, goal_lon)

            dx = goal_x - cur_x
            dy = goal_y - cur_y
            distance = math.sqrt(dx**2 + dy**2)

            # 3. Waypoint erreicht?
            if distance < self.distance_tolerance:
                rospy.loginfo(f"Waypoint {self.current_waypoint_index + 1} erreicht!")
                self.current_waypoint_index += 1
                self._publish(0.0, 0.0)
                return

            # 4. Fehler zum Ziel berechnen
            target_heading = math.atan2(dy, dx)
            angle_to_goal = target_heading - true_robot_heading
            angle_to_goal = math.atan2(math.sin(angle_to_goal), math.cos(angle_to_goal))
            angle_deg     = math.degrees(angle_to_goal)

            debugOutput = (
                f"Wegpunkt {self.current_waypoint_index+1} | "
                f"Distanz: {distance:.2f}m | "
                f"Robot-Angle: {math.degrees(true_robot_heading):.1f}° | "
                f"Target-Angle: {math.degrees(target_heading):.1f}° | "
                f"Error: {angle_deg:.1f}°"
            )
            rospy.loginfo_throttle(1, debugOutput)
            self.logger.info(debugOutput)

            # 5. Regellogik: Erst ausrichten, dann fahren
            if abs(angle_deg) > self.angle_tolerance:
                # WICHTIG: Das MINUS-Zeichen hier behebt den "Donut-Effekt" / das unendliche Drehen
                turn = -math.copysign(self.angular_velocity, angle_to_goal)
                self._publish(0.0, turn)
            else:
                # Sanftes Abbremsen nahe am Ziel
                speed = min(self.forward_velocity, distance * 1.5)
                speed = max(speed, 0.1)   # Mindestgeschwindigkeit etwas angehoben
                self._publish(speed, 0.0)

    # ════════════════════════════════════════════════════════════════════
    # HILFSMETHODEN
    # ════════════════════════════════════════════════════════════════════
    def _publish(self, linear: float, angular: float):
        msg = Twist()
        msg.linear.x  = linear
        msg.angular.z = angular
        self.cmd_vel_pub.publish(msg)

if __name__ == '__main__':
    try:
        rospy.init_node('navigation_node')
        node = NavigationNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass