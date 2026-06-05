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
from std_msgs.msg import UInt8, Empty

# ═══════════════════════════════════════════════════════════════════════
# KONSTANTEN
# ═══════════════════════════════════════════════════════════════════════
EARTH_RADIUS = 6_371_000.0   # m

class RTKStatus:
    NO_FIX = 0
    FLOAT  = 1
    FIXED  = 2


def quaternion_to_yaw(q) -> float:
    """Extracts yaw angle (heading) from a quaternion. Returns angle in radians."""
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
        self.wheel_base   = rospy.get_param('~wheel_base', 0.445)
        self.max_linear   = rospy.get_param('~max_linear', 1.0)
        self.max_angular  = rospy.get_param('~max_angular', 1.0)

        self.forward_velocity   = rospy.get_param('~forward_velocity', 0.5)
        self.angular_velocity   = rospy.get_param('~angular_velocity', 0.5)
        self.distance_tolerance   = rospy.get_param('~distance_tolerance', 0.15)
        # TODO is unused yet => probably remove
        self.angle_tolerance      = rospy.get_param('~angle_tolerance', 5.0)
        self.waypoints            = rospy.get_param('~waypoints', [])
        self.gps_to_nozzle_offset = rospy.get_param('~gps_to_nozzle_offset', 0.58)
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
        self.spray_until            = None   # rospy.Time: warten bis dieser Zeitpunkt

        # ── GPS-Tiefpassfilter ────────────────────────────────────────────
        self.gps_filter_alpha = 0.3   # EMA smoothing: lower = smoother, more lag

        # ── Heading-Tiefpassfilter (nur für Düsen-Distanzberechnung) ─────
        self.heading_smooth = None    # langsames EMA, verhindert Distanzsprünge

        # ── Kontinuierliche Heading-Nachkalibrierung ─────────────────────
        self.recalib_last_x = None   # GPS-Position beim letzten Kalibrierpunkt
        self.recalib_last_y = None

        # ── Auto-Kalibrierung (Kinematic Alignment) ──────────────────────
        self.nav_state = "WAITING_FOR_FIX"  # WAITING_FOR_FIX -> CALIBRATING -> NAVIGATING
        self.heading_offset = 0.0
        self.calib_start_x = None
        self.calib_start_y = None
        self.calib_start_time = None

    def _init_ros(self):
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel_controll', Twist, queue_size=1)
        self.spray_pub = rospy.Publisher("/cmd_spray", Empty, queue_size=1)
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
        if msg.status.status < self.min_gps_status:
            if self.has_fix:
                debugOutput = f"GPS-Fix verloren! aktueller status={msg.status.status}"
                rospy.logwarn_throttle(5, debugOutput)
                self.logger.warning(debugOutput)
                self.has_fix = False
            return

        self.has_fix = True # Fix wiederhergestellt / aktiv

        if self.current_lat is None:
            self.current_lat = msg.latitude
            self.current_lon = msg.longitude
        else:
            a = self.gps_filter_alpha
            self.current_lat = a * msg.latitude  + (1.0 - a) * self.current_lat
            self.current_lon = a * msg.longitude + (1.0 - a) * self.current_lon

        rospy.loginfo_throttle(5, f"GPS Fix: lat={self.current_lat:.7f}, lon={self.current_lon:.7f}")

        rtk              = self._parse_rtk_status(msg)

        if rtk == RTKStatus.NO_FIX:
            return

        if self.origin_lat is None:
            if rtk == RTKStatus.FIXED or rtk == RTKStatus.FLOAT:
                self.origin_lat = msg.latitude
                self.origin_lon = msg.longitude
                rospy.loginfo(f"RTK-Ursprung gesetzt: {msg.latitude}, {msg.longitude}")

    #TODO: extract => duplicate to gps node
    def _parse_rtk_status(self, msg):
        status = msg.status.status
        cov    = msg.position_covariance[0]
        if status < 0:    return RTKStatus.NO_FIX
        if cov <= 0.01:  return RTKStatus.FIXED
        elif cov <= 0.25: return RTKStatus.FLOAT
        else:             return RTKStatus.NO_FIX

    def _gps_quality_callback(self, msg):
        self.gps_quality = msg.data
        rospy.loginfo_throttle(5, f"GPS Quality = {self.gps_quality}")

        if self.gps_quality == 4 and not self.had_rtk_fix:
            self.had_rtk_fix = True
            rospy.loginfo("RTK FIX erreicht -> Navigation freigegeben")

    def _imu_callback(self, msg: Imu):
        q = msg.orientation
        self.heading = self._normalize_angle(quaternion_to_yaw(q))

        if self.heading_smooth is None:
            self.heading_smooth = self.heading
        else:
            # Kreisförmiger EMA: korrekte Behandlung des Winkelumbruchs
            a = 0.02
            self.heading_smooth = math.atan2(
                a * math.sin(self.heading) + (1.0 - a) * math.sin(self.heading_smooth),
                a * math.cos(self.heading) + (1.0 - a) * math.cos(self.heading_smooth),
            )

        if not self.has_imu:
            self.has_imu = True
            rospy.loginfo(f"IMU aktiv. Erster roher Heading: {math.degrees(self.heading):.1f}°")

    def _normalize_angle(self, angle: float):
        return math.atan2(math.sin(angle), math.cos(angle))

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
        if not self._preconditions_met():
            return

        cur_x, cur_y = self._gps_to_xy(self.current_lat, self.current_lon)

        if self.nav_state == "WAITING_FOR_FIX":
            self._handle_waiting_for_fix(cur_x, cur_y)
        elif self.nav_state == "CALIBRATING":
            self._handle_calibrating(cur_x, cur_y)
        elif self.nav_state == "NAVIGATING":
            self._handle_navigating(cur_x, cur_y)

    def _preconditions_met(self) -> bool:
        """Returns False (and stops the robot) if any required condition is not yet satisfied."""
        if not self.had_rtk_fix:
            rospy.logwarn_throttle(2, "Warte auf ersten RTK-Fix (status=4)...")
            self._publish(0.0, 0.0)
            return False

        if not self.has_fix:
            self._publish(0.0, 0.0)
            return False

        if self.current_waypoint_index >= len(self.waypoints):
            if not self.at_goal:
                rospy.loginfo("Alle Waypoints erreicht! Roboter stoppt.")
                self.at_goal = True
            self._publish(0.0, 0.0)
            return False

        if self.origin_lat is None or self.origin_lon is None:
            rospy.logwarn_throttle(2, "Warte auf GPS-Ursprung...")
            self._publish(0.0, 0.0)
            return False

        return True

    # ════════════════════════════════════════════════════════════════════
    # PHASE 1: AUTOMATISCHE KALIBRIERUNG (Kinematic Alignment)
    # ════════════════════════════════════════════════════════════════════
    def _handle_waiting_for_fix(self, cur_x: float, cur_y: float):
        """Records the calibration start position and transitions to CALIBRATING."""
        self.calib_start_x = cur_x
        self.calib_start_y = cur_y
        self.calib_start_time = rospy.Time.now()

        self.nav_state = "CALIBRATING"
        rospy.loginfo("Starte Auto-Kalibrierung: Fahre 1.5m geradeaus, um den GPS-Vektor zu messen...")

    def _handle_calibrating(self, cur_x: float, cur_y: float):
        """Drives straight and accumulates GPS displacement to compute the heading offset."""
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

        rospy.loginfo_throttle(0.5, f"Kalibriere... Gefahren: {distance_driven:.2f}m / 1.50m")

        calib_time = (rospy.Time.now() - self.calib_start_time).to_sec()

        # Bewegung plausibilisieren
        max_reasonable_distance = calib_time * 1.0  # max 1 m/s

        # Sanity check: detect GPS jumps (e.g., >1 m/s unrealistic)
        max_reasonable_distance = calib_time * 1.0
        if distance_driven > max_reasonable_distance:
            rospy.logwarn("GPS jump detected -> ignoring calibration")
            return

        # Wenn wir 1.0 Meter gefahren sind, ist der GPS Vektor stabil genug
        if distance_driven >= 1.0 and calib_time > 3.0:
            self._finalize_calibration(calib_dx, calib_dy, distance_driven)
        return

    def _finalize_calibration(self, calib_dx: float, calib_dy: float, distance_driven: float):
        """Computes the heading offset from the driven GPS displacement and transitions to NAVIGATING."""
        self._publish(0.0, 0.0) # Kurz stoppen

        if distance_driven < 1.0:
            return

        if abs(calib_dx) < 0.2 and abs(calib_dy) < 0.2:
            rospy.logwarn("GPS Bewegung zu klein für Heading")
            return

        # Echten Welt-Winkel aus der gefahrenen GPS-Strecke berechnen
        true_gps_heading = math.atan2(calib_dy, calib_dx)
        self.heading_offset = self._normalize_angle(true_gps_heading - self.heading)

        rospy.loginfo(
            f"Kalibrierung: dx={calib_dx:.2f}m, dy={calib_dy:.2f}m, "
            f"GPS-Winkel={math.degrees(true_gps_heading):.1f}°, "
            f"IMU-Roh={math.degrees(self.heading):.1f}°, "
            f"Offset={math.degrees(self.heading_offset):.1f}°"
        )
        self.nav_state = "NAVIGATING"
        rospy.loginfo("="*50)
        rospy.loginfo(f"KALIBRIERUNG ERFOLGREICH!")
        rospy.loginfo(f"GPS Welt-Winkel: {math.degrees(true_gps_heading):.1f}°")
        rospy.loginfo(f"Roher IMU-Winkel: {math.degrees(self.heading):.1f}°")
        rospy.loginfo(f"Berechneter Offset: {math.degrees(self.heading_offset):.1f}°")
        rospy.loginfo("="*50)

    # ════════════════════════════════════════════════════════════════════
    # PHASE 2: NORMALE NAVIGATION ZUM ZIEL
    # ════════════════════════════════════════════════════════════════════
    def _handle_navigating(self, cur_x: float, cur_y: float):
        """Proportional heading controller that steers the robot toward the current waypoint."""
        # Sprühwartezeit läuft noch → stehen bleiben
        if self.spray_until is not None:
            if rospy.Time.now() < self.spray_until:
                self._publish(0.0, 0.0)
                return
            self.spray_until = None

        # 1. Den echten Kompass-Winkel berechnen (Roh + Offset)
        true_robot_heading = self._compute_true_heading()

        # 2. Ziel-Wegpunkt berechnen
        goal_lat, goal_lon = self.waypoints[self.current_waypoint_index]
        goal_x, goal_y     = self._gps_to_xy(goal_lat, goal_lon)

        # 3. Waypoint-Erkennung: Düsenposition mit geglättetem Heading (kein Springen)
        smooth_nozzle_x = cur_x + self.gps_to_nozzle_offset * math.cos(self.heading_smooth)
        smooth_nozzle_y = cur_y + self.gps_to_nozzle_offset * math.sin(self.heading_smooth)
        dx_ant = goal_x - smooth_nozzle_x
        dy_ant = goal_y - smooth_nozzle_y
        distance = math.sqrt(dx_ant**2 + dy_ant**2)

        if distance < self.distance_tolerance:
            rospy.loginfo(f"Waypoint {self.current_waypoint_index + 1} erreicht!")
            self.current_waypoint_index += 1
            self.spray_pub.publish()
            self._publish(0.0, 0.0)
            self.spray_until = rospy.Time.now() + rospy.Duration(5.0)
            return

        # 4. Lenkwinkel: Düsenposition (GPS + Offset in Fahrtrichtung)
        nozzle_x = cur_x + self.gps_to_nozzle_offset * math.cos(true_robot_heading)
        nozzle_y = cur_y + self.gps_to_nozzle_offset * math.sin(true_robot_heading)
        dx = goal_x - nozzle_x
        dy = goal_y - nozzle_y

        target_heading = math.atan2(dy, dx)
        angle_to_goal  = self._normalize_angle(target_heading - true_robot_heading)

        debugOutput = (
            f"Wegpunkt {self.current_waypoint_index+1} | "
            f"Distanz: {distance:.2f}m | "
            f"Robot-Angle: {math.degrees(true_robot_heading):.1f}° | "
            f"IMU-Roh: {math.degrees(self.heading):.1f}° | "
            f"Target-Angle: {math.degrees(target_heading):.1f}° | "
        )
        rospy.loginfo_throttle(1, debugOutput)
        self.logger.info(debugOutput)

        k_p = 1.0
        angular_cmd = max(-self.angular_velocity,
                          min(self.angular_velocity, k_p * angle_to_goal))

        rospy.loginfo(
            f"Target={math.degrees(target_heading):.1f}° "
            f"Robot={math.degrees(true_robot_heading):.1f}° "
            f"Error={math.degrees(angle_to_goal):.1f}° "
            f"Angular={angular_cmd:.2f}"
        )

        # Geschwindigkeit: langsamer bei großem Winkel- oder Entfernungsfehler
        angle_factor    = max(0.0, 1.0 - abs(angle_to_goal) / math.radians(45))
        distance_factor = min(1.0, distance / 1.5)
        linear = self.forward_velocity * 0.2 * angle_factor * distance_factor
        linear = max(0.03, linear)   # Mindestgeschwindigkeit damit der Roboter nicht steckt
        self._publish(linear, angular_cmd)

        # 5. Kontinuierliche Heading-Nachkalibrierung aus dem GPS-Fahrvektor
        #    Nur wenn der Roboter geradeaus fährt (kleiner Winkelfehler) → GPS-Spur ist valide
        self._update_heading_offset(cur_x, cur_y, angle_to_goal)

    def _update_heading_offset(self, cur_x: float, cur_y: float, angle_to_goal: float):
        """Continuously recalibrates heading_offset from the GPS track while driving straight."""
        if self.recalib_last_x is None:
            self.recalib_last_x = cur_x
            self.recalib_last_y = cur_y
            return

        # Nur bei kleinem Winkelfehler – sonst spiegelt die GPS-Spur die Kurvenfahrt wider
        if abs(angle_to_goal) > math.radians(8):
            return

        dx = cur_x - self.recalib_last_x
        dy = cur_y - self.recalib_last_y
        dist = math.sqrt(dx**2 + dy**2)

        if dist < 0.5:
            return

        gps_track    = math.atan2(dy, dx)
        new_offset   = self._normalize_angle(gps_track - self.heading)

        # Kreisförmiger EMA: korrekte Behandlung des Winkelumbruchs
        a = 0.25
        self.heading_offset = math.atan2(
            a * math.sin(new_offset)        + (1.0 - a) * math.sin(self.heading_offset),
            a * math.cos(new_offset)        + (1.0 - a) * math.cos(self.heading_offset),
        )

        rospy.loginfo_throttle(2,
            f"Heading-Offset nachjustiert: {math.degrees(self.heading_offset):.1f}° "
            f"(GPS-Spur: {math.degrees(gps_track):.1f}°, IMU-Roh: {math.degrees(self.heading):.1f}°)"
        )
        self.recalib_last_x = cur_x
        self.recalib_last_y = cur_y

    def _compute_true_heading(self) -> float:
        """Applies the calibration offset to the raw IMU heading to get the world-frame heading."""
        true_heading = self.heading + self.heading_offset
        return math.atan2(math.sin(true_heading), math.cos(true_heading))

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
