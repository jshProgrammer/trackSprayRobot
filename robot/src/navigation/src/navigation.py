#!/usr/bin/env python3
"""
Navigation Node – EKF-basiert (RTK + IMU fusioniert).

Position UND Heading kommen jetzt aus der gefilterten EKF-Pose (/ekf/pose):
  - Position: GPS (RTK FIXED) fusioniert mit IMU -> während kurzer FLOAT-Phasen
    rechnet der EKF per IMU weiter (Dead-Reckoning), statt einzufrieren.
  - Heading: gyro-integriert UND kontinuierlich über die GPS-Fahrtrichtung
    eingenordet -> begrenzte Drift (früher nur einmalige Start-Kalibrierung).

Der ENU-Meter-Frame-Ursprung kommt latched über /gps/origin vom EKF, damit
Navigation und EKF im selben Koordinatensystem rechnen.
Sprühen erfolgt nur bei echtem RTK FIXED (/gps/quality == 4) und nach kurzer
Bestätigungszeit in Toleranz.
"""

import math
import rospy
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import NavSatFix
import os
import datetime
import logging
from std_msgs.msg import UInt8, Empty

# ═══════════════════════════════════════════════════════════════════════
# KONSTANTEN
# ═══════════════════════════════════════════════════════════════════════
EARTH_RADIUS = 6_371_000.0   # m


def quaternion_to_yaw(q) -> float:
    """Extracts yaw angle (heading) from a quaternion. Returns angle in radians."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class NavigationNode:
    def __init__(self):
        self.init_logging()
        self._init_params()
        self._init_state()
        self._init_ros()

        debugOutput = "Navigation Node gestartet (EKF-Pose-Modus)"
        rospy.loginfo(debugOutput)
        self.logger.info(debugOutput)
        self.logger.info(f"Waypoints: {self.waypoints}")

        debugOutput = "Warte auf stabilen RTK-FIXED und EKF-Pose..."
        rospy.loginfo(debugOutput)
        self.logger.info(debugOutput)

    # ════════════════════════════════════════════════════════════════════
    # INIT
    # ════════════════════════════════════════════════════════════════════
    def _init_params(self):
        self.forward_velocity     = rospy.get_param('~forward_velocity', 0.1)
        self.angular_velocity     = rospy.get_param('~angular_velocity', 0.5)
        self.distance_tolerance   = rospy.get_param('~distance_tolerance', 0.15)
        self.waypoints            = rospy.get_param('~waypoints', [])
        self.gps_to_nozzle_offset = rospy.get_param('~gps_to_nozzle_offset', 0.58)
        self.k_p                  = rospy.get_param('~steering_kp', 1.0)

        # ── RTK-Stabilität, Pose-Frische & Dead-Reckoning ──────────────────
        self.rtk_stable_sec     = rospy.get_param('~rtk_stable_sec', 3.0)        # FIXED-Streak bis Freigabe
        self.max_deadreckon_sec = rospy.get_param('~max_deadreckon_sec', 5.0)    # max. Fahrt ohne frischen FIXED
        self.odom_timeout       = rospy.get_param('~odom_timeout', 0.5)          # EKF-Pose darf nicht älter sein
        self.calib_distance     = rospy.get_param('~calib_distance', 1.0)        # m Geradeausfahrt zum Einnorden

        # ── Spray-Bestätigung ──────────────────────────────────────────────
        self.spray_confirm_sec     = rospy.get_param('~spray_confirm_sec', 0.5)
        self.require_fix_for_spray = rospy.get_param('~require_fix_for_spray', True)
        self.waypoint_pause_sec    = rospy.get_param('~waypoint_pause_sec', 5.0)

    def _init_state(self):
        # ── EKF-Pose (ENU-Meter-Frame um den geteilten Ursprung) ───────────
        self.cur_x          = None
        self.cur_y          = None
        self.heading        = None   # rad, absolutes Welt-Heading aus dem EKF
        self.last_pose_time = None
        self.has_pose       = False

        # ── Frame-Ursprung (latched vom EKF) ───────────────────────────────
        self.origin_lat = None
        self.origin_lon = None

        # ── RTK-Qualität / Freigabe ────────────────────────────────────────
        self.gps_quality      = 0
        self.rtk_ready        = False  # wird erst nach stabilem FIXED-Streak True
        self.fix_streak_start = None   # Beginn des ununterbrochenen FIXED-Streaks
        self.last_fix_time    = None   # Zeitpunkt des letzten RTK FIXED (quality == 4)

        # ── Waypoint-State ─────────────────────────────────────────────────
        self.current_waypoint_index = 0
        self.at_goal                = False
        self.in_tol_since           = None  # seit wann ununterbrochen in Toleranz
        self.pause_until            = None  # non-blocking Pause nach dem Sprühen

        # ── Startup-Ausrichtung (Geradeausfahrt zum Einnorden des EKF-θ) ───
        self.nav_state     = "WAITING_FOR_FIX"  # WAITING_FOR_FIX -> ALIGNING -> NAVIGATING
        self.align_start_x = None
        self.align_start_y = None

    def _init_ros(self):
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel_controll', Twist, queue_size=1)
        self.spray_pub   = rospy.Publisher('/cmd_spray', Empty, queue_size=1)
        rospy.Subscriber('/ekf/pose', PoseStamped, self._pose_callback)
        rospy.Subscriber('gps/origin', NavSatFix, self._origin_callback)
        rospy.Subscriber('gps/quality', UInt8, self._gps_quality_callback)
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
    def _pose_callback(self, msg: PoseStamped):
        """Gefilterte EKF-Pose: Position (ENU-Meter) + absolutes Heading."""
        self.cur_x          = msg.pose.position.x
        self.cur_y          = msg.pose.position.y
        self.heading        = normalize_angle(quaternion_to_yaw(msg.pose.orientation))
        self.last_pose_time = rospy.Time.now()
        if not self.has_pose:
            self.has_pose = True
            rospy.loginfo("EKF-Pose aktiv.")

    def _origin_callback(self, msg: NavSatFix):
        """Geteilter ENU-Frame-Ursprung vom EKF (latched, kommt einmalig)."""
        if self.origin_lat is None:
            self.origin_lat = msg.latitude
            self.origin_lon = msg.longitude
            debugOutput = f"Frame-Ursprung vom EKF übernommen: {msg.latitude}, {msg.longitude}"
            rospy.loginfo(debugOutput)
            self.logger.info(debugOutput)

    def _gps_quality_callback(self, msg: UInt8):
        """RTK-Qualität: Freigabe-Gate (stabiler FIXED-Streak) + Spray-Gate."""
        self.gps_quality = msg.data
        rospy.loginfo_throttle(5, f"GPS Quality = {self.gps_quality}")

        if self.gps_quality == 4:  # RTK FIXED
            self.last_fix_time = rospy.Time.now()
            if self.fix_streak_start is None:
                self.fix_streak_start = rospy.Time.now()
            streak = (rospy.Time.now() - self.fix_streak_start).to_sec()
            if not self.rtk_ready and streak >= self.rtk_stable_sec:
                self.rtk_ready = True
                debugOutput = f"RTK stabil ({streak:.1f}s FIXED am Stück) -> Navigation freigegeben"
                rospy.loginfo(debugOutput)
                self.logger.info(debugOutput)
        else:
            if self.fix_streak_start is not None and not self.rtk_ready:
                rospy.logwarn_throttle(5, "RTK-FIXED unterbrochen vor Freigabe – Streak zurückgesetzt")
            self.fix_streak_start = None

    # ════════════════════════════════════════════════════════════════════
    # KOORDINATEN-UMRECHNUNG (Waypoint lat/lon -> ENU-Meter, gleicher Frame)
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
        # Non-blocking Pause nach dem Sprühen.
        if self.pause_until is not None:
            if rospy.Time.now() < self.pause_until:
                self._publish(0.0, 0.0)
                return
            self.pause_until = None

        if not self._preconditions_met():
            return

        if self.nav_state == "WAITING_FOR_FIX":
            self._handle_waiting_for_fix()
        elif self.nav_state == "ALIGNING":
            self._handle_aligning()
        elif self.nav_state == "NAVIGATING":
            self._handle_navigating()

    def _preconditions_met(self) -> bool:
        """Returns False (and stops the robot) if any required condition is not yet satisfied."""
        if not self.rtk_ready:
            rospy.logwarn_throttle(2, f"Warte auf stabilen RTK-FIXED ({self.rtk_stable_sec:.0f}s am Stück)...")
            self._publish(0.0, 0.0)
            return False

        # EKF-Pose muss vorliegen und frisch sein (sonst EKF-Node tot/hängt).
        if not self.has_pose or self.last_pose_time is None or \
           (rospy.Time.now() - self.last_pose_time).to_sec() > self.odom_timeout:
            rospy.logwarn_throttle(2, "Keine frische EKF-Pose -> Stop")
            self._publish(0.0, 0.0)
            return False

        if self.origin_lat is None or self.origin_lon is None:
            rospy.logwarn_throttle(2, "Warte auf Frame-Ursprung (/gps/origin)...")
            self._publish(0.0, 0.0)
            return False

        # Dead-Reckoning-Limit: kurze FLOAT-Phasen überbrückt der EKF per IMU,
        # aber zu lange ohne frischen FIXED -> Drift -> sicherheitshalber stoppen.
        if self.last_fix_time is None or \
           (rospy.Time.now() - self.last_fix_time).to_sec() > self.max_deadreckon_sec:
            rospy.logwarn_throttle(2, f"Kein RTK-FIXED seit >{self.max_deadreckon_sec:.1f}s -> Stop")
            self._publish(0.0, 0.0)
            return False

        if self.current_waypoint_index >= len(self.waypoints):
            if not self.at_goal:
                rospy.loginfo("Alle Waypoints erreicht! Roboter stoppt.")
                self.at_goal = True
            self._publish(0.0, 0.0)
            return False

        return True

    # ════════════════════════════════════════════════════════════════════
    # PHASE 1: AUSRICHTUNG (Geradeausfahrt, damit der EKF θ über den
    #          GPS-Kurs einnordet, bevor wir auf den Wegpunkt zusteuern)
    # ════════════════════════════════════════════════════════════════════
    def _handle_waiting_for_fix(self):
        self.align_start_x = self.cur_x
        self.align_start_y = self.cur_y
        self.nav_state = "ALIGNING"
        rospy.loginfo(f"Ausrichtung: fahre {self.calib_distance:.1f}m geradeaus, "
                      f"damit der EKF das Heading über den GPS-Kurs einnordet...")

    def _handle_aligning(self):
        dx = self.cur_x - self.align_start_x
        dy = self.cur_y - self.align_start_y
        distance_driven = math.hypot(dx, dy)

        self._publish(self.forward_velocity, 0.0)  # geradeaus
        rospy.loginfo_throttle(0.5, f"Richte aus... {distance_driven:.2f}m / {self.calib_distance:.2f}m")

        if distance_driven >= self.calib_distance:
            self._publish(0.0, 0.0)
            self.nav_state = "NAVIGATING"
            debugOutput = (f"Ausrichtung fertig (EKF-Heading={math.degrees(self.heading):.1f}°) "
                           f"-> NAVIGATING")
            rospy.loginfo(debugOutput)
            self.logger.info(debugOutput)

    # ════════════════════════════════════════════════════════════════════
    # PHASE 2: NAVIGATION ZUM ZIEL
    # ════════════════════════════════════════════════════════════════════
    def _handle_navigating(self):
        """Proportional heading controller that steers the robot toward the current waypoint."""
        # Düsenposition: GPS/EKF-Punkt 58cm in Fahrtrichtung vorrechnen.
        nozzle_x = self.cur_x + self.gps_to_nozzle_offset * math.cos(self.heading)
        nozzle_y = self.cur_y + self.gps_to_nozzle_offset * math.sin(self.heading)

        goal_lat, goal_lon = self.waypoints[self.current_waypoint_index]
        goal_x, goal_y     = self._gps_to_xy(goal_lat, goal_lon)

        dx = goal_x - nozzle_x
        dy = goal_y - nozzle_y
        distance = math.hypot(dx, dy)

        # Waypoint erreicht? -> Spray erst nach Bestätigung (Dwell + RTK FIXED).
        fix_ok = (not self.require_fix_for_spray) or (self.gps_quality == 4)
        if distance < self.distance_tolerance and fix_ok:
            debugOutput = (f"Waypoint {self.current_waypoint_index + 1} bestätigt "
                               f"(Distanz {distance:.2f}m -> SPRAY")
            rospy.loginfo(debugOutput)
            self.logger.info(debugOutput)
            self.spray_pub.publish()
            self.current_waypoint_index += 1
            self.in_tol_since = None
            self.pause_until = rospy.Time.now() + rospy.Duration(self.waypoint_pause_sec)
            """
            self._publish(0.0, 0.0)  # anhalten und Position bestätigen lassen
            now = rospy.Time.now()
            if self.in_tol_since is None:
                self.in_tol_since = now
            dwell = (now - self.in_tol_since).to_sec()
            if dwell >= self.spray_confirm_sec:
                debugOutput = (f"Waypoint {self.current_waypoint_index + 1} bestätigt "
                               f"(Distanz {distance:.2f}m, {dwell:.1f}s stabil) -> SPRAY")
                rospy.loginfo(debugOutput)
                self.logger.info(debugOutput)
                self.spray_pub.publish()
                self.current_waypoint_index += 1
                self.in_tol_since = None
                self.pause_until = rospy.Time.now() + rospy.Duration(self.waypoint_pause_sec)
            else:
                rospy.loginfo_throttle(
                    0.5, f"In Toleranz, bestätige... {dwell:.1f}/{self.spray_confirm_sec:.1f}s")
            return
            """
        else:
            self.in_tol_since = None

        # Proportionaler Lenkungsbefehl basierend auf dem Winkel zum Ziel.
        target_heading = math.atan2(dy, dx)
        angle_to_goal  = normalize_angle(target_heading - self.heading)

        debugOutput = (
            f"Wegpunkt {self.current_waypoint_index+1} | "
            f"Distanz: {distance:.2f}m | "
            f"EKF-Heading: {math.degrees(self.heading):.1f}° | "
            f"Target-Angle: {math.degrees(target_heading):.1f}° | "
            f"Error: {math.degrees(angle_to_goal):.1f}°"
        )
        rospy.loginfo_throttle(1, debugOutput)
        self.logger.info(debugOutput)

        angular_cmd = self.k_p * angle_to_goal
        angular_cmd = max(-self.angular_velocity, min(self.angular_velocity, angular_cmd))


        if abs(angle_to_goal) > math.radians(45):
            linear = 0.05
        else:
            linear = self.forward_velocity
        self._publish(linear, angular_cmd)

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
