#!/usr/bin/env python3
"""
Navigation Node – GPS-basiert (RTK) + IMU Heading (Auto-Calibrated)
Position kommt direkt von gps/fix (NavSatFix).
Heading kommt von imu/data (Gyro-Integration) und wird über die ersten 1.5m
Fahrtstrecke automatisch zur GPS-Weltkarte ausgerichtet (Kinematic Alignment).
"""

import json
import math
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import UInt8, String
import os
import datetime
import logging
from std_msgs.msg import UInt8, Empty
from spray_counter import spray
from robot_msgs.status import StatusReporter

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
        self.wheel_base   = rospy.get_param('~wheel_base', 0.5)
        self.max_linear   = rospy.get_param('~max_linear', 1.0)
        self.max_angular  = rospy.get_param('~max_angular', 1.0)

        self.forward_velocity     = rospy.get_param('~forward_velocity', 0.5)
        self.angular_velocity     = rospy.get_param('~angular_velocity', 0.5)
        #TODO: might have to be changed
        self.waypoint_tolerance   = rospy.get_param('~waypoint_tolerance', 0.3)
        # Bypass-Punkte müssen nicht exakt getroffen werden (kein Spray) -> größere Toleranz
        self.bypass_tolerance     = rospy.get_param('~bypass_tolerance', 0.7)
        # Waypoints kommen primär aus einem JSON-File (vom Frontend auf den Pi geschrieben);
        # Fallback ist der rosparam ~waypoints (navigation.yaml), falls die Datei fehlt.
        self.waypoints_file       = rospy.get_param('~waypoints_file',
                                                    '/home/ubuntu/trackSprayRobot/shared_files/waypoints.json')
        self.waypoints            = self._load_waypoints()
        self.gps_to_nozzle_offset = rospy.get_param('~gps_to_nozzle_offset', 0.58)
        self.min_gps_status       = rospy.get_param('~min_gps_status', 0)
        self.obstacle_margin      = rospy.get_param('~obstacle_margin', 1.0)

        # ── RTK-Stabilität, Ausreißer-Schutz & Spray-Bestätigung ──
        self.rtk_stable_sec    = rospy.get_param('~rtk_stable_sec', 3.0)
        self.gps_timeout       = rospy.get_param('~gps_timeout', 2.0)
        self.max_gps_jump      = rospy.get_param('~max_gps_jump', 0.30)
        self.spray_confirm_sec = rospy.get_param('~spray_confirm_sec', 0.5)
        self.require_fix_for_spray = rospy.get_param('~require_fix_for_spray', True)
        self.waypoint_pause_sec    = rospy.get_param('~waypoint_pause_sec', 5.0)
        self.bypass_pause_sec      = rospy.get_param('~bypass_pause_sec', 5.0)

    def _load_waypoints(self):
        """Lädt Waypoints aus dem JSON-File; fällt auf rosparam ~waypoints zurück.

        Akzeptiert sowohl eine reine Liste [[lat, lon], ...] als auch die Objektform
        {"waypoints": [[lat, lon], ...]}. Bei Fehlern wird der rosparam-Fallback genutzt,
        damit ein fehlendes/kaputtes File die Navigation nicht crasht.
        """
        fallback = rospy.get_param('~waypoints', [])

        if not os.path.exists(self.waypoints_file):
            rospy.loginfo(f"Kein Waypoints-File ({self.waypoints_file}) -> rosparam-Fallback "
                          f"({len(fallback)} Punkte)")
            return fallback

        try:
            with open(self.waypoints_file, "r") as f:
                data = json.load(f)
            if isinstance(data, dict) and 'waypoints' in data:
                raw = data['waypoints']
            elif isinstance(data, list):
                raw = data
            else:
                raise ValueError("Ungültiges Waypoints-Format (erwarte Liste oder {'waypoints': [...]})")

            waypoints = [[float(p[0]), float(p[1])] for p in raw]
            rospy.loginfo(f"Waypoints aus {self.waypoints_file} geladen: {len(waypoints)} Punkte")
            return waypoints
        except (json.JSONDecodeError, ValueError, TypeError, KeyError, IndexError, OSError) as e:
            rospy.logwarn(f"Waypoints-File ungültig ({e}) -> rosparam-Fallback "
                          f"({len(fallback)} Punkte)")
            return fallback

    def _init_state(self):
        # ── GPS-State ────────────────────────────────────────────────────
        self.current_lat  = None
        self.current_lon  = None
        self.has_fix      = False
        self.rtk_ready    = False     # wird erst nach stabilem FIXED-Streak True
        self.gps_quality  = 0

        self.fix_streak_start = None  # Beginn des aktuellen ununterbrochenen FIXED-Streaks
        self.last_fix_time    = None  # Zeitpunkt der letzten akzeptierten FIXED-Position

        self.origin_lat   = None
        self.origin_lon   = None

        # ── Heading aus IMU ───────────────────────────────────────────────
        self.heading      = None   # rad, roher Wert aus der IMU (beginnt blind bei 0)
        self.has_imu      = False

        # ── Waypoint-State ───────────────────────────────────────────────
        self.current_waypoint_index = 0
        self.at_goal                = False
        self.in_tol_since           = None  # seit wann ununterbrochen in Toleranz (Spray-Bestätigung)
        self.pause_until            = None  # non-blocking Pause nach dem Sprühen

        # ── Obstacle Bypass-State ─────────────────────────────────────────
        self.bypass_targets = []    # Liste von (x, y) Umfahrungspunkten (leer = keine Umfahrung)

        # ── Auto-Kalibrierung (Kinematic Alignment) ──────────────────────
        self.nav_state = "WAITING_FOR_FIX"  # WAITING_FOR_FIX -> CALIBRATING -> NAVIGATING
        self.heading_offset = 0.0
        self.calib_start_x = None
        self.calib_start_y = None
        self.calib_start_time = None

        # ── Status-Reporting (Frontend) ──────────────────────────────────
        # Eigene Flanke für RTK_LOST<->RTK_RECOVERED, da der globale dedup im
        # StatusReporter durch andere Events (z.B. GOAL_REACHED) zurückgesetzt würde.
        self.rtk_lost = False

    def _init_ros(self):
        self.status = StatusReporter(source="navigation")
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel_controll', Twist, queue_size=1)
        self.spray_pub = rospy.Publisher("/cmd_spray", Empty, queue_size=1)
        self.bypass_request_pub = rospy.Publisher('/obstacle_bypass_request', String, queue_size=1)
        rospy.Subscriber('gps/fix', NavSatFix, self._gps_callback)
        rospy.Subscriber('gps/quality', UInt8, self._gps_quality_callback)
        rospy.Subscriber('imu/data', Imu, self._imu_callback)
        rospy.Subscriber('/obstacle_bypass_response', String, self._bypass_response_callback)
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
        rtk = self._parse_rtk_status(msg)

        # ── Startup-Gate: RTK muss erst STABIL FIXED sein ─────────────────
        # Ein einzelner FIXED-Treffer reicht NICHT. Wir verlangen einen
        # ununterbrochenen FIXED-Streak von rtk_stable_sec Sekunden, bevor
        # die Navigation freigegeben wird (deine Beobachtung: anfangs flackert
        # FIXED/FLOAT, erst danach wird es regelmäßig).
        if rtk == RTKStatus.FIXED:
            if self.fix_streak_start is None:
                self.fix_streak_start = rospy.Time.now()
            streak = (rospy.Time.now() - self.fix_streak_start).to_sec()
            if not self.rtk_ready and streak >= self.rtk_stable_sec:
                self.rtk_ready = True
                debugOutput = f"RTK stabil ({streak:.1f}s FIXED am Stück) -> Navigation freigegeben"
                rospy.loginfo(debugOutput)
                self.logger.info(debugOutput)
                self.status.info("RTK_READY", "RTK stabil – Navigation freigegeben")
        else:
            # Jeder Nicht-FIXED (FLOAT/DGPS/GPS/NO_FIX) unterbricht den Streak.
            if self.fix_streak_start is not None and not self.rtk_ready:
                rospy.logwarn_throttle(5, "RTK-FIXED unterbrochen vor Freigabe – Streak zurückgesetzt")
                self.status.warn("RTK_UNSTABLE", "RTK-FIXED unterbrochen vor Freigabe")
            self.fix_streak_start = None

        # ── Position NUR aus RTK FIXED übernehmen ─────────────────────────
        # FLOAT/DGPS/GPS verursachen 30-50cm-Sprünge. Statt sie zu nutzen,
        # halten wir die letzte gute FIXED-Position; der gps_timeout-Check in
        # _preconditions_met stoppt den Roboter, falls FIXED zu lange wegbleibt.
        if rtk != RTKStatus.FIXED:
            return

        new_lat = float(msg.latitude)
        new_lon = float(msg.longitude)

        # ── Ausreißer-Filter: physikalisch unmögliche Sprünge verwerfen ───
        # Bei max_linear m/s kann sich die Position pro Epoche nur begrenzt
        # ändern. Größere Sprünge sind RTK-Artefakte und werden ignoriert.
        if self.current_lat is not None and self.last_fix_time is not None:
            dt   = (rospy.Time.now() - self.last_fix_time).to_sec()
            jump = self._latlon_dist(self.current_lat, self.current_lon, new_lat, new_lon)
            max_plausible = self.max_gps_jump + self.max_linear * max(dt, 0.0)
            #TODO: tried 20 percent margin 
            if jump > 1.2 * max_plausible:
                debugOutput = (f"GPS-Sprung verworfen: {jump:.2f}m > erlaubt "
                               f"{max_plausible:.2f}m (dt={dt:.2f}s)")
                rospy.logwarn_throttle(2, debugOutput)
                self.logger.warning(debugOutput)
                return

        self.has_fix       = True
        self.current_lat   = new_lat
        self.current_lon   = new_lon
        self.last_fix_time = rospy.Time.now()

        # Gegenflanke zu RTK_LOST: frischer FIXED nach einem Verlust -> RECOVERED.
        if self.rtk_lost:
            self.rtk_lost = False
            self.status.info("RTK_RECOVERED", "RTK-Fix wieder da – Fahrt geht weiter", dedup=False)

        rospy.loginfo_throttle(5, f"GPS FIXED: lat={self.current_lat:.7f}, lon={self.current_lon:.7f}")

        if self.origin_lat is None:
            self.origin_lat = new_lat
            self.origin_lon = new_lon
            debugOutput = f"RTK-Ursprung gesetzt: {new_lat}, {new_lon}"
            rospy.loginfo(debugOutput)
            self.logger.info(debugOutput)

    #TODO: extract => duplicate to gps node
    def _parse_rtk_status(self, msg):
        status = msg.status.status
        cov    = msg.position_covariance[0]
        if status < 0:    return RTKStatus.NO_FIX
        if cov <= 0.01:  return RTKStatus.FIXED
        elif cov <= 0.25: return RTKStatus.FLOAT
        else:             return RTKStatus.NO_FIX

    def _gps_quality_callback(self, msg):
        # Wird als Quer-Check beim Sprühen genutzt (require_fix_for_spray).
        # Die Navigations-Freigabe selbst läuft über den stabilen FIXED-Streak
        # in _gps_callback (RTK-Status aus der Kovarianz der Fix-Nachricht).
        self.gps_quality = msg.data
        rospy.loginfo_throttle(5, f"GPS Quality = {self.gps_quality}")

    def _bypass_response_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            if isinstance(data, dict) and 'bypass_targets' in data:
                raw_targets = data['bypass_targets']
            elif isinstance(data, list):
                raw_targets = data
            else:
                raise ValueError("Ungültiges Bypass-Response-Format")

            self.bypass_targets = [
                (float(p[0]), float(p[1]))
                for p in raw_targets
            ]
            rospy.loginfo_throttle(2, f"Bypass-Antwort erhalten: {len(self.bypass_targets)} Punkte")
        except (json.JSONDecodeError, ValueError, TypeError, KeyError) as e:
            rospy.logwarn(f"Ungültige Bypass-Antwort: {e}")

    def _imu_callback(self, msg: Imu):
        q = msg.orientation
        self.heading = self._normalize_angle(quaternion_to_yaw(q))

        if not self.has_imu:
            self.has_imu = True
            rospy.loginfo(f"IMU aktiv. Erster roher Heading: {math.degrees(self.heading):.1f}°")

    def _normalize_angle(self, angle: float):
        return math.atan2(math.sin(angle), math.cos(angle))

    # ════════════════════════════════════════════════════════════════════
    # KOORDINATEN-UMRECHNUNG
    # ════════════════════════════════════════════════════════════════════
    def _latlon_dist(self, lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """Approximate planar distance in meters between two lat/lon points."""
        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)
        x = dlon * EARTH_RADIUS * math.cos(math.radians(lat1))
        y = dlat * EARTH_RADIUS
        return math.hypot(x, y)

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
        # Non-blocking Pause nach dem Sprühen (ersetzt das blockierende time.sleep,
        # das zuvor den 100-Hz-Timer-Thread für 5s einfror).
        if self.pause_until is not None:
            if rospy.Time.now() < self.pause_until:
                self._publish(0.0, 0.0)
                return
            self.pause_until = None

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
        if not self.rtk_ready:
            rospy.logwarn_throttle(2, f"Warte auf stabilen RTK-FIXED ({self.rtk_stable_sec:.0f}s am Stück)...")
            self._publish(0.0, 0.0)
            return False

        if not self.has_fix:
            self._publish(0.0, 0.0)
            return False

        # ── Frische-Check: ohne aktuelle FIXED-Position NICHT blind weiterfahren ──
        # Während kurzer FLOAT-Phasen halten wir die letzte Position; bleibt FIXED
        # länger als gps_timeout weg, wird gestoppt statt auf veralteter Position
        # weiterzufahren.
        if self.last_fix_time is None or \
           (rospy.Time.now() - self.last_fix_time).to_sec() > self.gps_timeout:
            rospy.logwarn_throttle(2, f"Kein frischer RTK-FIXED seit >{self.gps_timeout:.1f}s -> Stop")
            # Edge: nur beim ersten Eintritt melden; RTK_RECOVERED kommt aus _gps_callback.
            if not self.rtk_lost:
                self.rtk_lost = True
                self.status.error("RTK_LOST", "Kein frischer RTK-FIXED – Roboter stoppt", dedup=False)
            self._publish(0.0, 0.0)
            return False

        if self.current_waypoint_index >= len(self.waypoints):
            if not self.at_goal:
                rospy.loginfo("Alle Waypoints erreicht! Roboter stoppt.")
                self.at_goal = True
                self.status.info("GOAL_REACHED", "Alle Waypoints erreicht – Roboter stoppt")
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

        rospy.loginfo_throttle(0.5, f"Kalibriere... Gefahren: {distance_driven:.2f}m / 3.0m")

        calib_time = (rospy.Time.now() - self.calib_start_time).to_sec()

        # Bewegung plausibilisieren
        """
        max_reasonable_distance = calib_time * 1.0  # max 1 m/s

        #TODO: always errors => probably remove
        # Sanity check: detect GPS jumps (e.g., >1 m/s unrealistic)
        max_reasonable_distance = calib_time * 1.0
        if distance_driven > max_reasonable_distance:
            rospy.logwarn("GPS jump detected -> ignoring calibration")
            return
        """

        # Wenn wir 3.0 Meter gefahren sind, ist der GPS Vektor stabil genug
         #TODO: extract calibration distance to parameter
        if distance_driven >= 3.0 and calib_time > 3.0:
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
        # 1. Den echten Kompass-Winkel berechnen (Roh + Offset)
        true_robot_heading = self._compute_true_heading()

        # GPS sitzt 58cm hinter der Düse → Düsenposition in Fahrtrichtung vorrechnen
        nozzle_x = cur_x + self.gps_to_nozzle_offset * math.cos(true_robot_heading)
        nozzle_y = cur_y + self.gps_to_nozzle_offset * math.sin(true_robot_heading)

        # 2. Umfahrung aktiv? → erst die Bypass-Punkte der Reihe nach abfahren
        if self.bypass_targets:
            self._navigate_to_bypass(nozzle_x, nozzle_y)
            return

        # 3. Ziel-Wegpunkt berechnen
        goal_lat, goal_lon = self.waypoints[self.current_waypoint_index]
        goal_x, goal_y     = self._gps_to_xy(goal_lat, goal_lon)

        # 4. Anfrage an die Obstacle-Avoidance-Node senden
        request = {
            'robot_x': nozzle_x,
            'robot_y': nozzle_y,
            'goal_x': goal_x,
            'goal_y': goal_y,
            'origin_lat': self.origin_lat,
            'origin_lon': self.origin_lon,
            'obstacle_margin': self.obstacle_margin,
        }
        self.bypass_request_pub.publish(String(data=json.dumps(request)))

        dx = goal_x - nozzle_x
        dy = goal_y - nozzle_y
        distance = math.sqrt(dx**2 + dy**2)

        # 3. Waypoint erreicht? -> Spray erst nach Bestätigung (Dwell + RTK FIXED),
        #    damit ein einzelner Rausch-Fix nicht fälschlich auslöst.
        fix_ok = (not self.require_fix_for_spray) or (self.gps_quality == 4)
        if distance < self.waypoint_tolerance and fix_ok:
            self._publish(0.0, 0.0)  # anhalten und Position bestätigen lassen
            now = rospy.Time.now()
            debugOutput = (f"Waypoint {self.current_waypoint_index + 1} bestätigt "
                               f"(Distanz {distance:.2f}m) -> SPRAY")
            rospy.loginfo(debugOutput)
            self.logger.info(debugOutput)
            self.spray_pub.publish()
            rest = spray()
            rospy.loginfo(
                f"Noch {rest} Sprühstöße verfügbar"
            )
            self.current_waypoint_index += 1
            self.in_tol_since = None
            self.pause_until = rospy.Time.now() + rospy.Duration(self.waypoint_pause_sec)
            """
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
            # Toleranz verlassen oder kein FIXED -> Bestätigung zurücksetzen
            self.in_tol_since = None

        # 5. Proportionaler Lenkungsbefehl basierend auf dem Winkel zum Ziel
        self._steer_towards(nozzle_x, nozzle_y, goal_x, goal_y, label=f"Wegpunkt {self.current_waypoint_index+1}")

    def _navigate_to_bypass(self, cur_x: float, cur_y: float):
        """Fährt die Umfahrungspunkte der Reihe nach ab; entfernt jeden, wenn erreicht."""
        bx, by = self.bypass_targets[0]
        dist = math.sqrt((bx - cur_x) ** 2 + (by - cur_y) ** 2)

        rospy.loginfo_throttle(1, f"Distanz to bypass: {dist}")

        if dist < self.bypass_tolerance:
            self.bypass_targets.pop(0)
            if self.bypass_targets:
                rospy.loginfo("1. Umfahrungspunkt erreicht — weiter zum 2. Punkt")
            else:
                rospy.loginfo("Umfahrung abgeschlossen — weiter zum Waypoint")
            # An jedem Bypass-Punkt kurz stehen bleiben (non-blocking Pause)
            self._publish(0.0, 0.0)
            self.pause_until = rospy.Time.now() + rospy.Duration(self.bypass_pause_sec)
            return

        label = f"Bypass {'1/2' if len(self.bypass_targets) == 2 else '2/2'}"
        self._steer_towards(cur_x, cur_y, bx, by, label=label)

    # ════════════════════════════════════════════════════════════════════
    # HINDERNISERKENNUNG & UMFAHRUNG
    # ════════════════════════════════════════════════════════════════════

    def _steer_towards(self, cur_x: float, cur_y: float,
                       target_x: float, target_y: float, label: str = ""):
        """Proportionaler Regler: dreht und fährt den Roboter in Richtung (target_x, target_y)."""
        true_robot_heading = self._compute_true_heading()

        dx = target_x - cur_x
        dy = target_y - cur_y
        target_heading = math.atan2(dy, dx)
        angle_error = self._normalize_angle(target_heading - true_robot_heading)

        distance = math.sqrt(dx**2 + dy**2)

        debugOutput = (
            f"{label} | "
            f"Distanz: {distance:.2f}m | "
            f"Robot-Angle: {math.degrees(true_robot_heading):.1f}° | "
            f"IMU-Roh: {math.degrees(self.heading):.1f}° | "
            f"Target-Angle: {math.degrees(target_heading):.1f}°"
        )
        rospy.loginfo_throttle(1, debugOutput)
        self.logger.info(debugOutput)

        k_p = 1.0
        angular_cmd = max(-self.angular_velocity,
                          min(self.angular_velocity, k_p * angle_error))

        rospy.loginfo(
            f"Target={math.degrees(target_heading):.1f}° "
            f"Robot={math.degrees(true_robot_heading):.1f}° "
            f"Error={math.degrees(angle_error):.1f}° "
            f"Angular={angular_cmd:.2f}"
        )

        #TODO: attempt to reduce speed when distance <= 1.5
        if abs(angle_error) > math.radians(45):
            linear = 0.05
        else:
            linear = 0.1
        self._publish(linear, angular_cmd)

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
