#!/usr/bin/env python3
"""
Navigation Node – GPS-basiert (RTK) + IMU Heading (Auto-Calibrated)

Dünner Orchestrator: die Rechen-/Filter-/Kalibrier-Arbeit liegt im
importierbaren Package ``navigation`` (siehe src/navigation/*.py). Diese Node
hält nur den ROS-State, die State-Machine und die Pub/Sub-Verdrahtung.

Ablauf:
  WAITING_FOR_FIX -> CALIBRATING (1.5–3m geradeaus, Kinematic Alignment)
  -> NAVIGATING (Bézier-Leitpunkte als "Lotse", Bypass bei Hindernissen,
     Spray am Waypoint nach RTK-FIXED-Bestätigung)

Position kommt ausschließlich aus RTK-FIXED (gps/fix); Heading aus imu/data,
ausgerichtet über den in der Kalibrierung bestimmten Offset.
"""

import datetime
import json
import logging
import math
import os

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import String, Empty, Bool

from spray_counter import spray
from robot_msgs.status import StatusReporter, StatePublisher
from robot_msgs.msg import RobotState

from navigation.geo import quaternion_to_yaw, normalize_angle, gps_to_xy
from navigation.params import NavParams
from navigation.waypoint_guidepoints import load_waypoints, generate_guide_points
from navigation.motion_controller import MotionController
from navigation.rtk_tracker import RTKTracker
from navigation.heading_calibrator import HeadingCalibrator


class NavigationNode:
    def __init__(self):
        self._init_logging()
        self.p = NavParams.from_rosparam()
        self.waypoints = load_waypoints(self.p.waypoints_file)
        self._init_state()
        self._init_ros()

        debug_output = "Navigation Node gestartet (Auto-Kalibrierungs-Modus)"
        rospy.loginfo(debug_output)
        self.logger.info(debug_output)
        self.logger.info(f"Waypoints: {self.waypoints}")

        debug_output = "Warte auf GPS-Fix..."
        rospy.loginfo(debug_output)
        self.logger.info(debug_output)

    # ════════════════════════════════════════════════════════════════════
    # INIT
    # ════════════════════════════════════════════════════════════════════
    def _init_logging(self):
        log_dir = os.path.expanduser(
            f"~/trackRobotLogs/trackRobot_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}")
        os.makedirs(log_dir, exist_ok=True)
        log_file = os.path.join(log_dir, "navigation_node.log")

        self.logger = logging.getLogger("navigation_node")
        self.logger.setLevel(logging.INFO)
        file_handler = logging.FileHandler(log_file)
        formatter = logging.Formatter('%(asctime)s [%(levelname)s] %(message)s')
        file_handler.setFormatter(formatter)
        self.logger.addHandler(file_handler)

    def _init_state(self):
        # ── IMU-Heading (roh) ─────────────────────────────────────────────
        self.heading = None   # rad, roher Wert aus der IMU
        self.has_imu = False

        # ── Waypoint-State ───────────────────────────────────────────────
        self.current_waypoint_index = 0
        self.at_goal = False
        self.in_tol_since = None   # seit wann ununterbrochen in Toleranz (Spray-Bestätigung)
        self.pause_until = None    # non-blocking Pause nach dem Sprühen

        # ── Obstacle Bypass-State ─────────────────────────────────────────
        self.bypass_targets = []   # Liste von (x, y) Umfahrungspunkten (leer = keine Umfahrung)
        self.ignore_bypass_responses_until = None

        # ── Leitpunkt-State (virtuelle Waypoints, kein Spray) ─────────────
        self.guide_targets = []          # Liste von (x, y) Leitpunkten
        self.guide_waypoint_index = None  # für welchen Waypoint die Leitpunkte berechnet wurden

        # ── State-Machine ────────────────────────────────────────────────
        # WAITING_FOR_FIX -> CALIBRATING -> NAVIGATING; PAUSED merkt sich den
        # vorherigen Zustand in nav_state_before_pause.
        self.nav_state = "WAITING_FOR_FIX"
        self.nav_state_before_pause = None

        # ── Module mit eigenem State ─────────────────────────────────────
        self.rtk = RTKTracker(self.p, None, self.logger)   # status wird in _init_ros gesetzt
        self.calib = HeadingCalibrator(self.p, self.logger)

    def _init_ros(self):
        self.status = StatusReporter(source="navigation")
        self.rtk.set_status(self.status)
        self.state_pub = StatePublisher()
        # Startzustand sofort (latched) -> früh verbundenes Frontend sieht "IDLE".
        self.state_pub.publish(RobotState.STATE_IDLE, waypoint_total=len(self.waypoints))
        self.motion = MotionController(self.p, self.logger)
        self.spray_pub = rospy.Publisher("/cmd_spray", Empty, queue_size=1)
        self.bypass_request_pub = rospy.Publisher('/obstacle_bypass_request', String, queue_size=1)
        self.rtk.start_subscribers()
        rospy.Subscriber('imu/data', Imu, self._imu_callback)
        rospy.Subscriber('/obstacle_bypass_response', String, self._bypass_response_callback)
        rospy.Subscriber('/navigation_pause', Bool, self._pause_callback)
        rospy.Timer(rospy.Duration(0.01), self._control_loop)   # 100 Hz

    # ════════════════════════════════════════════════════════════════════
    # CALLBACKS
    # ════════════════════════════════════════════════════════════════════
    def _bypass_response_callback(self, msg: String):
        if self.nav_state == "PAUSED":
            rospy.loginfo_throttle(2, "Bypass-Antwort während Pause ignoriert")
            return

        # Nach einer manuellen Pause kann noch eine alte Bypass-Antwort zur
        # Position vor der Pause eintreffen. Kurz ignorieren, damit die
        # Navigation von der neuen Position frisch plant.
        if self.ignore_bypass_responses_until is not None:
            if rospy.Time.now() < self.ignore_bypass_responses_until:
                rospy.loginfo_throttle(2, "Veraltete Bypass-Antwort nach Pause ignoriert")
                return
            self.ignore_bypass_responses_until = None

        try:
            data = json.loads(msg.data)
            if isinstance(data, dict) and 'bypass_targets' in data:
                raw_targets = data['bypass_targets']
            elif isinstance(data, list):
                raw_targets = data
            else:
                raise ValueError("Ungültiges Bypass-Response-Format")

            self.bypass_targets = [(float(p[0]), float(p[1])) for p in raw_targets]

            # Echte Hindernis-Umfahrung hat Vorrang vor den virtuellen
            # Leitpunkten: Leitpunkte verwerfen, sie werden nach der
            # Umfahrung von der dann aktuellen Position neu erzeugt.
            if self.bypass_targets and self.guide_targets:
                rospy.loginfo("Hindernis erkannt -> Leitpunkte verworfen, Umfahrung hat Vorrang")
                self.guide_targets = []
                self.guide_waypoint_index = None

            rospy.loginfo_throttle(2, f"Bypass-Antwort erhalten: {len(self.bypass_targets)} Punkte")
        except (json.JSONDecodeError, ValueError, TypeError, KeyError) as e:
            rospy.logwarn(f"Ungültige Bypass-Antwort: {e}")

    def _imu_callback(self, msg: Imu):
        self.heading = normalize_angle(quaternion_to_yaw(msg.orientation))
        if not self.has_imu:
            self.has_imu = True
            rospy.loginfo(f"IMU aktiv. Erster roher Heading: {math.degrees(self.heading):.1f}°")

    def _pause_callback(self, msg: Bool):
        if msg.data:
            self._enter_paused()
        else:
            self._resume_from_pause()

    # ════════════════════════════════════════════════════════════════════
    # HAUPTREGELSCHLEIFE (100 Hz)
    # ════════════════════════════════════════════════════════════════════
    def _control_loop(self, event):
        if self.nav_state == "PAUSED":
            # User-Pause: Navigation gibt den Fahrkanal frei, damit der Controller
            # manuell auf /cmd_vel_controll steuern kann.
            return

        # Non-blocking Pause nach dem Sprühen (ersetzt das blockierende time.sleep,
        # das zuvor den 100-Hz-Timer-Thread einfror).
        if self.pause_until is not None:
            if rospy.Time.now() < self.pause_until:
                self.motion.stop()
                return
            self.pause_until = None

        if not self._preconditions_met():
            return

        cur_x, cur_y = gps_to_xy(self.rtk.current_lat, self.rtk.current_lon,
                                 self.rtk.origin_lat, self.rtk.origin_lon)

        if self.nav_state == "WAITING_FOR_FIX":
            self._initialize_calibration_state(cur_x, cur_y)
        elif self.nav_state == "CALIBRATING":
            self._handle_calibrating(cur_x, cur_y)
        elif self.nav_state == "NAVIGATING":
            self._handle_navigating(cur_x, cur_y)

    def _enter_paused(self):
        if self.nav_state == "PAUSED":
            return

        if self.nav_state not in ("WAITING_FOR_FIX", "CALIBRATING", "NAVIGATING"):
            rospy.logwarn(f"Pause in Zustand {self.nav_state} ignoriert")
            return

        self.nav_state_before_pause = self.nav_state
        self.nav_state = "PAUSED"
        self.pause_until = None
        self.in_tol_since = None
        self.motion.stop()
        self._publish_paused_state()
        self.status.info(
            "NAVIGATION_PAUSED",
            "Autonome Navigation pausiert – manuelle Steuerung ist möglich",
            dedup=False,
        )
        rospy.loginfo("Autonome Navigation pausiert; /cmd_vel_controll wird freigegeben")

    def _resume_from_pause(self):
        if self.nav_state != "PAUSED":
            return

        previous_state = self.nav_state_before_pause or "WAITING_FOR_FIX"
        self.nav_state_before_pause = None

        # Nach manueller Bewegung sind alte Zwischenziele nicht mehr verlässlich.
        # Der aktuelle Ziel-Waypoint bleibt erhalten und wird von der neuen
        # Position aus wieder angefahren.
        self.bypass_targets = []
        self.guide_targets = []
        self.guide_waypoint_index = None
        self.in_tol_since = None
        self.pause_until = None
        self.ignore_bypass_responses_until = rospy.Time.now() + rospy.Duration(0.2)

        if previous_state == "CALIBRATING":
            self.nav_state = "WAITING_FOR_FIX"
            self.state_pub.publish(RobotState.STATE_IDLE, waypoint_total=len(self.waypoints))
            message = "Navigation freigegeben – Kalibrierung wird von aktueller Position neu gestartet"
        else:
            self.nav_state = previous_state
            if self.nav_state == "NAVIGATING":
                self._publish_navigating_state()
                message = "Navigation freigegeben – aktueller Waypoint wird von neuer Position angefahren"
            else:
                self.state_pub.publish(RobotState.STATE_IDLE, waypoint_total=len(self.waypoints))
                message = "Navigation freigegeben"

        self.status.info("NAVIGATION_RESUMED", message, dedup=False)
        rospy.loginfo(message)

    def _publish_paused_state(self):
        if self.current_waypoint_index >= len(self.waypoints):
            self.state_pub.publish(RobotState.STATE_PAUSED, waypoint_total=len(self.waypoints))
            return

        goal_lat, goal_lon = self.waypoints[self.current_waypoint_index]
        self.state_pub.publish(
            RobotState.STATE_PAUSED,
            waypoint_index=self.current_waypoint_index + 1,
            waypoint_total=len(self.waypoints),
            target_lat=float(goal_lat),
            target_lon=float(goal_lon),
        )

    def _preconditions_met(self) -> bool:
        """Returns False (and stops the robot) if any required condition is not yet satisfied."""
        if not self.rtk.rtk_fix_initialized:
            rospy.logwarn_throttle(
                2, f"Warte auf stabilen RTK-FIXED ({self.p.rtk_stable_sec:.0f}s am Stück)...")
            self.motion.stop()
            return False

        # ── Frische-Check: ohne aktuelle FIXED-Position NICHT blind weiterfahren ──
        # Während kurzer FLOAT-Phasen halten wir die letzte Position; bleibt FIXED
        # länger als gps_timeout weg, wird gestoppt statt auf veralteter Position
        # weiterzufahren.
        if not self.rtk.is_fix_fresh() or not self.rtk.has_fix:
            rospy.logwarn_throttle(
                2, f"Kein frischer RTK-FIXED seit >{self.p.gps_timeout:.1f}s -> Stop")
            # Edge: nur beim ersten Eintritt melden; RTK_RECOVERED kommt aus dem RTKTracker.
            if not self.rtk.rtk_lost:
                self.rtk.rtk_lost = True
                self.status.error("RTK_LOST", "Kein frischer RTK-FIXED – Roboter stoppt", dedup=False)
            self.motion.stop()
            return False

        if self.current_waypoint_index >= len(self.waypoints):
            if not self.at_goal:
                rospy.loginfo("Alle Waypoints erreicht! Roboter stoppt.")
                self.at_goal = True
                self.status.info("GOAL_REACHED", "Alle Waypoints erreicht – Roboter stoppt")
                self.state_pub.publish(RobotState.STATE_GOAL_REACHED,
                                       waypoint_total=len(self.waypoints))
            self.motion.stop()
            return False

        return True

    # ════════════════════════════════════════════════════════════════════
    # PHASE 1: AUTOMATISCHE KALIBRIERUNG (Kinematic Alignment)
    # ════════════════════════════════════════════════════════════════════
    def _initialize_calibration_state(self, cur_x: float, cur_y: float):
        """Records the calibration start position and transitions to CALIBRATING."""
        self.calib.start(cur_x, cur_y)
        self.nav_state = "CALIBRATING"
        self.state_pub.publish(RobotState.STATE_CALIBRATING, waypoint_total=len(self.waypoints))
        rospy.loginfo("Starte Auto-Kalibrierung: Fahre geradeaus, um den GPS-Vektor zu messen...")

    def _handle_calibrating(self, cur_x: float, cur_y: float):
        """Drives straight and accumulates GPS displacement to compute the heading offset."""
        if self.heading is None:
            self.motion.stop()
            rospy.logwarn_throttle(2, "Warte auf IMU Daten für Kalibrierung...")
            return

        # Geradeaus fahren, während der GPS-Vektor gemessen wird
        self.motion.publish(self.calib.calib_speed(), 0.0)

        if self.calib.update(cur_x, cur_y, self.heading):
            self.motion.stop()  # kurz stoppen
            self.nav_state = "NAVIGATING"
            self._publish_navigating_state()

    def _publish_navigating_state(self):
        """Published STATE_NAVIGATING mit dem aktuellen Ziel-Wegpunkt (1-basiert + lat/lon)."""
        if self.current_waypoint_index >= len(self.waypoints):
            return
        goal_lat, goal_lon = self.waypoints[self.current_waypoint_index]
        self.state_pub.publish(
            RobotState.STATE_NAVIGATING,
            waypoint_index=self.current_waypoint_index + 1,
            waypoint_total=len(self.waypoints),
            target_lat=float(goal_lat),
            target_lon=float(goal_lon),
        )

    # ════════════════════════════════════════════════════════════════════
    # PHASE 2: NORMALE NAVIGATION ZUM ZIEL
    # ════════════════════════════════════════════════════════════════════
    def _handle_navigating(self, cur_x: float, cur_y: float):
        """Steuert den Roboter über Leitpunkte/Bypass zum aktuellen Waypoint."""
        # 1. Echtes Welt-Heading (Roh + Kalibrier-Offset)
        true_robot_heading = self.calib.compute_true_heading(self.heading)

        # GPS sitzt 58cm hinter der Düse → Düsenposition in Fahrtrichtung vorrechnen
        nozzle_x = cur_x + self.p.gps_to_nozzle_offset * math.cos(true_robot_heading)
        nozzle_y = cur_y + self.p.gps_to_nozzle_offset * math.sin(true_robot_heading)

        # 2. Umfahrung aktiv? → erst die Bypass-Punkte der Reihe nach abfahren
        if self.bypass_targets:
            self._navigate_to_bypass(nozzle_x, nozzle_y)
            return

        # 3. Ziel-Wegpunkt berechnen
        goal_lat, goal_lon = self.waypoints[self.current_waypoint_index]
        goal_x, goal_y = gps_to_xy(goal_lat, goal_lon, self.rtk.origin_lat, self.rtk.origin_lon)

        # 4. Virtuelle Leitpunkte für den aktuellen Waypoint erzeugen (einmalig
        #    pro Waypoint bzw. nach einer Umfahrung von der aktuellen Position aus)
        if self.p.guide_enabled and self.guide_waypoint_index != self.current_waypoint_index:
            self.guide_targets = generate_guide_points(
                nozzle_x, nozzle_y, true_robot_heading, goal_x, goal_y,
                self.p.guide_spacing, self.p.guide_min_goal_dist, self.p.guide_final_gap)
            self.guide_waypoint_index = self.current_waypoint_index
            if self.guide_targets:
                pts = ", ".join(f"({px:.2f}, {py:.2f})" for px, py in self.guide_targets)
                debug_output = (f"{len(self.guide_targets)} Leitpunkte für Waypoint "
                                f"{self.current_waypoint_index + 1} erzeugt: {pts}")
                rospy.loginfo(debug_output)
                self.logger.info(debug_output)

        # 5. Anfrage an die Obstacle-Avoidance-Node senden.
        #    Geprüft wird das Segment zum NÄCHSTEN tatsächlichen Fahrziel
        #    (Leitpunkt oder Waypoint), damit auch die gekrümmte Leitkurve
        #    gegen Hindernisse abgesichert ist.
        if self.guide_targets:
            check_x, check_y = self.guide_targets[0]
        else:
            check_x, check_y = goal_x, goal_y
        request = {
            'robot_x': nozzle_x,
            'robot_y': nozzle_y,
            'goal_x': check_x,
            'goal_y': check_y,
            'origin_lat': self.rtk.origin_lat,
            'origin_lon': self.rtk.origin_lon,
            'obstacle_margin': self.p.obstacle_margin,
        }
        self.bypass_request_pub.publish(String(data=json.dumps(request)))

        # 6. Leitpunkte aktiv? → erst die Leitpunkte der Reihe nach abfahren
        if self.guide_targets:
            self._navigate_to_guide(nozzle_x, nozzle_y)
            return

        dx = goal_x - nozzle_x
        dy = goal_y - nozzle_y
        distance = math.hypot(dx, dy)

        # 7. Waypoint erreicht? -> Spray nur mit RTK-FIXED-Quer-Check.
        fix_ok = (not self.p.require_fix_for_spray) or (self.rtk.gps_quality == 4)
        if distance < self.p.waypoint_tolerance and fix_ok:
            self.motion.stop()  # anhalten und Position bestätigen lassen
            debug_output = (f"Waypoint {self.current_waypoint_index + 1} bestätigt "
                            f"(Distanz {distance:.2f}m) -> SPRAY")
            rospy.loginfo(debug_output)
            self.logger.info(debug_output)
            self.spray_pub.publish()
            rest = spray()
            rospy.loginfo(f"Noch {rest} Sprühstöße verfügbar")
            self.current_waypoint_index += 1
            self.in_tol_since = None
            self.pause_until = rospy.Time.now() + rospy.Duration(self.p.waypoint_pause_sec)
            # Wegpunktwechsel -> neuer Ziel-Wegpunkt im State (GOAL_REACHED kommt ggf.
            # im nächsten _preconditions_met-Durchlauf, wenn keine Wegpunkte mehr da sind).
            self._publish_navigating_state()
        else:
            # Toleranz verlassen oder kein FIXED -> Bestätigung zurücksetzen
            self.in_tol_since = None

        # 8. Proportionaler Lenkungsbefehl basierend auf dem Winkel zum Ziel
        self._steer_towards(nozzle_x, nozzle_y, goal_x, goal_y,
                            label=f"Wegpunkt {self.current_waypoint_index + 1}")

    def _navigate_to_bypass(self, cur_x: float, cur_y: float):
        """Fährt die Umfahrungspunkte der Reihe nach ab; entfernt jeden, wenn erreicht."""
        bx, by = self.bypass_targets[0]
        dist = math.hypot(bx - cur_x, by - cur_y)
        rospy.loginfo_throttle(1, f"Distanz to bypass: {dist}")

        if dist < self.p.bypass_tolerance:
            self.bypass_targets.pop(0)
            if self.bypass_targets:
                rospy.loginfo("1. Umfahrungspunkt erreicht — weiter zum 2. Punkt")
            else:
                rospy.loginfo("Umfahrung abgeschlossen — weiter zum Waypoint")
            # An jedem Bypass-Punkt kurz stehen bleiben (non-blocking Pause)
            self.motion.stop()
            self.pause_until = rospy.Time.now() + rospy.Duration(self.p.bypass_pause_sec)
            return

        label = f"Bypass {'1/2' if len(self.bypass_targets) == 2 else '2/2'}"
        self._steer_towards(cur_x, cur_y, bx, by, label=label)

    def _navigate_to_guide(self, cur_x: float, cur_y: float):
        """Fährt die Leitpunkte der Reihe nach ab; KEIN Spray, nur Debug-Pause."""
        gx, gy = self.guide_targets[0]
        dist = math.hypot(gx - cur_x, gy - cur_y)

        if dist < self.p.guide_tolerance:
            self.guide_targets.pop(0)
            debug_output = (f"Leitpunkt erreicht (Distanz {dist:.2f}m), "
                            f"noch {len(self.guide_targets)} Leitpunkte bis "
                            f"Waypoint {self.current_waypoint_index + 1}")
            rospy.loginfo(debug_output)
            self.logger.info(debug_output)
            # Debug: kurz stehenbleiben, um das Verhalten beobachten zu können
            self.motion.stop()
            if self.p.guide_pause_sec > 0.0:
                self.pause_until = rospy.Time.now() + rospy.Duration(self.p.guide_pause_sec)
            return

        remaining = len(self.guide_targets)
        self._steer_towards(cur_x, cur_y, gx, gy,
                            label=f"Leitpunkt ({remaining} übrig, WP {self.current_waypoint_index + 1})")

    # ════════════════════════════════════════════════════════════════════
    # LENKUNG (dünner Wrapper: kalibriertes Heading -> MotionController)
    # ════════════════════════════════════════════════════════════════════
    def _steer_towards(self, cur_x: float, cur_y: float,
                       target_x: float, target_y: float, label: str = ""):
        true_robot_heading = self.calib.compute_true_heading(self.heading)
        self.motion.steer(cur_x, cur_y, target_x, target_y,
                          true_robot_heading, self.heading, label=label)


if __name__ == '__main__':
    try:
        rospy.init_node('navigation_node')
        node = NavigationNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
