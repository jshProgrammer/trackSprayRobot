"""Zentrale Sammlung aller Navigations-Parameter (rosparam).

Alle ~private Parameter werden an EINER Stelle gelesen, statt über die
Node verstreut. ``NavParams.from_rosparam()`` kapselt den rospy-Zugriff,
das Dataclass selbst ist ROS-frei und damit testbar.
"""

from dataclasses import dataclass

import rospy


@dataclass
class NavParams:
    # ── Kinematik / Geschwindigkeit ──────────────────────────────────────
    wheel_base: float
    max_linear: float
    max_angular: float
    forward_velocity: float
    angular_velocity: float

    # ── Toleranzen ───────────────────────────────────────────────────────
    waypoint_tolerance: float
    bypass_tolerance: float

    # ── Waypoints / Geometrie ────────────────────────────────────────────
    waypoints_file: str
    gps_to_nozzle_offset: float
    min_gps_status: int
    obstacle_margin: float

    # ── RTK-Stabilität, Ausreißer-Schutz & Spray-Bestätigung ────────────
    rtk_stable_sec: float
    gps_timeout: float
    max_gps_jump: float
    spray_confirm_sec: float
    require_fix_for_spray: bool
    waypoint_pause_sec: float
    bypass_pause_sec: float

    # ── Virtuelle Leitpunkte ("Lotse") ──────────────────────────────────
    guide_enabled: bool
    guide_spacing: float
    guide_tolerance: float
    guide_pause_sec: float
    guide_min_goal_dist: float
    guide_final_gap: float

    # ── Auto-Kalibrierung (Kinematic Alignment) ─────────────────────────
    calib_distance: float
    calib_min_time: float

    @classmethod
    def from_rosparam(cls) -> "NavParams":
        return cls(
            wheel_base=rospy.get_param('~wheel_base', 0.5),
            max_linear=rospy.get_param('~max_linear', 1.0),
            max_angular=rospy.get_param('~max_angular', 1.0),
            forward_velocity=rospy.get_param('~forward_velocity', 0.5),
            angular_velocity=rospy.get_param('~angular_velocity', 0.5),
            # TODO: might have to be changed
            waypoint_tolerance=rospy.get_param('~waypoint_tolerance', 0.3),
            # Bypass-Punkte müssen nicht exakt getroffen werden (kein Spray) -> größere Toleranz
            bypass_tolerance=rospy.get_param('~bypass_tolerance', 0.7),
            # Waypoints kommen primär aus einem JSON-File (vom Frontend auf den Pi geschrieben);
            # Fallback ist der rosparam ~waypoints (navigation.yaml), falls die Datei fehlt.
            waypoints_file=rospy.get_param(
                '~waypoints_file',
                '/home/ubuntu/trackSprayRobot/shared_files/waypoints.json'),
            gps_to_nozzle_offset=rospy.get_param('~gps_to_nozzle_offset', 0.58),
            min_gps_status=rospy.get_param('~min_gps_status', 0),
            obstacle_margin=rospy.get_param('~obstacle_margin', 1.0),
            rtk_stable_sec=rospy.get_param('~rtk_stable_sec', 3.0),
            gps_timeout=rospy.get_param('~gps_timeout', 2.0),
            max_gps_jump=rospy.get_param('~max_gps_jump', 0.30),
            spray_confirm_sec=rospy.get_param('~spray_confirm_sec', 0.5),
            require_fix_for_spray=rospy.get_param('~require_fix_for_spray', True),
            waypoint_pause_sec=rospy.get_param('~waypoint_pause_sec', 5.0),
            bypass_pause_sec=rospy.get_param('~bypass_pause_sec', 5.0),
            guide_enabled=rospy.get_param('~guide_enabled', True),
            # Abstand zwischen zwei Leitpunkten entlang der Kurve
            guide_spacing=rospy.get_param('~guide_spacing', 2.0),
            # Leitpunkte müssen (wie Bypass) nicht exakt getroffen werden
            guide_tolerance=rospy.get_param('~guide_tolerance', 0.7),
            # Debug: an jedem Leitpunkt kurz stehenbleiben
            guide_pause_sec=rospy.get_param('~guide_pause_sec', 2.0),
            # Unterhalb dieser Zieldistanz lohnen sich keine Leitpunkte mehr
            guide_min_goal_dist=rospy.get_param('~guide_min_goal_dist', 3.0),
            # Der letzte Leitpunkt liegt mind. so weit vorm Ziel -> freie, gerade Endanfahrt
            guide_final_gap=rospy.get_param('~guide_final_gap', 1.5),
            # Auto-Kalibrierung: Strecke geradeaus, bis der GPS-Vektor stabil ist
            calib_distance=rospy.get_param('~calib_distance', 3.0),
            calib_min_time=rospy.get_param('~calib_min_time', 3.0),
        )
