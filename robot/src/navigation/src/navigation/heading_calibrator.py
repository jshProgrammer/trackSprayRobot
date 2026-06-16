"""Kinematic Alignment: richtet das IMU-Heading an der GPS-Weltkarte aus.

Der Roboter fährt zu Beginn ein Stück geradeaus; aus dem gefahrenen
GPS-Vektor wird der Offset zwischen rohem IMU-Heading und Welt-Heading
bestimmt. Danach liefert ``compute_true_heading`` das kalibrierte Heading.

Fahr-/Stopp-Befehle und State-Publishing bleiben im Node; dieser Helfer
kapselt nur Mathematik und den Kalibrier-Fortschritt.
"""

import math

import rospy

from navigation.geo import normalize_angle

# Geradeausstrecke, ab der der GPS-Vektor als stabil genug gilt.
# TODO: nach Bedarf zu einem rosparam machen
CALIB_DISTANCE = 3.0
CALIB_MIN_TIME = 3.0


class HeadingCalibrator:
    def __init__(self, params, logger):
        self._p = params
        self._logger = logger

        self.heading_offset = 0.0
        self.calib_start_x = None
        self.calib_start_y = None
        self.calib_start_time = None

    def start(self, cur_x: float, cur_y: float):
        """Merkt sich die Startposition für die Kalibrierfahrt."""
        self.calib_start_x = cur_x
        self.calib_start_y = cur_y
        self.calib_start_time = rospy.Time.now()

    def update(self, cur_x: float, cur_y: float, raw_heading: float) -> bool:
        """Aktualisiert den Kalibrier-Fortschritt.

        Gibt True zurück, sobald genug Strecke gefahren wurde und der Offset
        erfolgreich bestimmt werden konnte (-> Kalibrierung abgeschlossen).
        """
        calib_dx = cur_x - self.calib_start_x
        calib_dy = cur_y - self.calib_start_y
        distance_driven = math.hypot(calib_dx, calib_dy)

        rospy.loginfo_throttle(
            0.5, f"Kalibriere... Gefahren: {distance_driven:.2f}m / {CALIB_DISTANCE:.1f}m")

        calib_time = (rospy.Time.now() - self.calib_start_time).to_sec()

        if distance_driven >= CALIB_DISTANCE and calib_time > CALIB_MIN_TIME:
            return self._finalize(calib_dx, calib_dy, distance_driven, raw_heading)
        return False

    def calib_speed(self) -> float:
        """Geschwindigkeit während der Kalibrierfahrt (halbe Vorwärtsgeschwindigkeit)."""
        return max(0.1, self._p.forward_velocity * 0.5)

    def _finalize(self, calib_dx, calib_dy, distance_driven, raw_heading) -> bool:
        if distance_driven < 1.0:
            return False

        if abs(calib_dx) < 0.2 and abs(calib_dy) < 0.2:
            rospy.logwarn("GPS Bewegung zu klein für Heading")
            return False

        # Echten Welt-Winkel aus der gefahrenen GPS-Strecke berechnen
        true_gps_heading = math.atan2(calib_dy, calib_dx)
        self.heading_offset = normalize_angle(true_gps_heading - raw_heading)

        rospy.loginfo(
            f"Kalibrierung: dx={calib_dx:.2f}m, dy={calib_dy:.2f}m, "
            f"GPS-Winkel={math.degrees(true_gps_heading):.1f}°, "
            f"IMU-Roh={math.degrees(raw_heading):.1f}°, "
            f"Offset={math.degrees(self.heading_offset):.1f}°"
        )
        rospy.loginfo("=" * 50)
        rospy.loginfo("KALIBRIERUNG ERFOLGREICH!")
        rospy.loginfo(f"GPS Welt-Winkel: {math.degrees(true_gps_heading):.1f}°")
        rospy.loginfo(f"Roher IMU-Winkel: {math.degrees(raw_heading):.1f}°")
        rospy.loginfo(f"Berechneter Offset: {math.degrees(self.heading_offset):.1f}°")
        rospy.loginfo("=" * 50)
        return True

    def compute_true_heading(self, raw_heading: float) -> float:
        """Wendet den Kalibrier-Offset auf das rohe IMU-Heading an (Welt-Frame)."""
        return normalize_angle(raw_heading + self.heading_offset)
