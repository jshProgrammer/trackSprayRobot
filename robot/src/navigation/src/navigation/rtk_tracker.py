"""RTK/GPS-Verarbeitung: Status-Parsing, FIXED-Streak-Gating, Ausreißer-Filter.

Kapselt den gesamten GPS-Positions-State der Navigation inklusive der
GPS-Subscriber (gps/fix, gps/quality). Der Node ruft einmal
``start_subscribers()`` auf und liest danach Position, Freigabe
(``rtk_fix_initialized``) und Frische (``last_fix_time``) ab.

Status-Reporter und Logger werden injiziert, rospy.Time wird direkt genutzt
(identisches Zeitverhalten wie zuvor in der Node). Der Konstruktor selbst
bleibt ROS-frei (testbar); ROS wird erst in ``start_subscribers()`` berührt.
"""

import rospy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import UInt8

from navigation.geo import latlon_dist


class RTKStatus:
    NO_FIX = 0
    FLOAT = 1
    FIXED = 2


def parse_rtk_status(msg) -> int:
    """Leitet den RTK-Status aus NavSatFix-Status und Kovarianz ab."""
    status = msg.status.status
    cov = msg.position_covariance[0]
    if status < 0:
        return RTKStatus.NO_FIX
    if cov <= 0.01:
        return RTKStatus.FIXED
    elif cov <= 0.25:
        return RTKStatus.FLOAT
    else:
        return RTKStatus.NO_FIX


class RTKTracker:
    def __init__(self, params, status, logger):
        self._p = params
        self._status = status
        self._logger = logger

        # ── GPS-State ────────────────────────────────────────────────────
        self.current_lat = None
        self.current_lon = None
        self.has_fix = False
        self.rtk_fix_initialized = False     # wird erst nach stabilem FIXED-Streak True
        self.gps_quality = 0

        self.fix_streak_start = None  # Beginn des aktuellen ununterbrochenen FIXED-Streaks
        self.last_fix_time = None     # Zeitpunkt der letzten akzeptierten FIXED-Position

        self.origin_lat = None
        self.origin_lon = None

        # Eigene Flanke für RTK_LOST<->RTK_RECOVERED, da der globale dedup im
        # StatusReporter durch andere Events (z.B. GOAL_REACHED) zurückgesetzt würde.
        self.rtk_lost = False

    def set_status(self, status):
        """Status-Reporter nachreichen (wird erst in _init_ros der Node erzeugt)."""
        self._status = status

    def start_subscribers(self):
        """Legt die GPS-Subscriber an. Erst nach rospy.init_node aufrufen."""
        rospy.Subscriber('gps/fix', NavSatFix, self.process_fix)
        rospy.Subscriber('gps/quality', UInt8, self._gps_quality_callback)

    def _gps_quality_callback(self, msg):
        # Wird als Quer-Check beim Sprühen genutzt (require_fix_for_spray).
        # Die Navigations-Freigabe selbst läuft über den stabilen FIXED-Streak
        # (RTK-Status aus der Kovarianz der Fix-Nachricht).
        self.set_quality(msg.data)
        rospy.loginfo_throttle(5, f"GPS Quality = {self.gps_quality}")

    def process_fix(self, msg):
        """Verarbeitet eine NavSatFix-Nachricht."""
        rtk = parse_rtk_status(msg)

        self.startup_calibrating(rtk)

        # ── Position NUR aus RTK FIXED übernehmen ─────────────────────────
        # FLOAT/DGPS/GPS verursachen 30-50cm-Sprünge. Statt sie zu nutzen,
        # halten wir die letzte gute FIXED-Position; der gps_timeout-Check
        # stoppt den Roboter, falls FIXED zu lange wegbleibt.
        if rtk != RTKStatus.FIXED:
            return

        new_lat = float(msg.latitude)
        new_lon = float(msg.longitude)

        # ── Ausreißer-Filter: physikalisch unmögliche Sprünge verwerfen ───
        # Bei max_linear m/s kann sich die Position pro Epoche nur begrenzt
        # ändern. Größere Sprünge sind RTK-Artefakte und werden ignoriert.
        if self.current_lat is not None and self.last_fix_time is not None:
            dt = (rospy.Time.now() - self.last_fix_time).to_sec()
            jump = latlon_dist(self.current_lat, self.current_lon, new_lat, new_lon)
            max_plausible = self._p.max_gps_jump + self._p.max_linear * max(dt, 0.0)
            # TODO: tried 20 percent margin
            if jump > 1.2 * max_plausible:
                debug_output = (f"GPS-Sprung verworfen: {jump:.2f}m > erlaubt "
                                f"{max_plausible:.2f}m (dt={dt:.2f}s)")
                rospy.logwarn_throttle(2, debug_output)
                self._logger.warning(debug_output)
                return

        self.has_fix = True
        self.current_lat = new_lat
        self.current_lon = new_lon
        self.last_fix_time = rospy.Time.now()

        # Gegenflanke zu RTK_LOST: frischer FIXED nach einem Verlust -> RECOVERED.
        if self.rtk_lost:
            self.rtk_lost = False
            self._status.info("RTK_RECOVERED", "RTK-Fix wieder da – Fahrt geht weiter", dedup=False)

        rospy.loginfo_throttle(5, f"GPS FIXED: lat={self.current_lat:.7f}, lon={self.current_lon:.7f}")

        if self.origin_lat is None:
            self.origin_lat = new_lat
            self.origin_lon = new_lon
            debug_output = f"RTK-Ursprung gesetzt: {new_lat}, {new_lon}"
            rospy.loginfo(debug_output)
            self._logger.info(debug_output)

    def startup_calibrating(self, rtk):
        # ── Startup-Gate: RTK muss erst STABIL FIXED sein ─────────────────
        # Ein einzelner FIXED-Treffer reicht NICHT. Wir verlangen einen
        # ununterbrochenen FIXED-Streak von rtk_stable_sec Sekunden, bevor
        # die Navigation freigegeben wird (Beobachtung: anfangs flackert
        # FIXED/FLOAT, erst danach wird es regelmäßig).
        if rtk == RTKStatus.FIXED:
            if self.fix_streak_start is None:
                self.fix_streak_start = rospy.Time.now()
            streak = (rospy.Time.now() - self.fix_streak_start).to_sec()
            if not self.rtk_fix_initialized and streak >= self._p.rtk_stable_sec:
                self.rtk_fix_initialized = True
                debug_output = f"RTK stabil ({streak:.1f}s FIXED am Stück) -> Navigation freigegeben"
                rospy.loginfo(debug_output)
                self._logger.info(debug_output)
                self._status.info("rtk_fix_initialized", "RTK stabil – Navigation freigegeben")
        else:
            # Jeder Nicht-FIXED (FLOAT/DGPS/GPS/NO_FIX) unterbricht den Streak.
            if self.fix_streak_start is not None and not self.rtk_fix_initialized:
                rospy.logwarn_throttle(5, "RTK-FIXED unterbrochen vor Freigabe – Streak zurückgesetzt")
                self._status.warn("RTK_UNSTABLE", "RTK-FIXED unterbrochen vor Freigabe")
            self.fix_streak_start = None


    def set_quality(self, quality: int):
        """Wird als Quer-Check beim Sprühen genutzt (require_fix_for_spray)."""
        self.gps_quality = quality

    def is_fix_fresh(self) -> bool:
        """True, wenn die letzte FIXED-Position jünger als gps_timeout ist."""
        if self.last_fix_time is None:
            return False
        return (rospy.Time.now() - self.last_fix_time).to_sec() <= self._p.gps_timeout
