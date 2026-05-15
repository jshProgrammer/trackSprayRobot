#!/usr/bin/env python3
"""
2.5D EKF + Stanley-Spurführungsregler für ROS Noetic.
Exklusiv optimiert für: Geradeausfahrt trotz Schlupf, Matsch und Bodenunebenheiten.

Ansteuerung: Fahrbefehl auf '/cmd_vel_controll' geben.
Der Node regelt den Kurs und schickt die korrigierten Motorbefehle auf '/cmd_vel'.
"""

import rospy
import numpy as np
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, NavSatFix
from tf.transformations import euler_from_quaternion

class RTKStatus:
    NO_FIX = 0
    FLOAT  = 1
    FIXED  = 2

class EKFNoeticNode:
    _gps_origin_lat = None
    _gps_origin_lon = None
    _rtk_status     = RTKStatus.NO_FIX

    def __init__(self):
        rospy.init_node('ekf_rtk_node', anonymous=True)

        # ── EKF-Zustand: [x, y, θ, v] ──────────────────────────────────
        self.x = np.zeros((4, 1))
        self.P = np.diag([1.0, 1.0, 0.1, 0.5])

        # Q (Prozessrauschen) ist höher angesetzt. Wenn die Räder durchdrehen,
        # vertraut der Filter primär den physikalischen IMU- und GPS-Messungen.
        self.Q = np.diag([0.06, 0.06, 0.02, 0.06])

        # Messrauschen
        self.R_imu = np.array([[0.005]]) 
        self.R_gps = {
            RTKStatus.FIXED: np.diag([0.0004, 0.0004]),  # RTK-Fixed (2 cm)
            RTKStatus.FLOAT: np.diag([0.09,   0.09  ]),  # RTK-Float (30 cm)
        }

        self.H_imu = np.array([[0, 0, 1, 0]])
        self.H_gps = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])

        # ── Regler & Spur-Parameter ───────────────────────────────────
        self.linear_speed = 0.0
        self.is_moving = False
        
        # Startpunkt der unsichtbaren Führungsschiene
        self.line_start_x = 0.0
        self.line_start_y = 0.0
        self.target_heading = 0.0

        # Regler-Gewinne (Steuern, wie aggressiv Schlupf korrigiert wird)
        self.kp_heading = 2.5   # Zieht die Nase schnell wieder geradeaus
        self.kd_heading = 0.4   # Verhindert, dass das Heck anfängt zu schwingen
        self.kp_track = 2.0     # Je höher, desto härter gibt er Gegenfeuer bei seitlichem Drift
        
        self._last_heading_error = 0.0
        self.last_time = rospy.Time.now()

        # ── Sensordaten-Zwischenspeicher ───────────────────────────────
        self.imu_w = 0.0        # Aktuelle echte Drehrate aus dem Gyroskop
        self.antenna_height = 1.2 # WICHTIG: Deine GPS-Antennenhöhe in Metern hier eintragen!
        self.roll = 0.0
        self.pitch = 0.0

        # ── ROS Schnittstellen ──────────────────────────────────────────
        rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        rospy.Subscriber('/gps/fix', NavSatFix, self.gps_callback)
        rospy.Subscriber('/cmd_vel_controll', Twist, self.cmd_vel_callback)
        
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Regler- und Filtertaktung auf 20 Hz
        rospy.Timer(rospy.Duration(0.05), self.control_loop)
        rospy.loginfo("EKF-Spurführungsregler aktiv. Bereit für unebenes Gelände.")

    @staticmethod
    def _wrap(a):
        return (a + math.pi) % (2 * math.pi) - math.pi

    def _latlon_to_meters(self, lat, lon):
        R = 6371000.0
        dlat = math.radians(lat - self._gps_origin_lat)
        dlon = math.radians(lon - self._gps_origin_lon)
        ref  = math.radians(self._gps_origin_lat)
        return dlon * R * math.cos(ref), dlat * R

    def _parse_rtk_status(self, msg):
        status = msg.status.status
        cov = msg.position_covariance[0]
        if status < 0: return RTKStatus.NO_FIX
        if cov <= 0.001: return RTKStatus.FIXED
        elif cov <= 0.25: return RTKStatus.FLOAT
        else: return RTKStatus.NO_FIX

    # ══════════════════════════════════════════════════════════════════
    # CALLBACKS
    # ══════════════════════════════════════════════════════════════════

    def cmd_vel_callback(self, msg):
        # Dynamisches Einrasten: Sobald Bewegung gefordert wird, nageln wir die Linie fest
        if not self.is_moving and abs(msg.linear.x) > 0.01:
            self.line_start_x = self.x[0, 0]
            self.line_start_y = self.x[1, 0]
            self.target_heading = self.x[2, 0] 
            rospy.loginfo(f"Spur fixiert auf aktueller Trajektorie! Kurs: {math.degrees(self.target_heading):.1f}°")
        
        self.linear_speed = msg.linear.x
        self.is_moving = abs(msg.linear.x) > 0.01

    def imu_callback(self, msg):
        # Echte Drehrate für den EKF-Predict sichern
        self.imu_w = msg.angular_velocity.z

        # Orientierung für Hebelarm-Korrektur auslesen
        q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        self.roll, self.pitch, yaw = euler_from_quaternion(q)

        # Winkel-Update in den Filter jagen
        self._ekf_update(z=np.array([[yaw]]), H=self.H_imu, R=self.R_imu, wrap_idx=0)

    def gps_callback(self, msg):
        rtk = self._parse_rtk_status(msg)
        self._rtk_status = rtk
        if rtk == RTKStatus.NO_FIX: return

        if self._gps_origin_lat is None:
            if rtk == RTKStatus.FIXED:
                self._gps_origin_lat = msg.latitude
                self._gps_origin_lon = msg.longitude
                rospy.loginfo(f"RTK-Nullpunkt kalibriert: {msg.latitude}, {msg.longitude}")
            return

        gx, gy = self._latlon_to_meters(msg.latitude, msg.longitude)

        # ── HEBELARM-KOMPENSATION FÜR UNEBENHEITEN ──
        # Nickt oder wankt der Roboter im Loch, wird die Antennenbewegung herausgerechnet
        gx = gx - self.antenna_height * math.sin(self.pitch)
        gy = gy + self.antenna_height * math.sin(self.roll)

        # Positions-Update in den Filter jagen
        self._ekf_update(z=np.array([[gx], [gy]]), H=self.H_gps, R=self.R_gps[rtk])

    # ══════════════════════════════════════════════════════════════════
    # FILTER MATH
    # ══════════════════════════════════════════════════════════════════

    def _ekf_predict(self, v, w, dt):
        px, py, th, _ = self.x.flatten()
        # Vorhersage nutzt die echte Drehrate der IMU (w)
        self.x = np.array([
            [px + v * math.cos(th) * dt],
            [py + v * math.sin(th) * dt],
            [self._wrap(th + w * dt)],
            [v],
        ])
        F = np.array([
            [1, 0, -v * math.sin(th) * dt,  math.cos(th) * dt],
            [0, 1,  v * math.cos(th) * dt,  math.sin(th) * dt],
            [0, 0,  1,                      0],
            [0, 0,  0,                      1],
        ])
        self.P = F @ self.P @ F.T + self.Q

    def _ekf_update(self, z, H, R, wrap_idx=None):
        innovation = z - H @ self.x
        if wrap_idx is not None:
            innovation[wrap_idx, 0] = self._wrap(innovation[wrap_idx, 0])
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ innovation
        self.x[2, 0] = self._wrap(self.x[2, 0])
        self.P = (np.eye(4) - K @ H) @ self.P

    # ══════════════════════════════════════════════════════════════════
    # REGLER-SCHLEIFE (20 Hz)
    # ══════════════════════════════════════════════════════════════════

    def control_loop(self, event):
        now = rospy.Time.now()
        dt = (now - self.last_time).to_sec()
        self.last_time = now
        if dt <= 0: return

        # EKF-Zustand fortschreiben mit echter Gyro-Drehrate
        self._ekf_predict(v=self.linear_speed, w=self.imu_w, dt=dt)

        cmd = Twist()
        if self.is_moving:
            current_x   = self.x[0, 0]
            current_y   = self.x[1, 0]
            current_yaw = self.x[2, 0]

            # 1. Winkelfehler ermitteln (Kurshalte-Regler)
            heading_error = self._wrap(self.target_heading - current_yaw)
            d_heading_error = (heading_error - self._last_heading_error) / dt
            self._last_heading_error = heading_error

            # 2. Querablagefehler berechnen (Spurtreue-Regler bei Schlupf)
            dx = current_x - self.line_start_x
            dy = current_y - self.line_start_y
            
            # Vorzeichen-Gefreite Berechnung: Positiv = Abweichung nach rechts, Negativ = nach links
            cross_track_error = math.sin(self.target_heading) * dx - math.cos(self.target_heading) * dy

            # 3. Steuersignal fusionieren
            # Kommt der Roboter nach rechts ab, korrigiert ein positives 'steering_out' nach links.
            # Das ROS-Fahrwerk gibt dadurch automatisch mehr Power auf den rechten Motor.
            steering_out = (self.kp_heading * heading_error) + \
                           (self.kd_heading * d_heading_error) + \
                           (self.kp_track * cross_track_error)

            cmd.linear.x = self.linear_speed
            cmd.angular.z = float(np.clip(steering_out, -1.5, 1.5))
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self._last_heading_error = 0.0

        # Ausgabebefehl an die echten Motoren publishen
        self.cmd_pub.publish(cmd)

if __name__ == '__main__':
    try:
        node = EKFNoeticNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass