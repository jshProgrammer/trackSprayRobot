#!/usr/bin/env python3
"""
EKF mit adaptivem RTK-GPS (Fixed / Float / No Fix)
+ IMU (Yaw via Gyroskop-Integration) + Encoder (Odometrie)
Für ROS Noetic

Fixes:
  - IMU-Montagerichtung konfigurierbar via ROS-Parameter
  - last_gyro_time und yaw_integrated korrekt initialisiert
  - Gyro-Bias-Schätzung beim Start (Stillstand)
  - Achsenauswahl: x / y / z, mit optionalem Vorzeichen-Flip
  - Sanity-Check: warnt wenn falsche Achse gewählt scheint

Ansteuerung: 
- angular.z = 0 -> Fährt wie auf Schienen starr geradeaus (für unebene Böden).
- angular.z != 0 -> Roboter lenkt dynamisch, Führungsschiene dreht sich mit.

Condition: pip3 install transforms3d
"""

import rospy
import numpy as np
import math
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import Twist
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

        # ── Zustandsvektor: [x, y, θ, v] ──────────────────────────────
        self.x = np.zeros((4, 1))

        # P = Kovarianzmatrix der Schätzung (Initialisierung mit großer Unsicherheit)
        self.P = np.diag([1.0, 1.0, 0.1, 0.5])

        # ── Prozessrauschen (Q) z.B. durch Wind, Schlupf ────────────────────────────────────────
        self.Q = np.diag([0.02, 0.02, 0.01, 0.02])

        # ── Messrauschen IMU ───────────────────────────────────────────
        self.R_imu = np.array([[0.005]])

        # ── Adaptives Messrauschen GPS ─────────────────────────────────
        self.R_gps = {
            RTKStatus.FIXED: np.diag([0.0004, 0.0004]),  # σ ≈ 2 cm
            RTKStatus.FLOAT: np.diag([0.09,   0.09  ]),  # σ ≈ 30 cm
        }

        # ── Messmatrizen (H) ───────────────────────────────────────────
        self.H_imu = np.array([[0, 0, 1, 0]])   # misst θ
        self.H_gps = np.array([[1, 0, 0, 0],
                               [0, 1, 0, 0]])    # misst x, y

        # ── IMU Montage-Konfiguration ──────────────────────────────────
        #
        # imu_yaw_axis:  Welche IMU-Achse entspricht der Roboter-Yaw-Rotation?
        #   'z'  → IMU flach montiert (Standardfall, Z zeigt nach oben)
        #   'x'  → IMU hochkant, X zeigt nach oben
        #   'y'  → IMU hochkant, Y zeigt nach oben
        #
        # imu_yaw_sign:  +1.0 oder -1.0
        #   Wenn sich der EKF-Yaw falsch herum dreht: auf -1.0 setzen
        #
        # Kurzanleitung zur Achsenfindung:
        #   1. rostopic echo /imu/data (angular_velocity)
        #   2. Roboter von Hand gegen Uhrzeigersinn drehen (= +Yaw laut REP-103)
        #   3. Welche Achse zeigt die größte positive Rate? Das ist imu_yaw_axis.
        #   4. Wenn die Rate negativ war: imu_yaw_sign auf -1.0 setzen.
        #
        self.imu_yaw_axis = rospy.get_param('~imu_yaw_axis', 'z')   # 'x' | 'y' | 'z'
        self.imu_yaw_sign = rospy.get_param('~imu_yaw_sign', 1.0)   # +1.0 | -1.0

        if self.imu_yaw_axis not in ('x', 'y', 'z'):
            rospy.logwarn(f"imu_yaw_axis='{self.imu_yaw_axis}' ungültig. Fallback auf 'z'.")
            self.imu_yaw_axis = 'z'

        rospy.loginfo(f"IMU Yaw-Achse: angular_velocity.{self.imu_yaw_axis}  "
                      f"Vorzeichen: {'+' if self.imu_yaw_sign > 0 else '-'}1")

        # ── Gyro-Bias-Schätzung (Drift-Kompensation) ──────────────────
        # Während der Roboter beim Start still steht, mitteln wir die
        # Gyro-Werte. Das ist der Bias, den wir dann abziehen.
        self._bias_samples      = []
        self._bias              = 0.0
        self._bias_window       = rospy.get_param('~bias_samples', 50)  # ~2.5 s bei 20 Hz
        self._bias_done         = False

        # ── Yaw-Integration ───────────────────────────────────────────
        self.yaw_integrated  = 0.0
        self.last_gyro_time  = rospy.Time.now()

        # ── Sanity-Check Schwelle ──────────────────────────────────────
        # Wenn die gewählte Achse beim Drehen weniger als 10% der
        # maximalen Rate liefert, warnen wir.
        self._sanity_max_rate = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self._sanity_done     = False
        self._sanity_count    = 0

        # ── Regler Parameter ───────────────────────────────────────────
        self.target_heading      = 0.0
        self.linear_speed        = 0.0
        self.kp                  = 2.5
        self.kd                  = 0.4
        self._last_heading_error = 0.0
        self.is_moving           = False

        # ── Zeitverwaltung ─────────────────────────────────────────────
        self.last_time = rospy.Time.now()

        # ── ROS Subscriber ─────────────────────────────────────────────
        rospy.Subscriber('/imu/data',          Imu,       self.imu_callback)
        rospy.Subscriber('/gps/fix',           NavSatFix, self.gps_callback)
        rospy.Subscriber('/cmd_vel_controll',  Twist,     self.cmd_vel_callback)
        # rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # ── ROS Publisher ──────────────────────────────────────────────
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # ── Regler-Timer (20 Hz) ───────────────────────────────────────
        rospy.Timer(rospy.Duration(0.05), self.control_loop)

        rospy.loginfo("EKF RTK Node gestartet")
        rospy.loginfo("Bitte Roboter für ~3 s still stehen lassen (Gyro-Bias-Kalibrierung)...")

    # ══════════════════════════════════════════════════════════════════
    # HILFSFUNKTIONEN
    # ══════════════════════════════════════════════════════════════════

    def _get_yaw_rate(self, msg):
        """Liest die konfigurierte Achse aus der IMU und wendet das Vorzeichen an."""
        rates = {
            'x': msg.angular_velocity.x,
            'y': msg.angular_velocity.y,
            'z': msg.angular_velocity.z,
        }
        return self.imu_yaw_sign * rates[self.imu_yaw_axis]

    def _parse_rtk_status(self, msg):
        status = msg.status.status
        cov    = msg.position_covariance[0]
        if status < 0:    return RTKStatus.NO_FIX
        if cov <= 0.001:  return RTKStatus.FIXED
        elif cov <= 0.25: return RTKStatus.FLOAT
        else:             return RTKStatus.NO_FIX

    @staticmethod
    def _wrap(a):
        return (a + math.pi) % (2 * math.pi) - math.pi

    def _latlon_to_meters(self, lat, lon):
        R    = 6371000.0
        dlat = math.radians(lat - self._gps_origin_lat)
        dlon = math.radians(lon - self._gps_origin_lon)
        ref  = math.radians(self._gps_origin_lat)
        return dlon * R * math.cos(ref), dlat * R

    # ══════════════════════════════════════════════════════════════════
    # SENSOR CALLBACKS
    # ══════════════════════════════════════════════════════════════════

    def cmd_vel_callback(self, msg):
        self.linear_speed   = msg.linear.x
        self.target_heading = msg.angular.z if msg.angular.z != 0 else self.x[2, 0]
        self.is_moving      = abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01

    
    """
    def imu_callback(self, msg):
        now = rospy.Time.now()
        dt  = (now - self.last_gyro_time).to_sec()
        self.last_gyro_time = now

        raw_rate = self._get_yaw_rate(msg)

        # ── Sanity-Check: maximale Raten aller Achsen aufzeichnen ──────
        if not self._sanity_done:
            self._sanity_max_rate['x'] = max(self._sanity_max_rate['x'],
                                              abs(msg.angular_velocity.x))
            self._sanity_max_rate['y'] = max(self._sanity_max_rate['y'],
                                              abs(msg.angular_velocity.y))
            self._sanity_max_rate['z'] = max(self._sanity_max_rate['z'],
                                              abs(msg.angular_velocity.z))
            self._sanity_count += 1
            if self._sanity_count >= 200:   # nach ~10 s
                self._sanity_done = True
                chosen = self._sanity_max_rate[self.imu_yaw_axis]
                best   = max(self._sanity_max_rate.values())
                if best > 0.05 and chosen < 0.1 * best:
                    best_axis = max(self._sanity_max_rate, key=self._sanity_max_rate.get)
                    rospy.logwarn(
                        f"[IMU Sanity] Die gewählte Achse '{self.imu_yaw_axis}' "
                        f"hatte nur {chosen:.3f} rad/s max, "
                        f"aber Achse '{best_axis}' hatte {best:.3f} rad/s. "
                        f"Bitte imu_yaw_axis auf '{best_axis}' setzen!")
                else:
                    rospy.loginfo(
                        f"[IMU Sanity] Achse '{self.imu_yaw_axis}' sieht korrekt aus "
                        f"(max {chosen:.3f} rad/s).")

        # ── Gyro-Bias-Kalibrierung (erster Stillstand) ─────────────────
        if not self._bias_done:
            self._bias_samples.append(raw_rate)
            if len(self._bias_samples) >= self._bias_window:
                self._bias      = float(np.mean(self._bias_samples))
                self._bias_done = True
                rospy.loginfo(f"Gyro-Bias kalibriert: {self._bias:.6f} rad/s "
                              f"(Achse: {self.imu_yaw_axis})")
            return   # noch keine EKF-Updates während Kalibrierung

        # ── Normale Yaw-Integration ────────────────────────────────────
        if 0 < dt < 0.1:
            yaw_rate             = raw_rate - self._bias
            self.yaw_integrated += yaw_rate * dt
            self.yaw_integrated  = self._wrap(self.yaw_integrated)

            self._ekf_update(
                z        = np.array([[self.yaw_integrated]]),
                H        = self.H_imu,
                R        = self.R_imu,
                wrap_idx = 0
            )
    """

    def imu_callback(self, msg):
        now = rospy.Time.now()
        dt  = (now - self.last_time).to_sec()
        self.last_time = now

        if not (0 < dt < 0.5):
            return

        # Wir extrahieren die saubere Drehrate (Z-Achse) direkt aus der Nachricht
        # Das MPU9250-Skript liefert diese bereits im korrekten ROS-Standard (rad/s)
        w = msg.angular_velocity.z
        
        # Die Lineargeschwindigkeit v kommt aus dem cmd_vel-Callback (Odometrie-Ersatz)
        v = self.linear_speed

        # ── MATHEMATISCH KORREKT: IMU treibt die Prädiktion (Vorhersage) an ──
        self._ekf_predict(v, w, dt)

    def gps_callback(self, msg):
        rtk              = self._parse_rtk_status(msg)
        self._rtk_status = rtk

        if rtk == RTKStatus.NO_FIX:
            return

        if self._gps_origin_lat is None:
            if rtk == RTKStatus.FIXED:
                self._gps_origin_lat = msg.latitude
                self._gps_origin_lon = msg.longitude
                rospy.loginfo(f"RTK-Ursprung gesetzt: {msg.latitude:.7f}, {msg.longitude:.7f}")
            return

        gx, gy = self._latlon_to_meters(msg.latitude, msg.longitude)
        self._ekf_update(
            z        = np.array([[gx], [gy]]),
            H        = self.H_gps,
            R        = self.R_gps[rtk],
            wrap_idx = None
        )

    # ══════════════════════════════════════════════════════════════════
    # EKF KERN LOGIK
    # ══════════════════════════════════════════════════════════════════

    def _ekf_predict(self, v, w, dt):
        px, py, th, _ = self.x.flatten()

        # Dead Reckoning: Hochrechnen der neuen Position basierend auf der aktuellen Geschwindigkeit und Drehrate
        self.x = np.array([
            [px + v * math.cos(th) * dt],
            [py + v * math.sin(th) * dt],
            [self._wrap(th + w * dt)   ],
            [v                         ],
        ])

        # Jakobi Matrix
        F = np.array([
            [1, 0, -v * math.sin(th) * dt, math.cos(th) * dt],
            [0, 1,  v * math.cos(th) * dt, math.sin(th) * dt],
            [0, 0,  1,                     0                 ],
            [0, 0,  0,                     1                 ],
        ])
        self.P = F @ self.P @ F.T + self.Q

    def _ekf_update(self, z, H, R, wrap_idx=None):
        # Differenz zwischen gemessener und erwarteter Messung
        innovation = z - H @ self.x
        if wrap_idx is not None:
            innovation[wrap_idx, 0] = self._wrap(innovation[wrap_idx, 0])

        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x      = self.x + K @ innovation
        self.x[2,0] = self._wrap(self.x[2, 0])
        self.P      = (np.eye(4) - K @ H) @ self.P

    # ══════════════════════════════════════════════════════════════════
    # REGLER
    # ══════════════════════════════════════════════════════════════════

    def control_loop(self, event):
        cmd = Twist()

        """
        if self.is_moving:
            heading_error = self._wrap(self.target_heading - self.x[2, 0])
            d_error       = (heading_error - self._last_heading_error) / 0.05
            self._last_heading_error = heading_error

            cmd.linear.x  = self.linear_speed
            cmd.angular.z = float(np.clip(
                self.kp * heading_error + self.kd * d_error, -1.5, 1.5))
        else:
            cmd.linear.x             = 0.0
            cmd.angular.z            = 0.0
            self._last_heading_error = 0.0

        self.cmd_pub.publish(cmd)
        """

        cmd = Twist()
        if self.is_moving:
            current_x   = self.x[0, 0]
            current_y   = self.x[1, 0]
            current_yaw = self.x[2, 0]

            # MODUS A: Der Benutzer lenkt aktiv von Hand (Kurve)
            if abs(self.commanded_angular_speed) > 0.01:
                cmd.linear.x = self.linear_speed
                cmd.angular.z = self.commanded_angular_speed
                
                # Wir verschieben den Spur-Startpunkt und Kurs fließend mit der Bewegung,
                # damit der Stanley-Regler nach der Kurve weich auf der neuen Richtung aufsetzt.
                self.line_start_x = current_x
                self.line_start_y = current_y
                self.target_heading = current_yaw
                self._last_heading_error = 0.0
                self._steering_active = True

            # MODUS B: Es soll nur die Spur gehalten werden (Geradeaus im Matsch)
            else:
                # Falls wir gerade aus einer Kurve kommen, die Spur jetzt final fixieren
                if getattr(self, '_steering_active', False):
                    self.line_start_x = current_x
                    self.line_start_y = current_y
                    self.target_heading = current_yaw
                    self._steering_active = False

                # Klassischer Stanley-Spurregler
                heading_error = self._wrap(self.target_heading - current_yaw)
                d_heading_error = (heading_error - self._last_heading_error) / dt
                self._last_heading_error = heading_error

                dx = current_x - self.line_start_x
                dy = current_y - self.line_start_y
                cross_track_error = math.sin(self.target_heading) * dx - math.cos(self.target_heading) * dy

                steering_out = (self.kp_heading * heading_error) + \
                               (self.kd_heading * d_heading_error) + \
                               (self.kp_track * cross_track_error)

                cmd.linear.x = self.linear_speed
                cmd.angular.z = float(np.clip(steering_out, -1.5, 1.5))
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self._last_heading_error = 0.0
            self._steering_active = False

        self.cmd_pub.publish(cmd)


if __name__ == '__main__':
    try:
        node = EKFNoeticNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass