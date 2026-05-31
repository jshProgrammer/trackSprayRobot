#!/usr/bin/env python3
"""
2.5D EKF + Stanley-Spurführungsregler für ROS Noetic.
Optimiert für: Geradeausfahrt trotz Schlupf, Matsch und Bodenunebenheiten.

Änderungen:
  - Mathematischen Fehler im imu_callback behoben (kein falsches Yaw-Update mehr).
  - IMU dient jetzt rein als Sensor-Lieferant für Prädiktion und Hebelarm-Kompensation.
"""

import rospy
import numpy as np
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import UInt8
class RTKStatus:
    NO_FIX = 0
    FLOAT  = 1
    FIXED  = 2

class EKFNoeticNode:
    def __init__(self):
        rospy.init_node('ekf_rtk_node', anonymous=True)

        # ── GPS Ursprung & RTK Status
        self._gps_origin_lat = None
        self._gps_origin_lon = None
        self._rtk_status = RTKStatus.NO_FIX
        self.gps_quality = 0  # from /gps/quality topic from gps_node

        # ── EKF-Zustand: [x, y, θ, v] ──────────────────────────────────
        self.x = np.zeros((4, 1))
        self.P = np.diag([1.0, 1.0, 0.1, 0.5])

        # Q (Prozessrauschen) bei Schlupf höher angesetzt, damit der Filter
        # bei durchdrehenden Rädern primär den realen Messungen vertraut.
        self.Q = np.diag([0.06, 0.06, 0.02, 0.06])

        # Messrauschen für GPS (RTK-Zustände)
        self.R_gps = {
            RTKStatus.FIXED: np.diag([0.0004, 0.0004]),  # RTK-Fixed (2 cm)
            RTKStatus.FLOAT: np.diag([0.09,   0.09  ]),  # RTK-Float (30 cm)
        }

        # Da wir kein absolutes Yaw-Update mehr fahren, benötigen wir nur die GPS-Messmatrix
        self.H_gps = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])

        # ── Regler & Spur-Parameter ───────────────────────────────────
        self.linear_speed = 0.0
        self.commanded_angular_speed = 0.0
        self.is_moving = False
        
        # Startpunkt der virtuellen Führungsschiene
        self.line_start_x = 0.0
        self.line_start_y = 0.0
        self.target_heading = 0.0

        # Stanley-Regler-Gewinne
        self.kp_heading = 2.5   # Zieht die Fahrzeugnase schnell wieder geradeaus
        self.kd_heading = 0.4   # Dämpft die Lenkung, verhindert Schwingen des Hecks
        self.kp_track = 2.0     # Hartes Gegensteuern bei seitlichem Abdriften (Querablage)
        
        self._last_heading_error = 0.0
        self.last_time = rospy.Time.now()

        # ── Sensordaten-Zwischenspeicher ───────────────────────────────
        self.imu_w = 0.0          # Drehrate um Z-Achse (w)
        #TODO: probably change back
        self.antenna_height =1.2 #0.26 # GPS-Antennenhöhe in Metern
        self.roll = 0.0
        self.pitch = 0.0

        # ── ROS Schnittstellen ──────────────────────────────────────────
        rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        rospy.Subscriber('/gps/fix', NavSatFix, self.gps_callback)
        rospy.Subscriber('/gps/quality', UInt8, self._gps_quality_callback)
        rospy.Subscriber('/cmd_vel_controll', Twist, self.cmd_vel_callback)

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Regler- und Filtertaktung fest auf 20 Hz (0.05s)
        rospy.Timer(rospy.Duration(0.05), self.control_loop)
        rospy.loginfo("EKF-Spurführungsregler aktualisiert und aktiv.")

    @staticmethod
    def _wrap(a):
        return (a + math.pi) % (2 * math.pi) - math.pi

    def _latlon_to_meters(self, lat, lon):
        R = 6371000.0
        dlat = math.radians(lat - self._gps_origin_lat)
        dlon = math.radians(lon - self._gps_origin_lon)
        ref  = math.radians(self._gps_origin_lat)
        return dlon * R * math.cos(ref), dlat * R

    def _gps_quality_callback(self, msg: UInt8):
        """
        Receive RTK quality directly from gps_node (consistent with navigation.py).
        Quality mapping: 4=RTK FIXED (cm), 5=RTK FLOAT (dm), 2=DGPS, 1=GPS, else=NO_FIX
        """
        self.gps_quality = msg.data

    def _parse_rtk_status(self, msg):
        """
        Convert gps_quality to RTK status for EKF.
        No more magic thresholds or reverse-engineering from covariance!
        """
        if self.gps_quality == 4:
            return RTKStatus.FIXED
        elif self.gps_quality == 5:
            return RTKStatus.FLOAT
        else:
            return RTKStatus.NO_FIX

    # ══════════════════════════════════════════════════════════════════
    # CALLBACKS
    # ══════════════════════════════════════════════════════════════════

    def cmd_vel_callback(self, msg):
        self.linear_speed = msg.linear.x
        self.commanded_angular_speed = msg.angular.z

        was_moving = self.is_moving
        self.is_moving = abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01

        # Dynamisches Einrasten der Spur, sobald sich der Roboter in Bewegung setzt
        #if not self.is_moving and abs(msg.linear.x) > 0.01:
        if self.is_moving and (not was_moving or (msg.angular.z == 0 and not hasattr(self, '_last_was_steering'))):
            self.line_start_x = self.x[0, 0]
            self.line_start_y = self.x[1, 0]
            self.target_heading = self.x[2, 0] 
            #TODO: only add to log file
            #rospy.loginfo(f"Spur eingerastet! Kurs: {math.degrees(self.target_heading):.1f}°")

        #self.linear_speed = msg.linear.x
        #self.is_moving = abs(msg.linear.x) > 0.01
       

    def imu_callback(self, msg: Imu):
        """
        IMU-Callback ohne Magnetometer.

        Gyroskop Z  → imu_w  (für EKF-Prädiktion)
        Accelerometer → Roll & Pitch via Gravitations-Projektion
                        (für GPS-Antenne-Hebelarm-Kompensation)

        Die imu_node encodiert nur Yaw in der Quaternion (x=0, y=0),
        daher extrahieren wir Roll/Pitch aus echten Beschleunigungsdaten.
        """
        # Drehrate um Z-Achse (wird in _ekf_predict für Yaw-Integration genutzt)
        self.imu_w = msg.angular_velocity.z

        # Roll & Pitch aus Erdbeschleunigung schätzen.
        # Gültig solange keine starke Linearbeschleunigung vorhanden ist –
        # bei einem Feldroboter mit moderaten Geschwindigkeiten ausreichend genau.
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z

        # Pitch: Neigung nach vorne/hinten
        self.pitch = math.atan2(-ax, math.sqrt(ay**2 + az**2))
        # Roll: Neigung zur Seite
        self.roll  = math.atan2(ay, az)

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
        # Nickt (Pitch) oder wankt (Roll) das Chassis in Schlaglöchern, 
        # rechnen wir das virtuelle "Eiern" der hohen Antenne heraus.
        gx = gx - self.antenna_height * math.sin(self.pitch)
        gy = gy + self.antenna_height * math.sin(self.roll)

        # Reines Positions-Update via GPS
        self._ekf_update(z=np.array([[gx], [gy]]), H=self.H_gps, R=self.R_gps[rtk])

        # ── DEBUG ──────────────────────────────────────────────────────
        """rospy.loginfo_throttle(1.0,
            f"[GPS] roh=({gx + self.antenna_height * math.sin(self.pitch):.3f}, "
            f"{gy - self.antenna_height * math.sin(self.roll):.3f}) | "
            f"kompensiert=({gx:.3f}, {gy:.3f}) | "
            f"RTK={'FIXED' if self._rtk_status==2 else 'FLOAT'}"
        )"""
        # ───────────────────────────────────────────────────────────────

    # ══════════════════════════════════════════════════════════════════
    # FILTER MATH
    # ══════════════════════════════════════════════════════════════════

    def _ekf_predict(self, v, w, dt):
        px, py, th, _ = self.x.flatten()
        # Zustand mathematisch sauber fortschreiben anhand der echten IMU-Drehrate w
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
    # REGLER-SCHLEIFE (20 Hz Taktung)
    # ══════════════════════════════════════════════════════════════════

    def control_loop(self, event):
        now = rospy.Time.now()
        dt = (now - self.last_time).to_sec()
        self.last_time = now
        if dt <= 0: return

        # 1. EKF-Schritt: Zustand mit real vergangenem dt prädizieren
        self._ekf_predict(v=self.linear_speed, w=self.imu_w, dt=dt)
        """
        cmd = Twist()
        if self.is_moving:
            current_x   = self.x[0, 0]
            current_y   = self.x[1, 0]
            current_yaw = self.x[2, 0]

            # ── STANLEY REGELUNG ──
            # A: Winkelfehler berechnen (PD-Anteil für Kursstabilität)
            heading_error = self._wrap(self.target_heading - current_yaw)
            d_heading_error = (heading_error - self._last_heading_error) / dt
            self._last_heading_error = heading_error

            # B: Querablagefehler berechnen (Versetzt das Fahrzeug im Matsch parallel?)
            dx = current_x - self.line_start_x
            dy = current_y - self.line_start_y
            cross_track_error = math.sin(self.target_heading) * dx - math.cos(self.target_heading) * dy

            # C: Reglersignale fusionieren
            steering_out = (self.kp_heading * heading_error) + \
                           (self.kd_heading * d_heading_error) + \
                           (self.kp_track * cross_track_error)

            cmd.linear.x = self.linear_speed
            cmd.angular.z = float(np.clip(steering_out, -1.5, 1.5))
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self._last_heading_error = 0.0

        # Steuerbefehl raus an die Motor-Treiber senden
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

                # ── DEBUG ──────────────────────────────────────────────────────
                """rospy.loginfo_throttle(0.5,
                    f"[SPURREGLER] "
                    f"heading_err={math.degrees(heading_error):+.2f}° | "
                    f"cross_track={cross_track_error:+.3f}m | "
                    f"d_heading={math.degrees(d_heading_error):+.2f}°/s | "
                    f"steering_out={steering_out:+.3f} | "
                    f"imu_w={self.imu_w:+.4f}rad/s"
                )"""
                # ───────────────────────────────────────────────────────────────

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