#!/usr/bin/env python3
"""
2.5D EKF + Stanley-Spurführungsregler für ROS Noetic.
Optimiert für: Geradeausfahrt trotz Schlupf, Matsch und Bodenunebenheiten.
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
        self.gps_quality = 0 

        # ── EKF-Zustand: [x, y, θ, v] ──────────────────────────────────
        self.x = np.zeros((4, 1))
        self.P = np.diag([1.0, 1.0, 0.1, 0.5])

        self.Q = np.diag([0.06, 0.06, 0.02, 0.06])

        self.R_gps = {
            RTKStatus.FIXED: np.diag([0.0004, 0.0004]),  # RTK-Fixed (2 cm)
            RTKStatus.FLOAT: np.diag([0.09,   0.09  ]),  # RTK-Float (30 cm)
        }

        self.H_gps = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])

        # NEU: Zwischenspeicher für den letzten GPS-Punkt zur Richtungsschätzung
        self._last_gx = None
        self._last_gy = None

        # ── Regler & Spur-Parameter ───────────────────────────────────
        self.linear_speed = 0.0
        self.commanded_angular_speed = 0.0
        self.is_moving = False
        
        self.line_start_x = 0.0
        self.line_start_y = 0.0
        self.target_heading = 0.0

        self.kp_heading = 2.5   
        self.kd_heading = 0.4   
        self.kp_track = 2.0     
        
        self._last_heading_error = 0.0
        self.last_time = rospy.Time.now()

        # ── Sensordaten-Zwischenspeicher ───────────────────────────────
        self.imu_w = 0.0          
        self.antenna_height = 0.26 
        self.roll = 0.0
        self.pitch = 0.0

        # ── ROS Schnittstellen ──────────────────────────────────────────
        rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        rospy.Subscriber('/gps/fix', NavSatFix, self.gps_callback)
        rospy.Subscriber('/gps/quality', UInt8, self._gps_quality_callback)
        rospy.Subscriber('/cmd_vel_controll', Twist, self.cmd_vel_callback)

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        rospy.Timer(rospy.Duration(0.05), self.control_loop)
        rospy.loginfo("EKF-Spurführungsregler inklusive GPS-Heading-Update aktiv.")

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
        self.gps_quality = msg.data

    def _parse_rtk_status(self, msg):
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

        # GEÄNDERT: Wir rasten die Spur hier NICHT mehr blind ein, da das Heading 
        # im Stillstand ungenau sein kann. Das Einrasten passiert jetzt kontrolliert 
        # in der control_loop, sobald wir echte Fahrdaten haben.
        if self.is_moving and not was_moving:
            self._spur_anforderung = True 

    def imu_callback(self, msg: Imu):
        self.imu_w = msg.angular_velocity.z
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z

        self.pitch = math.atan2(-ax, math.sqrt(ay**2 + az**2))
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

        # Hebelarm-Kompensation für Unebenheiten
        gx = gx - self.antenna_height * math.sin(self.pitch)
        gy = gy + self.antenna_height * math.sin(self.roll)

        # ── NEU: HEADING AUS GPS-BEWEGUNG ABLEITEN (Course over Ground) ──
        if self._last_gx is not None and self.linear_speed > 0.1:
            dx = gx - self._last_gx
            dy = gy - self._last_gy
            dist = math.hypot(dx, dy)
            
            # Wenn die Bewegung größer als 3 cm ist, berechnen wir den Fahrtvektor
            if dist > 0.03:
                gps_yaw = math.atan2(dy, dx)
                
                # Erweitertes EKF-Update für X, Y UND Yaw
                z = np.array([[gx], [gy], [gps_yaw]])
                H_3d = np.array([
                    [1, 0, 0, 0],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0]
                ])
                
                # Heading-Messrauschen gewichten (sehr vertrauenswürdig bei RTK-Fixed)
                yaw_noise = 0.005 if rtk == RTKStatus.FIXED else 0.05
                R_3d = np.zeros((3, 3))
                R_3d[0:2, 0:2] = self.R_gps[rtk]
                R_3d[2, 2] = yaw_noise
                
                # Update ausführen (dein mathematisches wrap_idx greift hier perfekt bei Index 2)
                self._ekf_update(z=z, H=H_3d, R=R_3d, wrap_idx=2)
                
                self._last_gx = gx
                self._last_gy = gy
                return # Beendet den Callback, da Update bereits erledigt

        # Reines Positions-Update, falls der Roboter steht oder zu langsam ist
        self._ekf_update(z=np.array([[gx], [gy]]), H=self.H_gps, R=self.R_gps[rtk])
        
        self._last_gx = gx
        self._last_gy = gy

    # ══════════════════════════════════════════════════════════════════
    # FILTER MATH (Bleibt identisch, da voll funktionsfähig)
    # ══════════════════════════════════════════════════════════════════

    def _ekf_predict(self, v, w, dt):
        px, py, th, _ = self.x.flatten()
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

        # EKF prädizieren
        self._ekf_predict(v=self.linear_speed, w=self.imu_w, dt=dt)

        cmd = Twist()
        if self.is_moving:
            current_x   = self.x[0, 0]
            current_y   = self.x[1, 0]
            current_yaw = self.x[2, 0]

            # MODUS A: Der Benutzer lenkt aktiv von Hand (Kurve)
            if abs(self.commanded_angular_speed) > 0.01:
                cmd.linear.x = self.linear_speed
                cmd.angular.z = self.commanded_angular_speed
                
                self.line_start_x = current_x
                self.line_start_y = current_y
                self.target_heading = current_yaw
                self._last_heading_error = 0.0
                self._steering_active = True
                self._spur_anforderung = False

            # MODUS B: Es soll nur die Spur gehalten werden (Geradeaus im Matsch)
            else:
                # NEU: Spur erst einrasten, wenn wir uns wirklich bewegen UND der EKF 
                # durch das GPS-Heading-Update die Chance hatte, sich einzunorden.
                if getattr(self, '_spur_anforderung', False) or getattr(self, '_steering_active', False):
                    self.line_start_x = current_x
                    self.line_start_y = current_y
                    self.target_heading = current_yaw
                    self._steering_active = False
                    self._spur_anforderung = False
                    rospy.loginfo(f"[SPUR] Neu fixiert auf stabilisiertes Heading: {math.degrees(self.target_heading):.1f}°")

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
            self._spur_anforderung = False

        self.cmd_pub.publish(cmd)

if __name__ == '__main__':
    try:
        node = EKFNoeticNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass