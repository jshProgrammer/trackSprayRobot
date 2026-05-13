#!/usr/bin/env python3
"""
EKF mit adaptivem RTK-GPS (Fixed / Float / No Fix)
+ IMU (Yaw) + Encoder (Odometrie)
Für ROS Noetic

Condition: run => pip3 install transforms3d
"""

import rospy
import numpy as np
import math
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

#TODO: find out coordinate system
#TODO: add odometry

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

        # ── State: [x, y, θ, v] ───────────────────────────────────────
        self.x = np.zeros((4, 1))
        self.P = np.diag([1.0, 1.0, 0.1, 0.5])

        # ── Prozessrauschen (Q) ────────────────────────────────────────
        # Etwas höher angesetzt, falls keine Encoder-Daten kommen
        self.Q = np.diag([0.02, 0.02, 0.01, 0.02])

        # ── Messrauschen IMU (R) ───────────────────────────────────────
        self.R_imu = np.array([[0.005]]) 

        # ── Adaptives Messrauschen GPS (R) ─────────────────────────────
        self.R_gps = {
            RTKStatus.FIXED: np.diag([0.0004, 0.0004]),  # σ ≈ 2 cm
            RTKStatus.FLOAT: np.diag([0.09,   0.09  ]),  # σ ≈ 30 cm
        }

        # ── Messmatrizen (H) ───────────────────────────────────────────
        self.H_imu = np.array([[0, 0, 1, 0]])       # misst θ
        self.H_gps = np.array([[1, 0, 0, 0],
                               [0, 1, 0, 0]])        # misst x, y

        # ── Regler Parameter ───────────────────────────────────────────
        self.target_heading       = 0.0
        self.linear_speed         = 0.0  # Standardmäßig 0
        self.kp                   = 2.5
        self.kd                   = 0.4
        self._last_heading_error  = 0.0
        self.is_moving            = False  # Flag für Bewegungsstatus

        # ── Zeitverwaltung ───────────────��─────────────────────────────
        self.last_time = rospy.Time.now()

        # ── ROS 1 Subscriber ───────────────────────────────────────────
        rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        #rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/gps/fix', NavSatFix, self.gps_callback)
        rospy.Subscriber('/cmd_vel_controll', Twist, self.cmd_vel_callback)  # CHANGED: Subscribe to cmd_vel

        # ── ROS 1 Publisher ────────────────────────────────────────────
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # ── Timer für Regler (20 Hz) ───────────────────────────────────
        rospy.Timer(rospy.Duration(0.05), self.control_loop)

        rospy.loginfo("EKF RTK Node für Noetic gestartet")

    # ══════════════════════════════════════════════════════════════════
    # HILFSFUNKTIONEN
    # ══════════════════════════════════════════════════════════════════

    def _parse_rtk_status(self, msg):
        status = msg.status.status
        cov    = msg.position_covariance[0]
        if status < 0: return RTKStatus.NO_FIX
        if cov <= 0.001: return RTKStatus.FIXED
        elif cov <= 0.25: return RTKStatus.FLOAT
        else: return RTKStatus.NO_FIX

    @staticmethod
    def _wrap(a):
        return (a + math.pi) % (2 * math.pi) - math.pi

    def _latlon_to_meters(self, lat, lon):
        R = 6371000.0
        dlat = math.radians(lat - self._gps_origin_lat)
        dlon = math.radians(lon - self._gps_origin_lon)
        ref  = math.radians(self._gps_origin_lat)
        return dlon * R * math.cos(ref), dlat * R

    # ══════════════════════════════════════════════════════════════════
    # SENSOR CALLBACKS
    # ══════════════════════════════════════════════════════════════════

    def cmd_vel_callback(self, msg):
        """CHANGED: Externe Bewegungsbefehle empfangen"""
        self.linear_speed = msg.linear.x
        self.target_heading = msg.angular.z if msg.angular.z != 0 else self.x[2, 0]
        self.is_moving = abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01

    """
    def imu_callback(self, msg):
        q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        _, _, yaw = euler_from_quaternion(q)
        
        # Falls keine Odometrie da ist, kann man hier auch den Predict triggern
        # Aber wir bleiben beim Standard-Update für die Ausrichtung
        self._ekf_update(
            z=np.array([[yaw]]),
            H=self.H_imu,
            R=self.R_imu,
            wrap_idx=0
        )
    """
    
    def imu_callback(self, msg):
        now = rospy.Time.now()
        dt = (now - self.last_gyro_time).to_sec()
        self.last_gyro_time = now
        
        if 0 < dt < 0.1:  # Sanity check
            # Gyro-Daten: angular_velocity.z (rad/s)
            yaw_rate = msg.angular_velocity.z
            
            # Yaw durch Integration
            self.yaw_integrated += yaw_rate * dt
            self.yaw_integrated = self._wrap(self.yaw_integrated)
            
            # EKF Update mit integriertem Yaw
            self._ekf_update(
                z=np.array([[self.yaw_integrated]]),
                H=self.H_imu,
                R=self.R_imu,
                wrap_idx=0
            )

    """
    def odom_callback(self, msg):
        now = rospy.Time.now()
        dt = (now - self.last_time).to_sec()
        self.last_time = now
        
        if 0 < dt < 1.0:
            self._ekf_predict(
                v=msg.twist.twist.linear.x,
                w=msg.twist.twist.angular.z,
                dt=dt
            )
    """

    def gps_callback(self, msg):
        rtk = self._parse_rtk_status(msg)
        self._rtk_status = rtk

        if rtk == RTKStatus.NO_FIX: return

        # Ursprungsbestimmung
        if self._gps_origin_lat is None:
            if rtk == RTKStatus.FIXED:
                self._gps_origin_lat = msg.latitude
                self._gps_origin_lon = msg.longitude
                rospy.loginfo(f"RTK-Ursprung gesetzt: {msg.latitude}, {msg.longitude}")
            return

        gx, gy = self._latlon_to_meters(msg.latitude, msg.longitude)
        self._ekf_update(
            z=np.array([[gx], [gy]]),
            H=self.H_gps,
            R=self.R_gps[rtk],
            wrap_idx=None
        )

    # ══════════════════════════════════════════════════════════════════
    # EKF KERN LOGIK
    # ══════════════════════════════════════════════════════════════════

    def _ekf_predict(self, v, w, dt):
        px, py, th, _ = self.x.flatten()

        # Neuer Zustand
        self.x = np.array([
            [px + v * math.cos(th) * dt],
            [py + v * math.sin(th) * dt],
            [self._wrap(th + w * dt)   ],
            [v                         ],
        ])

        # Jacobi-Matrix F
        F = np.array([
            [1, 0, -v * math.sin(th) * dt,  math.cos(th) * dt],
            [0, 1,  v * math.cos(th) * dt,  math.sin(th) * dt],
            [0, 0,  1,                      0                ],
            [0, 0,  0,                      1                ],
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
    # REGLER
    # ══════════════════════════════════════════════════════════════════

    def control_loop(self, event):
        """CHANGED: Nur applizieren wenn Bewegung gewünscht ist"""
        cmd = Twist()
        
        if self.is_moving:
            # Aktueller Kursfehler
            heading_error = self._wrap(self.target_heading - self.x[2, 0])
            
            # D-Anteil (Änderungsrate des Fehlers)
            d_error = (heading_error - self._last_heading_error) / 0.05
            self._last_heading_error = heading_error

            cmd.linear.x  = self.linear_speed
            # PD-Regler für die Lenkung
            cmd.angular.z = float(np.clip(
                self.kp * heading_error + self.kd * d_error, -1.5, 1.5))
        else:
            # Stillstand: beide Werte auf 0
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self._last_heading_error = 0.0
        
        self.cmd_pub.publish(cmd)

if __name__ == '__main__':
    try:
        node = EKFNoeticNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass