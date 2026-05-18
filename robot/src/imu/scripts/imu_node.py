#!/usr/bin/env python3

import rospy
import math
import tf.transformations as tft
from sensor_msgs.msg import Imu

from mpu9250_jmdev.mpu_9250 import MPU9250
from mpu9250_jmdev.registers import (
    AK8963_ADDRESS,
    MPU9050_ADDRESS_68,
    GFS_250, GFS_500, GFS_1000, GFS_2000,
    AFS_2G, AFS_4G, AFS_8G, AFS_16G,
    AK8963_BIT_16,
    AK8963_MODE_C8HZ
)


class MPU9250Node:

    GFS_RANGES = {250: GFS_250, 500: GFS_500, 1000: GFS_1000, 2000: GFS_2000}
    AFS_RANGES = {2: AFS_2G, 4: AFS_4G, 8: AFS_8G, 16: AFS_16G}

    GYRO_SENSITIVITY = {250: 131.0, 500: 65.5, 1000: 32.8, 2000: 16.4}

    def __init__(self):
        rospy.init_node("mpu9250_node")

        self.rate_hz = rospy.get_param("~rate", 100)
        self.gyro_fsr = rospy.get_param("~gyro_fsr", 250)
        self.accel_fsr = rospy.get_param("~accel_fsr", 2)

        self.frame_id = rospy.get_param("~frame_id", "imu_link")

        # ---------------- SENSOR INIT ----------------
        self.mpu = MPU9250(
            address_ak=AK8963_ADDRESS,
            address_mpu_master=MPU9050_ADDRESS_68,
            bus=1,
            gfs=self.GFS_RANGES[self.gyro_fsr],
            afs=self.AFS_RANGES[self.accel_fsr],
            mfs=AK8963_BIT_16,
            mode=AK8963_MODE_C8HZ
        )

        self.mpu.configureMPU6500(
            self.GFS_RANGES[self.gyro_fsr],
            self.AFS_RANGES[self.accel_fsr]
        )

        self.pub = rospy.Publisher("/imu/data", Imu, queue_size=10)

        # ---------------- YAW STATE ----------------
        self.yaw = 0.0
        self.last_time = rospy.Time.now()

        # gyro bias calibration
        self.bias_samples = []
        self.bias_z = 0.0
        self.bias_ready = False

    def run(self):
        rate = rospy.Rate(self.rate_hz)

        while not rospy.is_shutdown():

            now = rospy.Time.now()
            dt = (now - self.last_time).to_sec()
            self.last_time = now

            ax, ay, az = self.mpu.readAccelerometerMaster()
            gx, gy, gz = self.mpu.readGyroscopeMaster()

            # -------- gyro scaling (deg/s → rad/s) --------
            gz = (gz / self.GYRO_SENSITIVITY[self.gyro_fsr]) * (math.pi / 180.0)

            # -------- bias calibration (first ~100 samples) --------
            if not self.bias_ready:
                self.bias_samples.append(gz)
                if len(self.bias_samples) > 100:
                    self.bias_z = sum(self.bias_samples) / len(self.bias_samples)
                    self.bias_ready = True
                    rospy.loginfo(f"[IMU] Gyro bias Z calibrated: {self.bias_z:.6f}")
                rate.sleep()
                continue

            gz -= self.bias_z

            # -------- yaw integration --------
            self.yaw += gz * dt

            # optional wrap [-pi, pi]
            self.yaw = (self.yaw + math.pi) % (2 * math.pi) - math.pi

            # -------- quaternion --------
            q = tft.quaternion_from_euler(0, 0, self.yaw)

            # -------- IMU message --------
            msg = Imu()
            msg.header.stamp = now
            msg.header.frame_id = self.frame_id

            msg.orientation.x = q[0]
            msg.orientation.y = q[1]
            msg.orientation.z = q[2]
            msg.orientation.w = q[3]

            msg.angular_velocity.z = gz

            # covariance (simple but valid)
            msg.orientation_covariance = [
                0.05, 0, 0,
                0, 0.05, 0,
                0, 0, 0.2
            ]

            msg.angular_velocity_covariance = [
                0.02, 0, 0,
                0, 0.02, 0,
                0, 0, 0.02
            ]

            self.pub.publish(msg)

            rate.sleep()


if __name__ == "__main__":
    node = MPU9250Node()
    node.run()