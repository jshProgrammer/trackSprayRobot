#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import pigpio

# =========================
# GPIO CONFIG
# =========================
PIN_RIGHT = 13   # GPIO13 = PIN 33

# =========================
# ESC SETTINGS
# =========================
PWM_STOP    = 1500
PWM_MAX_FWD = 2000
PWM_MAX_BWD = 1000

pi = pigpio.pi()

if not pi.connected:
    raise RuntimeError("pigpio daemon not active! -> sudo pigpiod")


def set_motor_right(speed: float):
    pulse = int(1000000 * speed)

    pi.hardware_PWM(PIN_RIGHT, 10000, pulse)


def callback(msg: Twist):
    linear = msg.linear.x
    angular = msg.angular.z

    # Differential drive
    right_speed = linear - angular

    rospy.loginfo(f"Right speed: {right_speed:.2f}")

    set_motor_right(right_speed)


def shutdown():
    rospy.loginfo("Stopping motors...")

    # neutral signal
    pi.set_servo_pulsewidth(PIN_RIGHT, PWM_STOP)

    # PWM off
    pi.set_servo_pulsewidth(PIN_RIGHT, 0)

    pi.stop()


# =========================
# ROS NODE
# =========================
rospy.init_node("motor_driver")
rospy.Subscriber("/cmd_vel", Twist, callback, queue_size=10)
rospy.on_shutdown(shutdown)

rospy.loginfo("pigpio motor driver started")
rospy.spin()