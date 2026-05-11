#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
import time
import pigpio

class MotorDriver:

    def __init__(self):

        # =========================
        # PARAMETER LOAD
        # =========================
        self.pin_left  = rospy.get_param("~pin_left")
        self.pin_right = rospy.get_param("~pin_right")
        self.pin_spray = rospy.get_param("~pin_spray")

        self.pwm_stop    = rospy.get_param("~pwm_stop")
        self.pwm_max_fwd = rospy.get_param("~pwm_max_fwd")
        self.pwm_max_bwd = rospy.get_param("~pwm_max_bwd")

        self.motor_frequency = rospy.get_param("~motor_frequency")
        self.wheel_base = rospy.get_param("~wheel_base")

        self.max_linear  = rospy.get_param("~max_linear")
        self.max_angular = rospy.get_param("~max_angular")


        # =========================
        # PIGPIO INIT
        # =========================
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("pigpio daemon not running -> sudo pigpiod")
        
        self.pi.hardware_PWM(self.pin_right, self.motor_frequency, 0)
        self.pi.hardware_PWM(self.pin_left, self.motor_frequency, 0)
        self.pi.set_servo_pulsewidth(self.pin_spray, 0)
        
        # =========================
        # ROS NODE
        # =========================
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_callback, queue_size=1)
        rospy.Subscriber("/cmd_spray", Empty, self.spray_callback, queue_size=1)
        rospy.on_shutdown(self.shutdown)

        rospy.loginfo("Motor driver initialized")
        #rospy.spin()


    # =========================
    # MOTOR OUTPUT
    # =========================
    def set_motor_left(self, speed: float):
        pulse = int(1000000 * speed)    # 1kHz - 20 kHz

        self.pi.hardware_PWM(self.pin_left, self.motor_frequency, pulse)


    def set_motor_right(self, speed: float):
        pulse = int(1000000 * speed)

        self.pi.hardware_PWM(self.pin_right, self.motor_frequency, pulse)

    def set_servo_angle(self, angle):
        angle = max(0, min(270, angle))
        pulsewidth = 500 + (angle * (2000.0/270.0))
        self.pi.set_servo_pulsewidth(self.pin_spray, int(pulsewidth))

    def spray(self):
        rospy.loginfo("Spray")
        self.set_servo_angle(150)
        time.sleep(0.15)
        self.set_servo_angle(120)
        time.sleep(0.15)
        self.set_servo_angle(150)
        time.sleep(0.7)
        self.pi.set_servo_pulsewidth(self.pin_spray, 0)

    def spray_callback(self, msg: Empty):
        self.spray()


    # =========================
    # CMD_VEL CALLBACK
    # =========================
    def cmd_callback(self, msg: Twist):
        linear = max(min(msg.linear.x / self.max_linear, self.max_linear), 0)
        angular = max(min(msg.angular.z / self.max_angular, self.max_angular), 
                    -self.max_angular)

        left_speed = linear - (angular * self.wheel_base / 2)
        right_speed = linear + (angular * self.wheel_base / 2)

        rospy.logdebug(f"Left speed: {left_speed:.2f}, Right speed: {right_speed:.2f}")

        self.set_motor_left(left_speed)
        self.set_motor_right(right_speed)


    # =========================
    # SHUTDOWN
    # =========================
    def shutdown(self):
        rospy.loginfo("Stopping motors...")

        # neutral signal
        #self.pi.set_servo_pulsewidth(self.pin_left, self.pwm_stop)
        #self.pi.set_servo_pulsewidth(self.pin_right, self.pwm_stop)

        # PWM off
        self.pi.hardware_PWM(self.pin_left, self.motor_frequency, 0)
        self.pi.hardware_PWM(self.pin_right, self.motor_frequency, 0)
        self.pi.set_servo_pulsewidth(self.pin_spray, 0)

        self.pi.stop()

if __name__ == "__main__":
    rospy.init_node("motor_driver")
    MotorDriver()
    rospy.spin()