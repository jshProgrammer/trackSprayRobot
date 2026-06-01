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
        self.pin_left  = rospy.get_param("/motor_driver/pin_left")
        self.pin_right = rospy.get_param("/motor_driver/pin_right")
        self.pin_spray = rospy.get_param("/motor_driver/pin_spray")

        self.pwm_stop    = rospy.get_param("/motor_driver/pwm_stop")
        self.pwm_max_fwd = rospy.get_param("/motor_driver/pwm_max_fwd")
        self.pwm_max_bwd = rospy.get_param("/motor_driver/pwm_max_bwd")

        self.motor_frequency = rospy.get_param("/motor_driver/motor_frequency")
        self.wheel_base = rospy.get_param("/motor_driver/wheel_base")

        self.max_linear  = rospy.get_param("/motor_driver/max_linear")
        self.max_angular = rospy.get_param("/motor_driver/max_angular")


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
    
    """
    def cmd_callback(self, msg: Twist):
        linear = max(min(msg.linear.x / self.max_linear, self.max_linear), 0)
        angular = max(min(msg.angular.z / self.max_angular, self.max_angular), 
                    -self.max_angular)

        left_speed = linear - (angular * self.wheel_base / 2)
        right_speed = linear + (angular * self.wheel_base / 2)

        left_speed  = max(0.0, min(1.0, left_speed))
        right_speed = max(0.0, min(1.0, right_speed))

        rospy.logdebug(f"Left speed: {left_speed:.2f}, Right speed: {right_speed:.2f}")

        #TODO: remove hardcoded scale factor
        self.set_motor_left(left_speed)
        self.set_motor_right(right_speed)
    """

    def cmd_callback(self, msg: Twist):
        # 1. Mathematisch saubere Skalierung
        linear  = max(min(msg.linear.x  / self.max_linear,  1.0), 0.0)
        angular = max(min(msg.angular.z / self.max_angular,  1.0), -1.0)

        # 2. Saubere Kurvenberechnung (Verhältnis bleibt intakt!)
        left_speed  = linear - (angular * self.wheel_base / 2)
        right_speed = linear + (angular * self.wheel_base / 2)

        rospy.loginfo_throttle(
            1,
            f"PWM_DRIVE linear={linear} "
            f"PWM_DRIVE angular={angular} "
            f"left_speed_before={left_speed}"
            f"right_speed_before={right_speed}"
        )

        # 3. DIE SICHERHEITS-LEINE (Hardware-Deckel)
        # Anstatt 1.0 (100% Vollgas) begrenzen wir die Motoren hier hart auf z.B. 0.3 (30% Leistung).
        # Er kann jetzt physisch nicht schneller als 30% fahren, egal was das Gehirn befiehlt!
        MAX_SAFE_PWM = 0.15  # <-- Hier könnt ihr auf dem Feld hochgehen, wenn er gut fährt (z.B. 0.5)

        max_speed = max(abs(left_speed), abs(right_speed))

        if max_speed > MAX_SAFE_PWM:
            scale = MAX_SAFE_PWM / max_speed

            left_speed *= scale
            right_speed *= scale

        rospy.loginfo_throttle(
                1,
                f"left_speed_after={left_speed}"
                f"right_speed_after={right_speed}"
            )

        self.set_motor_left(left_speed)
        self.set_motor_right(right_speed)
    """

    def cmd_callback(self, msg: Twist):
        rospy.loginfo(f"CMD: linear={msg.linear.x:.3f}, angular={msg.angular.z:.3f}")

        linear  = max(min(msg.linear.x  / self.max_linear,  1.0), 0.0)
        angular = max(min(msg.angular.z / self.max_angular,  1.0), -1.0)

        left_speed  = linear - (angular * self.wheel_base / 2)
        right_speed = linear + (angular * self.wheel_base / 2)

        left_speed  = max(0.0, min(1.0, left_speed))
        right_speed = max(0.0, min(1.0, right_speed))

        rospy.loginfo(f"PWM: left={left_speed:.3f}, right={right_speed:.3f}")

        self.set_motor_left(left_speed)
        self.set_motor_right(right_speed)
    """

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