#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, Int32MultiArray
import time
import threading
import pigpio
from robot_msgs.status import StatusReporter

class MotorDriver:
    def __init__(self):

        # Status-Reporter zuerst, damit auch frühe Fehler ans Frontend gehen.
        # (rospy.init_node läuft bereits im __main__ vor MotorDriver().)
        self.status = StatusReporter(source="pwm_drive")

        # =========================
        # PARAMETER LOAD
        # =========================
        self.pin_left  = rospy.get_param("/motor_driver/pin_left")
        self.pin_left_reverse = rospy.get_param("/motor_driver/pin_left_reverse")
        self.left_motor_reversed = rospy.get_param("/motor_driver/left_motor_reversed", True)
        self.pin_right = rospy.get_param("/motor_driver/pin_right")
        self.pin_right_reverse = rospy.get_param("/motor_driver/pin_right_reverse")
        self.right_motor_reversed = rospy.get_param("/motor_driver/right_motor_reversed", False)
        self.pin_spray = rospy.get_param("/motor_driver/pin_spray")
        self.encoder_right_pins = (
            rospy.get_param("/motor_driver/encoder_right_a"),
            rospy.get_param("/motor_driver/encoder_right_b"),
            rospy.get_param("/motor_driver/encoder_right_c"),
        )
        self.encoder_left_pins = (
            rospy.get_param("/motor_driver/encoder_left_a"),
            rospy.get_param("/motor_driver/encoder_left_b"),
            rospy.get_param("/motor_driver/encoder_left_c"),
        )
        self.hardware_pwm_gpios = set(rospy.get_param("/motor_driver/hardware_pwm_gpios"))

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
            self.status.report_fatal("MOTOR_PIGPIOD_DOWN",
                                     "pigpio-Daemon läuft nicht – Roboter kann nicht fahren")
            raise RuntimeError("pigpio daemon not running -> sudo pigpiod")
        
        self._stop_motors(reset_modes=True)
        self._setup_encoders()
        self.pi.set_servo_pulsewidth(self.pin_spray, 0)
        
        # =========================
        # ROS NODE
        # =========================
        self.encoder_latest = []
        self.encoder_lock = threading.Lock()
        self.encoder_callbacks = []
        self.encoder_pub = rospy.Publisher(
            "/encoder_states",
            Int32MultiArray,
            queue_size=1,
            latch=True,
        )
        self._publish_encoder_state(force=True)
        self._setup_encoder_callbacks()
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_callback, queue_size=1)
        rospy.Subscriber("/cmd_spray", Empty, self.spray_callback, queue_size=1)
        rospy.on_shutdown(self.shutdown)

        rospy.loginfo("Motor driver initialized")
        #rospy.spin()


    # =========================
    # MOTOR OUTPUT
    # =========================
    def _motor_pins(self):
        return (
            self.pin_left,
            self.pin_right,
        )

    def _reverse_pins(self):
        return (
            self.pin_left_reverse,
            self.pin_right_reverse,
        )

    def _write_pwm(self, gpio, speed: float):
        speed = max(0.0, min(1.0, speed))

        if gpio in self.hardware_pwm_gpios:
            pulse = int(1000000 * speed)
            self.pi.hardware_PWM(gpio, self.motor_frequency, pulse)
            return

        self.pi.set_mode(gpio, pigpio.OUTPUT)
        self.pi.set_PWM_frequency(gpio, self.motor_frequency)
        self.pi.set_PWM_range(gpio, 255)
        self.pi.set_PWM_dutycycle(gpio, int(255 * speed))

    def _stop_motors(self, reset_modes=False):
        for gpio in self._motor_pins():
            self._write_pwm(gpio, 0)
            if reset_modes:
                self.pi.set_mode(gpio, pigpio.OUTPUT)
                self.pi.write(gpio, 0)

        for gpio in self._reverse_pins():
            self.pi.set_mode(gpio, pigpio.OUTPUT)
            self.pi.write(gpio, 0)

    def _set_reverse_pin(self, gpio, enabled: bool):
        self.pi.set_mode(gpio, pigpio.OUTPUT)
        self.pi.write(gpio, 1 if enabled else 0)

    def _set_bidirectional_motor(self, pwm_pin, reverse_pin, speed: float, reversed_motor: bool):
        duty = abs(speed)

        reverse_enabled = speed < 0
        if reversed_motor:
            reverse_enabled = not reverse_enabled

        self._set_reverse_pin(reverse_pin, reverse_enabled)

        if duty < 0.001:
            self._write_pwm(pwm_pin, 0)
            return

        self._write_pwm(pwm_pin, duty)

    def _encoder_pins(self):
        return self.encoder_right_pins + self.encoder_left_pins

    def _setup_encoders(self):
        for gpio in self._encoder_pins():
            self.pi.set_mode(gpio, pigpio.INPUT)
            self.pi.set_pull_up_down(gpio, pigpio.PUD_OFF)

    def _read_encoders(self):
        right = [self.pi.read(gpio) for gpio in self.encoder_right_pins]
        left = [self.pi.read(gpio) for gpio in self.encoder_left_pins]
        return right, left

    def _setup_encoder_callbacks(self):
        for gpio in self._encoder_pins():
            self.encoder_callbacks.append(
                self.pi.callback(gpio, pigpio.EITHER_EDGE, self.encoder_edge_callback)
            )

    def _cancel_encoder_callbacks(self):
        for callback in self.encoder_callbacks:
            callback.cancel()
        self.encoder_callbacks = []

    def _publish_encoder_state(self, force=False):
        right, left = self._read_encoders()
        encoder_state = right + left

        with self.encoder_lock:
            if not force and encoder_state == self.encoder_latest:
                return
            self.encoder_latest = encoder_state

        msg = Int32MultiArray()
        msg.data = encoder_state
        self.encoder_pub.publish(msg)
        rospy.loginfo_throttle(
            1,
            "ENCODER right=[%d,%d,%d] left=[%d,%d,%d]",
            right[0], right[1], right[2],
            left[0], left[1], left[2],
        )

    def encoder_edge_callback(self, gpio, level, tick):
        if level == pigpio.TIMEOUT:
            return
        self._publish_encoder_state()

    def set_motor_left(self, speed: float):
        self._set_bidirectional_motor(
            self.pin_left,
            self.pin_left_reverse,
            speed,
            self.left_motor_reversed,
        )

    def set_motor_right(self, speed: float):
        self._set_bidirectional_motor(
            self.pin_right,
            self.pin_right_reverse,
            speed,
            self.right_motor_reversed,
        )

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
        # 1. Mathematisch saubere Skalierung
        linear  = max(min(msg.linear.x  / self.max_linear,  1.0), 0.0)
        angular = max(min(msg.angular.z / self.max_angular,  1.0), -1.0)

        # 2. Saubere Kurvenberechnung (Verhältnis bleibt intakt!)
        left_speed  = linear - (angular * self.wheel_base / 2)
        right_speed = linear + (angular * self.wheel_base / 2)

        """
        rospy.loginfo_throttle(
            1,
            f"PWM_DRIVE linear={linear} "
            f"PWM_DRIVE angular={angular} "
            f"left_speed_before={left_speed}"
            f"right_speed_before={right_speed}"
        )
        """

        # 3. DIE SICHERHEITS-LEINE (Hardware-Deckel)
        # Anstatt 1.0 (100% Vollgas) begrenzen wir die Motoren hier hart auf z.B. 0.3 (30% Leistung).
        # Er kann jetzt physisch nicht schneller als 30% fahren, egal was das Gehirn befiehlt!
        MAX_SAFE_PWM = 0.15 # <-- Hier könnt ihr auf dem Feld hochgehen, wenn er gut fährt (z.B. 0.5)

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

    # =========================
    # SHUTDOWN
    # =========================
    def shutdown(self):
        rospy.loginfo("Stopping motors...")

        # neutral signal
        #self.pi.set_servo_pulsewidth(self.pin_left, self.pwm_stop)
        #self.pi.set_servo_pulsewidth(self.pin_right, self.pwm_stop)

        try:
            self._cancel_encoder_callbacks()
            self._stop_motors(reset_modes=True)
            self.pi.set_servo_pulsewidth(self.pin_spray, 0)
        finally:
            self.pi.stop()

if __name__ == "__main__":
    rospy.init_node("motor_driver")
    MotorDriver()
    rospy.spin()
