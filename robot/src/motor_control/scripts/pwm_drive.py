#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Empty, Int32MultiArray
import time
import threading
import pigpio
from encoder_reader import EncoderReader
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
        try:
            self.encoder_reader = EncoderReader.from_ros_params(
                self.pi,
                "/motor_driver",
            )
        except (KeyError, ValueError) as exc:
            self.status.report_fatal("MOTOR_ENCODER_CONFIG_INVALID", str(exc))
            raise
        self.pi.set_servo_pulsewidth(self.pin_spray, 0)
        
        # =========================
        # ROS NODE
        # =========================
        self.encoder_latest = []
        self.encoder_lock = threading.Lock()
        self.encoder_pub = rospy.Publisher(
            "/encoder_states",
            Int32MultiArray,
            queue_size=1,
            latch=True,
        )
        self.encoder_twist_pub = rospy.Publisher(
            "/wheel/twist",
            TwistStamped,
            queue_size=1,
        )
        self.encoder_twist_timer = None
        self._publish_encoder_state(force=True)
        self.encoder_reader.start(self._handle_encoder_change)
        if self.encoder_reader.can_measure_velocity:
            self.encoder_twist_timer = rospy.Timer(
                rospy.Duration(1.0 / self.encoder_reader.velocity_publish_rate),
                self._publish_encoder_twist,
            )
        else:
            rospy.logwarn(
                "Encoder velocity disabled: set wheel_radius, "
                "encoder_ticks_per_motor_revolution and "
                "encoder_motor_to_wheel_ratio in motor.yaml"
            )
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

    def _handle_encoder_change(self, snapshot):
        self._publish_encoder_state(snapshot=snapshot)

    def _publish_encoder_state(self, snapshot=None, force=False):
        snapshot = snapshot or self.encoder_reader.snapshot()
        right = snapshot["right_levels"]
        left = snapshot["left_levels"]
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
            "ENCODER right=[%d,%d,%d] left=[%d,%d,%d] ticks right=%d left=%d "
            "invalid right=%d/%d left=%d/%d",
            right[0], right[1], right[2],
            left[0], left[1], left[2],
            snapshot["right_ticks"],
            snapshot["left_ticks"],
            snapshot["right_invalid_states"],
            snapshot["right_invalid_transitions"],
            snapshot["left_invalid_states"],
            snapshot["left_invalid_transitions"],
        )

    def _publish_encoder_twist(self, event):
        sample = self.encoder_reader.sample_motion()
        if sample is None:
            return

        msg = TwistStamped()
        msg.header.stamp = sample["stamp"]
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = sample["linear_mps"]
        msg.twist.angular.z = sample["angular_radps"]
        self.encoder_twist_pub.publish(msg)

        rospy.loginfo_throttle(
            1,
            "WHEEL_TWIST v=%.3f m/s w=%.3f rad/s "
            "ticks right=%d left=%d delta right=%d left=%d",
            sample["linear_mps"],
            sample["angular_radps"],
            sample["right_ticks"],
            sample["left_ticks"],
            sample["right_delta"],
            sample["left_delta"],
        )

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
            if self.encoder_twist_timer is not None:
                self.encoder_twist_timer.shutdown()
            self.encoder_reader.close()
            self._stop_motors(reset_modes=True)
            self.pi.set_servo_pulsewidth(self.pin_spray, 0)
        finally:
            self.pi.stop()

if __name__ == "__main__":
    rospy.init_node("motor_driver")
    MotorDriver()
    rospy.spin()
