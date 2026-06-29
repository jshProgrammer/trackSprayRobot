#!/usr/bin/env python3
import math
import threading

import pigpio
import rospy


class HallWheelEncoder:
    def __init__(self, name, pins, hall_sequence, reverse=False):
        self.name = name
        self.pins = tuple(pins)
        self.sequence = tuple(hall_sequence)
        self.index_by_state = {
            state: index for index, state in enumerate(self.sequence)
        }
        self.reverse = reverse
        self.levels = [0, 0, 0]
        self.ticks = 0
        self.last_index = None
        self.invalid_states = 0
        self.invalid_transitions = 0

    def set_initial_levels(self, levels):
        self.levels = [int(level) for level in levels]
        self.last_index = self.index_by_state.get(tuple(self.levels))
        if self.last_index is None:
            self.invalid_states += 1

    def update(self, levels):
        self.levels = [int(level) for level in levels]
        index = self.index_by_state.get(tuple(self.levels))

        if index is None:
            self.invalid_states += 1
            self.last_index = None
            return 0

        if self.last_index is None:
            self.last_index = index
            return 0

        step = (index - self.last_index) % len(self.sequence)
        if step == 0:
            return 0

        half_sequence = len(self.sequence) / 2.0
        if step == half_sequence:
            self.invalid_transitions += 1
            self.last_index = index
            return 0

        delta = step if step < half_sequence else step - len(self.sequence)
        if self.reverse:
            delta = -delta

        self.ticks += int(delta)
        self.last_index = index
        return int(delta)


class EncoderReader:
    DEFAULT_HALL_SEQUENCE = ("001", "101", "100", "110", "010", "011")

    def __init__(
        self,
        pi,
        right_pins,
        left_pins,
        hall_sequence,
        right_reversed,
        left_reversed,
        wheel_radius,
        ticks_per_motor_revolution,
        motor_to_wheel_ratio,
        wheel_base,
        velocity_publish_rate,
    ):
        self.pi = pi
        self.lock = threading.Lock()
        self.callbacks = []
        self.on_change = None
        self.velocity_publish_rate = velocity_publish_rate

        self.right = HallWheelEncoder(
            "right",
            right_pins,
            hall_sequence,
            reverse=right_reversed,
        )
        self.left = HallWheelEncoder(
            "left",
            left_pins,
            hall_sequence,
            reverse=left_reversed,
        )
        self.gpio_to_wheel = {
            gpio: self.right for gpio in self.right.pins
        }
        self.gpio_to_wheel.update({
            gpio: self.left for gpio in self.left.pins
        })

        self.wheel_radius = float(wheel_radius)
        self.ticks_per_motor_revolution = float(ticks_per_motor_revolution)
        self.motor_to_wheel_ratio = float(motor_to_wheel_ratio)
        self.wheel_base = float(wheel_base)
        self.ticks_per_wheel_revolution = (
            self.ticks_per_motor_revolution * self.motor_to_wheel_ratio
        )
        self.meters_per_tick = None
        if self.wheel_radius > 0 and self.ticks_per_wheel_revolution > 0:
            self.meters_per_tick = (
                2.0 * math.pi * self.wheel_radius
            ) / self.ticks_per_wheel_revolution

        self.last_sample_time = rospy.Time.now()
        self.last_sample_right_ticks = 0
        self.last_sample_left_ticks = 0

        for gpio in self._all_pins():
            self.pi.set_mode(gpio, pigpio.INPUT)
            self.pi.set_pull_up_down(gpio, pigpio.PUD_OFF)

        self.right.set_initial_levels(self._read_levels(self.right.pins))
        self.left.set_initial_levels(self._read_levels(self.left.pins))

    @classmethod
    def from_ros_params(cls, pi, namespace="/motor_driver"):
        right_pins = (
            rospy.get_param(f"{namespace}/encoder_right_a"),
            rospy.get_param(f"{namespace}/encoder_right_b"),
            rospy.get_param(f"{namespace}/encoder_right_c"),
        )
        left_pins = (
            rospy.get_param(f"{namespace}/encoder_left_a"),
            rospy.get_param(f"{namespace}/encoder_left_b"),
            rospy.get_param(f"{namespace}/encoder_left_c"),
        )
        right_motor_reversed = rospy.get_param(
            f"{namespace}/right_motor_reversed",
            False,
        )
        left_motor_reversed = rospy.get_param(
            f"{namespace}/left_motor_reversed",
            False,
        )

        return cls(
            pi=pi,
            right_pins=right_pins,
            left_pins=left_pins,
            hall_sequence=cls.parse_hall_sequence(
                rospy.get_param(
                    f"{namespace}/encoder_hall_sequence",
                    cls.DEFAULT_HALL_SEQUENCE,
                )
            ),
            right_reversed=rospy.get_param(
                f"{namespace}/encoder_right_reversed",
                right_motor_reversed,
            ),
            left_reversed=rospy.get_param(
                f"{namespace}/encoder_left_reversed",
                left_motor_reversed,
            ),
            wheel_radius=rospy.get_param(f"{namespace}/wheel_radius", 0.0),
            ticks_per_motor_revolution=rospy.get_param(
                f"{namespace}/encoder_ticks_per_motor_revolution",
                0,
            ),
            motor_to_wheel_ratio=rospy.get_param(
                f"{namespace}/encoder_motor_to_wheel_ratio",
                1.0,
            ),
            wheel_base=rospy.get_param(f"{namespace}/wheel_base"),
            velocity_publish_rate=max(
                rospy.get_param(f"{namespace}/encoder_velocity_publish_rate", 20.0),
                0.1,
            ),
        )

    @classmethod
    def parse_hall_sequence(cls, raw_sequence):
        sequence = raw_sequence or cls.DEFAULT_HALL_SEQUENCE
        parsed = []

        for raw_state in sequence:
            if isinstance(raw_state, str):
                state = raw_state.strip()
                if state.startswith("0b"):
                    state = state[2:]
                if len(state) != 3 or any(bit not in "01" for bit in state):
                    raise ValueError(
                        "encoder_hall_sequence must contain 3-bit states"
                    )
                bits = tuple(int(bit) for bit in state)
            else:
                try:
                    bits = tuple(int(bit) for bit in raw_state)
                except TypeError:
                    value = int(raw_state)
                    bits = (
                        (value >> 2) & 1,
                        (value >> 1) & 1,
                        value & 1,
                    )

                if len(bits) != 3 or any(bit not in (0, 1) for bit in bits):
                    raise ValueError(
                        "encoder_hall_sequence must contain 3-bit states"
                    )

            parsed.append(bits)

        if len(parsed) < 3:
            raise ValueError("encoder_hall_sequence needs at least 3 states")
        if len(set(parsed)) != len(parsed):
            raise ValueError("encoder_hall_sequence contains duplicate states")

        return tuple(parsed)

    @property
    def can_measure_velocity(self):
        return self.meters_per_tick is not None

    def _all_pins(self):
        return self.right.pins + self.left.pins

    def _read_levels(self, pins):
        return [self.pi.read(gpio) for gpio in pins]

    def start(self, on_change=None):
        self.on_change = on_change
        for gpio in self._all_pins():
            self.callbacks.append(
                self.pi.callback(gpio, pigpio.EITHER_EDGE, self._edge_callback)
            )

    def close(self):
        for callback in self.callbacks:
            callback.cancel()
        self.callbacks = []

    def snapshot(self):
        with self.lock:
            return self._snapshot_locked()

    def _snapshot_locked(self):
        return {
            "right_levels": list(self.right.levels),
            "left_levels": list(self.left.levels),
            "right_ticks": self.right.ticks,
            "left_ticks": self.left.ticks,
            "right_invalid_states": self.right.invalid_states,
            "left_invalid_states": self.left.invalid_states,
            "right_invalid_transitions": self.right.invalid_transitions,
            "left_invalid_transitions": self.left.invalid_transitions,
        }

    def _edge_callback(self, gpio, level, tick):
        if level == pigpio.TIMEOUT:
            return

        wheel = self.gpio_to_wheel.get(gpio)
        if wheel is None:
            return

        with self.lock:
            wheel.update(self._read_levels(wheel.pins))
            snapshot = self._snapshot_locked()

        if self.on_change is not None:
            self.on_change(snapshot)

    def sample_motion(self, now=None):
        if not self.can_measure_velocity:
            return None

        now = now or rospy.Time.now()
        with self.lock:
            dt = (now - self.last_sample_time).to_sec()
            if dt <= 0:
                return None

            right_ticks = self.right.ticks
            left_ticks = self.left.ticks
            right_delta = right_ticks - self.last_sample_right_ticks
            left_delta = left_ticks - self.last_sample_left_ticks

            self.last_sample_time = now
            self.last_sample_right_ticks = right_ticks
            self.last_sample_left_ticks = left_ticks

        right_mps = right_delta * self.meters_per_tick / dt
        left_mps = left_delta * self.meters_per_tick / dt
        linear_mps = (right_mps + left_mps) / 2.0
        angular_radps = 0.0
        if self.wheel_base > 0:
            angular_radps = (right_mps - left_mps) / self.wheel_base

        return {
            "stamp": now,
            "dt": dt,
            "right_ticks": right_ticks,
            "left_ticks": left_ticks,
            "right_delta": right_delta,
            "left_delta": left_delta,
            "right_mps": right_mps,
            "left_mps": left_mps,
            "linear_mps": linear_mps,
            "angular_radps": angular_radps,
        }
