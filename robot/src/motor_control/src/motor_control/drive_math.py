#!/usr/bin/env python3
"""Helpers for differential-drive velocity conversion."""


def compute_wheel_speeds(linear, angular, wheel_base):
    """Compute left/right wheel speeds for a differential drive.

    The returned values are expressed in the robot's coordinate frame so that
    positive linear values mean forward motion and positive angular values mean
    a counter-clockwise rotation. The motor reversal flags from the YAML config
    are applied later by the motor driver.
    """
    if wheel_base <= 0:
        raise ValueError("wheel_base must be > 0")

    left_speed = linear - (angular * wheel_base / 2.0)
    right_speed = linear + (angular * wheel_base / 2.0)
    return left_speed, right_speed


def normalize_twist_command(linear, angular, max_linear, max_angular):
    """Normalize a Twist command to the [-1, 1] range while preserving sign."""
    if max_linear <= 0:
        raise ValueError("max_linear must be > 0")
    if max_angular <= 0:
        raise ValueError("max_angular must be > 0")

    linear_cmd = 0.0 if abs(linear) < 1e-9 else float(linear) / max_linear
    angular_cmd = 0.0 if abs(angular) < 1e-9 else float(angular) / max_angular

    if linear_cmd > 1.0:
        linear_cmd = 1.0
    elif linear_cmd < -1.0:
        linear_cmd = -1.0

    if angular_cmd > 1.0:
        angular_cmd = 1.0
    elif angular_cmd < -1.0:
        angular_cmd = -1.0

    return linear_cmd, angular_cmd
