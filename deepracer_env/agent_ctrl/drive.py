#################################################################################
#   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#################################################################################
"""Pure action → joint-command mapping for the ``ros2_control`` drive.

The agent's action is the 2-vector ``[steering_angle_deg, speed_m_s]``. Under
ros_control (the legacy stack) this was published as six ``std_msgs/Float64``
messages — one per controller. Under ``ros2_control`` the same six setpoints are
sent as two ``std_msgs/Float64MultiArray`` group commands (a
``forward_command_controller`` per group). **Only the transport changes; the
arithmetic below is preserved bit-for-bit from the legacy controller**, because
trained policies depend on it (see the audit: clamp-then-convert order matters,
and the wheel radius is version-locked).

Crucially — and matching the legacy behaviour exactly — there is **no Ackermann
decomposition here**: the same steering angle goes to *both* front hinges and the
same wheel angular velocity to *all four* wheels. The car's Ackermann geometry is
realised by the URDF joint constraints in Gazebo, not by this code.

Joint order
-----------
The returned arrays are positional and **must** line up with the ``joints:``
lists in ``config/ros2_control.yaml``:

* wheels:   ``[left_rear, right_rear, left_front, right_front]`` (velocity, rad/s)
* steering: ``[left_steering_hinge, right_steering_hinge]``      (position, rad)
"""
from __future__ import annotations

import math
from typing import List, Tuple

# Action bounds — the frozen gymnasium action space. Clamp BEFORE unit
# conversion (clamping in radians/rad-s would change the effective range).
MAX_STEERING_DEG = 30.0
MIN_STEERING_DEG = -30.0
MAX_SPEED_MPS = 4.0
MIN_SPEED_MPS = 0.1

# Wheel radius by simapp version (metres). Linear speed -> wheel angular velocity
# is ``speed / wheel_radius``; the radius is version-locked, so an off-by-one
# here silently changes how fast the car drives for a given action.
_WHEEL_RADIUS_BY_VERSION = {1.0: 0.1, 2.0: 0.0277, 3.0: 0.035}
DEFAULT_SIMAPP_VERSION = 5.0  # current default; uses the v3+ radius

# Canonical joint order (keep in lockstep with config/ros2_control.yaml).
WHEEL_JOINTS = (
    "left_rear_wheel_joint",
    "right_rear_wheel_joint",
    "left_front_wheel_joint",
    "right_front_wheel_joint",
)
STEERING_JOINTS = (
    "left_steering_hinge_joint",
    "right_steering_hinge_joint",
)


def get_wheel_radius(simapp_version: float = DEFAULT_SIMAPP_VERSION) -> float:
    """Return the wheel radius in metres for a simapp version.

    Args:
        simapp_version: The model version (1.0–5.0). Versions ``>= 3.0`` share
            the v3 radius (the kinematics-era geometry).

    Returns:
        Wheel radius in metres.
    """
    if simapp_version >= 3.0:
        return _WHEEL_RADIUS_BY_VERSION[3.0]
    return _WHEEL_RADIUS_BY_VERSION.get(simapp_version, _WHEEL_RADIUS_BY_VERSION[3.0])


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def action_to_joint_commands(
    steering_angle_deg: float,
    speed_mps: float,
    *,
    wheel_radius: float = None,
    simapp_version: float = DEFAULT_SIMAPP_VERSION,
) -> Tuple[List[float], List[float]]:
    """Map an agent action to ros2_control group commands.

    Args:
        steering_angle_deg: Desired steering angle in degrees (clamped to
            ``[-30, 30]``).
        speed_mps: Desired forward speed in m/s (clamped to ``[0.1, 4.0]``).
        wheel_radius: Override the wheel radius directly; if ``None`` it is
            derived from ``simapp_version``.
        simapp_version: Used to pick the wheel radius when ``wheel_radius`` is
            not given.

    Returns:
        A ``(wheel_velocities, steering_positions)`` tuple, where
        ``wheel_velocities`` is four identical rad/s values (one per wheel joint,
        in :data:`WHEEL_JOINTS` order) and ``steering_positions`` is two
        identical radian values (in :data:`STEERING_JOINTS` order).
    """
    if wheel_radius is None:
        wheel_radius = get_wheel_radius(simapp_version)

    steering_rad = _clamp(steering_angle_deg, MIN_STEERING_DEG, MAX_STEERING_DEG) * math.pi / 180.0
    angular_speed = _clamp(speed_mps, MIN_SPEED_MPS, MAX_SPEED_MPS) / wheel_radius

    wheel_velocities = [angular_speed] * len(WHEEL_JOINTS)
    steering_positions = [steering_rad] * len(STEERING_JOINTS)
    return wheel_velocities, steering_positions


def zero_commands() -> Tuple[List[float], List[float]]:
    """Return all-zero group commands — the stop sent on reset and in PARK/PAUSE.

    Returns:
        ``([0, 0, 0, 0], [0, 0])`` — zero wheel velocity, zero steering angle.
    """
    return [0.0] * len(WHEEL_JOINTS), [0.0] * len(STEERING_JOINTS)
