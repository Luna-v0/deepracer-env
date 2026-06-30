#################################################################################
#   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#################################################################################
"""The simulator-control seam: one interface over Gazebo Jetty.

This package is the single boundary between :mod:`deepracer_env` and the running
simulator. Everything the environment needs from the simulator — spawn, delete,
teleport, read state, step, recolour — goes through :class:`SimControl`, which
has interchangeable backends (``ros_gz`` today, ``simulation_interfaces``
tomorrow). See :mod:`deepracer_env.sim_control.interface` for the design.

Import policy
-------------
The value types (:mod:`.types`), the interface (:mod:`.interface`), and the
multi-arena layout (:mod:`.arena`) are **ROS-free** and import on any host — so
pose math and arena geometry are unit-testable without a ROS install. The
concrete backends and the rclpy plumbing import ``rclpy`` and are therefore
loaded lazily by :func:`~deepracer_env.sim_control.factory.make_sim_control`
(and re-exported here as a thin convenience) only inside the ROS 2 container.
"""
from deepracer_env.sim_control.arena import (
    DEFAULT_ARENA_SPACING_M,
    Arena,
    ArenaLayout,
)
from deepracer_env.sim_control.interface import (
    Capability,
    CapabilityNotSupported,
    NullSimControl,
    SimControl,
    SimControlDead,
    SimControlError,
    SimControlTimeout,
)
from deepracer_env.sim_control.types import (
    ColorRGBA,
    EntityState,
    IDENTITY_POSE,
    Pose,
    Quaternion,
    Twist,
    Vec3,
)

__all__ = [
    # value types
    "Vec3", "Quaternion", "Pose", "Twist", "EntityState", "ColorRGBA", "IDENTITY_POSE",
    # interface
    "SimControl", "NullSimControl", "Capability",
    "SimControlError", "SimControlTimeout", "SimControlDead", "CapabilityNotSupported",
    # arenas
    "Arena", "ArenaLayout", "DEFAULT_ARENA_SPACING_M",
    # factory (lazy)
    "make_sim_control",
]


def make_sim_control(*args, **kwargs):
    """Lazy proxy for :func:`deepracer_env.sim_control.factory.make_sim_control`.

    Imported on demand so that ``import deepracer_env.sim_control`` stays
    ROS-free; the factory (and the backend it builds) pull in ``rclpy`` only when
    actually called inside the container.
    """
    from deepracer_env.sim_control.factory import make_sim_control as _impl
    return _impl(*args, **kwargs)
