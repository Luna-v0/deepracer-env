#################################################################################
#   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#################################################################################
"""Process-wide ROS 2 node + simulator-control singletons.

The legacy stack reached ROS through one anonymous ``rospy`` node implicitly
created by ``rospy.init_node`` and through ad-hoc service proxies. ROS 2 has no
global node, so the ported env shares exactly one :class:`SimNode` (publishers /
subscriptions / parameters) and one :class:`SimControl` backend (spawn / pose /
step / recolour). Every ported module — the trackers, the controller, the
sensors, the world swap — pulls these from here instead of constructing their
own, so there is a single DDS participant and a single view of the simulator.

Both are created lazily on first use, so importing :mod:`deepracer_env` never
requires a live ROS graph (host unit tests, the world converter, etc.).
"""
from __future__ import annotations

import os
import threading
from typing import Optional

# Reentrant: get_sim_control() holds the lock while calling get_node(), which
# re-acquires it — a plain Lock would self-deadlock.
_LOCK = threading.RLock()
_NODE = None
_SIM = None


def world_name() -> str:
    """The active world/track name.

    Sourced from the ``WORLD_NAME`` environment variable (how ``dr-gym`` and the
    launch pass the initial track), defaulting to ``reinvent_base``. Runtime
    track swaps go through :meth:`DeepRacerEnv.set_world`, not this value.
    """
    return os.environ.get("WORLD_NAME", "reinvent_base")


def get_node():
    """Return the shared :class:`~deepracer_env.sim_control.rclpy_client.SimNode`.

    Created (and started spinning) on first call. Imported lazily so this module
    stays host-importable.
    """
    global _NODE
    with _LOCK:
        if _NODE is None:
            from deepracer_env.sim_control.rclpy_client import SimNode
            _NODE = SimNode("deepracer_env")
        return _NODE


def get_sim_control(world: Optional[str] = None):
    """Return the shared :class:`~deepracer_env.sim_control.interface.SimControl`.

    Args:
        world: Override the world name for the (first) backend construction;
            defaults to :func:`world_name`.

    Returns:
        The process-wide simulator-control backend (see
        :func:`deepracer_env.sim_control.make_sim_control`).
    """
    global _SIM
    with _LOCK:
        if _SIM is None:
            from deepracer_env.sim_control import make_sim_control
            _SIM = make_sim_control(world or world_name(), node=get_node())
        return _SIM


def reset_runtime() -> None:
    """Tear down the singletons and the rclpy context. Idempotent.

    Order matters to avoid a teardown segfault: stop the spinning executor and
    destroy the node *before* shutting down the rclpy context, so the background
    thread is not mid-spin when the underlying context is finalised.
    """
    global _NODE, _SIM
    with _LOCK:
        if _SIM is not None:
            try:
                _SIM.close()
            except Exception:  # noqa: BLE001
                pass
            _SIM = None
        if _NODE is not None:
            try:
                _NODE.destroy()  # stops the executor thread, then destroys the node
            except Exception:  # noqa: BLE001
                pass
            _NODE = None
        try:
            import rclpy
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass
