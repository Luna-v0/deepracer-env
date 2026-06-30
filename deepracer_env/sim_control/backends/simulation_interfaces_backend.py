#################################################################################
#   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#################################################################################
"""The future-primary :class:`SimControl` backend: ROS 2 ``simulation_interfaces``.

``simulation_interfaces`` is the cross-vendor ROS 2 standard for simulator
control (``SpawnEntity`` / ``DeleteEntity`` / ``GetEntityState`` /
``SetEntityState`` / ``StepSimulation`` / ``SetSimulationState`` …). Targeting it
makes the environment **simulator-agnostic**: the day a Jetty release (or a USD /
MuJoCo backend) ships a server for these services, the env flips to it by
changing one line in :mod:`deepracer_env.sim_control.factory` — no env code
changes.

Status on Lyrical / gz-sim 10.4 (2026-06): the *message package* is installed but
**no process serves the services yet** (only ``ros_gz_sim`` CLI helpers exist).
Hence :class:`~deepracer_env.sim_control.backends.ros_gz_backend.RosGzBackend` is
the active default and this backend is wired but dormant. It is written against
the exact, introspected srv fields so it is correct the moment a server appears;
:func:`is_available` lets the factory probe at runtime.

Field notes baked in below (from ``ros2 interface show``):
* ``SetEntityState`` needs ``set_pose`` / ``set_twist`` flags.
* ``StepSimulation`` requires the sim to be **paused** first (we drive that via
  ``SetSimulationState(PAUSED)``).
* Every reply carries ``result.result``; success is ``Result.RESULT_OK == 1``.
* An SDF string is passed in ``entity_resource.resource_string``.
"""
from __future__ import annotations

import logging
from typing import List

from deepracer_env.sim_control.interface import (
    Capability,
    SimControl,
    SimControlError,
)
from deepracer_env.sim_control.types import EntityState, Pose, Twist, Vec3, Quaternion, IDENTITY_POSE

LOG = logging.getLogger(__name__)

# Resolved lazily so importing this module never hard-requires the message pkg.
_RESULT_OK = 1


def is_available() -> bool:
    """Return True iff the ``simulation_interfaces`` message package imports.

    Note this only checks message availability, not whether a *server* is
    running; the factory pairs it with a service-presence probe.
    """
    try:
        import simulation_interfaces.srv  # noqa: F401
        return True
    except Exception:  # noqa: BLE001
        return False


class SimulationInterfacesBackend(SimControl):
    """Adapt the ROS 2 ``simulation_interfaces`` services to :class:`SimControl`."""

    def __init__(self, node, *, service_namespace: str = "") -> None:
        """Create service clients on *node*.

        Args:
            node: A spinning :class:`~deepracer_env.sim_control.rclpy_client.SimNode`.
            service_namespace: Optional prefix if the server namespaces its
                services (e.g. ``"/sim"``).
        """
        from simulation_interfaces import srv
        from simulation_interfaces import msg
        from deepracer_env.sim_control.rclpy_client import ServiceClientWrapper

        self._msg = msg
        self._srv = srv
        ns = service_namespace.rstrip("/")

        def name(s: str) -> str:
            return "{}/{}".format(ns, s) if ns else s

        self._spawn = ServiceClientWrapper(node, srv.SpawnEntity, name("spawn_entity"))
        self._delete = ServiceClientWrapper(node, srv.DeleteEntity, name("delete_entity"))
        self._get = ServiceClientWrapper(node, srv.GetEntityState, name("get_entity_state"))
        self._set = ServiceClientWrapper(node, srv.SetEntityState, name("set_entity_state"))
        self._entities = ServiceClientWrapper(node, srv.GetEntities, name("get_entities"))
        self._step = ServiceClientWrapper(node, srv.StepSimulation, name("step_simulation"))
        self._sim_state = ServiceClientWrapper(node, srv.SetSimulationState, name("set_simulation_state"))

    # -- capabilities ----------------------------------------------------------

    def supports(self, capability: str) -> bool:  # noqa: D102
        return capability in (Capability.DETERMINISTIC_STEP,)

    # -- helpers ---------------------------------------------------------------

    @staticmethod
    def _check(result, what: str) -> None:
        if getattr(result, "result", None) != _RESULT_OK:
            raise SimControlError(
                "{} failed (code {}): {}".format(
                    what, getattr(result, "result", "?"),
                    getattr(result, "error_message", "")))

    def _to_ros_pose(self, pose: Pose):
        from geometry_msgs.msg import Pose as RosPose
        rp = RosPose()
        rp.position.x, rp.position.y, rp.position.z = pose.position.as_tuple()
        rp.orientation.x = pose.orientation.x
        rp.orientation.y = pose.orientation.y
        rp.orientation.z = pose.orientation.z
        rp.orientation.w = pose.orientation.w
        return rp

    def _entity_state_msg(self, state: EntityState):
        es = self._msg.EntityState()
        es.pose = self._to_ros_pose(state.pose)
        es.twist.linear.x, es.twist.linear.y, es.twist.linear.z = state.twist.linear.as_tuple()
        es.twist.angular.x, es.twist.angular.y, es.twist.angular.z = state.twist.angular.as_tuple()
        return es

    # -- entity lifecycle ------------------------------------------------------

    def spawn_entity(self, name, sdf, pose=IDENTITY_POSE, *, allow_renaming=False):  # noqa: D102
        from geometry_msgs.msg import PoseStamped
        req = self._srv.SpawnEntity.Request()
        req.name = name
        req.allow_renaming = allow_renaming
        req.entity_resource.resource_string = sdf
        ps = PoseStamped()
        ps.pose = self._to_ros_pose(pose)
        req.initial_pose = ps
        resp = self._spawn(req)
        self._check(resp.result, "spawn_entity({})".format(name))
        return resp.entity_name or name

    def delete_entity(self, name):  # noqa: D102
        req = self._srv.DeleteEntity.Request()
        req.entity = name
        resp = self._delete(req)
        self._check(resp.result, "delete_entity({})".format(name))
        return True

    def list_entities(self):  # noqa: D102
        req = self._srv.GetEntities.Request()
        resp = self._entities(req)
        self._check(resp.result, "list_entities")
        return list(resp.entities)

    # -- state read / write ----------------------------------------------------

    def get_entity_state(self, name, *, reference_frame="world"):  # noqa: D102
        req = self._srv.GetEntityState.Request()
        req.entity = name
        resp = self._get(req)
        self._check(resp.result, "get_entity_state({})".format(name))
        s = resp.state
        return EntityState(
            pose=Pose(
                Vec3(s.pose.position.x, s.pose.position.y, s.pose.position.z),
                Quaternion(s.pose.orientation.x, s.pose.orientation.y,
                           s.pose.orientation.z, s.pose.orientation.w)),
            twist=Twist(
                Vec3(s.twist.linear.x, s.twist.linear.y, s.twist.linear.z),
                Vec3(s.twist.angular.x, s.twist.angular.y, s.twist.angular.z)))

    def set_entity_state(self, name, state, *, blocking=True):  # noqa: D102
        req = self._srv.SetEntityState.Request()
        req.entity = name
        req.state = self._entity_state_msg(state)
        req.set_pose = True
        req.set_twist = True
        resp = self._set(req)
        self._check(resp.result, "set_entity_state({})".format(name))
        return True

    # -- time control ----------------------------------------------------------

    def step(self, n=1):  # noqa: D102
        req = self._srv.StepSimulation.Request()
        req.steps = int(n)
        resp = self._step(req)
        self._check(resp.result, "step_simulation")

    def _set_state(self, state_value: int, what: str) -> None:
        req = self._srv.SetSimulationState.Request()
        req.state = self._msg.SimulationState()
        req.state.state = state_value
        resp = self._sim_state(req)
        self._check(resp.result, what)

    def pause(self):  # noqa: D102
        self._set_state(self._msg.SimulationState.STATE_PAUSED, "pause")

    def unpause(self):  # noqa: D102
        self._set_state(self._msg.SimulationState.STATE_PLAYING, "unpause")
