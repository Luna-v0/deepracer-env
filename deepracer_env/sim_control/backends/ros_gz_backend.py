#################################################################################
#   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#################################################################################
"""The working :class:`SimControl` backend for Gazebo Jetty.

Gazebo Jetty exposes its control plane as **gz-transport services** on
``/world/<world>/…`` — ``create`` / ``remove`` / ``set_pose`` / ``control``
(pause + deterministic ``multi_step``) / ``visual_config`` (recolour) — plus a
``/world/<world>/pose/info`` topic carrying every entity's pose. (Verified on
gz-sim 10.4.0.) Two facts shaped this backend:

* There are **no gz-transport Python bindings** in the Lyrical vendor packages,
  so we drive those services through the ``gz service`` / ``gz topic`` CLIs via
  :mod:`subprocess`. Every control-plane operation here is **cold path** (track
  spawn, obstacle placement, per-episode teleport / recolour), so the per-call
  CLI overhead is irrelevant. The *hot* paths bypass this backend entirely:
  wheel commands go through ``ros2_control`` topics and sensor frames arrive on
  bridged ROS topics.
* ``visual_config`` and ``light_config`` are **native** gz services, so visual /
  lighting domain randomisation needs no custom plugin — this is what lets
  Route B delete the legacy C++ ``SystemPlugin`` outright.

Pose reads are *batched*: one ``/world/<world>/pose/info`` snapshot per
:meth:`refresh_state` populates a cache that :meth:`get_entity_state` serves for
every entity — the same "one read serves all cars per step" pattern the legacy
``GetModelStateTracker`` used.
"""
from __future__ import annotations

import logging
import os
import subprocess
import tempfile
import time
from typing import Dict, List, Optional, Tuple

from deepracer_env.sim_control.interface import (
    Capability,
    SimControl,
    SimControlDead,
    SimControlError,
)
from deepracer_env.sim_control.types import (
    ColorRGBA,
    EntityState,
    Pose,
    Quaternion,
    Twist,
    Vec3,
    IDENTITY_POSE,
)

LOG = logging.getLogger(__name__)


def _fmt_pose(pose: Pose) -> str:
    """Render a :class:`Pose` as a gz protobuf-text ``pose`` fragment."""
    p, o = pose.position, pose.orientation
    return (
        "position {{x: {} y: {} z: {}}} "
        "orientation {{x: {} y: {} z: {} w: {}}}"
    ).format(p.x, p.y, p.z, o.x, o.y, o.z, o.w)


class RosGzBackend(SimControl):
    """Control a running gz-sim Jetty world over its gz-transport services."""

    def __init__(
        self,
        world_name: str,
        *,
        gz_timeout_ms: int = 5000,
        cli_timeout_s: float = 10.0,
        refresh_min_interval_s: float = 0.04,
    ) -> None:
        """Bind to a live gz world.

        Args:
            world_name: The gz world name (the ``<world name=...>`` of the loaded
                SDF), used to build the ``/world/<world>/…`` service prefix.
            gz_timeout_ms: Per-service timeout handed to ``gz service``.
            cli_timeout_s: Wall-clock cap on each ``subprocess`` invocation.
            refresh_min_interval_s: Minimum wall-clock gap between (non-forced)
                pose snapshots. The gz ``/clock`` callback asks for a refresh
                every tick (up to ~1 kHz); each snapshot is a CLI subprocess, so
                we coalesce them to ~25 Hz. Blocking reads pass ``force=True``.
        """
        self._world = world_name
        self._prefix = "/world/{}".format(world_name)
        self._gz_timeout_ms = gz_timeout_ms
        self._cli_timeout_s = cli_timeout_s
        self._refresh_min_interval_s = refresh_min_interval_s
        self._last_refresh_t = 0.0
        self._paused = False
        # batched pose cache: name -> (Pose, monotonic_t); previous snapshot kept
        # so twist can be finite-differenced (gz pose/info carries no velocity).
        self._pose_cache: Dict[str, Tuple[Pose, float]] = {}
        self._prev_pose_cache: Dict[str, Tuple[Pose, float]] = {}
        # Fast pose path: subscribe to the bridged `dynamic_pose/info` ROS topic
        # (continuous) instead of forking a `gz topic` subprocess per refresh —
        # the gz CLI snapshot was ~77% of training wall time on the feature/pose
        # path. Populated by a background callback; refresh_state reads the latest.
        # Falls back to the gz CLI when the bridge/topic isn't up.
        self._sub_cache: Dict[str, Tuple[Pose, float]] = {}
        self._pose_sub = None
        self._pose_sub_tried = False
        # Fast teleport path: a persistent rclpy client for the bridged set_pose
        # service, instead of forking a `gz service` per reset (~270ms = the
        # dominant short-episode/reset cost). Falls back to the gz CLI.
        self._set_pose_client = None
        self._set_pose_tried = False

    # -- capabilities ----------------------------------------------------------

    def supports(self, capability: str) -> bool:  # noqa: D102 (inherited)
        return capability in (
            Capability.DETERMINISTIC_STEP,
            Capability.VISUAL_RECOLOR,
            Capability.LIGHTING,
        )

    # -- gz CLI plumbing -------------------------------------------------------

    def _run(self, argv: List[str]) -> str:
        """Run a ``gz`` CLI command, returning stdout or raising on death."""
        try:
            proc = subprocess.run(
                argv, capture_output=True, text=True, timeout=self._cli_timeout_s
            )
        except subprocess.TimeoutExpired as ex:
            if not self._gz_alive():
                raise SimControlDead("gz did not respond: {}".format(ex))
            raise SimControlError("gz CLI timed out: {}".format(" ".join(argv)))
        return proc.stdout

    def _service(self, service: str, reqtype: str, reptype: str, req: str) -> str:
        """Call one gz-transport service via ``gz service -s``.

        Args:
            service: Service name suffix under ``/world/<world>/`` (e.g.
                ``"create"``) or an absolute name if it starts with ``/``.
            reqtype: gz request message type (e.g. ``"gz.msgs.EntityFactory"``).
            reptype: gz response message type.
            req: The request as gz protobuf text.

        Returns:
            The command's stdout.
        """
        name = service if service.startswith("/") else "{}/{}".format(self._prefix, service)
        return self._run([
            "gz", "service", "-s", name,
            "--reqtype", reqtype, "--reptype", reptype,
            "--timeout", str(self._gz_timeout_ms), "--req", req,
        ])

    @staticmethod
    def _ok(stdout: str) -> bool:
        """True if a ``gz.msgs.Boolean`` reply reported ``data: true``."""
        return "data: true" in stdout

    def _gz_alive(self, timeout: float = 3.0) -> bool:
        """Return True iff gz still lists this world's services.

        Turns a dead simulator into a catchable :class:`SimControlDead` instead
        of a hang — the Jetty analogue of the legacy ``gazebo_alive`` guard.
        """
        try:
            out = subprocess.run(
                ["gz", "service", "-l"], capture_output=True, text=True, timeout=timeout
            ).stdout
        except Exception:  # noqa: BLE001
            return False
        return "{}/control".format(self._prefix) in out

    # -- entity lifecycle ------------------------------------------------------

    def spawn_entity(self, name, sdf, pose=IDENTITY_POSE, *, allow_renaming=False):  # noqa: D102
        # gz EntityFactory spawns most reliably from a file; write the SDF out so
        # multi-line model descriptions need no protobuf-text escaping.
        fd, path = tempfile.mkstemp(suffix=".sdf", prefix="dr_spawn_")
        try:
            with os.fdopen(fd, "w") as fh:
                fh.write(sdf)
            req = 'sdf_filename: "{}" name: "{}" allow_renaming: {} pose {{{}}}'.format(
                path, name, "true" if allow_renaming else "false", _fmt_pose(pose))
            out = self._service("create", "gz.msgs.EntityFactory", "gz.msgs.Boolean", req)
        finally:
            try:
                os.remove(path)
            except OSError:
                pass
        if not self._ok(out):
            if not self._gz_alive():
                raise SimControlDead("gz died spawning {!r}".format(name))
            raise SimControlError("spawn of {!r} rejected: {}".format(name, out.strip()))
        return name

    def delete_entity(self, name):  # noqa: D102
        out = self._service(
            "remove", "gz.msgs.Entity", "gz.msgs.Boolean", 'name: "{}" type: MODEL'.format(name))
        if not self._ok(out) and not self._gz_alive():
            raise SimControlDead("gz died deleting {!r}".format(name))
        return self._ok(out)

    def list_entities(self):  # noqa: D102
        out = self._run(["gz", "model", "--list"])
        names = []
        for line in out.splitlines():
            line = line.strip()
            if line.startswith("- "):
                names.append(line[2:].strip())
        return names

    # -- state read / write ----------------------------------------------------

    def _ensure_pose_sub(self) -> None:
        """Lazily subscribe to the bridged ``dynamic_pose/info`` ROS topic.

        The launch bridges gz ``/world/<world>/dynamic_pose/info`` (gz.msgs.Pose_V)
        to a ROS ``tf2_msgs/TFMessage``; a background callback caches each entity's
        latest pose so :meth:`refresh_state` is subprocess-free. No-op (and the gz
        CLI fallback stays in effect) if rclpy / the shared node / the topic isn't
        available — e.g. host-side tests with no live bridge.
        """
        if self._pose_sub_tried:
            return
        self._pose_sub_tried = True
        try:
            from rclpy.qos import qos_profile_sensor_data
            from tf2_msgs.msg import TFMessage

            from deepracer_env.runtime import get_node

            node = get_node()
            topic = "{}/dynamic_pose/info".format(self._prefix)
            self._pose_sub = node.create_subscription(
                TFMessage, topic, self._on_pose_tf, qos_profile_sensor_data)
            LOG.info("RosGzBackend: subscribed to bridged pose topic %s", topic)
        except Exception as ex:  # noqa: BLE001
            LOG.warning("RosGzBackend: pose subscription unavailable (%s); "
                        "falling back to gz CLI pose snapshots", ex)
            self._pose_sub = None

    def _on_pose_tf(self, msg) -> None:
        """Cache the latest pose of every entity from a bridged TFMessage."""
        now = time.monotonic()
        cache = dict(self._sub_cache)  # copy-on-write so refresh_state reads atomically
        for tf in msg.transforms:
            t = tf.transform.translation
            r = tf.transform.rotation
            pose = Pose(position=Vec3(t.x, t.y, t.z),
                        orientation=Quaternion(r.x, r.y, r.z, r.w))
            cache[tf.child_frame_id] = (pose, now)
        self._sub_cache = cache

    def refresh_state(self, force: bool = False) -> None:
        """Refresh the entity pose cache.

        Fast path: copy the latest poses from the bridged ``dynamic_pose/info``
        subscription (no subprocess). The previous snapshot is retained so
        :meth:`get_entity_state` can finite-difference a velocity. Falls back to a
        throttled ``gz topic`` snapshot when the subscription has no data yet.
        """
        self._ensure_pose_sub()
        if self._pose_sub is not None and self._sub_cache:
            snapshot = self._sub_cache  # atomic ref (callback rebinds, never mutates)
            if snapshot is not self._pose_cache:
                self._prev_pose_cache = self._pose_cache
                self._pose_cache = snapshot
            return

        now = time.monotonic()
        if not force and (now - self._last_refresh_t) < self._refresh_min_interval_s:
            return
        self._last_refresh_t = now
        out = self._run([
            "gz", "topic", "-e", "-n", "1",
            "-t", "{}/pose/info".format(self._prefix),
        ])
        poses = self._parse_pose_v(out)
        if poses:
            self._prev_pose_cache = self._pose_cache
            self._pose_cache = {n: (p, time.monotonic()) for n, p in poses.items()}

    def get_entity_state(self, name, *, reference_frame="world"):  # noqa: D102
        if name not in self._pose_cache:
            # Lazy forced refresh if the caller never primed the cache.
            self.refresh_state(force=True)
        if name not in self._pose_cache:
            raise SimControlError("entity {!r} not found in pose snapshot".format(name))
        pose, t = self._pose_cache[name]
        twist = self._estimate_twist(name, pose, t)
        return EntityState(pose=pose, twist=twist)

    def _estimate_twist(self, name: str, pose: Pose, t: float) -> Twist:
        """Finite-difference a velocity from the previous pose snapshot."""
        prev = self._prev_pose_cache.get(name)
        if prev is None:
            return Twist()
        p0, t0 = prev
        dt = t - t0
        if dt <= 1e-6:
            return Twist()
        lin = Vec3((pose.position.x - p0.position.x) / dt,
                   (pose.position.y - p0.position.y) / dt,
                   (pose.position.z - p0.position.z) / dt)
        dyaw = pose.orientation.yaw - p0.orientation.yaw
        # wrap to [-pi, pi] so a 359°->1° step reads as +2°, not -358°.
        while dyaw > 3.141592653589793:
            dyaw -= 2 * 3.141592653589793
        while dyaw < -3.141592653589793:
            dyaw += 2 * 3.141592653589793
        return Twist(linear=lin, angular=Vec3(0.0, 0.0, dyaw / dt))

    def _ensure_set_pose_client(self) -> None:
        """Lazily create the rclpy client for the bridged set_pose service.

        Uses a DEDICATED node + executor (not the shared background SimNode), so
        the call can ``spin_until_future_complete`` synchronously — the same
        pattern ``ros2 service call`` uses, which works where a call_async on the
        background-spun node did not.
        """
        if self._set_pose_tried:
            return
        self._set_pose_tried = True
        try:
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from ros_gz_interfaces.srv import SetEntityPose

            from deepracer_env.sim_control.rclpy_client import ensure_rclpy_initialized

            ensure_rclpy_initialized()
            self._set_pose_node = Node("deepracer_set_pose_client")
            self._set_pose_exec = SingleThreadedExecutor()
            self._set_pose_exec.add_node(self._set_pose_node)
            cli = self._set_pose_node.create_client(
                SetEntityPose, "{}/set_pose".format(self._prefix))
            if cli.wait_for_service(timeout_sec=5.0):
                self._set_pose_client = cli
                LOG.info("RosGzBackend: using bridged set_pose service client")
            else:
                LOG.warning("RosGzBackend: set_pose service bridge not up; gz-CLI fallback")
                self._set_pose_client = None
        except Exception as ex:  # noqa: BLE001
            LOG.warning("RosGzBackend: set_pose client unavailable (%s); gz-CLI fallback", ex)
            self._set_pose_client = None

    def set_entity_state(self, name, state, *, blocking=True):  # noqa: D102
        # gz `set_pose` sets pose only; twist is re-settled by physics + the
        # zeroed wheel commands the reset path already issues. (See note in the
        # module docstring; a twist-set needs the entity component API.)
        self._ensure_set_pose_client()
        if self._set_pose_client is not None:
            from ros_gz_interfaces.msg import Entity
            from ros_gz_interfaces.srv import SetEntityPose

            req = SetEntityPose.Request()
            req.entity.name = name
            req.entity.type = Entity.MODEL
            p, o = state.pose.position, state.pose.orientation
            req.pose.position.x, req.pose.position.y, req.pose.position.z = p.x, p.y, p.z
            req.pose.orientation.x = o.x
            req.pose.orientation.y = o.y
            req.pose.orientation.z = o.z
            req.pose.orientation.w = o.w
            import rclpy

            future = self._set_pose_client.call_async(req)
            rclpy.spin_until_future_complete(
                self._set_pose_node, future, self._set_pose_exec, timeout_sec=2.0)
            if future.done():
                res = future.result()
                return bool(getattr(res, "success", True))
            # timed out -> disable the client (no per-reset 2s penalty) and use
            # the gz CLI from here on.
            LOG.warning("set_pose service call timed out for %r; disabling client, gz-CLI fallback", name)
            self._set_pose_client = None

        req_txt = 'name: "{}" {}'.format(name, _fmt_pose(state.pose))
        out = self._service("set_pose", "gz.msgs.Pose", "gz.msgs.Boolean", req_txt)
        if not self._ok(out) and not self._gz_alive():
            raise SimControlDead("gz died teleporting {!r}".format(name))
        return self._ok(out)

    # -- time control ----------------------------------------------------------

    def step(self, n=1):  # noqa: D102
        # `multi_step` advances exactly n ticks and leaves the world paused —
        # the deterministic, race-free advance the legacy pause/unpause lacked.
        out = self._service(
            "control", "gz.msgs.WorldControl", "gz.msgs.Boolean",
            "pause: true multi_step: {}".format(int(n)))
        self._paused = True
        if not self._ok(out) and not self._gz_alive():
            raise SimControlDead("gz died stepping")

    def pause(self):  # noqa: D102
        self._service("control", "gz.msgs.WorldControl", "gz.msgs.Boolean", "pause: true")
        self._paused = True

    def unpause(self):  # noqa: D102
        self._service("control", "gz.msgs.WorldControl", "gz.msgs.Boolean", "pause: false")
        self._paused = False

    # -- visual domain randomisation ------------------------------------------

    def set_visual_color(self, entity, link, visual, diffuse, *, ambient=None, blocking=True):  # noqa: D102
        amb = ambient if ambient is not None else ColorRGBA(
            diffuse.r * 0.6, diffuse.g * 0.6, diffuse.b * 0.6, diffuse.a)
        # gz Visual carries the target by name + parent_name (the link); the
        # material block recolours it in the rendering scene.
        req = (
            'name: "{visual}" parent_name: "{link}" material {{'
            ' ambient {{r: {ar} g: {ag} b: {ab} a: {aa}}}'
            ' diffuse {{r: {dr} g: {dg} b: {db} a: {da}}} }}'
        ).format(visual=visual, link=link,
                 ar=amb.r, ag=amb.g, ab=amb.b, aa=amb.a,
                 dr=diffuse.r, dg=diffuse.g, db=diffuse.b, da=diffuse.a)
        out = self._service("visual_config", "gz.msgs.Visual", "gz.msgs.Boolean", req)
        return self._ok(out)

    def set_light(self, name, *, diffuse=None, specular=None, direction=None, blocking=True):  # noqa: D102
        # gz-sim's native /world/<w>/light_config takes a gz.msgs.Light. Only the
        # fields we set are sent; gz keeps the rest. Used for lighting DR.
        parts = ['name: "{}"'.format(name)]
        if diffuse is not None:
            parts.append("diffuse {{r: {} g: {} b: {} a: {}}}".format(
                diffuse.r, diffuse.g, diffuse.b, diffuse.a))
        if specular is not None:
            parts.append("specular {{r: {} g: {} b: {} a: {}}}".format(
                specular.r, specular.g, specular.b, specular.a))
        if direction is not None:
            dx, dy, dz = direction
            parts.append("direction {{x: {} y: {} z: {}}}".format(dx, dy, dz))
        out = self._service("light_config", "gz.msgs.Light", "gz.msgs.Boolean", " ".join(parts))
        return self._ok(out)

    # -- parsing ---------------------------------------------------------------

    @staticmethod
    def _parse_pose_v(text: str) -> Dict[str, Pose]:
        """Parse ``gz topic`` text dump of a ``gz.msgs.Pose_V`` into name->Pose.

        The dump nests ``pose { name … position { x y z } orientation { … } }``.
        We track brace depth so the ``x/y/z`` inside ``position`` and
        ``orientation`` land in the right slot and any other nested sub-message
        (e.g. a header) is skipped.
        """
        out: Dict[str, Pose] = {}
        depth = 0
        cur_name: Optional[str] = None
        pos = [0.0, 0.0, 0.0]
        ori = [0.0, 0.0, 0.0, 1.0]
        sub: Optional[str] = None
        for raw in text.splitlines():
            line = raw.strip()
            if not line:
                continue
            if line.endswith("{"):
                key = line[:-1].strip().rstrip(":").strip()
                depth += 1
                if key == "pose" and depth == 1:
                    cur_name, pos, ori, sub = None, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], None
                elif key in ("position", "orientation"):
                    sub = key
                else:
                    sub = "ignore"
                continue
            if line == "}":
                depth -= 1
                if depth == 1:
                    sub = None
                elif depth == 0 and cur_name is not None:
                    out[cur_name] = Pose(Vec3(*pos), Quaternion(*ori))
                    cur_name = None
                continue
            if ":" not in line:
                continue
            field, _, value = line.partition(":")
            field, value = field.strip(), value.strip()
            if field == "name" and depth == 1:
                cur_name = value.strip('"')
            elif sub == "position" and field in ("x", "y", "z"):
                pos[{"x": 0, "y": 1, "z": 2}[field]] = float(value)
            elif sub == "orientation" and field in ("x", "y", "z", "w"):
                ori[{"x": 0, "y": 1, "z": 2, "w": 3}[field]] = float(value)
        return out
