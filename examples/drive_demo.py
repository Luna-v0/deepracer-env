#!/usr/bin/env python3
#################################################################################
#   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#################################################################################
"""Drive the DeepRacer car around a track and log its trajectory.

A small, self-contained integration demo for the ROS 2 Lyrical / Gazebo Jetty
stack. It dogfoods three pieces of the port at once:

* the :class:`~deepracer_env.sim_control.backends.ros_gz_backend.RosGzBackend`
  seam — to **teleport** the car onto the track start (``set_entity_state``) and
  to **read** the car pose every step (``get_entity_state``);
* :func:`deepracer_env.agent_ctrl.drive.action_to_joint_commands` — to turn a
  ``[steering_deg, speed]`` action into ``ros2_control`` group commands; and
* the live ``ros2_control`` controllers — by publishing ``Float64MultiArray`` to
  ``/wheels_velocity_controller/commands`` and
  ``/steering_position_controller/commands``.

Steering uses a tiny pure-pursuit law against the track centre-line waypoints
(``routes/<world>.npy``), so the car actually follows the loop. The pose samples
are written to a CSV (``--out``) that ``examples/render_demo_video.py`` turns
into an MP4.

Run it *inside* the container after the stack is up (gz + the car + active
controllers) — see ``examples/demo_bringup.sh``.
"""
from __future__ import annotations

import argparse
import csv
import math
import time

import numpy as np
import rclpy
from std_msgs.msg import Float64MultiArray

from deepracer_env.agent_ctrl.drive import action_to_joint_commands
from deepracer_env.sim_control.backends.ros_gz_backend import RosGzBackend
from deepracer_env.sim_control.rclpy_client import SimNode
from deepracer_env.sim_control.types import EntityState, Pose


def _heading_to(frm, to) -> float:
    """Planar heading (rad) from point ``frm`` to point ``to``."""
    return math.atan2(to[1] - frm[1], to[0] - frm[0])


def _wrap(angle: float) -> float:
    """Wrap an angle to ``[-pi, pi]``."""
    return math.atan2(math.sin(angle), math.cos(angle))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--world", default="reinvent_base")
    parser.add_argument("--car", default="car")
    parser.add_argument("--routes", default="/sim/routes")
    parser.add_argument("--out", default="/out/traj.csv")
    parser.add_argument("--duration", type=float, default=22.0)
    parser.add_argument("--speed", type=float, default=1.6, help="target speed m/s")
    parser.add_argument("--lookahead", type=int, default=8, help="waypoints ahead")
    parser.add_argument("--rate", type=float, default=10.0, help="control Hz")
    args = parser.parse_args()

    centre = np.load("{}/{}.npy".format(args.routes, args.world))[:, :2]
    n = len(centre)

    rclpy.init()
    node = SimNode("drive_demo")
    backend = RosGzBackend(args.world)
    wheels_pub = node.create_publisher(Float64MultiArray, "/wheels_velocity_controller/commands", 10)
    steer_pub = node.create_publisher(Float64MultiArray, "/steering_position_controller/commands", 10)

    # Teleport onto the start line, heading toward the next waypoint (dogfoods
    # the per-entity reset that powers decoupled multi-arena episodes).
    start_yaw = _heading_to(centre[0], centre[args.lookahead % n])
    backend.set_entity_state(
        args.car, EntityState(pose=Pose.at(centre[0][0], centre[0][1], 0.06, start_yaw)))
    time.sleep(1.0)

    rows = []
    t0 = time.monotonic()
    dt = 1.0 / args.rate
    while time.monotonic() - t0 < args.duration:
        backend.refresh_state()
        try:
            state = backend.get_entity_state(args.car)
        except Exception:  # noqa: BLE001 — first frames may precede the snapshot
            time.sleep(dt)
            continue
        x, y = state.pose.position.x, state.pose.position.y
        yaw = state.pose.orientation.yaw

        # pure pursuit: aim at the lookahead point past the nearest waypoint
        nearest = int(np.argmin(np.hypot(centre[:, 0] - x, centre[:, 1] - y)))
        target = centre[(nearest + args.lookahead) % n]
        heading_err = _wrap(_heading_to((x, y), target) - yaw)
        steering_deg = max(-30.0, min(30.0, math.degrees(heading_err) * 0.8))

        wheel_vel, steer_pos = action_to_joint_commands(steering_deg, args.speed)
        wheels_pub.publish(Float64MultiArray(data=wheel_vel))
        steer_pub.publish(Float64MultiArray(data=steer_pos))

        rows.append((time.monotonic() - t0, x, y, yaw, steering_deg))
        time.sleep(dt)

    # stop the car
    wheels_pub.publish(Float64MultiArray(data=[0.0] * 4))
    steer_pub.publish(Float64MultiArray(data=[0.0, 0.0]))

    with open(args.out, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["t", "x", "y", "yaw", "steer_deg"])
        w.writerows(rows)
    print("wrote {} samples to {}".format(len(rows), args.out))

    backend.close()
    node.destroy()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
