#!/usr/bin/env python3
#################################################################################
#   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#################################################################################
"""Record a decoupled multi-arena + domain-randomization episode (real sim data).

Run inside the container after the multi-car stack is up (N namespaced cars +
controllers — see multi_arena_record_bringup.sh). For each arena it:

* samples that arena's domain randomization with the real per-arena
  :class:`DomainRandomizer` (independent seed) — a random start position, a
  CW/CCW direction, and a track colour;
* recolours that arena's track and teleports its car to the random start, both
  through the :class:`SimControl` seam (per-entity — decoupled);
* drives the car around its own track with a small pure-pursuit law
  (:func:`deepracer_env.agent_ctrl.drive.action_to_joint_commands`), publishing
  to ``/racecar_i/...`` ros2_control topics;
* logs every car's pose each tick.

Writes ``traj.csv`` (t, car, x, y, yaw in arena-LOCAL frame) and ``dr.json`` (the
sampled DR per arena) for ``render_multiarena_video.py``.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import time

import numpy as np
import rclpy
from std_msgs.msg import Float64MultiArray

from deepracer_env.agent_ctrl.drive import action_to_joint_commands
from deepracer_env.domain_randomizations.spec import RandomizationSpec
from deepracer_env.domain_randomizations.domain_randomizer import DomainRandomizer
from deepracer_env.sim_control.arena import ArenaLayout
from deepracer_env.sim_control.backends.ros_gz_backend import RosGzBackend
from deepracer_env.sim_control.rclpy_client import SimNode
from deepracer_env.sim_control.types import EntityState, Pose


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--world", default="reinvent_base")
    p.add_argument("--n_cars", type=int, default=int(os.environ.get("N_CARS", "3")))
    p.add_argument("--spacing", type=float, default=300.0)
    p.add_argument("--routes", default="/ws/simulation/routes")
    p.add_argument("--out", default="/out")
    p.add_argument("--duration", type=float, default=20.0)
    p.add_argument("--speed", type=float, default=1.6)
    p.add_argument("--lookahead", type=int, default=8)
    args = p.parse_args()

    centre = np.load("{}/{}.npy".format(args.routes, args.world))[:, :2]
    n_wp = len(centre)
    layout = ArenaLayout(args.n_cars, [args.world], spacing=args.spacing,
                         car_name_fmt="racecar_{index}", track_entity_fmt="racetrack_{index}")

    # Full DR catalog so the video shows it; each arena gets its own seed.
    spec = RandomizationSpec(visual_recolor=True, lighting=True, start_position=True,
                             direction=True, steering_bias=True, steering_bias_deg=3.0)

    rclpy.init()
    node = SimNode("multi_arena_record")
    sim = RosGzBackend(args.world)

    pubs = {}      # car -> (wheels_pub, steering_pub)
    dr = {}        # car -> sampled EpisodeRandomization
    for arena in layout:
        ns = arena.car_name
        pubs[ns] = (
            node.create_publisher(Float64MultiArray, "/{}/wheels_velocity_controller/commands".format(ns), 10),
            node.create_publisher(Float64MultiArray, "/{}/steering_position_controller/commands".format(ns), 10),
        )
        ep = DomainRandomizer(spec, np.random.default_rng(arena.dr_seed),
                              track_entity_name=arena.track_entity_name).sample()
        dr[ns] = ep
        # recolour this arena's track (best-effort; native gz visual_config)
        if ep.track_color is not None:
            try:
                # reinvent_base's track model is one link 'fullLink' / visual
                # 'visual' (verified) — recolor that for a visible per-arena tint.
                sim.set_visual_color(arena.track_entity_name, "fullLink", "visual", ep.track_color,
                                     ambient=ep.track_color)
            except Exception:  # noqa: BLE001
                pass
        # teleport the car to its random start (arena origin + local start pose)
        i0 = int((ep.start_ndist or 0.0) * n_wp) % n_wp
        sx, sy = centre[i0]
        yaw = math.atan2(centre[(i0 + args.lookahead) % n_wp][1] - sy,
                         centre[(i0 + args.lookahead) % n_wp][0] - sx)
        if ep.reverse_dir:
            yaw = _wrap(yaw + math.pi)
        sim.set_entity_state(arena.car_name, EntityState(
            pose=Pose.at(sx + arena.origin.x, sy + arena.origin.y, 0.06, yaw)))
    time.sleep(1.0)

    rows = []
    t0 = time.monotonic()
    dt = 0.1
    while time.monotonic() - t0 < args.duration:
        sim.refresh_state()
        for arena in layout:
            ns = arena.car_name
            try:
                st = sim.get_entity_state(ns)
            except Exception:  # noqa: BLE001
                continue
            # arena-local pose (decoupled reward/geometry frame)
            lx, ly = st.pose.position.x - arena.origin.x, st.pose.position.y - arena.origin.y
            yaw = st.pose.orientation.yaw
            nearest = int(np.argmin(np.hypot(centre[:, 0] - lx, centre[:, 1] - ly)))
            tx, ty = centre[(nearest + args.lookahead) % n_wp]
            steer = math.degrees(_wrap(math.atan2(ty - ly, tx - lx) - yaw)) * 0.8
            steer = max(-30.0, min(30.0, steer + math.degrees(dr[ns].steering_bias_rad)))
            wv, sp = action_to_joint_commands(steer, args.speed)
            pubs[ns][0].publish(Float64MultiArray(data=wv))
            pubs[ns][1].publish(Float64MultiArray(data=sp))
            rows.append((round(time.monotonic() - t0, 3), arena.index, round(lx, 4), round(ly, 4), round(yaw, 4)))
        time.sleep(dt)

    # stop
    for ns in pubs:
        pubs[ns][0].publish(Float64MultiArray(data=[0.0] * 4))
        pubs[ns][1].publish(Float64MultiArray(data=[0.0, 0.0]))

    with open(os.path.join(args.out, "traj.csv"), "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["t", "arena", "x", "y", "yaw"])
        w.writerows(rows)
    with open(os.path.join(args.out, "dr.json"), "w") as fh:
        json.dump({str(layout.get(i).index): layout_dr(dr, layout, i) for i in range(len(layout))}, fh, indent=2)
    print("recorded {} pose rows for {} arenas".format(len(rows), len(layout)), flush=True)

    sim.close()
    node.destroy()
    rclpy.shutdown()


def layout_dr(dr, layout, i):
    ep = dr[layout.get(i).car_name]
    return {
        "start_ndist": ep.start_ndist,
        "reverse_dir": ep.reverse_dir,
        "steering_bias_deg": math.degrees(ep.steering_bias_rad),
        "track_color": None if ep.track_color is None else
        [round(ep.track_color.r, 3), round(ep.track_color.g, 3), round(ep.track_color.b, 3)],
    }


if __name__ == "__main__":
    raise SystemExit(main())
