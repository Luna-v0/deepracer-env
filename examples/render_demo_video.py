#!/usr/bin/env python3
#################################################################################
#   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#################################################################################
"""Render the demo MP4 from a recorded trajectory.

Left panel: the *real* car trajectory from ``examples/drive_demo.py`` (driven
live via ros2_control on Gazebo Jetty) tracing the converted ``reinvent_base``
track. Right 2x2: the same drive replayed across four **decoupled arenas**
(positions phase-shifted, tracks tinted differently) to illustrate the tiled
multi-arena design — built from the real
:class:`~deepracer_env.sim_control.arena.ArenaLayout`.

Runs on the host (no ROS); needs numpy + matplotlib + ffmpeg.
"""
from __future__ import annotations

import argparse
import csv
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter, FuncAnimation

from deepracer_env.sim_control.arena import ArenaLayout


def load_track(routes_dir, world):
    a = np.load(os.path.join(routes_dir, world + ".npy"))
    return a[:, 0:2], a[:, 2:4], a[:, 4:6]  # center, inner, outer


def load_traj(path):
    xs, ys, yaws = [], [], []
    with open(path) as fh:
        for row in csv.DictReader(fh):
            xs.append(float(row["x"])); ys.append(float(row["y"])); yaws.append(float(row["yaw"]))
    return np.array(xs), np.array(ys), np.array(yaws)


def draw_track(ax, center, inner, outer, fill="#d9d9d9"):
    ax.fill(np.r_[outer[:, 0], inner[::-1, 0]], np.r_[outer[:, 1], inner[::-1, 1]],
            color=fill, zorder=0)
    for b in (inner, outer):
        ax.plot(np.r_[b[:, 0], b[0, 0]], np.r_[b[:, 1], b[0, 1]], color="#444", lw=1)
    ax.plot(np.r_[center[:, 0], center[0, 0]], np.r_[center[:, 1], center[0, 1]],
            ls="--", color="#888", lw=0.8)
    ax.set_aspect("equal"); ax.set_xticks([]); ax.set_yticks([])


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--routes", default="simulation/routes")
    p.add_argument("--world", default="reinvent_base")
    p.add_argument("--traj", default="/tmp/dr_drive/traj.csv")
    p.add_argument("--out", default="/tmp/dr_drive/deepracer_jetty_demo.mp4")
    p.add_argument("--fps", type=int, default=15)
    args = p.parse_args()

    center, inner, outer = load_track(args.routes, args.world)
    xs, ys, yaws = load_traj(args.traj)
    n = len(xs)

    fig = plt.figure(figsize=(13, 6.5))
    fig.suptitle("DeepRacer-Env — ROS 2 Lyrical + Gazebo Jetty (gz-sim 10.4) · live ros2_control drive",
                 fontsize=13, fontweight="bold")

    # --- left: live single-car drive --------------------------------------
    axL = fig.add_axes([0.04, 0.06, 0.44, 0.84])
    draw_track(axL, center, inner, outer)
    axL.set_title("Live drive on converted reinvent_base\n(seam teleport → pure-pursuit → ros2_control)",
                  fontsize=10)
    (trail,) = axL.plot([], [], color="#1f77b4", lw=2)
    (carL,) = axL.plot([], [], "o", color="#d62728", ms=10)
    headL = axL.annotate("", xy=(0, 0), xytext=(0, 0),
                         arrowprops=dict(arrowstyle="->", color="#d62728", lw=2))

    # --- right: 4 decoupled arenas ----------------------------------------
    layout = ArenaLayout(4, [args.world], base_seed=7)
    tints = ["#ffe0e0", "#e0ffe0", "#e0e0ff", "#fff3cc"]  # per-arena "visual DR"
    arena_axes, arena_cars = [], []
    for i, arena in enumerate(layout):
        r, c = divmod(i, 2)
        ax = fig.add_axes([0.54 + c * 0.225, 0.50 - r * 0.42, 0.20, 0.36])
        draw_track(ax, center, inner, outer, fill=tints[i])
        ax.set_title("arena {}  ·  {}  ·  seed {}".format(arena.index, arena.car_name, arena.dr_seed),
                     fontsize=8)
        (car,) = ax.plot([], [], "o", color="#d62728", ms=7)
        arena_axes.append(ax); arena_cars.append(car)
    fig.text(0.765, 0.94, "Decoupled multi-arena: 4 cars · 1 simulator · independent track+DR+episode",
             ha="center", fontsize=10, fontweight="bold")

    def update(k):
        trail.set_data(xs[:k + 1], ys[:k + 1])
        carL.set_data([xs[k]], [ys[k]])
        d = 0.4
        headL.set_position((xs[k], ys[k]))
        headL.xy = (xs[k] + d * np.cos(yaws[k]), ys[k] + d * np.sin(yaws[k]))
        for i, car in enumerate(arena_cars):
            j = (k + i * n // 4) % n  # phase-shift: independent episodes
            car.set_data([xs[j]], [ys[j]])
        return [trail, carL, headL, *arena_cars]

    anim = FuncAnimation(fig, update, frames=n, interval=1000 / args.fps, blit=False)
    anim.save(args.out, writer=FFMpegWriter(fps=args.fps, bitrate=2400))
    print("wrote", args.out)


if __name__ == "__main__":
    raise SystemExit(main())
