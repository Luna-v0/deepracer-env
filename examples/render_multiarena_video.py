#!/usr/bin/env python3
#################################################################################
#   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#################################################################################
"""Render the decoupled multi-arena + DR demo MP4 from recorded sim data.

Consumes ``traj.csv`` + ``dr.json`` from ``multi_arena_record.py`` (real per-arena
trajectories in arena-local frame, and the sampled DR per arena) and the track
centre-line, and animates one panel per arena: the track tinted by that arena's
randomized colour, the car driving from its randomized start, labelled with the
arena's DR. Host-side; needs numpy + matplotlib + ffmpeg.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
from collections import defaultdict

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter, FuncAnimation


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--routes", default="simulation/routes")
    p.add_argument("--world", default="reinvent_base")
    p.add_argument("--data", default="/tmp/dr_drive")
    p.add_argument("--out", default="/tmp/dr_drive/deepracer_multiarena_dr.mp4")
    p.add_argument("--fps", type=int, default=15)
    args = p.parse_args()

    wp = np.load(os.path.join(args.routes, args.world + ".npy"))
    center, inner, outer = wp[:, 0:2], wp[:, 2:4], wp[:, 4:6]
    dr = json.load(open(os.path.join(args.data, "dr.json")))

    per = defaultdict(list)  # arena -> [(x,y,yaw), ...] in record order
    with open(os.path.join(args.data, "traj.csv")) as fh:
        for r in csv.DictReader(fh):
            per[int(r["arena"])].append((float(r["x"]), float(r["y"]), float(r["yaw"])))
    arenas = sorted(per)
    frames = max(len(per[a]) for a in arenas)

    n = len(arenas)
    cols = min(n, 3)
    rows = int(math.ceil(n / cols))
    fig, axes = plt.subplots(rows, cols, figsize=(4.4 * cols, 4.0 * rows), squeeze=False)
    fig.suptitle("DeepRacer-Env · ROS 2 Lyrical + Gazebo Jetty — decoupled multi-arena + domain randomization",
                 fontsize=13, fontweight="bold")

    artists = {}
    for k, a in enumerate(arenas):
        ax = axes[k // cols][k % cols]
        d = dr.get(str(a), {})
        col = d.get("track_color") or [0.8, 0.8, 0.8]
        tint = [0.55 + 0.45 * c for c in col]  # lighten for the surface fill
        ax.fill(np.r_[outer[:, 0], inner[::-1, 0]], np.r_[outer[:, 1], inner[::-1, 1]],
                color=tint, zorder=0)
        for b in (inner, outer):
            ax.plot(np.r_[b[:, 0], b[0, 0]], np.r_[b[:, 1], b[0, 1]], color="#333", lw=1)
        ax.plot(np.r_[center[:, 0], center[0, 0]], np.r_[center[:, 1], center[0, 1]],
                ls="--", color="#888", lw=0.7)
        ax.set_aspect("equal"); ax.set_xticks([]); ax.set_yticks([])
        st = d.get("start_ndist"); bias = d.get("steering_bias_deg", 0.0)
        ddir = "CW" if d.get("reverse_dir") else "CCW"
        ax.set_title("arena {} (racecar_{})\nstart {:.0f}% · {} · bias {:+.1f}°".format(
            a, a, 100 * (st or 0.0), ddir, bias or 0.0), fontsize=9)
        (trail,) = ax.plot([], [], color="#1f77b4", lw=2)
        (car,) = ax.plot([], [], "o", color="#d62728", ms=10)
        head = ax.annotate("", xy=(0, 0), xytext=(0, 0),
                           arrowprops=dict(arrowstyle="->", color="#d62728", lw=2))
        artists[a] = (trail, car, head)
    for k in range(n, rows * cols):
        axes[k // cols][k % cols].axis("off")

    def update(f):
        out = []
        for a in arenas:
            traj = per[a]
            j = min(f, len(traj) - 1)
            xs = [t[0] for t in traj[:j + 1]]; ys = [t[1] for t in traj[:j + 1]]
            x, y, yaw = traj[j]
            trail, car, head = artists[a]
            trail.set_data(xs, ys); car.set_data([x], [y])
            head.set_position((x, y))
            head.xy = (x + 0.4 * math.cos(yaw), y + 0.4 * math.sin(yaw))
            out += [trail, car, head]
        return out

    anim = FuncAnimation(fig, update, frames=frames, interval=1000 / args.fps, blit=False)
    anim.save(args.out, writer=FFMpegWriter(fps=args.fps, bitrate=2400))
    print("wrote", args.out)


if __name__ == "__main__":
    raise SystemExit(main())
