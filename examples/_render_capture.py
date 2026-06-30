#!/usr/bin/env python3
#################################################################################
#   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#################################################################################
"""Capture bridged camera image topics to PNG frames (for the real-render demo).

Subscribes to one or more `sensor_msgs/Image` topics (gz camera sensors bridged
via ros_gz_image) and saves the latest frame from each — composited side by
side — as numbered PNGs under ``--out`` at ``--rate``. A throwaway harness for
the GPU render video; ``ffmpeg`` then turns the frames into MP4.
"""
from __future__ import annotations

import argparse
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from PIL import Image as PILImage


def _to_rgb(msg: Image) -> np.ndarray:
    """sensor_msgs/Image -> HxWx3 uint8 RGB (handles row stride + rgb/bgr)."""
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    row = msg.step if msg.step else msg.width * 3
    img = buf[: row * msg.height].reshape(msg.height, row)[:, : msg.width * 3]
    img = img.reshape(msg.height, msg.width, 3)
    if (msg.encoding or "rgb8").lower().startswith("bgr"):
        img = img[:, :, ::-1]
    return img


class Capture(Node):
    def __init__(self, topics):
        super().__init__("render_capture")
        self._topics = topics
        self._latest = {t: None for t in topics}
        self._count = 0
        # Image topics are best-effort (sensor QoS); a reliable sub gets nothing.
        for t in topics:
            self.create_subscription(Image, t, self._mk_cb(t), qos_profile_sensor_data)

    def _mk_cb(self, topic):
        def cb(msg):
            try:
                self._latest[topic] = _to_rgb(msg)
                self._count += 1
            except Exception:  # noqa: BLE001
                pass
        return cb

    def composite(self):
        frames = [self._latest[t] for t in self._topics]
        if any(f is None for f in frames):
            return None
        h = min(f.shape[0] for f in frames)
        return np.concatenate([f[:h] for f in frames], axis=1) if len(frames) > 1 else frames[0]


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--topics", required=True)
    p.add_argument("--out", default="/out/frames")
    p.add_argument("--rate", type=float, default=15.0)
    p.add_argument("--duration", type=float, default=20.0)
    args = p.parse_args()
    os.makedirs(args.out, exist_ok=True)

    rclpy.init()
    node = Capture([t for t in args.topics.split(",") if t])
    t0 = time.monotonic()
    last_save = 0.0
    i = 0
    period = 1.0 / args.rate
    while time.monotonic() - t0 < args.duration:
        rclpy.spin_once(node, timeout_sec=0.02)
        now = time.monotonic()
        if now - last_save >= period:
            frame = node.composite()
            if frame is not None:
                PILImage.fromarray(frame).save(os.path.join(args.out, "f{:05d}.png".format(i)))
                i += 1
            last_save = now
    print("received {} msgs, saved {} frames".format(node._count, i), flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
