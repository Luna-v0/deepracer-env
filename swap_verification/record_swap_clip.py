#!/usr/bin/env python3
"""Record ONE continuous onboard-camera clip while the track swaps under the car.

This is the "model's-eye view of the world changing" video: a background thread
samples the car's front camera at a steady 30 fps (so it keeps recording even
during the ~1 s physics-pause of a swap), while the main thread drives a few
seconds on each world and calls ``env.set_world(next)`` between them. The result
is a single mp4 where the track visibly morphs from one layout to the next with
no cut.

Run inside the simapp container (sim already up), with the edited package on the
path and WORLD_NAME set:

    rosparam set WORLD_NAME reinvent_base
    PYTHONPATH=/workspace:$PYTHONPATH python3 swap_verification/record_swap_clip.py
"""
import os
import sys
import time
import threading

import numpy as np
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

OUT = os.environ.get("SWAP_OUT", "/workspace/swap_verification")
ONBOARD_TOPIC = "/racecar/camera/zed/rgb/image_rect_color"
# Worlds to walk through, in order. First one is whatever Gazebo loaded.
WORLDS = os.environ.get(
    "CLIP_WORLDS", "reinvent_base,arctic_pro,Spain_track,reinvent_base"
).split(",")
SECONDS_PER_WORLD = float(os.environ.get("CLIP_SECONDS", "4.0"))
FPS = 30

_bridge = CvBridge()
_state = {"frame": None, "label": "", "banner": ""}


def _cam_cb(msg):
    try:
        _state["frame"] = _bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    except Exception as ex:  # noqa: BLE001
        print("cv_bridge error:", ex, flush=True)


# Width of the info sidebar drawn to the RIGHT of the camera image.
SIDEBAR = 300


def _fmt(elapsed):
    m, s = divmod(elapsed, 60.0)
    return "{:02d}:{:05.2f}".format(int(m), s)


def _put(canvas, text, org, scale, color, thick=2):
    cv2.putText(canvas, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color,
                thick, cv2.LINE_AA)


def _compose(img, label, banner, elapsed):
    """Overlay the wall clock + world label directly on the camera frame."""
    canvas = img.copy()
    h, w = canvas.shape[:2]
    # wall clock, top-right
    clk = "T+ " + _fmt(elapsed)
    (tw, _th), _bl = cv2.getTextSize(clk, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
    cv2.rectangle(canvas, (w - tw - 22, 0), (w, 34), (0, 0, 0), -1)
    _put(canvas, clk, (w - tw - 12, 23), 0.6, (255, 255, 255), 2)
    # current world, bottom strip
    cv2.rectangle(canvas, (0, h - 34), (w, h), (0, 0, 0), -1)
    _put(canvas, "WORLD: {}".format(label), (10, h - 11), 0.6, (0, 255, 0), 2)
    # transient swap banner, top-left
    if banner:
        cv2.rectangle(canvas, (0, 0), (w - tw - 24, 34), (0, 0, 0), -1)
        _put(canvas, banner, (10, 23), 0.55, (0, 200, 255), 2)
    return canvas


class Recorder(threading.Thread):
    """Append the latest camera frame to the mp4 at a fixed wall-clock rate."""

    def __init__(self, path):
        super().__init__(daemon=True)
        self.path = path
        self.writer = None
        self.size = None
        self._stop_evt = threading.Event()
        self.n = 0

    def run(self):
        period = 1.0 / FPS
        start = None
        while not self._stop_evt.is_set():
            t0 = time.time()
            f = _state["frame"]
            if f is not None:
                if self.writer is None:
                    h, w = f.shape[:2]
                    self.size = (w, h)
                    self.writer = cv2.VideoWriter(
                        self.path, cv2.VideoWriter_fourcc(*"mp4v"), FPS, self.size)
                    start = t0
                frame = _compose(f, _state["label"], _state["banner"], t0 - start)
                self.writer.write(frame)
                self.n += 1
            dt = period - (time.time() - t0)
            if dt > 0:
                time.sleep(dt)

    def stop(self):
        self._stop_evt.set()
        self.join(timeout=5)
        if self.writer is not None:
            self.writer.release()


def main():
    rospy.init_node("record_swap_clip", anonymous=True, disable_signals=True)
    rospy.Subscriber(ONBOARD_TOPIC, Image, _cam_cb, queue_size=1)

    from deepracer_env.environments.deepracer_env import DeepRacerEnv

    def reward(params):
        return float(params.get("progress", 0.0))

    print("building env on {} ...".format(WORLDS[0]), flush=True)
    env = DeepRacerEnv(reward_fn=reward, sensors=["FRONT_FACING_CAMERA"])
    env.reset()
    # wait for the first frame
    t0 = time.time()
    while _state["frame"] is None and time.time() - t0 < 20:
        time.sleep(0.05)

    out_path = os.path.join(OUT, "model_view_track_swap.mp4")
    rec = Recorder(out_path)
    rec.start()

    drive = np.array([0.0, 1.2], dtype=np.float32)
    for i, world in enumerate(WORLDS):
        if i > 0:
            _state["banner"] = "set_world('{}')  ...".format(world)
            print("swapping -> {}".format(world), flush=True)
            env.set_world(world)
            env.reset()
            _state["banner"] = ""
        _state["label"] = world
        # drive for SECONDS_PER_WORLD wall-clock seconds
        t_end = time.time() + SECONDS_PER_WORLD
        while time.time() < t_end:
            _, _, term, trunc, _ = env.step(drive)
            if term or trunc:
                env.reset()

    time.sleep(0.5)
    rec.stop()
    env.close()
    print("wrote {} ({} frames, {:.1f}s)".format(
        out_path, rec.n, rec.n / float(FPS)), flush=True)


if __name__ == "__main__":
    try:
        main()
    except Exception as ex:  # noqa: BLE001
        import traceback
        traceback.print_exc()
        sys.exit(1)
