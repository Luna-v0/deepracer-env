#!/usr/bin/env python3
"""Live-sim verification for DeepRacerEnv.set_world() runtime track swap.

Run *inside* the simapp container, after `deepracer_env.launch` has brought up
Gazebo + the car, with PYTHONPATH=/workspace so the EDITED deepracer_env (the
one carrying set_world) shadows the image's installed copy:

    PYTHONPATH=/workspace python3 /workspace/swap_verification/verify_swap.py main
    PYTHONPATH=/workspace python3 /workspace/swap_verification/verify_swap.py oa

`main` covers Phase 3a (assertions), 3b (onboard + top-down PNGs), 3c (drive
mp4) and 3d (multi-swap stress). `oa` covers Phase 3e (object-avoidance swap).
All artifacts + a results JSON land in /workspace/swap_verification/.
"""
import os
import sys
import time
import json
import math

import numpy as np

OUT = os.environ.get("SWAP_OUT", "/workspace/swap_verification")
os.makedirs(OUT, exist_ok=True)

import rospy  # noqa: E402
import rospkg  # noqa: E402
from geometry_msgs.msg import Pose  # noqa: E402
from gazebo_msgs.srv import (  # noqa: E402
    GetWorldProperties, GetModelState, SpawnModel, SetModelState,
)
from gazebo_msgs.msg import ModelState  # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402

import cv2  # noqa: E402
from cv_bridge import CvBridge  # noqa: E402

PKG = "deepracer_simulation_environment"
ONBOARD_TOPIC = "/racecar/camera/zed/rgb/image_rect_color"
TOPCAM_NS = "topcam"
TOPCAM_TOPIC = "/{}/zed/rgb/image_rect_color".format(TOPCAM_NS)

RESULTS = {"phase": None, "checks": [], "artifacts": []}


def log(msg):
    print("[verify] {}".format(msg), flush=True)
    sys.stdout.flush()


def record(name, ok, detail=""):
    RESULTS["checks"].append({"name": name, "ok": bool(ok), "detail": str(detail)})
    log("{} {}: {}".format("PASS" if ok else "FAIL", name, detail))


def route_path(world):
    rp = rospkg.RosPack().get_path(PKG)
    return os.path.join(rp, "routes", "{}.npy".format(world))


# --------------------------------------------------------------------------- #
# Camera capture
# --------------------------------------------------------------------------- #
class Grabber:
    def __init__(self, topic):
        self.bridge = CvBridge()
        self.frame = None
        self.topic = topic
        self.sub = rospy.Subscriber(topic, Image, self._cb, queue_size=1)

    def _cb(self, msg):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as ex:  # noqa: BLE001
            log("cv_bridge error on {}: {}".format(self.topic, ex))

    def grab(self, timeout=20.0, fresh=True):
        if fresh:
            self.frame = None
        t0 = time.time()
        while self.frame is None and time.time() - t0 < timeout:
            time.sleep(0.05)
        return self.frame

    def save(self, path, timeout=20.0):
        f = self.grab(timeout=timeout)
        if f is not None:
            cv2.imwrite(path, f)
            RESULTS["artifacts"].append(os.path.basename(path))
            log("saved {} ({}x{}, mean={:.1f})".format(
                path, f.shape[1], f.shape[0], float(f.mean())))
            return True
        log("NO FRAME for {}".format(path))
        return False


# --------------------------------------------------------------------------- #
# Top-down overhead camera (spawned once, repositioned per track)
# --------------------------------------------------------------------------- #
def topcam_sdf():
    return """<?xml version="1.0"?>
<sdf version="1.6">
  <model name="topcam">
    <static>true</static>
    <link name="camera_link">
      <sensor type="camera" name="cam">
        <update_rate>15.0</update_rate>
        <camera name="cam">
          <horizontal_fov>1.5</horizontal_fov>
          <image><width>720</width><height>720</height><format>R8G8B8</format></image>
          <clip><near>0.1</near><far>500</far></clip>
        </camera>
        <plugin name="cc" filename="libgazebo_ros_camera.so">
          <alwaysOn>true</alwaysOn>
          <updateRate>15</updateRate>
          <cameraName>zed</cameraName>
          <imageTopicName>rgb/image_rect_color</imageTopicName>
          <cameraInfoTopicName>rgb/camera_info</cameraInfoTopicName>
          <frameName>camera_link</frameName>
        </plugin>
      </sensor>
    </link>
  </model>
</sdf>"""


def _downward_pose(cx, cy, z):
    from deepracer_env.track_geom.utils import euler_to_quaternion
    p = Pose()
    p.position.x = cx
    p.position.y = cy
    p.position.z = z
    q = euler_to_quaternion(roll=1.5708, pitch=1.5708, yaw=3.14159)
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q
    return p


def topcam_pose_for(td):
    x_min, y_min, x_max, y_max = td.outer_border.bounds
    cx, cy = (x_min + x_max) / 2.0, (y_min + y_max) / 2.0
    extent = max(x_max - x_min, y_max - y_min)
    # h such that a 1.5 rad fov sees the whole extent with margin.
    z = (0.5 * extent * 1.4) / math.tan(0.75) + 2.0
    return _downward_pose(cx, cy, z)


def spawn_topcam(td):
    rospy.wait_for_service("/gazebo/spawn_sdf_model", timeout=30)
    spawn = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
    spawn("topcam", topcam_sdf(), TOPCAM_NS, topcam_pose_for(td), "")
    log("spawned top-down camera")


def move_topcam(td):
    rospy.wait_for_service("/gazebo/set_model_state", timeout=30)
    setm = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
    ms = ModelState()
    ms.model_name = "topcam"
    ms.pose = topcam_pose_for(td)
    setm(ms)


# --------------------------------------------------------------------------- #
# Gazebo introspection
# --------------------------------------------------------------------------- #
def model_names():
    rospy.wait_for_service("/gazebo/get_world_properties", timeout=30)
    return list(rospy.ServiceProxy(
        "/gazebo/get_world_properties", GetWorldProperties)().model_names)


def waypoints_of(td):
    return np.asarray(td.get_way_pnts(), dtype=float)


# --------------------------------------------------------------------------- #
# Phase 3a-3d
# --------------------------------------------------------------------------- #
def run_main():
    RESULTS["phase"] = "main"
    from deepracer_env.environments.deepracer_env import DeepRacerEnv
    from deepracer_env.track_geom.track_data import TrackData
    from shapely.geometry import Point

    def reward(params):
        return float(params.get("progress", 0.0))

    log("building env on reinvent_base ...")
    env = DeepRacerEnv(reward_fn=reward, sensors=["FRONT_FACING_CAMERA"])
    onboard = Grabber(ONBOARD_TOPIC)
    obs, info = env.reset()
    log("first reset done")

    td0 = TrackData.get_instance()
    spawn_topcam(td0)
    time.sleep(2.0)

    # On-track probe point for reinvent_base (centerline start).
    wp0 = waypoints_of(td0)
    probe = (float(wp0[len(wp0) // 3][0]), float(wp0[len(wp0) // 3][1]))
    d_center_before = td0.center_line.distance(Point(*probe))
    on_track_before = bool(any(td0.points_on_track([Point(*probe)])))
    loop_before, ccw_before = bool(td0.is_loop), bool(td0.is_ccw)
    n_track_before = sum(1 for m in model_names() if m == "racetrack")

    onboard_ok = onboard.save(os.path.join(OUT, "before_reinvent_base.png"))
    record("onboard_before_frame", onboard_ok, "reinvent_base start-line view")
    record("racetrack_count_before", n_track_before == 1, "count={}".format(n_track_before))
    move_topcam(td0)
    time.sleep(1.5)
    Grabber(TOPCAM_TOPIC)  # warm up topic
    time.sleep(1.0)
    top = Grabber(TOPCAM_TOPIC)
    top.save(os.path.join(OUT, "topdown_reinvent_base.png"))

    # ---- swap to arctic_pro ----
    target = "arctic_pro"
    log("set_world({}) ...".format(target))
    t0 = time.time()
    env.set_world(target)
    swap_secs = time.time() - t0
    record("set_world_no_exception", True, "{:.1f}s".format(swap_secs))

    td1 = TrackData.get_instance()
    n_track_after = sum(1 for m in model_names() if m == "racetrack")
    record("racetrack_count_after", n_track_after == 1, "count={}".format(n_track_after))

    route = np.load(route_path(target))
    wp_after = waypoints_of(td1)
    wp_match = (wp_after.shape == route[:, 0:2].shape
                and np.allclose(wp_after, route[:, 0:2]))
    record("waypoints_match_route_file", wp_match,
           "td={} route={}".format(wp_after.shape, route[:, 0:2].shape))

    loop_after, ccw_after = bool(td1.is_loop), bool(td1.is_ccw)
    log("is_loop {}->{}  is_ccw {}->{}".format(
        loop_before, loop_after, ccw_before, ccw_after))

    d_center_after = td1.center_line.distance(Point(*probe))
    on_track_after = bool(any(td1.points_on_track([Point(*probe)])))
    geom_changed = (abs(d_center_after - d_center_before) > 1e-3
                    or on_track_after != on_track_before)
    record("reward_geometry_moved", geom_changed,
           "probe d_center {:.3f}->{:.3f}, on_track {}->{}".format(
               d_center_before, d_center_after, on_track_before, on_track_after))

    obs, info = env.reset()
    onboard.save(os.path.join(OUT, "after_arctic_pro.png"))
    move_topcam(td1)
    time.sleep(1.5)
    top = Grabber(TOPCAM_TOPIC)
    top.save(os.path.join(OUT, "topdown_arctic_pro.png"))

    # ---- 200-step episode on the new track ----
    steps = 0
    for _ in range(200):
        obs, r, term, trunc, info = env.step(np.array([0.0, 1.5], dtype=np.float32))
        steps += 1
        if term or trunc:
            obs, info = env.reset()
    record("episode_200_steps_on_new_track", steps == 200, "ran {} steps".format(steps))

    # ---- 3c: drive mp4 on arctic_pro ----
    mp4 = os.path.join(OUT, "drive_arctic_pro.mp4")
    f = onboard.grab()
    if f is not None:
        h, w = f.shape[:2]
        vw = cv2.VideoWriter(mp4, cv2.VideoWriter_fourcc(*"mp4v"), 15.0, (w, h))
        env.reset()
        nframes = 0
        for _ in range(90):
            env.step(np.array([0.0, 1.5], dtype=np.float32))
            fr = onboard.grab(timeout=2.0)
            if fr is not None:
                vw.write(fr)
                nframes += 1
        vw.release()
        RESULTS["artifacts"].append(os.path.basename(mp4))
        record("drive_mp4_written", nframes > 30, "{} frames".format(nframes))

    # ---- 3d: multi-swap stress (incl. reinvent_base model.sdf + Vegas outlier) ----
    seq = ["Spain_track", "Tokyo_Training_track", "Vegas_track", "reinvent_base"]
    for w in seq:
        log("multi-swap -> {}".format(w))
        env.set_world(w)
        td = TrackData.get_instance()
        route = np.load(route_path(w))
        wps = waypoints_of(td)
        ok = wps.shape == route[:, 0:2].shape and np.allclose(wps, route[:, 0:2])
        n = sum(1 for m in model_names() if m == "racetrack")
        env.reset()
        # run a short episode to surface stale collision geometry
        collided = False
        for _ in range(60):
            _, _, term, _, info = env.step(np.array([0.0, 1.2], dtype=np.float32))
            if info.get("is_crashed"):
                collided = True
            if term:
                env.reset()
        record("swap_{}".format(w), ok and n == 1,
               "waypoints_match={} racetrack_count={} crashed_in_episode={}".format(
                   ok, n, collided))
        move_topcam(td)
        time.sleep(1.5)
        top = Grabber(TOPCAM_TOPIC)
        top.save(os.path.join(OUT, "topdown_{}.png".format(w)))

    env.close()
    finish()


# --------------------------------------------------------------------------- #
# Phase 3e: object-avoidance swap
# --------------------------------------------------------------------------- #
def run_oa():
    RESULTS["phase"] = "oa"
    from deepracer_env.environments.deepracer_env import DeepRacerEnv
    from deepracer_env.object_avoidance import ObjectAvoidanceConfig
    from deepracer_env.object_avoidance.config import PLACEMENT_RANDOM
    from deepracer_env.track_geom.track_data import TrackData

    def reward(params):
        return float(params.get("progress", 0.0))

    track_a = os.environ.get("OA_TRACK_A", "reinvent_base")
    track_b = os.environ.get("OA_TRACK_B", "arctic_pro")
    seed = 7

    oa = ObjectAvoidanceConfig(n_obstacles=3, placement=PLACEMENT_RANDOM)
    log("building OA env on {} ...".format(track_a))
    env = DeepRacerEnv(reward_fn=reward, sensors=["FRONT_FACING_CAMERA"], object_avoidance=oa)
    env.reset(seed=seed)

    mgr = env._obstacle_manager
    names_a = list(mgr.spawned_names)
    td_a = TrackData.get_instance()
    present_a = [n for n in model_names() if n.startswith("obstacle")]
    registered_a = [n for n in td_a.object_poses.keys() if n.startswith("obstacle")]
    record("oa_obstacles_present_before", len(present_a) == 3,
           "gazebo={} registered={}".format(present_a, registered_a))

    # ---- swap to track_b, reset with same seed ----
    env.set_world(track_b)
    env.reset(seed=seed)

    mgr2 = env._obstacle_manager
    # (a) trackA's obstacle models gone from Gazebo? (only trackB's remain)
    present_after = [n for n in model_names() if n.startswith("obstacle")]
    record("oa_stale_models_gone", len(present_after) == 3,
           "obstacles in gazebo after swap: {}".format(present_after))

    # (b) manager bound to NEW TrackData (waypoints == trackB route)
    td_b = TrackData.get_instance()
    mgr_td_is_new = mgr2._track_data is td_b
    routeb = np.load(route_path(track_b))[:, 0:2]
    wb = waypoints_of(td_b)
    record("oa_manager_rebound_to_new_track",
           mgr_td_is_new and wb.shape == routeb.shape and np.allclose(wb, routeb),
           "mgr._track_data is new={} waypoints_match={}".format(
               mgr_td_is_new, wb.shape == routeb.shape))

    # (c) new obstacles lie near trackB centerline, NOT trackA's
    from shapely.geometry import Point
    poses = [p for n, p in td_b.object_poses.items() if n.startswith("obstacle")]
    routea_line = None
    td_a_wp = np.load(route_path(track_a))[:, 0:2]
    max_d_b = 0.0
    for p in poses:
        d = td_b.center_line.distance(Point(p.position.x, p.position.y))
        max_d_b = max(max_d_b, d)
    on_b = max_d_b < 2.0  # obstacles within a lane-width of trackB centerline
    record("oa_obstacles_on_new_track", on_b,
           "max dist to trackB centerline = {:.2f}m".format(max_d_b))

    # ---- determinism: same seed twice -> same layout ----
    env.set_world(track_a)
    env.set_world(track_b)
    env.reset(seed=seed)
    layout1 = sorted([(round(p.position.x, 3), round(p.position.y, 3))
                      for n, p in TrackData.get_instance().object_poses.items()
                      if n.startswith("obstacle")])
    env.set_world(track_a)
    env.set_world(track_b)
    env.reset(seed=seed)
    layout2 = sorted([(round(p.position.x, 3), round(p.position.y, 3))
                      for n, p in TrackData.get_instance().object_poses.items()
                      if n.startswith("obstacle")])
    record("oa_deterministic_layout", layout1 == layout2,
           "layout1={} layout2={}".format(layout1, layout2))

    # ---- photo: top-down of trackB with obstacles ----
    spawn_topcam(td_b)
    move_topcam(td_b)
    time.sleep(2.0)
    top = Grabber(TOPCAM_TOPIC)
    top.save(os.path.join(OUT, "oa_{}.png".format(track_b)))

    env.close()
    finish()


def finish():
    n_ok = sum(1 for c in RESULTS["checks"] if c["ok"])
    n = len(RESULTS["checks"])
    RESULTS["summary"] = "{}/{} checks passed".format(n_ok, n)
    path = os.path.join(OUT, "results_{}.json".format(RESULTS["phase"]))
    with open(path, "w") as fh:
        json.dump(RESULTS, fh, indent=2)
    log("=== {} ===".format(RESULTS["summary"]))
    log("results written to {}".format(path))


if __name__ == "__main__":
    rospy.init_node("swap_verify", anonymous=True, disable_signals=True)
    mode = sys.argv[1] if len(sys.argv) > 1 else "main"
    try:
        if mode == "oa":
            run_oa()
        else:
            run_main()
    except Exception as ex:  # noqa: BLE001
        import traceback
        traceback.print_exc()
        record("FATAL_{}".format(mode), False, repr(ex))
        finish()
        sys.exit(1)
