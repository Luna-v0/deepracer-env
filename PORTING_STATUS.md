# ROS 1 Noetic + Gazebo 11 → ROS 2 Lyrical + Gazebo Jetty — Porting Status

Branch: `feat/ros2-lyrical-gazebo-jetty-port` (deepracer-env) ·
`feat/ros2-lyrical-jetty-base` (dr-gym). Build/test path: **Docker**.
Target: Ubuntu 26.04 · ROS 2 Lyrical · **Gazebo Jetty (gz-sim 10.4.0, DART)** · Python 3.12.

This file tracks what is **done & verified**, what is **authored**, and what
**remains**. See the mkdocs site under `docs/` for the architecture.

## Done & verified in a real Gazebo Jetty container

| Area | Evidence |
|---|---|
| **Substrate image** | `docker/Dockerfile.base` builds `deepracer-env-build-core:lyrical`; `gz sim --version` = 10.4.0. Every `ros-lyrical-*` package resolves on apt. |
| **gz control plane** | `create` / `set_pose` / `control` (pause + deterministic `multi_step`) / `pose/info` all exercised end-to-end via `gz service`/`gz topic`. |
| **SimControl seam** | `deepracer_env/sim_control/` — ports-&-adapters; `RosGzBackend` is the working backend. 18 host unit tests green (`sim_control/tests/`). |
| **World conversion** | `scripts/world_converter.py` — 127 worlds converted to Jetty SDF; `reinvent_base.sdf` loads as a controllable world (racetrack mesh resolved, control service up). 6 host tests green. |
| **Robot + drive** | `urdf/deepracer/deepracer_gz.urdf.xacro` + `config/ros2_control.yaml`; in-container the car spawns, all 3 controllers (`joint_state_broadcaster`, `wheels_velocity_controller`, `steering_position_controller`) go **active**, and the car **drives** (pose x: -0.0005 → 8.45). |
| **End-to-end demo** | `examples/drive_demo.py` teleports the car onto the track via the seam and drives it around `reinvent_base` (pure-pursuit + `drive.py` + ros2_control), logging 134 real pose samples → `examples/render_demo_video.py` → the demo MP4. |
| **ament build** | `colcon build` of `deepracer_simulation_environment` (ament_cmake) succeeds; the 3-stage image chain (`Dockerfile.base/build/runtime`) builds. |
| **Docs** | mkdocs site (`docs/`, `mkdocs.yml`): architecture, the seam, multi-arena, DR catalog, contract, drive, world conversion, deployment, + ADR 0001 (Route B) and ADR 0002 (stack). |
| **dr-gym** | `Dockerfile` (noetic→lyrical, `ros2 launch`, WORLD_NAME via env) + `bootstrap.sh` (tracks the port branch); `export_bundle.py` unchanged. |

## Key decisions (see ADRs)

- **Route B**: the ~1,700-LOC Gazebo-classic C++ `SystemPlugin` and `deepracer_msgs`
  are **deleted** (`COLCON_IGNORE`d). Model/link/pose/pause use native gz services;
  visual + lighting DR use native `/world/<w>/visual_config` + `/light_config` —
  so **zero custom C++** remains.
- **Drive**: `forward_command_controller` groups (ros2_controllers 6.7 dropped
  `velocity_controllers`/`position_controllers`), commanded with `Float64MultiArray`.
- **Decoupled multi-arena** is a first-class pillar: every seam verb is per-entity;
  `ArenaLayout` (host-tested) tiles N tracks with per-arena origin/seed and an
  arena-local reward frame.

## Authored, pending in-stack verification

- ROS 2 launch `launch/deepracer_env.launch.py` — encodes the verified bring-up
  sequence (gz + rsp + spawn + bridge + controller spawners with `--param-file`).
  Needs a run via `ros2 launch` once the package is installed in the image.

## Remaining work (next sessions)

1. **Env-module rclpy port (the bulk of Phase 4).** The public `gymnasium.Env`
   surface is frozen and the seam it needs is done + dogfooded, but the 22
   `deepracer_env` modules that still `import rospy` must be switched to the seam
   / `rclpy_client`:
   - `environments/deepracer_env.py` (reset/step/pose/set_world) → `SimControl`.
   - `agent_ctrl/rollout_agent_ctrl.py` → publish via `drive.action_to_joint_commands`
     to the ros2_control topics (the math is already extracted + tested in
     `agent_ctrl/drive.py`).
   - `sensors/sensors_rollout.py`, `cameras/` → rclpy subscriptions on the bridged
     topics (decode logic unchanged).
   - `environments/world_swap.py`, `object_avoidance/` → `SimControl.spawn/delete_entity`.
   - `track_geom/`, `reset/` are pure-numpy — only the `rospy.get_param` /
     `rospkg` lookups change (`ament_index_python`).
2. **AWS/kinesis strip** — delete `deepracer_env/boto/`, the `aws_utils`, the
   `mp4_saving` KVS paths, and the surgical removals catalogued in the audit
   (move `ModelMetadataKeys` out of `boto/` first).
3. **DR consolidation (Phase 7)** — one per-arena `DomainRandomizer` over the full
   catalog (start pos/dir, friction, visual recolor, lighting, sensor noise,
   steering/motor), surfacing applied-DR + ground-truth feature vector in the
   observation `info` as labels for the camera→feature dataset.
4. **Live N-car multi-arena** — per-car namespaced controller managers so multiple
   cars drive via ros2_control in one process (the `ArenaLayout` + per-entity seam
   are ready; the controller-namespacing is the remaining piece).
5. **Behavioral parity (Phase 7 gate)** — old-vs-new lap comparison on
   `reinvent_base`; tune friction/drive gains.
6. **Headless camera rendering** — the camera/LiDAR sensors are defined in the
   URDF, but rendering them headless needs EGL/GPU (OGRE2 won't render under
   software GL in the plain container). Required for the camera-observation
   dataset; run on a GPU host or with an EGL-capable base.
