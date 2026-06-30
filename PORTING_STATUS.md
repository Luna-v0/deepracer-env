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
| **Python control plane ported** | All 22 `deepracer_env` modules moved off rospy/gazebo_msgs/deepracer_msgs/boto onto the SimControl seam + rclpy (26 modules import clean in-container). The `gazebo_tracker` layer is re-homed onto the seam (same API); `runtime.py` holds the shared node + backend singletons; `sim_control/compat.py` provides the gazebo_msgs shims. |
| **gym.make → reset → step** | `DeepRacerEnv` constructs (contract intact), `reset()` teleports the car via the seam, and `step()` reads pose + computes reward/progress + evaluates reset rules — verified end-to-end against live gz Jetty (`examples/_func_test.py`). |
| **AWS/kinesis stripped** | `deepracer_env/boto/` deleted; boto/S3/CloudWatch removed from utils, exception_handler, node monitors; `ModelMetadataKeys` relocated to `agent_ctrl/model_metadata.py`. |
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

The env-module rclpy port and the AWS strip (formerly items 1–2 here) are **done
and verified** — see the table above. Randomization status: **random start
position (`RANDOM_START`) and CW/CCW direction (`RANDOM_DIRECTION`/`ALT_DIR`) are
implemented and survive the port**; the rest of the DR catalog is item 1 below.

1. **DR consolidation (Phase 7)** — DONE: one per-arena `DomainRandomizer`
   (`domain_randomizations/spec.py` + `domain_randomizer.py`) over the full
   catalog — visual recolor (native gz `visual_config`), lighting (native gz
   `light_config`, new `SimControl.set_light`), friction, start position,
   direction, steering bias, motor delay, sensor noise — wired into `RolloutCtrl`
   (per-car seed, scoped to the car's own track entity), with the applied `dr_*`
   labels surfaced in the observation `info` for the camera→feature dataset.
   Verified in-container (random start relocated the car; all `dr_*` in `info`).
   Remaining nuance: per-*episode* friction is sampled + surfaced but not yet
   applied at runtime (needs the gz `wheel_slip` service); the exact track
   link/visual names for recolor are unverified headless (no GPU render).
2. **Live N-car multi-arena** — DONE (authored + verified): `multi_arena.launch.py`
   spawns N decoupled arenas (`ArenaLayout`) in one gz world, each car with its own
   **namespaced** `gz_ros2_control` controller_manager (`/racecar_i/...`). Key fix:
   never set `<controller_manager_name>` to a value containing `/` (gz turns it into
   a node-name remap, which aborts the gz server) — namespace alone yields the FQN.
3. **Behavioral parity (Phase 7 gate)** — old-vs-new lap comparison on
   `reinvent_base`; tune friction/drive gains. (Note: the env step is currently
   paced by the sensor's blocking `get_state`; with `sensors=[]` it free-runs.)
4. **Headless camera rendering** — the camera/LiDAR sensors are defined in the
   URDF, but rendering them headless needs EGL/GPU (OGRE2 won't render under
   software GL in the plain container). Required for the camera-observation
   dataset; run on a GPU host or with an EGL-capable base.
5. **Sensor-obs in-stack test** — `reset/step` is verified with `sensors=[]`
   (control plane). Verifying with `CAMERA`/`LIDAR` obs needs the ros_gz sensor
   bridge + headless rendering (item 4).
