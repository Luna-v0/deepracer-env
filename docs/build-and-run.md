# Build & run (Docker)

Docker is the supported build and test path for the modernized DeepRacer
simulation stack. This page shows how the container images fit together, how to
build the base image, how worlds and meshes reach the simulator through
`GZ_SIM_RESOURCE_PATH`, how to launch a world headless or with a GUI, and how to
pick a simulator backend with `DR_SIM_BACKEND`.

For the bigger picture see the [architecture overview](architecture/overview.md).
For what the env actually does once a world is running, see
[the SimControl seam](architecture/sim-control-seam.md).

## The target stack

The container targets a single, modern stack. Everything below is delivered by
the ROS apt repository — there is **no osrf Gazebo apt list** (that path is
unsupported on Lyrical).

| Component | Version | How it arrives |
| --- | --- | --- |
| OS | Ubuntu 26.04 "resolute" | `FROM ros:lyrical-ros-base` |
| ROS | ROS 2 Lyrical Luth | base image |
| Simulator | Gazebo Jetty (`gz-sim` 10.4.0) | `ros-lyrical-ros-gz-sim` (pulls `ros-lyrical-gz-sim-vendor` transitively) |
| Physics | DART 6.16.6 | bundled with `gz-sim` |
| ROS↔gz bridge | `ros_gz` 3.0.9 | `ros-lyrical-ros-gz-bridge`, `ros-lyrical-ros-gz-image` |
| Drive stack | `ros2_control` 6.7 + `gz_ros2_control` | `ros-lyrical-ros2-control*`, `ros-lyrical-gz-ros2-control` |
| Python | 3.12 (in-container) | base image |

> The physical car is a **separate** target. It runs Foxy/Jazzy, not Lyrical,
> and that is expected — see [Deployment is a separate path](#deployment-is-a-separate-path).

## The three-stage image chain

The Docker build is split into three stages so the slow, rarely-changing
substrate is cached independently of the fast, frequently-changing workspace.

```
ros:lyrical-ros-base
        │  docker/Dockerfile.base
        ▼
deepracer-env-build-core:lyrical      ← Stage 1: BASE (substrate only)
        │  docker/Dockerfile.build
        ▼
deepracer-env-build-bundle            ← Stage 2: BUILD (colcon-compiled workspace)
        │  docker/Dockerfile.runtime
        ▼
deepracer-env (runnable)              ← Stage 3: RUNTIME (slim, with install/ overlay)
```

| Stage | Dockerfile | What it contains | Why it is separate |
| --- | --- | --- | --- |
| **base** | `docker/Dockerfile.base` | Jetty + `ros2_control` + the `ros_gz` bridge + `simulation_interfaces` + the colcon/ament toolchain. **No DeepRacer code.** | Substrate changes rarely; cache it so workspace rebuilds stay cheap. |
| **build** | `docker/Dockerfile.build` | The DeepRacer workspace copied on top of the base, compiled with `rosdep` + `colcon build` into `install/`. | The workspace changes constantly; rebuild only this layer. |
| **runtime** | `docker/Dockerfile.runtime` | A slim runnable image: the compiled bundle plus the entrypoint that sources the overlay. | Ship/run without the build toolchain. |

> **Port status.** Stage 1 (`docker/Dockerfile.base`) is ported and verified on
> Lyrical/Jetty. The base header (`docker/Dockerfile.base`) is the canonical
> description of the substrate; the build and runtime stages follow the same
> chain and source whatever `ROS_DISTRO` the base sets (`lyrical`).

## Build the base image

Run from the repository root (the build context is `.`):

```bash
docker build -f docker/Dockerfile.base -t deepracer-env-build-core:lyrical .
```

What this stage does (see `docker/Dockerfile.base`):

- Starts from `ros:lyrical-ros-base` and pins `ROS_DISTRO=lyrical`.
- Installs the simulation substrate by **ROS apt packages**, not by exact
  version — Lyrical is a single LTS track, so `ros-lyrical-*` already pins the
  supported set. Gazebo Jetty arrives via `ros-lyrical-ros-gz-sim` (which pulls
  `ros-lyrical-gz-sim-vendor` transitively).
- Installs `ros-lyrical-simulation-interfaces` — the simulator-agnostic standard
  used by the future backend (see [DR_SIM_BACKEND](#choosing-a-backend-dr_sim_backend)).
- Refreshes `rosdep` for the Lyrical list so the build stage's `rosdep install`
  resolves.
- Sets headless-render defaults:

  ```dockerfile
  ENV GZ_IP=127.0.0.1 \
      LIBGL_ALWAYS_SOFTWARE=1
  ```

The convenience wrapper `build.sh` builds the same image (it tags it
`<prefix>/deepracer-env-build-core:latest` and then chains the build/runtime
stages); the explicit `docker build` above is the minimal substrate build.

## Worlds and resources: `GZ_SIM_RESOURCE_PATH`

Gazebo resolves `model://…` URIs inside a world file through
`GZ_SIM_RESOURCE_PATH` — exactly as the classic stack used `GAZEBO_MODEL_PATH`.
Point it at the directory that **contains `models/` and `meshes/`**. In this
repo that directory is `simulation/`:

```
simulation/
├── worlds/   ← .world (classic) and converted .sdf (Jetty)
├── models/   ← model:// targets
└── meshes/   ← .dae geometry (renderer-agnostic, unchanged by the port)
```

### Convert classic worlds first

Gazebo Jetty needs **SDF** worlds with physics and the system plugins wired in;
the shipped `.world` files are classic templates that carry neither. Convert
them with the host-testable batch tool `scripts/world_converter.py` (127 worlds
already converted and load-verified in Jetty):

```bash
# Convert the whole worlds/ directory in place (.world -> .sdf)
python3 scripts/world_converter.py

# Or a single world by name (no extension)
python3 scripts/world_converter.py --world 2022_april_open
```

Conversion injects the five system plugins (`Physics`, `UserCommands`,
`SceneBroadcaster`, `Contact`, `Sensors`) plus `<physics>` and `<gravity>`, and
swaps the classic `model://sun` include for a native directional light. Those
plugins are what make the `/world/<world>/…` services and the
`/world/<world>/pose/info` topic exist — without them a raw load gives you a
static, uncontrollable scene. Details: `scripts/world_converter.py`.

### Mount the resources into the container

Bind-mount `simulation/` and export `GZ_SIM_RESOURCE_PATH` to its in-container
path:

```bash
docker run --rm -it \
  -v "$PWD/simulation:/sim" \
  -e GZ_SIM_RESOURCE_PATH=/sim \
  deepracer-env-build-core:lyrical \
  gz sim -s -r /sim/worlds/2022_april_open.sdf
```

The base image's inherited ROS entrypoint already sources
`/opt/ros/lyrical/setup.bash`, so the `gz` CLI is on `PATH`.

## Run a world: headless vs GUI

Gazebo Jetty splits into a **server** (`gz sim -s`, physics + sensors, no
window) and a **GUI** (`gz sim`, adds the OGRE-next render window). Training and
CI use the server; the GUI is only for inspection and demo recording.

### Headless (server only)

This is the default for training and tests. No X server is required; the base
image sets `LIBGL_ALWAYS_SOFTWARE=1` for a software-GL path.

```bash
# inside the container, with GZ_SIM_RESOURCE_PATH set
gz sim -s -r /sim/worlds/2022_april_open.sdf
```

- `-s` — server only (headless).
- `-r` — run immediately instead of starting paused. Omit `-r` if you want the
  env to drive stepping deterministically through the control service (see
  [Verify the control plane](#verify-the-control-plane)).

### GUI

Forward an X server from a Linux host:

```bash
xhost +local:root
docker run --rm -it \
  -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$PWD/simulation:/sim" -e GZ_SIM_RESOURCE_PATH=/sim \
  deepracer-env-build-core:lyrical \
  gz sim -r /sim/worlds/2022_april_open.sdf      # no -s -> GUI + server
```

For a windowless GUI (e.g. recording video in CI), run it under `xvfb`:

```bash
xvfb-run -s "-screen 0 1280x720x24" gz sim -r /sim/worlds/2022_april_open.sdf
```

### GPU rendering

Camera and LiDAR sensors render through OGRE-next. Software GL works but is slow.
For GPU rendering, run with the NVIDIA container runtime and override the
software-GL default:

```bash
docker run --rm -it --gpus all \
  -e LIBGL_ALWAYS_SOFTWARE=0 \
  -v "$PWD/simulation:/sim" -e GZ_SIM_RESOURCE_PATH=/sim \
  deepracer-env-build-core:lyrical \
  gz sim -s -r /sim/worlds/2022_april_open.sdf
```

The runtime stage ships the NVIDIA EGL vendor config
(`docker/files/10_nvidia.json`) for this path.

## Verify the control plane

A converted world should expose its `gz-transport` services and the pose topic.
This is the **cold path** the `RosGzBackend` drives through the `gz service` /
`gz topic` CLIs (`deepracer_env/sim_control/backends/ros_gz_backend.py`); the
hot path — wheel commands and sensor frames — bypasses it on `ros2_control` and
bridged ROS topics.

```bash
# list the control-plane services for the running world
gz service -l | grep '/world/'
#   /world/<world>/create        gz.msgs.EntityFactory -> gz.msgs.Boolean
#   /world/<world>/remove        gz.msgs.Entity
#   /world/<world>/set_pose      gz.msgs.Pose            (pose only; twist is finite-differenced)
#   /world/<world>/control       gz.msgs.WorldControl    (pause / deterministic multi_step)
#   /world/<world>/visual_config gz.msgs.Visual          (native recolour)
#   /world/<world>/light_config  gz.msgs.Light           (native lighting)

# poses stream on a topic (not a service)
gz topic -l | grep pose/info        # /world/<world>/pose/info
```

Deterministic stepping (confirmed on `gz-sim` 10.4.0) — pause, then advance N
steps:

```bash
gz service -s /world/<world>/control \
  --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 3000 \
  --req 'pause: true, multi_step: 200'
```

If these are missing, the world was loaded without the system plugins — re-run
`scripts/world_converter.py` and reload the `.sdf`.

## Choosing a backend: `DR_SIM_BACKEND`

The env reaches the simulator through one swappable adapter, chosen by
`make_sim_control` in `deepracer_env/sim_control/factory.py`. Set
`DR_SIM_BACKEND` to force a choice:

| `DR_SIM_BACKEND` | Adapter | Status |
| --- | --- | --- |
| `ros_gz` | `RosGzBackend` (drives `gz service` / `gz topic`) | **Working default.** |
| `simulation_interfaces` | `SimulationInterfacesBackend` | Future primary, **dormant** — nothing serves the `simulation_interfaces` ROS services on `ros_gz` 3.0.9 yet. |
| `null` | `NullSimControl` | Null Object for host tests (no simulator). |

Selection order (from `factory.py`):

1. The `DR_SIM_BACKEND` environment variable, if set.
2. The `prefer=` argument to `make_sim_control`.
3. Auto: `simulation_interfaces` **only if** its messages *and* a live server
   are both present; otherwise `ros_gz`.

```bash
# force the working gz-transport backend (the default today)
export DR_SIM_BACKEND=ros_gz

# run host-only unit tests with no simulator
DR_SIM_BACKEND=null pytest
```

The `null` backend is what lets the seam, [tiled multi-arena](architecture/multi-arena.md),
and the [drive layer](architecture/drive-and-control.md) be unit-tested off a
live simulator.

## Build & runtime stages

Once the base image exists, the remaining two stages turn the workspace into a
runnable image:

- **Build** (`docker/Dockerfile.build`) copies the workspace on top of
  `deepracer-env-build-core`, runs `rosdep install` and `colcon build`, and
  produces the compiled `install/` overlay.
- **Runtime** (`docker/Dockerfile.runtime`) assembles a slim image from the
  compiled bundle. Its entrypoint (`docker/files/entrypoint.sh`) sources both
  the ROS distro and the overlay so every child process inherits the right
  environment:

  ```bash
  source /opt/ros/${ROS_DISTRO}/setup.bash   # ROS_DISTRO=lyrical
  source /opt/simapp/setup.bash              # the colcon overlay
  exec "$@"
  ```

`build.sh` wraps all three stages end to end (`-f` forces `--no-cache`,
`-p <prefix>` overrides the image prefix, `-a "cpu gpu"` selects runtime
variants).

## Deployment is a separate path

The Docker workflow on this page is **sim and training only**. Trained policies
export to ONNX and run on the car through a ROS-distro-agnostic inference engine
(OpenVINO / TFLite). The sim-to-real boundary is the **observation/action
contract**, not the ROS ABI — so the physical car running **Foxy/Jazzy** while
the simulator runs **Lyrical** is fine and expected. See the
[architecture overview](architecture/overview.md) for the deployment story.

## Quick reference

```bash
# 1. Build the substrate (Stage 1)
docker build -f docker/Dockerfile.base -t deepracer-env-build-core:lyrical .

# 2. Convert classic worlds to Jetty SDF (host tool, no container needed)
python3 scripts/world_converter.py

# 3. Run a world headless, with resources mounted
docker run --rm -it \
  -v "$PWD/simulation:/sim" -e GZ_SIM_RESOURCE_PATH=/sim \
  deepracer-env-build-core:lyrical \
  gz sim -s -r /sim/worlds/2022_april_open.sdf

# 4. Inspect the control plane
gz service -l | grep '/world/'
gz topic   -l | grep pose/info

# 5. Pick a backend
export DR_SIM_BACKEND=ros_gz     # working default
DR_SIM_BACKEND=null pytest       # host-only tests, no simulator
```
