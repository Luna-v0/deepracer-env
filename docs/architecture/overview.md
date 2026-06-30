# Architecture overview

This page is the map. It explains the modernized DeepRacer simulation stack, why
it is built the way it is, how the layers fit together, and where to go next for
detail. If you read one page first, read this one.

## TL;DR

- We moved off ROS 1 Noetic + Gazebo classic and onto **ROS 2 Lyrical Luth +
  Gazebo Jetty**, running in Docker.
- The whole environment talks to the simulator through **one seam**: the
  `SimControl` port in [`deepracer_env/sim_control/`](../../deepracer_env/sim_control/interface.py),
  with swappable backends.
- The **~1,700-line custom C++ Gazebo plugin and its custom services are gone**.
  Zero custom C++ survives. Everything maps to native gz-transport and ROS 2
  services.
- The **training contract is frozen byte-for-byte**: same action space, same
  observation dict, same 26 reward-param keys, same `reset`/`step`/`set_world`
  signatures. Training code (dr-gym) barely notices the move.

## Why we did this: future-proofing over hardware parity

The legacy stack was pinned to a dead platform. ROS 1 is end-of-life, Gazebo
classic is end-of-life, and the custom C++ system plugin was a maintenance
liability that re-implemented things the simulator now does natively.

The guiding decision for this port: **optimize for future-proofing, not for
bit-exact parity with the legacy hardware/sim behavior.** Concretely that means:

- Prefer **native and standard services** (gz-transport, ROS 2
  `simulation_interfaces`) over bespoke plugins, even where a custom plugin would
  reproduce old behavior more literally.
- Concentrate all simulator coupling behind **one swappable seam**, so the next
  platform move is a config change, not a rewrite.
- Keep the **sim-to-real boundary at the obs/action contract**, not at the ROS
  ABI. The physical car does not need to run the same ROS distro as the
  simulator (see [Deployment & sim-to-real](deployment.md)).

The payoff is that the simulator, the ROS distro, and even the backend transport
can each change underneath the environment without touching training code.

## The layered stack

Data and control flow top to bottom. Each arrow is a stable interface; the layer
above never reaches around it.

```
┌──────────────────────────────────────────────────────────────┐
│  Training (dr-gym)                                             │
│  SB3 PPO · VecEnv · CNN / feature-vector policies · export    │
└───────────────────────────┬──────────────────────────────────┘
                            │  gymnasium API (reset / step / set_world)
                            ▼
┌──────────────────────────────────────────────────────────────┐
│  Environment (gymnasium.Env)                                  │
│  DeepRacerEnv  ·  MultiAgentDeepRacerEnv                       │
│  reward params · reset rules · sensors · domain randomization  │
└───────────────────────────┬──────────────────────────────────┘
                            │  8 per-entity verbs (the FROZEN seam)
                            ▼
┌──────────────────────────────────────────────────────────────┐
│  SimControl seam  (deepracer_env/sim_control/)                │
│  port: interface.py · value types: types.py · arenas: arena.py │
│  factory.make_sim_control() selects the adapter               │
└───────────────────────────┬──────────────────────────────────┘
                            │  Strategy pattern (interchangeable adapters)
              ┌─────────────┼──────────────────────┐
              ▼             ▼                       ▼
      ┌──────────────┐ ┌────────────────────┐ ┌──────────────┐
      │ RosGzBackend │ │ SimulationInter-   │ │ NullSimControl│
      │ (working     │ │ facesBackend       │ │ (host tests, │
      │  default)    │ │ (future primary,   │ │  Null Object)│
      │              │ │  dormant)          │ │              │
      └──────┬───────┘ └─────────┬──────────┘ └──────────────┘
            │                   │
            ▼                   ▼
┌──────────────────────────────────────────────────────────────┐
│  Gazebo Jetty (gz-sim 10.4.0, DART 6.16.6 physics)           │
│  gz-transport services on /world/<world>/ · ros2_control      │
│  · ros_gz_bridge for sensors                                  │
└──────────────────────────────────────────────────────────────┘
```

The environment classes are real: `DeepRacerEnv` (`gymnasium.Env`) lives in
[`deepracer_env/environments/deepracer_env.py`](../../deepracer_env/environments/deepracer_env.py)
and `MultiAgentDeepRacerEnv` in
[`deepracer_env/environments/multi_agent_env.py`](../../deepracer_env/environments/multi_agent_env.py).

### The seam in one sentence

`SimControl` is a small **port** (abstract interface) of exactly eight verbs;
the backends are **adapters**. The environment depends only on the port, so the
backend is swappable in one place and mockable for tests. The full design is on
[The SimControl seam](sim-control-seam.md).

The eight verbs, all defined in
[`interface.py`](../../deepracer_env/sim_control/interface.py):

| Verb | Purpose |
| --- | --- |
| `spawn_entity` | Insert a model from SDF/URDF (track, obstacle, car) |
| `delete_entity` | Remove a named entity |
| `list_entities` | List top-level entities in the world |
| `get_entity_state` | Read a model's pose + twist (hot path, every step) |
| `set_entity_state` | Teleport a model — how a car resets to the start line |
| `step` / `pause` / `unpause` | Time control (deterministic stepping where supported) |
| `set_visual_color` / `set_visual_transparency` | Optional visual domain randomization |

**Every verb is per-entity, never world-global.** That single rule is what
enables decoupled multi-arena: resetting `car_3` teleports only `car_3` and never
touches the world or the other cars. A global `reset()` exists but deliberately
raises by default, to protect the decoupling. See
[Tiled multi-arena](multi-arena.md).

## What moved, what stayed

| Concern | Legacy | Now |
| --- | --- | --- |
| OS | Ubuntu (Noetic era) | **Ubuntu 26.04 "resolute"** |
| ROS | ROS 1 **Noetic** | **ROS 2 Lyrical Luth** |
| Simulator | **Gazebo classic** | **Gazebo Jetty** (gz-sim 10.4.0) |
| Physics | ODE (classic) | **DART 6.16.6** |
| Python | 2/3 mix | **3.12** in-container |
| Sim coupling | ~1,700-LOC C++ system plugin + `deepracer_msgs` | **`SimControl` seam → native gz/ROS 2 services. Zero custom C++.** |
| Drive | `velocity_controllers` / `position_controllers` | **`ForwardCommandController`** (ros2_control 6.7) |
| Build/run | host + apt | **Docker** (`ros:lyrical-ros-base` base) |
| AWS/Kinesis/RoboMaker (~28 files) | present | **stripped, not ported** |

What **stayed** — on purpose, to keep training stable:

- The **frozen contract**: action space `Box(low=[-30, 0.1], high=[30, 4.0])`,
  the observation `Dict` keyed by sensor name, the **26 reward-param keys**, and
  the `reset` / `step` / `set_world` signatures. See [The frozen
  contract](contract.md).
- **Tracks, route geometry, and reward parameters** — the 127 worlds were
  converted, not redesigned (see [World conversion](world-conversion.md)).
- The **deployment path**: policies still export to ONNX; the on-device engine is
  ROS-distro-agnostic (see [Deployment & sim-to-real](deployment.md)).

**Extensions** (user-authorized, contract-compatible): per-arena `set_world`
(lifts the old "multi-car can't rotate worlds" limit), and an enriched
observation `info` carrying applied-DR state plus a ground-truth feature vector
as labels for a future camera→feature-vector model.

## Route B: the C++ plugin is gone

The legacy path reached the simulator through a custom Gazebo-classic
`SystemPlugin` plus a pile of custom `deepracer_msgs` services. We chose **Route
B: eliminate the plugin entirely.** Every operation maps to a native service:

- Model / link / pose / pause → native **gz-transport services** on
  `/world/<world>/` (`create`, `remove`, `set_pose`, `control`, `state`).
- Visual recolor + lighting (the visual-DR shim) → native
  **`/world/<world>/visual_config`** and **`/world/<world>/light_config`**
  services — verified present, so even visual DR needs **no custom C++**.

Net result: **zero custom C++ survives.** Detail and the verified service catalog
are in [Backends](backends.md).

## The four migration layers

The port is organized as four layers. The frozen contract runs vertically
through all of them — each layer changed its implementation while preserving the
interface above it.

### Layer 1 — Platform substrate

Ubuntu 26.04, ROS 2 Lyrical, Gazebo Jetty (gz-sim 10.4.0, DART 6.16.6 physics),
Python 3.12, all in Docker. Gazebo is delivered via
`ros-lyrical-gz-sim-vendor` (no osrf apt list). The base image
[`docker/Dockerfile.base`](../../docker/Dockerfile.base) builds
`deepracer-env-build-core:lyrical` from `ros:lyrical-ros-base` — substrate only,
no DeepRacer code. See [Docker build & runtime](docker-build.md).

### Layer 2 — Simulator control plane (the seam)

The `SimControl` port and its adapters, in
[`deepracer_env/sim_control/`](../../deepracer_env/sim_control/__init__.py). This
is where the C++ plugin and custom services dissolved into eight per-entity
verbs. `factory.make_sim_control` selects the backend via the `DR_SIM_BACKEND`
env var or an auto-probe. Value objects in `types.py`
(`Vec3`/`Quaternion`/`Pose`/`Twist`/`EntityState`/`ColorRGBA`) keep the
environment ROS-message-free and host-testable. See
[The SimControl seam](sim-control-seam.md) and [Backends](backends.md).

### Layer 3 — Robot control and sensing

Actuation moved to **ros2_control 6.7**, which dropped
`velocity_controllers`/`position_controllers`; we use
`forward_command_controller/ForwardCommandController` — one per group (4 wheels
on a velocity interface, 2 steering hinges on a position interface), commanded
via `std_msgs/Float64MultiArray`, with `joint_state_broadcaster` for
`/joint_states` and the `gz_ros2_control` plugin in the URDF. Drive code is in
[`deepracer_env/agent_ctrl/drive.py`](../../deepracer_env/agent_ctrl/drive.py).
Sensors (camera, stereo, LIDAR) are bridged from gz via `ros_gz_bridge` /
`ros_gz_image`. See [Drive control](drive-control.md) and
[Observations & sensors](observations.md).

### Layer 4 — Worlds, tracks, and domain randomization

Classic `.world` files were converted to Jetty `.sdf` by
[`scripts/world_converter.py`](../../scripts/world_converter.py) — 127 worlds
converted and load-verified, with the five required system plugins, physics, and
native lighting injected. Domain randomization is consolidated per-arena and
seeded independently, with applied-DR state surfaced into the observation `info`
as dataset labels. See [World conversion](world-conversion.md),
[Tiled multi-arena](multi-arena.md), and
[Domain randomization](domain-randomization.md).

## Where to go next

| Page | What it covers |
| --- | --- |
| [The SimControl seam](sim-control-seam.md) | Ports-&-adapters design, the 8 verbs, value types, `rclpy_client` |
| [Backends](backends.md) | `RosGzBackend`, `SimulationInterfacesBackend`, `NullSimControl`, the gz service catalog |
| [Drive control](drive-control.md) | ros2_control, ForwardCommandController, action→joint mapping |
| [Observations & sensors](observations.md) | CNN vs feature-vector modes, camera/stereo/LIDAR bridging |
| [Tiled multi-arena](multi-arena.md) | `Arena`/`ArenaLayout`, grid offsets, per-entity resets |
| [Domain randomization](domain-randomization.md) | The DR catalog, per-arena seeding, DR-as-labels |
| [World conversion](world-conversion.md) | `world_converter.py`, classic `.world` → Jetty `.sdf` |
| [The frozen contract](contract.md) | Action/observation spaces, 26 reward params, allowed extensions |
| [Deployment & sim-to-real](deployment.md) | ONNX export, the contract-as-boundary, the car runs Foxy/Jazzy |
| [Docker build & runtime](docker-build.md) | Base image, build/test path, in-container toolchain |
