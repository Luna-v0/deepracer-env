# ADR 0001: Eliminate the C++ plugin (Route B)

- **Status:** Accepted
- **Date:** 2026-06-30
- **Deciders:** DeepRacer environment maintainers
- **Supersedes:** the legacy Gazebo-classic `SystemPlugin` + `deepracer_msgs` services
- **Related:** [Architecture overview](../architecture/overview.md), [The SimControl seam](../architecture/sim-control-seam.md), [Drive and control](../architecture/drive-and-control.md), [Tiled multi-arena](../architecture/multi-arena.md)

## Context

The legacy DeepRacer simulation reached Gazebo classic through a custom
**~1,700-LOC Gazebo-classic `SystemPlugin`** plus a set of custom
`deepracer_msgs` services. That plugin provided model/link/pose access, pause and
stepping control, and — the only genuinely DeepRacer-specific capability — visual
recolour and lighting changes for domain randomization.

The port to **Ubuntu 26.04 "resolute" + ROS 2 Lyrical Luth + Gazebo Jetty
(gz-sim 10.4.0, DART 6.16.6 physics)** forced a decision about that plugin. ROS 1
Noetic and Gazebo classic are both end-of-life, and the plugin re-implemented a
control plane that the modern simulator now exposes natively. Two paths were on
the table:

- **Route A — port the plugin.** Rewrite the ~1,700 lines as a Gazebo Jetty
  (gz-sim) system plugin and re-issue the custom `deepracer_msgs` services on
  ROS 2. This reproduces legacy behaviour most literally but carries forward a
  bespoke C++ artifact that must be maintained, rebuilt, and re-validated against
  every future gz-sim / ROS release.
- **Route B — eliminate the plugin.** Map every operation the plugin performed
  onto **native gz-transport services** and **ros2_control standards**, including
  the visual/lighting domain-randomization shim, leaving **zero custom C++** in
  the stack.

The guiding principle for the whole port is to **optimize for future-proofing
over bit-exact parity** with the legacy hardware/sim behaviour (see the
[architecture overview](../architecture/overview.md)). The pivotal verification
that made Route B viable: Gazebo Jetty serves **`visual_config` and
`light_config` as native gz services**, so even visual DR — the one thing that
historically justified custom C++ — needs no plugin.

## Decision

**Adopt Route B: eliminate the C++ plugin entirely.**

Every capability the legacy plugin and its `deepracer_msgs` services provided is
re-expressed through native, standard interfaces:

### Model, pose, and time control → native gz-transport services

The simulator control plane is reached through gz-transport services on
`/world/<world>/…`, driven by the `RosGzBackend`
(`deepracer_env/sim_control/backends/ros_gz_backend.py`) behind the `SimControl`
port (`deepracer_env/sim_control/interface.py`). Verified present on gz-sim
10.4.0:

| Operation | gz service / topic | Request → reply | Notes |
|-----------|--------------------|-----------------|-------|
| Spawn | `/world/<w>/create` | `gz.msgs.EntityFactory` → `Boolean` | via `sdf_filename` |
| Delete | `/world/<w>/remove` | `gz.msgs.Entity` → `Boolean` | |
| Teleport | `/world/<w>/set_pose` | `gz.msgs.Pose` → `Boolean` | **pose only**, not twist |
| Step / pause | `/world/<w>/control` | `gz.msgs.WorldControl` → `Boolean` | `pause: true/false`, `multi_step: N` (deterministic stepping confirmed) |
| World state | `/world/<w>/state` | → `SerializedStepMap` | |
| Pose read | `/world/<w>/pose/info` (topic) | `gz.msgs.Pose_V` | pose only; twist is finite-differenced |

There are no gz-transport Python bindings in the Lyrical vendor packages, so the
backend drives these services through the `gz service` / `gz topic` CLI. Every
control-plane operation is cold path (track spawn, obstacle placement,
per-episode teleport), so the CLI overhead is irrelevant.

### Visual recolour + lighting → native `visual_config` / `light_config`

The visual-DR shim — the sole remaining excuse for custom C++ — is served by
**native** gz services:

| Operation | gz service | Request → reply |
|-----------|------------|-----------------|
| Recolour | `/world/<w>/visual_config` | `gz.msgs.Visual` → `Boolean` |
| Lighting | `/world/<w>/light_config` | (native light config) |

Because both are native, **no custom plugin is needed for domain randomization.**
This is the fact that lets Route B retire the plugin outright.

### Actuation → ros2_control standards

Drive moved to **ros2_control 6.7**, which dropped `velocity_controllers` /
`position_controllers`. We use `forward_command_controller/ForwardCommandController`
— one per group: wheels (velocity interface, 4 joints) and steering (position
interface, 2 joints) — commanded via `std_msgs/Float64MultiArray`, with
`joint_state_broadcaster` publishing `/joint_states` and the `gz_ros2_control`
`GazeboSimSystem` plugin in the URDF. Drive code lives in
`deepracer_env/agent_ctrl/drive.py`. See [Drive and control](../architecture/drive-and-control.md).

> **Dependency:** these services and the pose topic exist only if the world SDF
> loads the `Physics`, `UserCommands`, `SceneBroadcaster`, `Contact`, and
> `Sensors` system plugins. `scripts/world_converter.py` injects them during the
> classic `.world` → Jetty `.sdf` conversion (127 worlds converted and
> load-verified).

### Net result

**Zero custom C++ survives.** The custom `deepracer_msgs` package is removed, not
ported. All simulator coupling is concentrated behind the `SimControl` seam and
expressed through native gz-transport and ROS 2 standard interfaces.

## Consequences

### Positive

- **Maintenance.** No bespoke C++ to build, debug, or carry. The ~1,700-LOC
  plugin and the custom `deepracer_msgs` service definitions are deleted.
- **Future-proofing.** Native gz services and ros2_control standards track the
  simulator's own release cadence instead of requiring a parallel plugin rebuild
  for each new gz-sim / ROS version.
- **Simulator-agnosticism.** With coupling behind the `SimControl` port, the
  dormant `SimulationInterfacesBackend` can target the cross-vendor ROS 2
  `simulation_interfaces` standard, making the eventual move to another simulator
  (or a server-backed standard) a config change, not a rewrite.
- **Determinism.** `control` with `multi_step: N` advances exactly N ticks and
  leaves the world paused — race-free, reproducible stepping that the legacy
  pause/unpause dance lacked.
- **Testability.** No live simulator or C++ toolchain is required to exercise the
  control plane; `NullSimControl` and ROS-free value types stand in for host
  tests.

### Negative

- **Dependency on gz native services.** The stack now relies on the
  `/world/<w>/…` services (`create`, `remove`, `set_pose`, `control`, `state`,
  `visual_config`, `light_config`) and the `pose/info` topic being present and
  stable. If a future gz-sim release changes or removes one, we adapt the backend
  rather than owning the implementation outright.
- **Required world plugins.** Every world SDF must carry the five system plugins;
  a world that omits them silently loses these services and poses. The world
  converter mitigates this, but hand-authored worlds must follow suit.
- **Indirect transport on the working backend.** With no gz-transport Python
  bindings in the Lyrical vendor packages, `RosGzBackend` shells out to the `gz`
  CLI. Acceptable because every such call is cold path, but it is a `subprocess`
  hop rather than an in-process API.

### Neutral

- **Pose-only writes / reads.** `set_pose` sets pose, not twist, and `pose/info`
  carries pose only; twist on reset re-settles via physics and the zeroed wheel
  commands, and read twist is finite-differenced. This matches the contract the
  reset/reward code already relied on.
- **Frozen training contract is untouched.** Action space
  `Box(low=[-30, 0.1], high=[30, 4.0])`, the observation `Dict`, the 26
  reward-param keys, and the `reset` / `step` / `set_world` signatures are kept
  byte-for-byte. Route B changes *how* the env reaches the simulator, not *what*
  it promises.
- **Deployment is unaffected.** The sim-to-real boundary is the obs/action
  contract, not the ROS ABI; policies still export to ONNX, and the physical car
  continues to run Foxy/Jazzy rather than Lyrical.

## Alternatives considered

| Option | Outcome | Why not |
|--------|---------|---------|
| **Route A — port the plugin to gz-sim** | Rejected | Reproduces legacy behaviour most literally but perpetuates a ~1,700-LOC bespoke C++ artifact and custom services that must be rebuilt and re-validated against every future gz-sim / ROS release — the exact maintenance liability the port set out to remove. |
| **Route B — eliminate the plugin** (chosen) | Accepted | Native `visual_config` / `light_config` plus standard gz-transport and ros2_control interfaces cover every capability, so the plugin can be deleted with zero custom C++ remaining. |
