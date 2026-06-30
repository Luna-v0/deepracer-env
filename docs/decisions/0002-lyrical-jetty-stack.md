# ADR 0002: Target ROS 2 Lyrical Luth + Gazebo Jetty

- **Status:** Accepted
- **Date:** 2026-06-30
- **Deciders:** deepracer-env maintainers
- **Supersedes:** the implicit ROS 1 Noetic + Gazebo classic baseline
- **Related:** [Architecture overview](../architecture/overview.md) ·
  [The SimControl seam](../architecture/sim-control-seam.md) ·
  [Drive & control](../architecture/drive-and-control.md)

## Context

The legacy environment was pinned to a dead platform:

- **ROS 1 Noetic** reached end of life in May 2025. There will be no further ROS 1
  releases.
- **Gazebo classic (Gazebo 11)** reached end of life in January 2025. The classic
  simulator is frozen; all upstream work moved to the new Gazebo (`gz-sim`).

Staying put means running unsupported, unpatched middleware indefinitely and
maintaining a ~1,700-line custom C++ Gazebo-classic system plugin that
re-implements behavior the modern simulator now provides natively (see
[the seam](../architecture/sim-control-seam.md)).

Moving forces one unavoidable choice: **which ROS 2 + Gazebo pair do we target?**
The realistic candidates are the two most recent LTS-aligned stacks:

| Stack | ROS 2 distro | Gazebo | Ubuntu | Maturity today | Supported until |
|-------|--------------|--------|--------|----------------|-----------------|
| **Lyrical + Jetty** (chosen) | Lyrical Luth | Jetty (`gz-sim` 10.4.0) | 26.04 "resolute" | Newer, fewer rough edges shaken out | **2031** |
| Jazzy + Harmonic | Jazzy Jalisco | Harmonic | 24.04 | More battle-tested, larger community | 2029 |

The tension is **support horizon vs. immediate maturity**, and a secondary
question of **hardware parity**: the physical DeepRacer ships with older ROS 2
distros (Foxy, and Jazzy on newer images), so Jazzy would line the simulator up
more closely with the car.

A guiding principle was set in [ADR 0001](../architecture/overview.md) and the
[architecture overview](../architecture/overview.md): **optimize for
future-proofing, not for bit-exact parity with the legacy hardware/sim
behavior.** All simulator coupling is concentrated behind one swappable seam, and
the sim-to-real boundary is the obs/action contract — not the ROS ABI. That
principle drives this decision.

## Decision

**Target ROS 2 Lyrical Luth + Gazebo Jetty.**

Concretely, the supported substrate is:

| Component | Pinned value |
|-----------|--------------|
| OS (in container) | Ubuntu 26.04 "resolute" |
| ROS 2 distro | Lyrical Luth (LTS) |
| Simulator | Gazebo Jetty = `gz-sim` 10.4.0 |
| Gazebo delivery | `ros-lyrical-gz-sim-vendor` (pulled transitively by `ros-gz-sim`) — **no osrf apt list** |
| Physics engine | DART 6.16.6 |
| Python | 3.12 (in-container) |
| Build/test path | Docker only |
| Base image | `ros:lyrical-ros-base` → [`docker/Dockerfile.base`](../../docker/Dockerfile.base) builds `deepracer-env-build-core:lyrical` (substrate only, no DeepRacer code) |

Two delivery details are load-bearing:

1. **Gazebo arrives via the ROS vendor package**, not the osrf Gazebo apt
   repository. On Lyrical the osrf apt list is the *unsupported* path;
   `ros-lyrical-gz-sim-vendor` (pulled in by `ros-lyrical-ros-gz-sim`) is the
   supported one. See the package set in
   [`docker/Dockerfile.base`](../../docker/Dockerfile.base).
2. **Lyrical is a single LTS track**, so packages are pinned by distro
   (`ros-lyrical-*`) rather than by exact version — the distro tag already pins
   the supported set.

We chose Lyrical + Jetty **because of its longer support horizon (to 2031, two
years beyond Jazzy)**, accepting that it is less mature today, and explicitly
**preferring future-proofing over drag-and-drop hardware parity** with the
physical car.

## Alternatives considered

### Jazzy Jalisco + Gazebo Harmonic (the strongest alternative)

- **Pros:** More mature and battle-tested *today* — Harmonic has had more release
  cycles, the `ros_gz` bridge is more proven, and community Q&A is deeper. Jazzy
  also matches the newer physical-car images more closely, giving better
  drag-and-drop hardware parity.
- **Cons:** Supported only to **2029**. Choosing it would mean planning the *next*
  platform migration almost as soon as this one lands.
- **Verdict:** Rejected. Maturity is a transient advantage that Lyrical inherits
  as it ages; the two extra years of support are permanent. Hardware parity is a
  non-goal here because the sim-to-real boundary is the obs/action contract, not
  the ROS distro (see [Consequences](#consequences)).

### Stay on ROS 1 Noetic + Gazebo classic

- **Verdict:** Rejected outright. Both are end-of-life; this is the status quo the
  port exists to escape.

### Lyrical + osrf Gazebo apt packages (instead of the ROS vendor package)

- **Verdict:** Rejected. The osrf apt list is the unsupported integration path on
  Lyrical. The `ros-lyrical-gz-sim-vendor` route is the maintained one and keeps
  the whole stack on a single, distro-pinned apt source.

## Consequences

### Positive

- **Longest available support runway** — patched, supported middleware through
  **2031**.
- **Zero custom C++ survives.** Targeting modern Gazebo lets every legacy verb
  map to native `gz-transport` / ROS 2 services (Route B), including visual and
  lighting domain randomization via native `/world/<w>/visual_config` and
  `/world/<w>/light_config`. See [the seam](../architecture/sim-control-seam.md)
  and [drive & control](../architecture/drive-and-control.md).
- **Simple, single-track apt pinning** via `ros-lyrical-*` — no per-package
  version matrix, no osrf list to keep in sync.
- **Python 3.12** in-container, current tooling.
- **Future moves are config, not rewrites** — all simulator coupling lives behind
  the `SimControl` port, so the next platform jump is a backend swap.

### Negative / caveats

- **Less mature than Jazzy/Harmonic today.** Some modern standards are not yet
  served on this stack: nothing serves the `simulation_interfaces` ROS services
  on `ros_gz` 3.0.9 yet, so the `SimulationInterfacesBackend` ships **dormant**.
  The working default is `RosGzBackend`, which drives `gz-transport` services via
  the `gz service` / `gz topic` CLI. This is by design — see
  [the seam](../architecture/sim-control-seam.md).
- **Thinner community knowledge.** Fewer existing Q&A threads and recipes for a
  newer distro; expect to read upstream source more often.
- **Hardware-parity caveat (explicit and accepted):** the **physical car runs
  Foxy/Jazzy, NOT Lyrical.** This is fine and expected. The sim-to-real boundary
  is the **obs/action contract**, not the ROS ABI — policies export to ONNX and
  the on-device inference engine (OpenVINO/TFLite) is ROS-distro-agnostic. The
  simulator and the car deliberately do **not** need to share a ROS distro.
- **Headless rendering needs care.** `gz-sim` renders through OGRE-next; the base
  image defaults to the software-GL path (`LIBGL_ALWAYS_SOFTWARE=1`) so the
  server runs with no X server. See [`docker/Dockerfile.base`](../../docker/Dockerfile.base).

### Neutral

- The frozen training contract (action space, observation dict, 26 reward-param
  keys, `reset`/`step`/`set_world` signatures) is unaffected by this choice — it
  is preserved byte-for-byte regardless of the ROS distro underneath.
- dr-gym coupling stays thin (~95% insulated): only the Dockerfile base tag and
  `/opt/ros` sourcing change from `noetic` to `lyrical`.

## Verification

This decision was checked against reality before acceptance, not assumed:

- **The whole package set exists on apt** (`packages.ros.org`). Every dependency
  in [`docker/Dockerfile.base`](../../docker/Dockerfile.base) resolves as a
  `ros-lyrical-*` package, including the simulator (`ros-lyrical-ros-gz-sim` +
  the transitive `ros-lyrical-gz-sim-vendor`), the bridge (`ros-lyrical-ros-gz-bridge`,
  `-ros-gz-image`, `-ros-gz-interfaces`), the drive stack
  (`ros-lyrical-gz-ros2-control`, `-ros2-control`, `-ros2-controllers`), and the
  Route B standard (`ros-lyrical-simulation-interfaces`). The audited version
  table lives in the project memory note `lyrical-jetty-substrate-verified`.

  ```bash
  # The supported install path, all from the single ROS apt source:
  apt-get install -y \
      ros-lyrical-ros-gz-sim ros-lyrical-ros-gz-bridge \
      ros-lyrical-ros-gz-image ros-lyrical-ros-gz-interfaces \
      ros-lyrical-gz-ros2-control ros-lyrical-ros2-control ros-lyrical-ros2-controllers \
      ros-lyrical-simulation-interfaces
  # NB: no osrf gazebo apt list is added — gz-sim 10.4.0 comes via the vendor pkg.
  ```

- **The base image builds.** `ros:lyrical-ros-base` →
  [`docker/Dockerfile.base`](../../docker/Dockerfile.base) produces
  `deepracer-env-build-core:lyrical` (substrate only).
- **Jetty actually loads our content.** 127 legacy `.world` files were converted
  to Jetty `.sdf` and load-verified in `gz-sim` 10.4.0 via
  [`scripts/world_converter.py`](../../scripts/world_converter.py).

## References

- [Architecture overview](../architecture/overview.md)
- [The SimControl seam](../architecture/sim-control-seam.md)
- [Drive & control](../architecture/drive-and-control.md)
- [`docker/Dockerfile.base`](../../docker/Dockerfile.base)
- [`deepracer_env/sim_control/`](../../deepracer_env/sim_control/)
- [`scripts/world_converter.py`](../../scripts/world_converter.py)
