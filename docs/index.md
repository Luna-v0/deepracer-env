# DeepRacer-Env (ROS 2 Lyrical / Gazebo Jetty)

DeepRacer-Env is the simulation environment for training AWS DeepRacer policies,
rebuilt on a modern robotics stack: **Ubuntu 26.04 "resolute", ROS 2 Lyrical Luth,
and Gazebo Jetty (`gz-sim` 10.4.0)** with the DART 6.16.6 physics engine. It exposes
a Gym-style `reset`/`step` environment whose observation and action contract is frozen
byte-for-byte against the legacy ROS 1 / Gazebo-classic environment, so existing
training code (dr-gym) and exported policies keep working unchanged.

The port follows **Route B**: the legacy ~1,700-LOC Gazebo-classic C++ system plugin and
its custom `deepracer_msgs` services are eliminated entirely. Every simulator interaction
now flows through standard, native Gazebo services. Zero custom C++ survives.

## Headline capabilities

- **Decoupled multi-arena.** Run `X` cars in a single Gazebo process, each in its own
  tiled arena with an independent track, episode lifecycle, and seeded domain
  randomization. This lifts the legacy "multi-car can't rotate worlds / visual DR is
  primary-agent-only" coupling. See [Tiled multi-arena](architecture/multi-arena.md).
- **Simulator-agnostic seam.** A ports-and-adapters layer (`deepracer_env/sim_control/`)
  defines an 8-verb abstract `SimControl` port and swappable backends. The default
  `RosGzBackend` drives `gz-transport` services; the environment itself stays free of ROS
  message types and is host-testable. See [the SimControl seam](architecture/sim-control-seam.md).
- **127 converted tracks.** `scripts/world_converter.py` converts classic `.world` files to
  Jetty `.sdf`; all 127 worlds are converted and load-verified in Gazebo Jetty. See
  [World file conversion](architecture/world-conversion.md).
- **Frozen training contract.** Action space `Box(low=[-30, 0.1], high=[30, 4.0])`, a Dict
  observation space keyed by sensor name, 26 reward-param keys, and the `reset`/`step`/
  `set_world` signatures are preserved exactly. See
  [the observation/action contract](architecture/observation-action-contract.md).
- **ONNX deployment, unchanged.** Policies export to ONNX and run on the physical car's
  distro-agnostic inference engine (OpenVINO/TFLite). The sim-to-real boundary is the
  obs/action contract, not the ROS ABI. See [Deployment & sim-to-real](deployment.md).

## Start here

| Page | What it covers |
| --- | --- |
| [Architecture overview](architecture/overview.md) | How the pieces fit together, end to end. |
| [The SimControl seam](architecture/sim-control-seam.md) | The 8-verb port, backends, and value objects. |
| [Tiled multi-arena](architecture/multi-arena.md) | Many cars, one process, decoupled arenas. |
| [Observation/action contract](architecture/observation-action-contract.md) | The frozen interface and observation modes. |
| [Drive & ros2_control](architecture/drive-and-control.md) | Forward command controllers and the action mapping. |
| [Domain randomization catalog](architecture/domain-randomization.md) | Per-arena, independently seeded DR. |
| [World file conversion](architecture/world-conversion.md) | Classic `.world` to Jetty `.sdf`. |
| [Build & run (Docker)](build-and-run.md) | The containerized build and run path. |
| [Deployment & sim-to-real](deployment.md) | ONNX export and the physical car. |
| [ADR 0001: Eliminate the C++ plugin (Route B)](decisions/0001-route-b-eliminate-cpp-plugin.md) | Why no custom C++ survives. |
| [ADR 0002: Target ROS 2 Lyrical + Gazebo Jetty](decisions/0002-lyrical-jetty-stack.md) | Why this stack. |
