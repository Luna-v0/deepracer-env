# The SimControl seam

`deepracer_env.sim_control` is the **single boundary** between the environment and
the running simulator. Everything the env needs from Gazebo — spawn, delete,
teleport, read state, step, recolour — passes through one small interface,
`SimControl`. Concrete backends sit behind it and translate to the actual
transport (gz-transport services today, ROS 2 `simulation_interfaces` tomorrow).

This is a textbook **ports-and-adapters (hexagonal)** design:

- **Port** — `SimControl`, the abstract interface the env depends on
  (`deepracer_env/sim_control/interface.py`).
- **Adapters** — interchangeable backends selected by a factory (Strategy
  pattern), under `deepracer_env/sim_control/backends/`.
- **Value objects** — plain Python dataclasses that cross the seam, keeping the
  env free of ROS message types (`deepracer_env/sim_control/types.py`).

If you only remember one thing: **every verb is per-entity, never world-global.**
That single rule is what makes decoupled multi-arena possible. See
[tiled multi-arena](tiled-multi-arena.md) for where that pays off.

## Why a seam exists

The legacy stack reached the simulator through a custom Gazebo-classic
`SystemPlugin` of roughly **1,700 lines of C++**, plus **14 distinct**
`gazebo_msgs` / `deepracer_msgs` services called from **99 sites** across 13
modules. The ROS↔sim coupling was smeared everywhere, untestable off a live
simulator, and impossible to swap.

The seam collapses all of that into **8 verbs behind one interface**. Every one
of those 99 call sites maps to one verb. The coupling now lives in exactly one
place, is swappable via config (Strategy), and is mockable for host tests via a
Null Object. As part of this same effort ([Route B](route-b-no-cpp.md)), the
custom C++ plugin is **eliminated entirely** — the visual / lighting
domain-randomisation shim that justified custom C++ is now served by *native* gz
services, so zero custom C++ survives.

## The port: `SimControl` and its 8 verbs

`SimControl` is an `abc.ABC`. The eight verbs below are the entire vocabulary the
environment uses to talk to a simulator.

| # | Verb | Signature (abbreviated) | Replaces (legacy) |
|---|------|--------------------------|-------------------|
| 1 | `spawn_entity` | `(name, sdf, pose=IDENTITY_POSE, *, allow_renaming=False) -> str` | `gazebo_msgs/SpawnModel` |
| 2 | `delete_entity` | `(name) -> bool` | `gazebo_msgs/DeleteModel` |
| 3 | `list_entities` | `() -> List[str]` | `GetWorldProperties.model_names` |
| 4 | `get_entity_state` | `(name, *, reference_frame="world") -> EntityState` | `GetModelState(s)` |
| 5 | `set_entity_state` | `(name, state, *, blocking=True) -> bool` | `SetModelState(s)` |
| 6 | `step` | `(n=1) -> None` | pause/unpause stepping dance |
| 7 | `pause` / `unpause` | `() -> None` | `/gazebo/(un)pause_physics` |
| 8 | `set_visual_color` / `set_visual_transparency` | `(entity, link, visual, …) -> bool` | `deepracer_msgs/SetVisualColors` |

Verbs 1–7 are `@abc.abstractmethod` (every backend must implement them). The
visual verbs (8), plus `get_link_state`, are **optional**: their default
implementation raises `CapabilityNotSupported`, and callers guard with
`supports()` so optional features degrade gracefully instead of crashing.

`get_entity_state` (verb 4) is the hot read path — the car's state is queried
every step to compute the 26 reward parameters and evaluate the reset rules.
`set_entity_state` (verb 5) is the hot write path — it teleports a single car
back to the start line on reset, and it is strictly per-entity (the cornerstone
of multi-arena).

### The deliberately-absent verb: `reset`

There is no world-global reset in the eight verbs. `SimControl.reset()` exists
but its default **raises** `CapabilityNotSupported`:

```python
def reset(self) -> None:
    raise CapabilityNotSupported(
        "Global world reset is disabled to protect multi-arena decoupling; "
        "reset a single car with set_entity_state() instead.")
```

A global reset would clobber every arena sharing the simulator at once. Per-episode
resets must teleport the single car via `set_entity_state`. The method is kept
only for single-arena bring-up and tests, and it fails loudly so misuse is caught.

## Value objects: a ROS-message-free contract

The env, the reset rules, and the domain randomisers speak **only** the
dataclasses in `types.py`. Backends translate them to/from ROS 2 / gz message
classes at the very edge.

| Type | Fields | Notes |
|------|--------|-------|
| `Vec3` | `x, y, z` (m) | Frozen; supports `+`, `-`, `as_tuple()` |
| `Quaternion` | `x, y, z, w` | `from_yaw(rad)` / `.yaw` — the car only ever rotates about +Z |
| `Pose` | `position: Vec3`, `orientation: Quaternion` | `Pose.at(x, y, z=0, yaw=0)` convenience ctor |
| `Twist` | `linear: Vec3`, `angular: Vec3` (rad/s) | Spatial velocity |
| `EntityState` | `pose: Pose`, `twist: Twist` | The unit read by verb 4 / written by verb 5 |
| `ColorRGBA` | `r, g, b, a` in `[0, 1]` | Gazebo material range |

`IDENTITY_POSE = Pose()` is exported as a module constant so call sites can spawn
at the world origin without building a throwaway object.

All types are frozen dataclasses using SI units and Gazebo's right-handed, Z-up
world frame. Keeping the contract in plain Python (not `geometry_msgs`) buys three
things:

- **Testability** — pose math and arena geometry unit-test on a host with no ROS
  installed. `types.py` imports nothing beyond `math` and `dataclasses`.
- **ABI insulation** — a change in the underlying message ABI (ROS 1 `Pose` vs
  ROS 2 `Pose` vs `gz.msgs.Pose`) is absorbed inside one backend, not scattered
  across the old 99 call sites.
- **Intent** — `EntityState` says "a pose and a twist for one named thing,"
  exactly the vocabulary the reset/reward code already uses.

This is also why the package import surface is split: `types`, `interface`, and
`arena` are ROS-free and import on any host; backends and `rclpy_client` import
`rclpy` and are loaded lazily, only inside the container (see the import policy in
`deepracer_env/sim_control/__init__.py`).

## Why per-entity, never world-global

Every verb names the entity it acts on. There is no "set all model states," no
"world reset," no implicit "primary agent." This is the load-bearing choice:

- One shared `gz` process can host *N* independent arenas.
- Resetting `car_3` teleports only `car_3`; it never touches the world or the
  other cars.
- Each arena owns its own episode lifecycle, its own seeded domain randomisation,
  and its own track at a grid offset.

This generalises the legacy `MultiAgentDeepRacerEnv` and breaks the old
"visual DR = primary-agent-only / world-shared" coupling. The full layout logic
lives in `deepracer_env/sim_control/arena.py`; see
[tiled multi-arena](tiled-multi-arena.md).

## The three backends + factory selection (Strategy)

Three adapters implement (or stub) the port. The factory picks one at runtime —
the Strategy pattern in practice.

| Backend | Module | Role | State today |
|---------|--------|------|-------------|
| `RosGzBackend` | `backends/ros_gz_backend.py` | Working default; drives gz-transport services via the `gz service` / `gz topic` CLI | **Active** |
| `SimulationInterfacesBackend` | `backends/simulation_interfaces_backend.py` | Future primary; targets the cross-vendor ROS 2 `simulation_interfaces` standard | **Dormant** — message pkg installed, but nothing serves the services on ros_gz 3.0.9 / gz-sim 10.4 yet |
| `NullSimControl` | `interface.py` | Null Object; records calls, returns identity/empty values | Host tests / dry imports |

### `factory.make_sim_control` — the chooser

`deepracer_env/sim_control/factory.py` selects the backend in this order:

1. The `DR_SIM_BACKEND` environment variable, if set
   (`"ros_gz"` | `"simulation_interfaces"` | `"null"`).
2. The `prefer=` argument.
3. **Auto**: `simulation_interfaces` only if both its message package imports
   *and* a live server is visible (`is_available()` + a service-presence probe
   for `simulation_interfaces/srv/SetEntityState`); otherwise `ros_gz`.

```python
from deepracer_env.sim_control import make_sim_control

sim = make_sim_control("racetrack_world")   # auto -> RosGzBackend today
# or force one:
#   DR_SIM_BACKEND=null python ...          # host tests
#   make_sim_control(world, prefer="simulation_interfaces", node=sim_node)
```

Because the choice is data-driven, the eventual move to the standard is a config
change, not a code change.

### `RosGzBackend` — the working default

Gazebo Jetty exposes its control plane as gz-transport services on
`/world/<world>/…`, plus a pose topic. There are **no gz-transport Python
bindings** in the Lyrical vendor packages, so this backend shells out to the `gz`
CLI via `subprocess`. That is fine: every control-plane op here is **cold path**
(track spawn, obstacle placement, per-episode teleport / recolour). The hot paths
bypass this backend entirely — wheel commands go through `ros2_control` topics
and sensor frames arrive on bridged ROS topics (see [the drive layer](drive-layer.md)).

Verb-to-service mapping (verified on gz-sim 10.4.0):

| Verb | gz service / topic | Request → reply |
|------|--------------------|-----------------|
| `spawn_entity` | `/world/<w>/create` | `gz.msgs.EntityFactory` → `Boolean` (via `sdf_filename`) |
| `delete_entity` | `/world/<w>/remove` | `gz.msgs.Entity` → `Boolean` |
| `list_entities` | `gz model --list` | (CLI) |
| `get_entity_state` | `/world/<w>/pose/info` (topic) | batched `Pose_V` snapshot |
| `set_entity_state` | `/world/<w>/set_pose` | `gz.msgs.Pose` → `Boolean` (**pose only**) |
| `step` | `/world/<w>/control` | `gz.msgs.WorldControl` (`pause: true multi_step: N`) → `Boolean` |
| `pause` / `unpause` | `/world/<w>/control` | `gz.msgs.WorldControl` (`pause: true/false`) |
| `set_visual_color` | `/world/<w>/visual_config` | `gz.msgs.Visual` → `Boolean` (**native**) |

Three details worth knowing:

- **Deterministic stepping is confirmed.** `step(n)` sends
  `pause: true multi_step: n`, which advances exactly *n* ticks and leaves the
  world paused — race-free and reproducible, a strict upgrade over the legacy
  pause/unpause dance. The backend advertises `Capability.DETERMINISTIC_STEP`.
- **Pose reads are batched.** `refresh_state()` takes one `/world/<w>/pose/info`
  snapshot per step into a cache that `get_entity_state` serves for every entity
  — the "one read serves all cars per step" pattern (the heir of the legacy
  `GetModelStateTracker`). The gz pose topic carries **pose only**, so twist is
  finite-differenced against the previous snapshot, with yaw wrapped to
  `[-π, π]`.
- **`set_pose` sets pose only**; the reset path re-settles twist via physics plus
  the zeroed wheel commands it already issues. Visual recolour is a **native**
  `visual_config` service (ambient defaults to `0.6 × diffuse`), which is exactly
  why the legacy C++ plugin can be deleted.

A dead simulator is turned into a catchable `SimControlDead` rather than a hang:
`_gz_alive()` checks whether `gz service -l` still lists `/world/<w>/control`,
the Jetty analogue of the legacy `gazebo_alive` guard.

> System plugins `Physics`, `UserCommands`, `SceneBroadcaster`, `Contact`, and
> `Sensors` must be present in the world SDF, or these services and the pose topic
> don't exist. The [world converter](world-conversion.md) injects them.

### `SimulationInterfacesBackend` — future primary, wired but dormant

`simulation_interfaces` is the cross-vendor ROS 2 standard for simulator control.
Targeting it makes the env simulator-agnostic: the day a Jetty release (or a USD /
MuJoCo backend) ships a server, the env flips to it by changing one line in the
factory.

It is written against the exact, introspected `srv` fields so it is correct the
moment a server appears:

| Verb | Service type | Notes |
|------|--------------|-------|
| `spawn_entity` | `SpawnEntity` | SDF goes in `entity_resource.resource_string` |
| `delete_entity` | `DeleteEntity` | |
| `list_entities` | `GetEntities` | |
| `get_entity_state` | `GetEntityState` | |
| `set_entity_state` | `SetEntityState` | needs `set_pose` / `set_twist` flags |
| `step` | `StepSimulation` | requires the sim be PAUSED first |
| `pause` / `unpause` | `SetSimulationState` | `STATE_PAUSED` / `STATE_PLAYING` |

Every reply carries `result.result`; success is `Result.RESULT_OK == 1`,
checked by `_check()`. `is_available()` only verifies the message package imports;
the factory pairs it with a live service-presence probe before selecting it.

### `NullSimControl` — the Null Object

Lets the environment be constructed and exercised with **zero** ROS/simulator
dependency. Reads return identity/empty values; writes are recorded in `.calls`
for test assertions. `supports()` returns `True` for everything so optional-path
code is exercised too. Select it with `DR_SIM_BACKEND=null`.

## rclpy plumbing: `SimNode` and `ServiceClientWrapper`

`deepracer_env/sim_control/rclpy_client.py` is the ROS 2 replacement for the
legacy `rospy_wrappers` / `rospy.init_node` machinery. It imports `rclpy` at
import time, so it loads only inside the container.

**`ensure_rclpy_initialized()`** — idempotent `rclpy.init`, lock-guarded, so
constructing an environment twice in one process (common in HPO sweeps) never
double-inits.

**`SimNode(Node)`** — the *single* node the environment owns, spun on a
background daemon thread by a `SingleThreadedExecutor`. Owning one node (rather
than one per sensor) keeps the DDS graph small and makes multi-car namespacing a
matter of per-topic names. The background executor lets subscription callbacks
(filling the camera / LiDAR / pose double-buffers) run while gymnasium
`step`/`reset` execute synchronously on the main thread — the same threading
model the legacy `rospy` callbacks relied on. `start_spinning` / `stop_spinning`
/ `destroy` are all idempotent and teardown-safe.

**`ServiceClientWrapper`** — the rclpy heir of the legacy `ServiceProxyWrapper`.
It keeps the *retry* behaviour but raises a typed `SimControlError` instead of
killing the process (a RoboMaker quirk), so a looping caller can recover. Because
`SimNode` already spins its executor on a background thread, calls go through
`call_async` and block on the future via a `threading.Event` — never
`spin_until_future_complete`, which would fight the background spin:

```python
future = self._client.call_async(request)
done = threading.Event()
future.add_done_callback(lambda _f: done.set())
if not done.wait(timeout_sec):
    raise SimControlTimeout(...)
return future.result()
```

The constructor waits for the server (`wait_for_service`) and raises
`SimControlError` if it never appears. Only the `simulation_interfaces` backend
uses this wrapper today; `RosGzBackend` talks to the CLI, not ROS services.

## Error types and capability negotiation

All backend errors derive from one base, so callers can catch coarsely or finely:

| Exception | Meaning | Caller action |
|-----------|---------|---------------|
| `SimControlError` | Base class; a service was rejected or failed | Surface / abort the op |
| `SimControlTimeout` | A service didn't answer in time | Retry or escalate |
| `SimControlDead` | The simulator process is unreachable | Checkpoint and restart the container (mirrors the legacy `WorldSwapError` contract) |
| `CapabilityNotSupported` | The active backend can't do this op | Guard with `supports()`; treat as optional |

Optional features negotiate via `supports(capability)` against the `Capability`
constants:

- `DETERMINISTIC_STEP` — exact integer stepping (both real backends advertise it).
- `VISUAL_RECOLOR` — runtime material colour / transparency (`RosGzBackend` only).
- `LINK_STATE` — per-link pose/twist reads via `get_link_state`.

Domain-randomisation code checks `supports(Capability.VISUAL_RECOLOR)` before
recolouring, so a backend without it degrades gracefully instead of crashing.

## How the seam collapses the legacy coupling

| Legacy | After the seam |
|--------|----------------|
| ~1,700-LOC Gazebo-classic C++ `SystemPlugin` | Eliminated — native gz services ([Route B](route-b-no-cpp.md)) |
| 14 `gazebo_msgs` / `deepracer_msgs` services | 8 verbs on one interface |
| 99 call sites across 13 modules | Each maps to one verb; coupling lives in one package |
| ROS messages threaded through env logic | ROS-free dataclasses; ABI absorbed at the backend edge |
| Hard ROS dependency to run any test | Host-testable; `NullSimControl` + ROS-free `types`/`interface`/`arena` |
| World-shared, primary-agent-only ops | Strictly per-entity → decoupled multi-arena |
| One hard-wired transport | Strategy: swap backend via `DR_SIM_BACKEND` / factory |

The frozen env contract (action space `Box(low=[-30, 0.1], high=[30, 4.0])`,
Dict observation space, 26 reward keys, reset/step/set_world signatures) is kept
byte-for-byte across the seam — the rewrite changes *how* the env reaches the
simulator, not *what* the env promises.

## Related pages

- [Route B: no custom C++](route-b-no-cpp.md) — how native gz services retire the
  1,700-LOC plugin.
- [Tiled multi-arena](tiled-multi-arena.md) — what per-entity verbs unlock.
- [The drive layer](drive-layer.md) — the hot-path control that bypasses this seam.
- [World conversion](world-conversion.md) — injecting the system plugins the
  gz services depend on.
- [Domain randomization](domain-randomization.md) — the consumer of the optional
  visual / lighting verbs.
