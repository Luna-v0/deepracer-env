# Tiled multi-arena (decoupled multi-car)

Run a *dynamic* number of cars in **one** Gazebo process, each fully decoupled:
its own track, its own domain randomization (DR), and its own episode lifecycle.
Resetting `car_3` teleports only `car_3` — it never touches the world or any
neighbour. This is the headline capability of the port, and it generalizes the
legacy `MultiAgentDeepRacerEnv` while lifting its "multi-car can't rotate worlds"
limit.

The geometry and bookkeeping live in
[`deepracer_env/sim_control/arena.py`](../../deepracer_env/sim_control/arena.py)
(`Arena`, `ArenaLayout` — host-tested, imports no ROS and no simulator). The
actual spawning, teleporting, and stepping is done by a `SimControl` backend that
*consumes* `Arena` objects — see [the seam](sim-control-seam.md).

## Why this works: every verb is per-entity

The enabling design choice is in the seam, not in this module. The `SimControl`
port ([`interface.py`](../../deepracer_env/sim_control/interface.py)) exposes
eight verbs, and **every one is per-entity, never world-global**:

```
spawn_entity   delete_entity   list_entities
get_entity_state   set_entity_state
step   pause/unpause   set_visual_color / set_visual_transparency
```

A global `SimControl.reset()` exists only as a deliberately-discouraged
single-arena bring-up helper; its default implementation *raises* so that misuse
in a multi-arena world is loud:

```python
# interface.py — reset() default
raise CapabilityNotSupported(
    "Global world reset is disabled to protect multi-arena decoupling; "
    "reset a single car with set_entity_state() instead."
)
```

Because state writes go through `set_entity_state` (a per-entity teleport, backed
by the gz `/world/<world>/set_pose` service), N arenas can share one world with
zero cross-talk.

## Spatial tiling

Each arena's track is laid at a distinct world offset on a square grid, far
enough apart that cars can neither see nor collide with one another. The spacing
default is **300 m** (`DEFAULT_ARENA_SPACING_M`), preserved from the legacy
`MultiAgentDeepRacerEnv` — large enough that the biggest shipped track plus its
camera far-clip cannot reach a neighbour.

`ArenaLayout.grid_offsets(n, spacing)` fills a row-major
`ceil(sqrt(n)) x ceil(sqrt(n))` grid so the bounding box stays compact:

```python
cols = int(math.ceil(math.sqrt(n)))
offsets = [((i % cols) * spacing, (i // cols) * spacing) for i in range(n)]
```

Arena 0 is **always** at `(0, 0)` — the car-0 track is the one the world SDF
loads at world-load time; the others are spawned around it via `spawn_entity`.

### A 2x2 tiled layout (N=4, spacing=300 m)

World frame is right-handed, Z-up; `+x` to the right, `+y` away. Row-major fill
puts arenas 0–1 on the first row, 2–3 on the second.

```
            +y
             ^
   600 m  ───┼──────────────────────────
             │
             │   ┌──────────┐   ┌──────────┐
   300 m ────┤   │ arena 2  │   │ arena 3  │
             │   │ car_2    │   │ car_3    │
             │   │ racetrack_2  │ racetrack_3
             │   │ origin   │   │ origin   │
             │   │ (0,300)  │   │ (300,300)│
             │   └──────────┘   └──────────┘
             │
             │   ┌──────────┐   ┌──────────┐
     0 m ────┤   │ arena 0  │   │ arena 1  │
             │   │ car_0    │   │ car_1    │
             │   │ racetrack_0  │ racetrack_1
             │   │ origin   │   │ origin   │
             │   │ (0,0)    │   │ (300,0)  │
             │   └──────────┘   └──────────┘
             │
             └───┼──────────────┼─────────────> +x
               0 m           300 m
```

Each tile is an independent island: its own track mesh, its own namespaced car,
its own DR seed, its own episode clock.

## The `Arena` record

`Arena` is a frozen dataclass — one decoupled racing instance:

| Field | Meaning |
| --- | --- |
| `index` | Zero-based arena index. |
| `car_name` | Namespace + entity name of the car (`"car_0"`). Drives every per-car topic, e.g. `/car_0/camera/zed/rgb/image_rect_color`, `/car_0/scan`. |
| `track_name` | The `routes/<name>.npy` / `models/<name>` key. **Arenas may run different tracks.** |
| `track_entity_name` | The Gazebo entity name of the spawned track mesh (`"racetrack_0"`), unique so deletes/recolours target exactly one arena. |
| `origin` | `Vec3` world-frame offset of this arena's track origin. |
| `dr_seed` | Independent DR seed, so two arenas on the *same* track still randomize differently. |

## Naming

Names are templated and unique-per-index, which is what lets per-entity verbs
address exactly one arena:

* Cars: `car_{index}` → `car_0`, `car_1`, … (override via `car_name_fmt`).
* Track meshes: `racetrack_{index}` → `racetrack_0`, … (override via
  `track_entity_fmt`).
* Topics inherit the car namespace: `/car_2/...`.

## Per-arena reset: `set_pose` only, never world reset

An episode reset for arena *i* is a single `set_entity_state` (per-entity
teleport) on `car_i` to its start pose. The world is never reset, physics is
never globally rewound, and no neighbour observes a discontinuity. This is the
cornerstone of decoupled lifecycles — arenas terminate, reset, and re-randomize
on their own clocks.

## Arena-local reward frame

Track geometry — progress, `distance_from_center`, `closest_waypoints` — is
defined in each track's own coordinates (loaded from `routes/<track>.npy`).
Because every arena hosts a possibly-different track translated to a different
offset, the reward for car *i* is computed in arena *i*'s **local** frame:

1. Read the car's world pose (`get_entity_state`).
2. Subtract the arena origin: `ArenaLayout.to_local(arena, world_pose)`.
3. Evaluate against that arena's un-offset track.

```python
def to_local(self, arena, world_pose):
    # orientation unchanged: arenas are pure translations of one another
    return Pose(world_pose.position - arena.origin, world_pose.orientation)
```

`to_world` is the inverse — it turns a track-relative start pose from
`routes/<track>.npy` into the world pose to teleport the car to. Keeping one
canonical, un-shifted track per arena (and moving the cheap transform to
read-time) is what lets heterogeneous tracks coexist in one world. The legacy
code instead pre-offset each car's `TrackData` copy; the two are equivalent.

## Per-arena DR seeds

Arena *i* gets `base_seed + i` as its DR seed. Each arena's randomization —
start position, start direction, recovery resets, wheel friction, visual recolour,
lighting, sensor noise, steering bias, motor delay — draws from its own seeded
stream, so two arenas on the same track diverge. The applied-DR state for each
arena is surfaced in the observation `info` as dataset labels. For the full
catalog see [domain randomization](domain-randomization.md).

This breaks the old "visual DR = primary-agent-only / world-shared" coupling:
visual recolour now targets a single `racetrack_i` entity via the native gz
`/world/<world>/visual_config` service (and lighting via `light_config`), so each
arena recolours independently with **zero custom C++**.

## Worked `ArenaLayout` example

```python
from deepracer_env.sim_control.arena import ArenaLayout

# Three cars: two on reinvent_base, one on Bowtie_track — independent DR each.
layout = ArenaLayout(
    n_arenas=3,
    tracks=["reinvent_base", "Bowtie_track", "reinvent_base"],
    base_seed=1000,
)

for a in layout.arenas:
    print(a.index, a.car_name, a.track_name, a.track_entity_name,
          a.origin.as_tuple(), a.dr_seed)
```

Output (cols = `ceil(sqrt(3))` = 2, so a 2-wide grid):

```
0 car_0 reinvent_base  racetrack_0 (0.0,   0.0,   0.0) 1000
1 car_1 Bowtie_track   racetrack_1 (300.0, 0.0,   0.0) 1001
2 car_2 reinvent_base  racetrack_2 (0.0,   300.0, 0.0) 1002
```

Constructor knobs:

| Arg | Default | Notes |
| --- | --- | --- |
| `n_arenas` | — | `>= 1`. Same code path serves 1 or 64 cars. |
| `tracks` | — | Length 1 (broadcast to all arenas — "same track, independent DR") **or** length `n_arenas` ("different track per arena"). Any other length raises `ValueError`. |
| `spacing` | `300.0` | Grid spacing in metres between adjacent origins. |
| `base_seed` | `0` | Arena *i* seed = `base_seed + i`. |
| `car_name_fmt` | `"car_{index}"` | `str.format` template; receives `index`. |
| `track_entity_fmt` | `"racetrack_{index}"` | Template; receives `index`. |

Single-track broadcast (8 cars, one track, eight DR streams):

```python
layout = ArenaLayout(n_arenas=8, tracks=["reinvent_base"], base_seed=0)
```

## Contrast with the legacy `MultiAgentDeepRacerEnv`

| Aspect | Legacy `MultiAgentDeepRacerEnv` | Tiled multi-arena |
| --- | --- | --- |
| Tracks | One shared world; all cars on the **same** track | Per-arena track; arenas may differ |
| World rotation | **Multi-car couldn't rotate worlds** (locked) | Per-arena `set_world` lifts the limit |
| Reset | Coupled to world / primary agent | Per-entity `set_pose` on one car only |
| Reward frame | Pre-offset `TrackData` copy per car | Read-time `to_local` against a canonical track |
| Visual DR | Primary-agent-only / world-shared | Per-arena, native `visual_config`, zero C++ |
| Sim plumbing | Custom Gazebo-classic `SystemPlugin` (~1,700 LOC) + `deepracer_msgs` | Standard gz services behind the `SimControl` seam |

The frozen obs/action contract is preserved byte-for-byte (action
`Box(low=[-30, 0.1], high=[30, 4.0])`, Dict observation, 26 reward keys,
`reset`/`step`/`set_world` signatures). Per-arena `set_world` is a user-authorised
*extension* of that contract, not a change to it.

## Where this sits

* Geometry / naming / seeding: [`arena.py`](../../deepracer_env/sim_control/arena.py).
* Per-entity verbs that make decoupling possible: [the seam](sim-control-seam.md).
* The DR streams each arena seeds: [domain randomization](domain-randomization.md).
* Turning a classic `.world` into the per-arena track meshes: [world conversion](world-conversion.md).
* Driving each namespaced car: [drive layer](drive.md).

## Live N-car bring-up (implementation)

The N decoupled arenas are launched by
[`launch/multi_arena.launch.py`](../../simulation/src/deepracer_simulation_environment/launch/multi_arena.launch.py):
one `gz` server, then per arena `i` a track instance `racetrack_i` at the grid
offset, a namespaced `robot_state_publisher`, a spawned car `racecar_i`, and that
car's own **namespaced `gz_ros2_control` controller_manager** at
`/racecar_i/controller_manager`. `RolloutCtrl` publishes each car's commands to
`/racecar_i/wheels_velocity_controller/commands` + `/racecar_i/steering_position_controller/commands`.

Two namespacing rules make multiple controller-managers coexist in one server
(both learned the hard way):

* **Never** set `<controller_manager_name>` to a value containing `/` — gz turns
  it into a node-name remap (`-r __node:=…`), and a node *name* may not contain
  `/`; the uncaught parse error aborts the whole gz server. Use the
  `<ros><namespace>` element alone; the FQN becomes `/racecar_i/controller_manager`.
* `config/ros2_control.yaml` uses `/**/`-prefixed keys
  (`/**/controller_manager`, `/**/wheels_velocity_controller`, …) so the params
  reach the *namespaced* controller nodes. `/**` matches zero-or-more namespace
  levels, so the **one** file serves both the single-car run (`/controller_manager`)
  and every arena (`/racecar_i/controller_manager`).

**Verified** in a real gz Jetty container: two namespaced controller-managers
come up and activate hardware in one server, and both cars drive simultaneously
from their `/racecar_i/...` command topics.
