# Domain randomization catalog

Domain randomization (DR) is how the simulator manufactures the visual and
dynamical variety that a sim-to-real policy needs to generalize. This page is the
single consolidated catalog of every DR knob in the port: what it perturbs, how
it is realized on Gazebo Jetty, and whether it is independent per arena.

Two things make the Jetty DR story different from the legacy stack:

- **Every randomization is per-arena.** A run can host *N* decoupled cars in one
  Gazebo process (see [Tiled multi-arena](multi-arena.md)). Each arena seeds its
  own DR and every simulator write is per-entity, so recoloring `racetrack_3` or
  teleporting `car_3` never touches a neighbor. This breaks the legacy coupling
  where visual DR was world-shared and applied only by the primary agent.
- **Zero custom C++.** Visual recolor and lighting now ride **native** gz services
  (`visual_config`, `light_config`), so the ~1,700-LOC Gazebo-classic plugin that
  used to back visual DR is gone. See [the SimControl seam](sim-control-seam.md).

The downstream payoff: the DR values actually applied each episode are surfaced in
the observation `info` dict as **dataset labels** for the future
camera-to-feature-vector model (see
[Applied-DR state as dataset labels](#applied-dr-state-as-dataset-labels)).

## Per-arena independence

`ArenaLayout` assigns each arena an independent DR seed
(`deepracer_env/sim_control/arena.py`):

```python
Arena(index=i,
      car_name="car_i",
      track_name=...,             # may differ per arena
      track_entity_name="racetrack_i",
      origin=Vec3(dx_i, dy_i, 0),
      dr_seed=base_seed + i)      # independent randomisation
```

Two arenas running the *same* track with `base_seed + i` seeds still randomize
differently. Independence holds at three layers:

| Layer | What makes it per-arena |
| --- | --- |
| Seeding | `Arena.dr_seed = base_seed + index` (`arena.py`) |
| Reset | Cars reset via per-entity `set_entity_state` only — never a world reset (`sim_control/interface.py`) |
| Visual / lighting | `visual_config` / `light_config` target a named entity (`racetrack_i`), so recolor is scoped to one arena (`backends/ros_gz_backend.py`) |

The abstract port deliberately *disables* the global `reset()` verb to protect
this decoupling; per-episode resets go through `set_entity_state` on the single
car (`deepracer_env/sim_control/interface.py`).

## The catalog

| Knob | Scope | Mechanism on Jetty | Per-arena? |
| --- | --- | --- | --- |
| Start position (`start_ndist`) | Per-episode reset | `random_start` flag → sample `rng.random()` ∈ [0,1) along centerline, teleport via per-entity `set_pose` | Yes — own `dr_reset_rng`, own car teleport |
| Start direction (`reverse_dir`) | Per-episode reset | `random_direction` flag → `reverse_dir = rng.random() < 0.5` on the arena's `TrackData` | Yes — per-car track state + RNG |
| Recovery resets (`NUMBER_OF_RESETS`) | Per-episode behavior | Config int; `0` = off-track is immediately terminal; `>0` recover-and-continue N times | Yes — per-car controller counter |
| Wheel friction (`mu`) | Per-spawn (Jetty: per-episode capable) | Xacro arg `friction_mu` → SDF surface friction `mu1`/`mu2`; sampled into `GYM_DR_FRICTION_MU` | Yes — each car spawned with its own friction |
| Visual recolor (track + background) | Per-episode reset | Native `visual_config` gz service (`gz.msgs.Visual`); contrast gate 0.10, `ambient = 0.6 × diffuse` | Yes — targets `racetrack_i` |
| Lighting | Per-episode / per-run | Native `light_config` gz service | Yes — per-arena lights |
| Sensor noise | Per-spawn / Python | SDF `<noise>` Gaussian (LiDAR mean 0.0, stddev 0.01) or a Python sensor layer | Yes — per-car sensor config |
| Steering bias | Action layer | Pure-Python offset added to the action before clamp-and-convert | Yes — per-car action layer |
| Motor delay | Action layer | Pure-Python command buffer delaying the applied action by N steps | Yes — per-car action layer |

The rest of this page details each knob.

## Reset-time knobs

These are sampled in the per-car controller at episode boundary
(`deepracer_env/agent_ctrl/rollout_agent_ctrl.py`). All three share one dedicated,
reproducible RNG (`self._dr_reset_rng_`), independent of global NumPy state.

### Start position — `start_ndist`

When `random_start` is enabled, `finish_episode()` samples a fresh normalized
distance for the next episode:

```python
if self._start_pos_behavior_['random_start']:
    self._data_dict_['start_ndist'] = float(self._dr_reset_rng_.random())
elif self._start_pos_behavior_['change_start']:
    # deterministic round-robin advance fallback
    self._data_dict_['start_ndist'] = (
        self._data_dict_['start_ndist']
        + self._start_pos_behavior_['round_robin_advance_dist']) % 1.0
```

Any `ndist` ∈ [0, 1) is a valid on-track start — the start pose is interpolated
from the track lanes, then the car is teleported there with the per-entity
`set_pose`. `random_start` takes precedence over the deterministic round-robin
advance (`CHANGE_START`). Config keys: `RANDOM_START`, `CHANGE_START`,
`ROUND_ROBIN_ADVANCE_DIST`, `START_POSITION_OFFSET` in
`deepracer_env/agent_ctrl/constants.py`.

### Start direction — `reverse_dir`

When `random_direction` is enabled, each episode flips an independent coin:

```python
if self._start_pos_behavior_['random_direction']:
    self._track_data_.reverse_dir = bool(self._dr_reset_rng_.random() < 0.5)
elif self._start_pos_behavior_['alternate_dir']:
    self._track_data_.reverse_dir = not self._track_data_.reverse_dir
```

`random_direction` takes precedence over the deterministic alternation
(`ALT_DIR`). Because `reverse_dir` lives on the arena's own `TrackData`, two
arenas can drive opposite directions on the same track.

### Recovery resets — `NUMBER_OF_RESETS`

This is a reset-*behavior* knob rather than a sampled value, but it belongs in the
DR catalog because it changes how failure is presented to the policy.

- Default `0`: an off-track is immediately terminal — a clean, immediate failure
  signal.
- `>0`: the DeepRacer "recover-and-continue" behavior — the car is teleported back
  and the **same** episode continues up to N times.

```python
# deepracer_env/environments/deepracer_env.py
ctrl_const.ConfigParams.NUMBER_OF_RESETS.value:
    int(os.getenv('GYM_DR_NUMBER_OF_RESETS', '0')),
```

History note: a non-zero default once let a drifting off-track car flail for
~19 s before reset; the default was moved to `0`. Override with
`GYM_DR_NUMBER_OF_RESETS` if recover-and-continue is wanted for sample
efficiency. Each car's controller owns its own reset counter
(`rollout_agent_ctrl.py`).

## Dynamics knobs

### Wheel friction — `mu`

Friction is a **spawn-time** parameter today: a xacro arg sets the wheel ODE
surface coefficients `mu1`/`mu2`. Each car can be spawned at a different friction,
so it is per-arena out of the box.

```xml
<!-- simulation/urdf/deepracer/racecar.xacro -->
<!-- friction_mu: wheel ODE surface mu (mu1/mu2), per-spawn friction DR. Default
     1.5 (baseline grip). dr-gym samples it from the DR `friction` Range and passes
     GYM_DR_FRICTION_MU so each run/worker spawns at a different friction. -->
<xacro:arg name="friction_mu" default="1.5" />
```

Default `1.5` is the baseline grip. dr-gym samples it from the `friction` Range and
passes `GYM_DR_FRICTION_MU` at spawn. Jetty can also retune friction *per-episode*
through the entity surface component; the spawn-time path is what ships today.

### Sensor noise

LiDAR carries Gaussian range noise via the SDF `<noise>` block, with knobs exposed
as xacro args (`simulation/urdf/deepracer/racecar.xacro`) and defaults mirrored in
`deepracer_env/sensors/constants.py`:

| Parameter | Default |
| --- | --- |
| `lidar_360_degree_noise_mean` / `LIDAR_360_DEGREE_NOISE_MEAN` | `0.0` |
| `lidar_360_degree_noise_stddev` / `LIDAR_360_DEGREE_NOISE_STDDEV` | `0.01` |

Camera noise can be added the same way (SDF `<noise>`) or applied in a Python
sensor layer on the decoded frame. Sensor config is per-car, so noise is
per-arena.

## Action-layer knobs (pure Python)

Steering bias and motor delay are realized entirely in the **pure-Python action
layer** — no simulator round-trip — as a pre-processing step on the agent action
before it is mapped to joint commands by
`deepracer_env/agent_ctrl/drive.py::action_to_joint_commands`. Because the layer is
per-car, both are independent per arena.

- **Steering bias**: a fixed angular offset added to the commanded steering angle,
  applied *before* the `[-30, 30]` clamp and the degrees→radians conversion (clamp
  order is load-bearing; see `drive.py`).
- **Motor delay**: the applied action is buffered so the car responds N steps late,
  emulating actuation latency.

The arithmetic in `drive.py` itself is preserved bit-for-bit from the legacy
controller (same steering angle to both hinges, same angular velocity to all four
wheels, no Ackermann decomposition in code); the DR bias/delay sit *in front of*
that mapping.

## Visual and lighting DR (native gz services)

### Visual recolor

Every episode (gated by `GYM_DR_VISUAL_DR`), the track surface and the
background/surround are recolored. The randomizer
(`deepracer_env/domain_randomizations/visual_randomizer.py`) samples per-channel
RGB and enforces a minimum contrast so the track stays separable in the camera
frame:

- Track surface: full-hue diffuse; ambient = `0.6 × diffuse`.
- Background: resampled (up to 8 tries) until its squared RGB distance from the
  track color is `≥ 0.10`.

On Jetty this is flushed through the **native** `visual_config` service — no custom
plugin. `RosGzBackend.set_visual_color` builds the `gz.msgs.Visual` request and
defaults ambient to `0.6 × diffuse` when not supplied
(`deepracer_env/sim_control/backends/ros_gz_backend.py`):

```text
gz service -s /world/<world>/visual_config \
  --reqtype gz.msgs.Visual --reptype gz.msgs.Boolean \
  --req 'name: "<visual>" parent_name: "<link>" material {
           ambient { r: .. g: .. b: .. a: .. }
           diffuse { r: .. g: .. b: .. a: .. } }'
```

Because the request names the visual and its parent link, the recolor targets one
arena's `racetrack_i`. The legacy "track visuals are world-shared, recolor once
from the primary agent" constraint no longer applies — each arena recolors its own
track entity.

The track model exposes two recolorable channels: `track::visual` (the full
track-surface mesh; lane/edge/center lines are baked into this one visual and
cannot be split from the road) and `background::visual` (the surround the camera
sees beyond the track). Visual recolor advertises the `visual_recolor`
[capability](sim-control-seam.md); callers guard with `supports()` so it degrades
gracefully on a backend that lacks it.

### Lighting

Lighting DR rides the native `light_config` gz service (verified present on
gz-sim 10.4.0). Like `visual_config`, it needs no custom C++ and is scoped per
entity/world, so per-arena lighting variety is available without disturbing
neighbors.

## Seeding and determinism

| RNG | Seed source | Purpose |
| --- | --- | --- |
| `Arena.dr_seed` | `base_seed + index` (`arena.py`) | Top-level per-arena seed |
| `_dr_reset_rng_` | `START_POSITION_OFFSET × 1e6` | Start position + direction schedule |
| `_visual_dr_rng_` | `GYM_DR_VISUAL_DR_SEED` (else nondeterministic) | Visual color schedule |

Visual colors are drawn from a separate RNG so they never perturb the
start/direction schedule. Given the same seeds and config, the start, direction,
and visual schedules replay exactly.

### Environment-variable gates

| Variable | Default | Effect |
| --- | --- | --- |
| `GYM_DR_VISUAL_DR` | `0` (off) | Enable per-episode visual recolor (`1`/`true` only) |
| `GYM_DR_VISUAL_DR_SEED` | unset | Seed the visual color RNG |
| `GYM_DR_FRICTION_MU` | `1.5` | Per-spawn wheel friction |
| `GYM_DR_NUMBER_OF_RESETS` | `0` | Recover-and-continue count |

## Applied-DR state as dataset labels

The frozen observation contract permits one user-authorized extension: enriching
the per-step `info` dict with the **applied-DR state** plus the **ground-truth
feature vector**. Together these are the supervised labels for the future
camera-to-feature-vector model — the model that lets the CNN-policy path
(camera → CNN) and the feature-vector path share training data.

`info` is assembled in `DeepRacerEnv.step`
(`deepracer_env/environments/deepracer_env.py`), which already surfaces the crash /
off-track / object flags from the reward params:

```python
info['objects_location'] = list(reward_params.get('objects_location', []))
info['is_crashed']       = bool(reward_params.get('is_crashed', False))
info['is_offtrack']      = bool(reward_params.get('is_offtrack', False))
info['closest_objects']  = list(reward_params.get('closest_objects', [-1, -1]))
```

The DR extension adds the values that were actually applied this episode — for
example the sampled `start_ndist` and `reverse_dir`, the spawn `friction_mu`, the
sampled track/background colors, lighting parameters, sensor-noise parameters, and
the action-layer `steering_bias` / `motor_delay`. A camera frame paired with its
applied-DR labels and its ground-truth feature vector is exactly the
`(image → features)` training pair the downstream model needs. Because each arena
randomizes independently, one multi-arena run yields a diverse, self-labeling
dataset.

## See also

- [The SimControl seam](sim-control-seam.md) — the per-entity port that makes DR per-arena
- [Tiled multi-arena](multi-arena.md) — how arenas tile, seed, and stay decoupled
- [World conversion](world-conversion.md) — how the 5 system plugins + lights land in each `.sdf`
- [Drive control](drive-control.md) — the action→joint mapping the action-layer DR sits in front of
