# Observation/action contract

This is the **frozen boundary** between a trained policy and the simulator. Every
RL algorithm, every exported ONNX bundle, and the physical car all agree on the
shapes and meanings defined here. The port behind [the sim-control seam](sim-control-seam.md)
can change its backend, its ROS distro, even its physics engine — but the
observation and action surface stays **byte-for-byte stable**. That stability is
what makes the sim-to-real boundary the *contract*, not the ROS ABI.

> **Why "frozen" is load-bearing.** A policy trained months ago must still load
> and run. If the action bounds shift or a reward-param key is renamed, every
> checkpoint silently mis-drives or every reward function `KeyError`s. Treat this
> page as a change-control gate: the [must-never-change vs. may-grow](#what-must-never-change-vs-what-may-grow)
> table at the bottom is the rule.

## The frozen contract

Three things are nailed down: the **action space**, the **observation space**
(a `Dict`), and the **26 reward-parameter keys**. Plus the signatures of
`reset` / `step` / `set_world`.

### Action space

A continuous `gymnasium.spaces.Box` of shape `(2,)`:

```python
# deepracer_env/environments/deepracer_env.py
DEFAULT_ACTION_SPACE = gymnasium.spaces.Box(
    low=np.array([-30.0, 0.1], dtype=np.float32),
    high=np.array([30.0, 4.0], dtype=np.float32),
    dtype=np.float32,
)
```

| Index | Name | Units | Range | Sign convention |
|-------|------|-------|-------|-----------------|
| `0` | `steering_angle_deg` | degrees | `[-30, 30]` | positive turns the car **left** |
| `1` | `speed_m_s` | metres/second | `[0.1, 4.0]` | forward speed |

The action is mapped to `ros2_control` joint commands in
`deepracer_env/agent_ctrl/drive.py`. The arithmetic is preserved bit-for-bit
from the legacy controller because trained policies depend on it:

- **Clamp before converting.** Steering is clamped to `[-30, 30]` *then*
  multiplied by `pi/180`; speed is clamped to `[0.1, 4.0]` *then* divided by the
  wheel radius. Clamping in radians/rad-s would change the effective range.
- **Wheel radius is version-locked.** simapp v5 (the default) uses
  `wheel_radius = 0.035 m`. An off-by-one here silently changes how fast the car
  drives for a given action.
- **No Ackermann decomposition in code.** The *same* steering angle goes to both
  front hinges and the *same* angular velocity to all four wheels. Gazebo's joint
  geometry realises the Ackermann split — see [the drive layer](drive-and-control.md).

### Observation space

A `gymnasium.spaces.Dict` whose **keys are the active sensor `Input.value`
strings** and whose values are per-sensor `Box`es. The exact set of keys depends
on which sensors are configured; the default is a single front camera
(`["FRONT_FACING_CAMERA"]`).

```python
env = DeepRacerEnv(reward_fn=my_reward,
                   sensors=[Input.CAMERA.value, Input.LIDAR.value])
# env.observation_space == Dict({
#     "FRONT_FACING_CAMERA": Box(0, 255, (120, 160, 3), uint8),
#     "LIDAR":               Box(0.15, 1.0, (64,), float32),
# })
```

The space is assembled by `deepracer_env/sensors/utils.py:get_observation_space`
and merged across sensors by the `CompositeSensor`. See the
[sensor topics and shapes](#sensor-topics-and-shapes) table.

### The 26 reward-parameter keys

`step()`'s reward function receives a flat `dict` with **exactly these 26 keys**,
defined by `RewardParam` in `deepracer_env/agent_ctrl/constants.py`. The key
strings and their semantics are part of the frozen contract: reward functions
written by users (and the dr-gym reward library) index them by name.

| # | Key | Type | Default | Meaning |
|---|-----|------|---------|---------|
| 1 | `all_wheels_on_track` | bool | `True` | all four wheels inside the track |
| 2 | `x` | float | `0.0` | car x position |
| 3 | `y` | float | `0.0` | car y position |
| 4 | `heading` | float | `0.0` | car heading angle (deg) |
| 5 | `distance_from_center` | float | `0.0` | distance from centerline |
| 6 | `projection_distance` | float | `0.0` | distance projected onto centerline |
| 7 | `progress` | float | `0.0` | track progress, `[0, 1]` (as %) |
| 8 | `steps` | int | `0` | steps taken this episode |
| 9 | `speed` | float | `0.0` | car speed (m/s) |
| 10 | `steering_angle` | float | `0.0` | car steering angle (deg) |
| 11 | `track_width` | float | `0.0` | track width |
| 12 | `track_length` | float | `0.0` | track length |
| 13 | `waypoints` | list[tuple] | `0` | centerline waypoints as `(x, y)` |
| 14 | `closest_waypoints` | list[int] | `[0, 0]` | `[prev, next]` waypoint indices |
| 15 | `is_left_of_center` | bool | `False` | car left of centerline |
| 16 | `is_reversed` | bool | `False` | car facing reverse direction |
| 17 | `closest_objects` | list[int] | `[-1, -1]` | `[prev, next]` object indices (`-1` = none) |
| 18 | `objects_location` | list[tuple] | `[]` | all object `(x, y)` locations |
| 19 | `objects_left_of_center` | list[bool] | `[]` | each object left of centerline |
| 20 | `object_in_camera` | bool | `False` | any object in the camera frustum |
| 21 | `objects_speed` | list[float] | `[]` | each object's speed |
| 22 | `objects_heading` | list[float] | `[]` | each object's heading |
| 23 | `objects_distance_from_center` | list[float] | `[]` | each object's distance from center |
| 24 | `objects_distance` | list[float] | `[]` | each object's centerline projection |
| 25 | `is_crashed` | bool | `False` | crashed into object/bot car |
| 26 | `is_offtrack` | bool | `False` | all four wheels off-track |

`RewardParam.make_default_param()` returns this dict pre-filled with defaults;
`RewardParam.validate_dict()` raises if any key is missing.

### `reset` / `step` / `set_world` signatures

From `deepracer_env/environments/deepracer_env.py`:

```python
def reset(self, *, seed=None, options=None
          ) -> Tuple[Dict[str, np.ndarray], Dict[str, Any]]:
    # returns (observation, info)

def step(self, action: np.ndarray
         ) -> Tuple[Dict[str, np.ndarray], float, bool, bool, Dict[str, Any]]:
    # returns (observation, reward, terminated, truncated, info)

def set_world(self, world_name: str) -> None:
    # swap the rendered track at runtime, between episodes only
```

Contract details that must not drift:

- **`step` returns the standard gymnasium 5-tuple.** `terminated` is `True` on a
  natural end (lap complete, off-track, time-up). `truncated` is **always
  `False`** — there is no external time limit beyond what the reset-rules manager
  handles.
- **`action` is `[steering_angle_deg, speed_m_s]`** — the action space above.
- **`reset` returns `(obs, info)`**; `info` carries at least `objects_location`
  and `is_crashed`.
- **`set_world` is a between-episodes operation.** It may only be called *after*
  `reset()` has run at least once and *before* the next `reset()` — never
  mid-episode. It pauses physics, deletes and respawns the track model, rebuilds
  the `TrackData` geometry the reward and reset rules depend on, teleports the
  car, and drains stale sensor frames. Calling it before the first `reset()`
  raises `RuntimeError`; a missing-asset world raises `ValueError`; a dead
  `gzserver` mid-swap raises `WorldSwapError` (checkpoint and restart the sim
  container).

## Observation modes

The `Dict` observation supports two distinct policy front-ends. Both consume the
*same* env; they differ only in which keys the policy reads.

### 1. CNN-policy mode (camera)

The camera image is the policy input. dr-gym's `DeepRacerCNN`
(`gym_dr/networks.py`) is a config-driven `BaseFeaturesExtractor`: each image key
in the obs `Dict` runs through the DeepRacer conv stack, non-image keys are
flattened, per-key outputs are concatenated and projected to `features_dim`. SB3
hands it observations already preprocessed (channels-first, frame-stacked
grayscale, e.g. `(N, 4, 120, 160)`). `normalize_images=False` matches the
physical car, which feeds raw uint8-valued floats.

This is the **deployable** path: the camera-only actor is what ships to the car.

### 2. Feature-vector mode

The CNN input is replaced by a low-dimensional **state feature vector** built
from the reward-param elements (see `gym_dr/perception.py:ACTOR_FEATURES` and
`gym_dr/environment.py:FeatureObsWrapper`). Training over the feature vector is
far faster and is the basis of the asymmetric actor-critic work, where the critic
sees privileged ground-truth features the actor cannot. See dr-gym's
`docs/reports/perception.md`.

The bridge between the two: a future **camera → feature-vector model** is trained
on ground-truth feature labels (an [authorised extension](#authorised-extensions)
of the `info` dict), so a feature-trained policy can eventually run from the
camera. The non-negotiable guardrail is that the **deployed actor never touches
privileged sim state** — asymmetry lives only in the discarded critic.

## Sensor topics and shapes

Sensors are enumerated by `Input` in `deepracer_env/sensors/constants.py`. Each
subscribes to a per-car namespaced topic and produces a fixed-shape `Box`. Images
are decoded with PIL and resized to `TRAINING_IMAGE_SIZE = (160, 120)` (width ×
height); the obs `Box` shape is `(H, W, C) = (120, 160, C)`.

| `Input` value | ROS topic (per car) | Message | Obs `Box` | Notes |
|---------------|---------------------|---------|-----------|-------|
| `FRONT_FACING_CAMERA` | `/<car>/camera/zed/rgb/image_rect_color` | `Image` | `Box(0, 255, (120, 160, 3), uint8)` | RGB, the default sensor |
| `OBSERVATION` | `/<car>/camera/zed/rgb/image_rect_color` | `Image` | `Box(0, 255, (120, 160, 3), uint8)` | v1-compatible single camera |
| `LEFT_CAMERA` | `/<car>/camera/zed/rgb/image_rect_color` | `Image` | `Box(0, 255, (120, 160, 3), uint8)` | same topic, distinct space |
| `STEREO_CAMERAS` | `…/zed/rgb/image_rect_color` + `…/zed_right/rgb/image_rect_color_right` | `Image` ×2 | `Box(0, 255, (120, 160, 2), uint8)` | greyscale stereo pair |
| `LIDAR` | `/<car>/scan` | `LaserScan` | `Box(0.15, 1.0, (64,), float32)` | 64 rays |
| `SECTOR_LIDAR` | `/<car>/scan` | `LaserScan` | `Box(0.0, 1.0, (8,), float32)` | binary per-sector flags |
| `DISCRETIZED_SECTOR_LIDAR` | `/<car>/scan` | `LaserScan` | `Box(0.0, 1.0, (n_sectors·m,), float32)` | needs `model_metadata.lidar_config` |

LIDAR hardware constants (`deepracer_env/sensors/constants.py`): **64 samples**,
FOV `±2.61799 rad`, raw range `0.15 – 12.0 m`, noise stddev `0.01`. The training
obs normalises range to `[0.15, 1.0]`.

Topics are bridged from gz-sim into ROS 2 by `ros_gz_bridge` / `ros_gz_image`.
The world SDF must carry the `Sensors` system plugin or these topics never
publish.

## Authorised extensions

The contract is frozen, but two **additive** extensions are explicitly
sanctioned. Both *grow* the surface without breaking any existing field.

### Per-arena `set_world`

Today `set_world` swaps the single shared track. The authorised extension is to
make it **per-arena**, lifting the legacy "multi-car can't rotate worlds" limit.
Because every verb behind the seam is per-entity (delete/spawn the arena's own
`racetrack_i` at its grid offset — see [tiled multi-arena](tiled-multi-arena.md)),
each arena can rotate its own track independently without disturbing its
neighbours. This generalises the old global-world-swap coupling; it does **not**
change the `reset`/`step` shapes.

### `info` enrichment for the dataset

`step()` already surfaces `objects_location`, `is_crashed`, `is_offtrack`, and
`closest_objects` into `info`. The authorised growth is to add, per step:

- **applied domain-randomization state** (which DR was sampled this episode —
  friction, recolor, lighting, sensor noise, steering bias, motor delay), and
- the **ground-truth feature vector** as labels,

so the `info` stream doubles as a labelled dataset for the future
camera → feature-vector model. See [domain randomization](domain-randomization.md)
for the DR catalogue that gets logged. `info` is **additive only**: new keys may
appear, existing keys never change meaning.

## What must never change vs. what may grow

| Frozen — never change | May grow — additive only |
|-----------------------|--------------------------|
| Action `Box` bounds `[-30, 0.1] → [30, 4.0]`, dtype `float32`, order `[steering_deg, speed_mps]` | New **optional sensors** (new `Input` members → new obs `Dict` keys) |
| The **26 reward-param keys** — names, types, semantics | New keys in the **`info`** dict (applied-DR state, feature labels) |
| Observation is a **`Dict` keyed by sensor name**; each sensor's `Box` shape/dtype/bounds | **`set_world`** target scope (global → per-arena) |
| `reset` / `step` / `set_world` signatures; the gymnasium **5-tuple** with `truncated == False` | Backend swap behind [the seam](sim-control-seam.md) (RosGz → simulation_interfaces) — does not touch obs/action |
| Drive arithmetic: clamp-then-convert, version-locked wheel radius, no in-code Ackermann | DR catalogue (more randomizers), provided they only feed `info` |

The single rule: **a checkpoint trained against this contract must keep driving
correctly.** Anything that could break that — renaming a key, narrowing a bound,
adding a required action dimension — is forbidden. Anything purely additive that a
trained policy can ignore is allowed.

## See also

- [The sim-control seam](sim-control-seam.md) — the per-entity port the env drives.
- [The drive layer](drive-and-control.md) — action → `ros2_control` joint commands.
- [Tiled multi-arena](tiled-multi-arena.md) — N decoupled cars, per-arena reset.
- [Domain randomization](domain-randomization.md) — the catalogue logged into `info`.
- [Deployment & sim-to-real](deployment.md) — why this contract, not the ROS ABI,
  is the boundary; the car runs Foxy/Jazzy, not Lyrical.
