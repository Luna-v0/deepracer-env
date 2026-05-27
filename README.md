# deepracer-env

A [Gymnasium](https://gymnasium.farama.org/) environment for the AWS DeepRacer
autonomous-driving simulation, powered by **ROS Noetic** and **Gazebo 11**.

Instead of training against a fixed network with a plug-in reward function, this
repository exposes the full simulation as a standard `gymnasium.Env` so you can
bring any RL algorithm or neural-network architecture you like.

---

## Repository structure

```
deepracer-env/
├── deepracer_env/          # pip-installable Python package
│   ├── __init__.py         # registers DeepRacer-v0 with Gymnasium
│   ├── environments/       # DeepRacerEnv (gymnasium.Env subclass)
│   ├── sensors/            # Camera, LIDAR, sector-LIDAR sensor drivers
│   ├── agents/             # Agent (sensor + controller bundle)
│   ├── agent_ctrl/         # RolloutCtrl — Gazebo/ROS action publisher
│   ├── track_geom/         # Track geometry and waypoint utilities
│   └── ...
├── simulation/             # ROS/Gazebo workspace (meshes, worlds, URDF, routes)
│   ├── src/                # ROS colcon workspace — built inside Docker
│   ├── meshes/
│   ├── worlds/
│   └── urdf/
├── docker/                 # Dockerfiles and pip requirements for the container
│   ├── Dockerfile.base     # OS + ROS Noetic + Gazebo 11 system dependencies
│   ├── Dockerfile.build    # Compiles simulation/src/ with colcon
│   ├── Dockerfile.runtime  # Final runnable image
│   └── Dockerfile.dev-user # Dev variant: adds a non-root local user
├── examples/
│   └── train.py            # Minimal PPO training example (Stable-Baselines3)
├── sample-config/          # Example reward function and model metadata
├── pyproject.toml          # Package metadata and build configuration
├── requirements.txt        # Minimal Python runtime dependencies
├── build.sh                # Builds the Docker image chain
├── build-dev-bundle.sh     # Developer workflow: colcon build into local volume
└── VERSION
```

---

## How this fits into your project

**`deepracer_env` communicates with Gazebo exclusively through ROS**, which means
it can only run inside an environment that has ROS Noetic installed.  It cannot
be used as a standalone Python package on a bare machine.

The intended usage pattern is:

```
your-project/               ← your RL project repository
├── reward.py
├── train.py
├── pyproject.toml          ← depends on deepracer-env
└── Dockerfile              ← FROM the simulation image built here
```

Your training code and `deepracer_env` both run **inside a container that extends
the simulation image produced by this repository**.  You edit your code locally;
the container provides the ROS + Gazebo infrastructure.

---

## Step-by-step: using deepracer-env in your project

### Step 1 — Build the simulation image

Clone this repository and build the runtime Docker image:

```bash
git clone https://github.com/your-org/deepracer-env.git
cd deepracer-env
./build.sh -a cpu          # or -a gpu  for NVIDIA GPU support
```

This produces the image `awsdeepracercommunity/deepracer-env:<VERSION>-cpu`.

### Step 2 — Create your project repository

Create a new repository with this minimal structure:

```
my-deepracer-project/
├── reward.py
├── train.py
├── requirements.txt
├── pyproject.toml
└── Dockerfile
```

#### `pyproject.toml`

Do **not** list `deepracer-env` as a dependency here — it is already installed
in the base simulation image. Only declare your own project's dependencies:

```toml
[project]
name = "my-deepracer-project"
version = "0.1.0"
requires-python = ">=3.8"
dependencies = [
    "stable-baselines3",
]
```

#### `requirements.txt`

List the Python packages your training script directly imports e.g.:

```
stable-baselines3
```

Add any other packages your `train.py` requires (e.g. `numpy`, `pandas`, `wandb`).

#### `Dockerfile`

Extend the simulation image, install your dependencies first (for better layer
caching), then install your project on top of it:

```dockerfile
ARG SIMAPP_TAG=latest-cpu
FROM awsdeepracercommunity/deepracer-env:${SIMAPP_TAG}

# Install third-party dependencies first — cached unless requirements.txt changes
COPY requirements.txt /workspace/requirements.txt
RUN pip install --no-cache-dir -r /workspace/requirements.txt

# Copy and install your project
COPY . /workspace
RUN pip install --no-cache-dir -e /workspace

# Default: run your training script
ENTRYPOINT ["/bin/bash", "-c"]
CMD ["source /opt/ros/noetic/setup.bash && source /opt/simapp/setup.bash && ./run.sh run deepracer_env.launch & sleep 5 && python3 /workspace/train.py"]
```

#### `train.py`

```python
from deepracer_env.environments.deepracer_env import DeepRacerEnv
from stable_baselines3 import PPO


def reward_function(params: dict) -> float:
    if not params["all_wheels_on_track"]:
        return 1e-3
    return float(params["progress"] * params["speed"] / 4.0)


env = DeepRacerEnv(reward_fn=reward_function)

model = PPO("MultiInputPolicy", env, verbose=1)
model.learn(total_timesteps=100_000)
model.save("my_model")
env.close()
```

### Step 3 — Build and run your project image

Inside your project repository:

```bash
docker build \
  --build-arg SIMAPP_TAG=<VERSION>-cpu \
  -t my-deepracer-project:latest .
```

> **Rebuilding after changes to the base image:** Docker caches the `FROM` layer
> by tag. If you rebuilt `deepracer-env` with the same tag, add `--no-cache` to
> force the project image to pick up the new base:
> ```bash
> docker build --no-cache --build-arg SIMAPP_TAG=<VERSION>-cpu -t my-deepracer-project:latest .
> ```

```bash
docker run --rm \
  -e WORLD_NAME=reinvent_base \
  -e ENABLE_GUI=False \
  my-deepracer-project:latest
```

Set `ENABLE_GUI=True` and expose port `5900` to view the Gazebo simulation
over VNC:

```bash
docker run --rm \
  -e WORLD_NAME=reinvent_base \
  -e ENABLE_GUI=True \
  -p 5900:5900 \
  my-deepracer-project:latest
```

Then connect a **desktop VNC client** (e.g. [TigerVNC](https://tigervnc.org/),
RealVNC Viewer) to `localhost:5900`. No password is set.

#### Simulation speed: `RTF_OVERRIDE`

Control the simulation speed via the `RTF_OVERRIDE` environment variable
(Real-Time Factor):

```bash
docker run --rm \
  -e WORLD_NAME=reinvent_base \
  -e RTF_OVERRIDE=2.0 \
  my-deepracer-project:latest
```

| Value | Effect |
|-------|--------|
| `1.0` | Real-time simulation |
| `> 1.0` | Faster than real-time (e.g., `2.0` = 2× speed) |
| `< 1.0` | Slower than real-time (e.g., `0.5` = half speed) |

The actual achievable speed depends on your hardware — if your system cannot
compute physics fast enough at the requested rate, Gazebo will throttle down
to the maximum your hardware can sustain.

> **Note:** Browser-based noVNC clients will not work — port 5900 is a raw VNC
> port, not a WebSocket endpoint.

### Step 4 — Iterate without rebuilding the image

Mount your project source as a volume to pick up code changes instantly without
rebuilding:

```bash
docker run --rm \
  -e WORLD_NAME=reinvent_base \
  -v $(pwd):/workspace \
  my-deepracer-project:latest \
  bash -c "source /opt/ros/noetic/setup.bash && source /opt/simapp/setup.bash && ./run.sh run deepracer_env.launch & sleep 5 && python3 /workspace/train.py"
```

---

## API reference

### Action space

`Box([-30, 0.1], [30, 4.0], dtype=float32)` — `[steering_angle_deg, speed_m_s]`

### Observation space

`Dict` — one entry per active sensor:

| Sensor key | Shape | dtype |
|------------|-------|-------|
| `CAMERA` / `OBSERVATION` / `LEFT_CAMERA` | `(120, 160, 3)` | uint8 |
| `STEREO` | `(120, 160, 2)` | uint8 |
| `LIDAR` | `(64,)` | float32 |
| `SECTOR_LIDAR` | `(8,)` | float32 |
| `DISCRETIZED_SECTOR_LIDAR` | `(N×M,)` | float32 |

### Reward function parameters

The callable passed as `reward_fn` receives a `dict` with the following keys:

| Key | Type | Description |
|-----|------|-------------|
| `all_wheels_on_track` | bool | All four wheels on track |
| `progress` | float | Lap progress 0–100 % |
| `speed` | float | Current speed m/s |
| `steering_angle` | float | Current steering angle in degrees |
| `distance_from_center` | float | Distance from track centre line |
| `is_left_of_center` | bool | Car is left of centre line |
| `waypoints` | list | All track waypoints |
| `closest_waypoints` | list[int] | Indices of previous and next waypoints |
| `steps` | int | Step count within the current episode |
| `is_offtrack` | bool | All four wheels off the track this step |
| `is_crashed` | bool | Bounding-box collision with an obstacle this step |
| `objects_location` | list[(x, y)] | World positions of all spawned obstacles (empty when OA off) |
| `closest_objects` | list[int] | `[prev_idx, next_idx]` in the obstacle list; `-1` means none |
| `objects_distance` | list[float] | Each obstacle's projection distance along the centerline |
| `object_in_camera` | bool | Any obstacle inside the camera frustum |

The object-related keys are always present (defaults are empty list / `False` /
`[-1, -1]`) so reward functions never need `params.get(...)`.

### Customising sensors

```python
from deepracer_env.sensors.constants import Input

env = DeepRacerEnv(
    reward_fn=my_reward,
    sensors=[Input.CAMERA.value, Input.LIDAR.value],
)
```

Available sensors: `CAMERA`, `LEFT_CAMERA`, `STEREO`, `LIDAR`, `SECTOR_LIDAR`,
`DISCRETIZED_SECTOR_LIDAR`.

### Customising controller parameters

```python
import deepracer_env.agent_ctrl.constants as ctrl_const

env = DeepRacerEnv(
    reward_fn=my_reward,
    config={
        ctrl_const.ConfigParams.COLLISION_PENALTY.value: 5.0,
        ctrl_const.ConfigParams.OFF_TRACK_PENALTY.value: 2.0,
        ctrl_const.ConfigParams.NUMBER_OF_RESETS.value: 0,
    },
)
```

### Object Avoidance

Spawn N static obstacles (cardboard-box-shaped) on the track every episode and
expose their state to the reward function. The measurement side (bbox
collision detection, frustum check, `objects_*` reward params) is always
active in the env; the `object_avoidance` kwarg enables the spawn side.

```python
from deepracer_env import DeepRacerEnv
from deepracer_env.object_avoidance import ObjectAvoidanceConfig

def my_reward(p):
    if p["is_crashed"]:
        return -10.0
    if not p["all_wheels_on_track"]:
        return 1e-3
    return float(p["progress"] * p["speed"] / 4.0)

env = DeepRacerEnv(
    reward_fn=my_reward,
    object_avoidance=ObjectAvoidanceConfig(
        n_obstacles=3,
        placement="random_on_waypoints",   # or "fixed" / "callable"
        min_spacing_m=2.5,
        lane="any",                        # any / inner / outer / center
        terminate_on_collision=True,       # AWS DeepRacer OA default
    ),
)

obs, info = env.reset(seed=42)
# info["objects_location"] -> [[x, y], [x, y], [x, y]]
```

`ObjectAvoidanceConfig` fields:

| Field | Default | Notes |
|-------|---------|-------|
| `enabled` | `True` | Set `False` to skip the spawn side entirely |
| `n_obstacles` | `2` | Number of boxes per episode |
| `placement` | `"random_on_waypoints"` | Strategy — see below |
| `fixed_positions` | `None` | List of `(x, y)` when `placement="fixed"` |
| `placement_fn` | `None` | Callable `(np_random, track_data) -> list[(x, y[, yaw])]` for `placement="callable"` |
| `min_spacing_m` | `2.0` | Enforced via rejection sampling along the centerline |
| `lane` | `"any"` | Which lane to project obstacles onto |
| `terminate_on_collision` | `True` | `False` keeps `is_crashed` live across the trajectory (used for constrained-RL cost signals) |
| `obstacle_sdf_path` | bundled `obstacle_box.sdf` | Custom Gazebo SDF |
| `name_prefix` | `"obstacle"` | Must contain `"obstacle"` so the mercy-reset path still recognises it |

Layouts are deterministic per `env.reset(seed=...)` call — same seed → same
layout, different seed → different layout. On `env.close()` every spawned
obstacle is removed from Gazebo and from `TrackData`.

See `examples/train_object_avoidance.py` for a Stable-Baselines3 PPO trainer
that uses this feature end to end.

### Full control — bring your own Agent

Build an `Agent` yourself and pass it directly; `reward_fn`, `sensors`, and
`config` are then ignored:

```python
from deepracer_env.agents.agent import Agent
from deepracer_env.sensors.composite_sensor import CompositeSensor
from deepracer_env.agent_ctrl.rollout_agent_ctrl import RolloutCtrl

sensor = CompositeSensor()
ctrl   = RolloutCtrl(my_config, my_metrics, is_training=True)
env    = DeepRacerEnv(agent=Agent(sensor, ctrl))
```

---

## Building the simulation image

```bash
./build.sh -a cpu    # CPU-only image
./build.sh -a gpu    # NVIDIA GPU image (requires nvidia-docker)
./build.sh -a "cpu gpu" -p myreeg  # both architectures, custom registry prefix
```

The build chain:

1. `Dockerfile.base` — installs ROS Noetic + Gazebo 11 into Ubuntu 20.04
2. `Dockerfile.build` — compiles `simulation/src/` with colcon
3. `Dockerfile.runtime` — assembles the final image

## Developer workflow

To compile the ROS packages locally (output on your disk rather than baked into
an image):

```bash
./build-dev-bundle.sh          # compiles into ./install/
```

To also start Gazebo immediately after building, use the `-g` flag. This
requires a few environment variables and the `sagemaker-local` Docker network:

```bash
# One-time network setup
docker network create sagemaker-local

# Set required variables (on Windows/Git Bash, $USER may not be set — whoami is used automatically)
export DR_SIMAPP_IMAGE=$(cat VERSION)-cpu   # or -gpu
export DR_WORLD_NAME=reinvent_base
export USER_UID=$(id -u)
export USER_GID=$(id -g)

./build-dev-bundle.sh -g
```

To iterate without rebuilding the image, volume-mount the local `install/` and
`deepracer_env/` into a running container:

```bash
docker run --rm \
  -e WORLD_NAME=reinvent_base \
  -v $(pwd)/install:/opt/simapp \
  -v $(pwd)/deepracer_env:/usr/local/lib/python3.8/dist-packages/deepracer_env \
  my-deepracer-project:latest
```

> **Windows / Git Bash note:** `$USER` is not set by default on Windows. The
> `build-dev-bundle.sh` script falls back to `whoami` automatically. If you
> encounter UID/GID related errors, verify that `id -u` and `id -g` return
> valid values in your shell.

---

## License

Apache License 2.0 — see [LICENSE](LICENSE).
