# Drive & ros2_control

How an agent action becomes wheel and steering motion in Gazebo. This page covers
the pure action→joint math, the `ros2_control` controller groups that carry the
setpoints, and the joint-order contract the two sides share.

The driving stack has two halves:

1. **A pure Python mapping** — `deepracer_env/agent_ctrl/drive.py` turns the
   2-vector action into joint setpoints. No ROS, no Gazebo, no I/O. Host-tested.
2. **A `ros2_control` runtime** — controllers in the simulated car apply those
   setpoints to the URDF joints through the `gz_ros2_control` system plugin.

The boundary between them is two `std_msgs/Float64MultiArray` messages. See also
[the sim-control seam](sim-control-seam.md) for the surrounding ports-and-adapters
design.

## The action

The frozen gymnasium action space is `Box(low=[-30, 0.1], high=[30, 4.0])`:

| Index | Meaning             | Units | Range        |
|-------|---------------------|-------|--------------|
| `0`   | `steering_angle`    | deg   | `[-30, 30]`  |
| `1`   | `speed`             | m/s   | `[0.1, 4.0]` |

These bounds are part of the frozen obs/action contract and must not change —
trained policies depend on them byte-for-byte.

## Action → joint math

The whole conversion lives in
[`action_to_joint_commands`](../../deepracer_env/agent_ctrl/drive.py). Two rules
make it correct, and both matter:

### 1. Clamp, then convert

Inputs are clamped in their **native units** (degrees, m/s) *before* unit
conversion. Clamping after conversion (in radians / rad·s⁻¹) would change the
effective range, so the order is load-bearing.

```python
steering_rad  = clamp(steering_deg, -30, 30) * math.pi / 180.0
angular_speed = clamp(speed_mps, 0.1, 4.0) / wheel_radius
```

### 2. No Ackermann decomposition

The **same** steering angle goes to *both* front hinges, and the **same** angular
velocity goes to *all four* wheels. The drive code does no left/right or
front/rear differential. The car's Ackermann geometry is realised by the URDF
joint constraints in Gazebo — not in this code.

```python
wheel_velocities  = [angular_speed] * 4   # all wheels identical
steering_positions = [steering_rad]  * 2  # both hinges identical
```

### Wheel radius is version-locked

Linear speed becomes wheel angular velocity by dividing by the wheel radius. The
radius is keyed to the simapp model version, so an off-by-one here silently
changes how fast the car drives for a given action.

| `simapp_version` | wheel radius (m) |
|------------------|------------------|
| `1.0`            | `0.1`            |
| `2.0`            | `0.0277`         |
| `>= 3.0`         | `0.035`          |

The current default is `simapp_version = 5.0`, which uses the `>= 3.0` radius of
**0.035 m**. `get_wheel_radius()` resolves the value; you can also pass
`wheel_radius=` to override it directly.

### Worked example

Full-lock right, full speed, v5:

```python
from deepracer_env.agent_ctrl.drive import action_to_joint_commands

wheels, steering = action_to_joint_commands(30.0, 4.0, simapp_version=5.0)
# wheels   == [114.2857, 114.2857, 114.2857, 114.2857]  rad/s  (= 4.0 / 0.035)
# steering == [0.5236, 0.5236]                            rad    (= 30 deg)
```

Out-of-range inputs clamp first. `action_to_joint_commands(90.0, 0.0)` yields
steering `radians(30)` and wheels `0.1 / 0.035` — the speed is clamped *up* to the
`0.1` m/s floor before division.

### Stop command

[`zero_commands()`](../../deepracer_env/agent_ctrl/drive.py) returns
`([0, 0, 0, 0], [0, 0])` directly, bypassing the speed clamp. This is the explicit
stop published on reset and in PARK/PAUSE — the one case where wheel velocity is
allowed below the `0.1` m/s action floor.

## ros2_control runtime

`ros2_control` 6.7 removed the `velocity_controllers` and `position_controllers`
packages. The drive uses the generic
`forward_command_controller/ForwardCommandController` instead — **one controller
per group**:

| Group      | Controller type            | Interface  | Joints | Units |
|------------|----------------------------|------------|--------|-------|
| wheels     | `ForwardCommandController` | `velocity` | 4      | rad/s |
| steering   | `ForwardCommandController` | `position` | 2      | rad   |

Each group is commanded with a single `std_msgs/Float64MultiArray` whose entries
are positional — they line up with that group's `joints:` list (see the contract
below). The two arrays returned by `action_to_joint_commands` map straight onto
the two group command topics.

```
drive.action_to_joint_commands(steer_deg, speed) ->
    ([v, v, v, v], [a, a])
         |              |
         v              v
  Float64MultiArray  Float64MultiArray
  -> wheels group     -> steering group
     (velocity)          (position)
```

> Transport note: under the legacy ros_control stack these six setpoints were six
> separate `std_msgs/Float64` messages (one controller per joint). Under
> `ros2_control` they are two grouped `Float64MultiArray` messages. **Only the
> transport changed — the arithmetic in `drive.py` is preserved bit-for-bit.**

### Joint state feedback

A `joint_state_broadcaster/JointStateBroadcaster` publishes `/joint_states` (per
car, namespaced) so consumers can read back joint positions and velocities. It is
a broadcaster, not a controller — it issues no commands.

### The Gazebo bridge: gz_ros2_control

The simulated car's URDF carries the `gz_ros2_control` system plugin, which loads
the `GazeboSimSystem` hardware interface. This is what binds the `ros2_control`
controllers to the actual Gazebo (gz-sim 10.4.0, Jetty) joints — the controllers
write command interfaces, `GazeboSimSystem` applies them to DART, and reads state
back each physics step. There is **no custom C++** here; `gz_ros2_control` and
`GazeboSimSystem` are upstream components.

This replaces the legacy `transmission` blocks
(`hardware_interface/EffortJointInterface`) still visible in the old xacros at
`simulation/urdf/deepracer/macros.xacro`.

## The joint-order contract

The positional arrays in `drive.py` only mean anything if their order matches the
controller configuration in `config/ros2_control.yaml`. The two **must** stay in
lockstep. The canonical order is defined in `drive.py` as `WHEEL_JOINTS` and
`STEERING_JOINTS`:

| Group    | Index 0                       | Index 1                        | Index 2              | Index 3               |
|----------|-------------------------------|--------------------------------|----------------------|-----------------------|
| wheels   | `left_rear_wheel_joint`       | `right_rear_wheel_joint`       | `left_front_wheel_joint` | `right_front_wheel_joint` |
| steering | `left_steering_hinge_joint`   | `right_steering_hinge_joint`   | —                    | —                     |

These names match the joints declared in the car URDFs under
`simulation/urdf/deepracer/` (e.g. `deepracer_stereo_cam_lidar.urdf`).

The controller manifest pins the same order. Its shape:

```yaml
controller_manager:
  ros__parameters:
    wheels_velocity_controller:
      type: forward_command_controller/ForwardCommandController
    steering_position_controller:
      type: forward_command_controller/ForwardCommandController
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

wheels_velocity_controller:
  ros__parameters:
    interface_name: velocity
    joints:
      - left_rear_wheel_joint      # array index 0
      - right_rear_wheel_joint     # array index 1
      - left_front_wheel_joint     # array index 2
      - right_front_wheel_joint    # array index 3

steering_position_controller:
  ros__parameters:
    interface_name: position
    joints:
      - left_steering_hinge_joint  # array index 0
      - right_steering_hinge_joint # array index 1
```

If you reorder either list, reorder the other to match. Because all wheel values
are identical and both steering values are identical, a wrong order will *not*
crash and may not even look wrong in simple cases — it will only surface as subtly
incorrect motion. Treat the order as a contract, not a convenience.

## Why this split

Keeping the math ROS-free has three payoffs:

- **Host-testable.** `drive.py` runs and is unit-tested without ROS or Gazebo
  (`deepracer_env/sim_control/tests/test_sim_control_core.py` checks the exact
  arithmetic, the clamp-before-convert order, and the version radii).
- **Policy-stable.** The conversion is the sim-to-real contract surface. Freezing
  it bit-for-bit means a policy trained on the legacy stack drives identically on
  the ported stack — the [seam](sim-control-seam.md) changed underneath it, the
  numbers did not.
- **Transport-agnostic.** Swapping six `Float64` topics for two
  `Float64MultiArray` group commands was a one-place change; the action layer was
  untouched.

## Multi-car note

Every controller and the joint-state broadcaster are namespaced per car
(`/car_i/...`), so each car in a [tiled arena](sim-control-seam.md) drives
independently within the one shared gz-sim process. Steering bias and motor delay
domain randomization are applied in the pure-Python action layer, *upstream* of
`action_to_joint_commands`, so they never touch the controller config.
