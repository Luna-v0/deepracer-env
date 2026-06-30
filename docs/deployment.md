# Deployment & sim-to-real

The ROS 2 port changed how the *simulator* is built and driven. It did **not**
change how a trained policy gets onto a physical DeepRacer. The deployment path
is the same one the legacy stack used, because the sim-to-real boundary was never
the ROS ABI — it is the **observation/action contract**. As long as that contract
holds, the car and the simulator are free to run completely different software
stacks.

If you are looking for the broader picture first, start with the
[architecture overview](architecture/overview.md).

## TL;DR

- A trained policy ships to the car as a **frozen graph** (ONNX or TF `.pb`),
  packaged by dr-gym's `scripts/export_bundle.py`.
- The on-car **inference engine (OpenVINO / TFLite) is ROS-distro-agnostic**. No
  ROS message types cross the sim-to-real boundary.
- The **physical car runs ROS 2 Foxy/Jazzy**; the simulator runs **ROS 2 Lyrical
  Luth**. That mismatch is expected and fine — see [below](#the-car-runs-foxyjazzy-the-sim-runs-lyrical-and-thats-fine).
- The only thing that has to match on both sides is the **frozen contract**:
  action space `Box(low=[-30, 0.1], high=[30, 4.0])` and the observation `Dict`
  keyed by sensor name.
- A separate goal — a **camera → feature-vector model** — is fed by labels this
  environment collects in sim: every observation's `info` carries the
  ground-truth feature vector alongside the camera image.

## The boundary is the contract, not the ROS ABI

The guiding decision for the whole port was to keep the sim-to-real boundary at
the obs/action contract. Concretely, that means the *only* artifact that travels
from training to the car is the policy graph plus its `model_metadata.json`.
Nothing about how the simulator is wired — gz-transport services, the
`SimControl` seam in
[`deepracer_env/sim_control/`](../deepracer_env/sim_control/interface.py),
ros2_control, the `ros_gz` bridge — is part of the deliverable.

| Crosses the boundary | Stays in the simulator |
| --- | --- |
| The trained policy as a frozen graph (`.onnx` / `.pb`) | The ROS 2 distro (Lyrical) |
| `model_metadata.json` (action space + sensor config) | Gazebo Jetty, DART physics, gz-transport services |
| The observation/action contract it was trained against | The `SimControl` port and its backends |
| — | ros2_control / `ForwardCommandController`, the URDF |
| — | `ros_gz_bridge` / `ros_gz_image` sensor bridging |

Because the policy is a frozen graph, it carries no ROS dependency at all. It is
a pure tensor-in/tensor-out function: feed it the observation tensors in the
shapes it trained on, read back the action. The on-car runtime supplies those
tensors from the car's own sensors and consumes the action with the car's own
firmware.

## The car runs Foxy/Jazzy, the sim runs Lyrical — and that's fine

| | Simulator (this repo) | Physical car |
| --- | --- | --- |
| OS | Ubuntu 26.04 "resolute" | Ubuntu (Foxy/Jazzy era) |
| ROS | ROS 2 Lyrical Luth | ROS 2 **Foxy / Jazzy** |
| Where the policy runs | dr-gym (training) | OpenVINO / TFLite inference engine |
| Coupling to the policy | obs/action contract | obs/action contract |

This looks like a version skew that should cause problems, and under the legacy
design — where the C++ system plugin and `deepracer_msgs` were the integration
surface — it would have. It does not here, for one reason: **no ROS interface is
shared across the boundary.** The car never deserializes a message produced by
the simulator's ROS distro, and the simulator never produces one for the car. The
distros are independent because the thing that ships between them — the frozen
graph — is ROS-free.

The on-device inference engine reinforces this. OpenVINO and TFLite load a graph
and run tensors; they have no notion of a ROS distro. So the car can stay on its
vendor-supported Foxy/Jazzy image while the simulator moves forward to Lyrical
(and, later, to whatever comes next) without either side caring what the other
runs.

## The export path

Export is handled by dr-gym, unchanged by the ROS 2 port. The CLI is
`dr-gym/scripts/export_bundle.py`, a thin argparse wrapper around
`gym_dr.export.export_bundle` (in `dr-gym/gym_dr/export.py`).

```bash
# From an SB3 .zip (our training output), with metadata rendered from app.py:
uv run python scripts/export_bundle.py \
    --model artifacts/<run>/final_model.zip \
    --app app.py \
    --output bundle.tar.gz

# If a sibling <model>.model_metadata.json exists, metadata is auto-detected:
uv run python scripts/export_bundle.py \
    --model artifacts/<run>/final_model.zip \
    --output bundle.tar.gz
```

The input extension drives the bundle:

| Input | Action | In-tar artifact |
| --- | --- | --- |
| SB3 `.zip` | `model.policy` exported via `torch.onnx.export` (deterministic action) | `agent/agent.onnx` |
| `.onnx` | copied verbatim | `agent/agent.onnx` |
| `.pb` (TF frozen graph) | copied verbatim | `agent/agent.pb` |

Every path produces the same on-disk contract, so the on-device loader treats
them identically:

```text
bundle.tar.gz
├── model_metadata.json
└── agent/
    └── agent.{pb,onnx}
```

Two details matter for the car to accept the bundle:

- **Extensions stay truthful.** The on-device loader
  (`aws-deepracer/aws-deepracer-systems-pkg`) dispatches on the file extension —
  `.pb` for TF protobufs, `.onnx` for ONNX. Override the in-tar filename with
  `--bundle-filename` only if your target expects a specific name.
- **`model_metadata.json` `version` is never mutated.** That field is a contract
  with the on-device loader, so the exporter passes the metadata's schema through
  unchanged. For SB3 exports rendered from `app.py`, the metadata comes from
  `experiment.action_space.to_model_metadata_dict()`.

For dict observation spaces (DeepRacer's default), each observation key becomes a
**named ONNX input** so the on-device loader can feed them as a dict. The
exported graph returns the deterministic action — matching
`model.predict(obs, deterministic=True)`.

## On-device inference: OpenVINO / TFLite

On the car, the frozen graph is loaded by OpenVINO or TFLite. Both are
self-contained inference runtimes: they need the graph and the input tensors,
nothing more. Because the loader dispatches on the bundle's file extension, the
same `export_bundle.py` output works regardless of which engine the car uses, and
regardless of the car's ROS distro.

## What both sides must agree on

The contract is frozen byte-for-byte (see the
[architecture overview](architecture/overview.md#what-moved-what-stayed) for the
full statement). For deployment, the parts that both the simulator and the car
must realize identically are:

- **Action space** `Box(low=[-30, 0.1], high=[30, 4.0])`, interpreted as
  `[steering_deg, speed_mps]`. In simulation,
  [`deepracer_env/agent_ctrl/drive.py`](../deepracer_env/agent_ctrl/drive.py)
  decodes it: clamp steering to `[-30, 30]` then convert to radians, clamp speed
  to `[0.1, 4.0]` then divide by the wheel radius (v5 = `0.035 m`) to get wheel
  angular velocity. The car's firmware applies the equivalent decode. See
  [drive and control](architecture/drive-and-control.md) for the full mapping.
- **Observation `Dict` keyed by sensor name**, in the shapes the policy trained
  on — for the camera, a `160x120` `uint8` image; for LIDAR, a `64`-ray scan.

As long as the car presents observations in those shapes and consumes the action
in that space, the policy behaves on the car as it did in training. That
equivalence — not a shared ROS stack — is what "sim-to-real" means here.

## Camera → feature-vector: collecting labels in sim

The environment supports two observation modes (see the
[architecture overview](architecture/overview.md)):

1. **CNN policy mode** — the camera image feeds a shared CNN (`gym_dr`'s
   `DeepRacerCNN`) that the policy sits on top of.
2. **Feature-vector mode** — a compact state feature vector replaces the CNN
   input.

Feature-vector mode is cheaper and more sample-efficient, but it depends on a
feature vector that, on a real car, has to come from *somewhere*. The plan is a
**camera → feature-vector model**: a learned front-end that turns the car's
camera image into the same feature vector a feature-vector policy expects. That
front-end needs a labeled dataset of `(camera image, ground-truth feature
vector)` pairs — exactly what simulation can produce and a real car cannot.

This environment collects those labels as a contract-compatible **extension**:
every observation's `info` is enriched with the **applied domain-randomization
state plus the ground-truth feature vector**, alongside the camera image in the
observation itself. The image is the model's input; the ground-truth feature
vector is its label. Because labels are emitted per observation, a camera policy
run doubles as a dataset generator.

Two properties of the rest of the stack make these labels good training data:

- **Per-arena domain randomization, seeded independently** (see
  [tiled multi-arena](architecture/multi-arena.md)) gives the dataset broad
  visual coverage — recolored tracks and backgrounds, varied lighting, sensor
  noise — so the learned front-end generalizes from sim to the real track.
- The ground-truth feature vector is computed in each arena's **local frame**, so
  labels are consistent even when multiple tracks share one Gazebo process.

The applied-DR fields in `info` also let a downstream consumer filter or weight
samples by the randomization that produced them.

## dr-gym coupling stays thin

Deployment touches dr-gym, but the ROS 2 port barely does. dr-gym is roughly 95%
insulated from the simulator move. The only changes are:

- the **Dockerfile base tag** and the `/opt/ros` sourcing line (noetic →
  lyrical), and
- the corresponding **`bootstrap.sh` branch**.

`scripts/export_bundle.py` and the export logic in `gym_dr/export.py` are
**unchanged**. The deployment artifact and its on-car contract are identical
before and after the port.

## See also

| Page | What it covers |
| --- | --- |
| [Architecture overview](architecture/overview.md) | The map: layers, the seam, the frozen contract |
| [Drive and control](architecture/drive-and-control.md) | Action → joint mapping, the on-car-equivalent decode |
| [Tiled multi-arena](architecture/multi-arena.md) | Per-arena DR and local-frame labels that feed the dataset |
