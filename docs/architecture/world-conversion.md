# World file conversion (127 tracks)

The 127 shipped tracks ship as Gazebo-**classic** `.world` files. They load under
Gazebo Jetty (`gz-sim` 10.4.0) but produce a *static, uncontrollable* scene: the
car never falls onto the track, and none of the gz-transport services the backend
drives actually exist. The converter
([`scripts/world_converter.py`](../../scripts/world_converter.py)) rewrites each
classic world into a Jetty `.sdf` that is physically simulated and fully
controllable, while preserving every reusable element verbatim.

The transform is pure `xml.etree` — no ROS, no Gazebo — so it is unit-testable on
any host and runs as a fast batch over the whole `worlds/` directory. All 127
worlds have been converted and load-verified in Jetty.

## Why classic `.world` files won't drive under Jetty

A classic world is a near-identical template: a `<scene>`, a classic
`model://sun` include, one or more authored `<light>` elements, and one or more
track `<include>` blocks. Crucially, it carries **no `<physics>` and no system
plugins**.

That was fine for classic `gzserver`, which wired physics and its core systems in
itself. Jetty does not. Without the systems baked into the world SDF:

* There is **no gravity / no physics step** — the spawned car never settles onto
  the track and nothing moves.
* The control surface is **absent**. The
  [`RosGzBackend`](../../deepracer_env/sim_control/backends/ros_gz_backend.py)
  drives `gz-transport` services under `/world/<world>/`
  (`create`, `set_pose`, `control`, `state`) and reads poses from the
  `/world/<world>/pose/info` topic. Those services and topics are *provided by
  the system plugins*. No plugins in the SDF, no services — and every per-entity
  verb of [the seam](sim-control-seam.md) fails.

So conversion has to supply what classic `gzserver` used to inject implicitly.

## What the converter injects

The converter prepends a fixed, canonical block to every world so all 127 tracks
get an identical, drivable configuration. The block is copied from the shipped
`empty.sdf` and extended with the `Sensors` system for camera/LiDAR arenas.

| Injected | Kind | Why it's needed |
| --- | --- | --- |
| `<physics name="1ms">` (`max_step_size` 0.001, RTF 1.0) | physics step | Deterministic 1 ms simulation step; backs `control` multi-step stepping. |
| `<gravity>0 0 -9.8</gravity>` | gravity | Car settles onto the track instead of floating. |
| `gz::sim::systems::Physics` | system plugin | DART (6.16.6) rigid-body simulation. |
| `gz::sim::systems::UserCommands` | system plugin | Serves `create` / `set_pose` (`remove`) / `control` — spawn, teleport, step. |
| `gz::sim::systems::SceneBroadcaster` | system plugin | Publishes `/world/<world>/pose/info` and feeds the GUI. |
| `gz::sim::systems::Contact` | system plugin | Collision / contact events (off-track detection). |
| `gz::sim::systems::Sensors` (`<render_engine>ogre2</render_engine>`) | system plugin | Renders camera and LiDAR sensors. |

That is the **5 system plugins + physics + gravity**. Mapping the plugins to the
control surface they unlock:

| Plugin | Provides | Seam verb / read it enables |
| --- | --- | --- |
| `UserCommands` | `/world/<w>/create`, `set_pose`, `remove`, `control` | `spawn_entity`, `set_entity_state` (teleport), `delete_entity`, `step`, `pause`/`unpause` |
| `SceneBroadcaster` | `/world/<w>/pose/info`, `state` | `get_entity_state`, `list_entities` |
| `Physics` | DART stepping | makes `step` actually advance dynamics |
| `Sensors` | camera/LiDAR rendering | observation modes (CNN image, LiDAR scan) |
| `Contact` | contact events | off-track / collision signals |

The native visual-DR services (`/world/<w>/visual_config`,
`/world/<w>/light_config`) are part of gz's standard surface and need no plugin
injection — which is why visual domain randomization survives with zero custom
C++. See [domain randomization](domain-randomization.md).

The exact injected text lives in the `SYSTEM_PLUGINS` constant in
[`world_converter.py`](../../scripts/world_converter.py).

## The sun swap: classic include to native light

Classic worlds pull the sun in as a model include:

```xml
<include>
  <uri>model://sun</uri>
  ...
</include>
```

`model://sun` is a Gazebo-classic asset that does not exist under Jetty. The
converter **drops that include** and injects a native directional light named
`sun` in its place (`SUN_LIGHT` constant):

```xml
<light type="directional" name="sun">
  <cast_shadows>false</cast_shadows>
  <pose>0 0 15 0 0 0</pose>
  <diffuse>0.9 0.9 0.9 1</diffuse>
  <specular>0.2 0.2 0.2 1</specular>
  <direction>-0.3 0.2 -0.9</direction>
</light>
```

The classic `model://ground_plane` include (where present) is dropped too — Jetty
tracks bring their own asphalt mesh and collision. Both drops are handled by
`_is_dropped_include`, keyed on the `_DROP_INCLUDE_MODELS = ("sun", "ground_plane")`
tuple.

## What it preserves

Everything reusable carries over **verbatim**:

* **`<scene>`** — ambient color, shadow settings.
* **Authored `<light>` point/spot lights** — these are valid SDF in Jetty and are
  copied unchanged (e.g. the `"Light 1"` point light in `reinvent_base`).
* **Every track `<include>`** — including **multi-include** worlds. For example,
  `reinvent_base_jeremiah` includes both `reinvent_lines_walls` and
  `reinvent_grass_asphalt`; the converter keeps **both** meshes.
* **Any other authored element** (e.g. `<spherical_coordinates>`) is passed
  through.

Only the classic `<physics>` / `<gravity>` elements are stripped — and only so the
injected canonical pair guarantees the same drivable configuration across all 127
worlds. The `.dae` mesh files themselves are renderer-agnostic and are never
touched.

## Resource resolution via `GZ_SIM_RESOURCE_PATH`

The converter does **not** rewrite `model://` URIs. Track includes still read like
`model://models/reinvent_base`, and the model SDFs in turn reference meshes like
`model://meshes/reinvent/reinvent_base.dae`. Jetty resolves the `model://` prefix
against `GZ_SIM_RESOURCE_PATH` at load time, exactly as the classic stack used
`GAZEBO_MODEL_PATH`.

Point `GZ_SIM_RESOURCE_PATH` at the directory that contains both `models/` and
`meshes/` — that is the `simulation/` directory:

```
simulation/
├── models/      # model://models/<track>  -> model.sdf
├── meshes/      # model://meshes/<track>/<file>.dae
└── worlds/      # converted .sdf files
```

```bash
export GZ_SIM_RESOURCE_PATH=/path/to/deepracer-env/simulation
```

With that set, both `model://models/...` and `model://meshes/...` resolve relative
to the same root. No URI edits, no mesh edits.

## How to run it

The CLI defaults to the in-repo `simulation/worlds/` directory and writes `.sdf`
files alongside the `.world` sources.

Convert the whole directory (the default — all 127 tracks):

```bash
python3 scripts/world_converter.py
# converted 127 worlds -> .../simulation/worlds
```

Convert a single world by name (no extension):

```bash
python3 scripts/world_converter.py --world reinvent_base
# wrote .../simulation/worlds/reinvent_base.sdf
```

Convert from one directory to another:

```bash
python3 scripts/world_converter.py \
  --in-dir  /path/to/classic/worlds \
  --out-dir /path/to/jetty/worlds
```

| Flag | Default | Meaning |
| --- | --- | --- |
| `--in-dir` | `simulation/worlds` (relative to the script) | Directory of classic `.world` files. |
| `--out-dir` | same as `--in-dir` | Where `.sdf` files are written. |
| `--world` | — | Convert a single world by stem (e.g. `reinvent_base`); otherwise the whole directory is batched. |

Batch mode prints a count and continues past failures — any world that fails to
parse is reported to `stderr` as `FAILED <name>: <reason>` and skipped, so one bad
file never aborts the run.

## Worked example

Input (`reinvent_base.world`, abbreviated):

```xml
<sdf version="1.7">
<world name="reinvent_base">
  <scene>...</scene>
  <include><uri>model://sun</uri>...</include>
  <light type="point" name="Light 1">...</light>
  <include>
    <uri>model://models/reinvent_base</uri>
    <name>racetrack</name>
  </include>
</world>
</sdf>
```

Output (`reinvent_base.sdf`, abbreviated): the injected physics + 5 plugins +
native `sun` light come first, then the preserved `<scene>`, the preserved point
light, and the preserved track include:

```xml
<sdf version="1.10">
  <world name="reinvent_base">
    <physics name="1ms" type="ignored">...</physics>
    <gravity>0 0 -9.8</gravity>
    <plugin filename="gz-sim-physics-system" .../>
    <plugin filename="gz-sim-user-commands-system" .../>
    <plugin filename="gz-sim-scene-broadcaster-system" .../>
    <plugin filename="gz-sim-contact-system" .../>
    <plugin filename="gz-sim-sensors-system" ...><render_engine>ogre2</render_engine></plugin>
    <light type="directional" name="sun">...</light>
    <scene>...</scene>                                <!-- preserved -->
    <light type="point" name="Light 1">...</light>    <!-- preserved -->
    <include><uri>model://models/reinvent_base</uri>...</include>  <!-- preserved -->
  </world>
</sdf>
```

Note the SDF version bump to `1.10` (broadly safe for `gz-sim` 10, which reads up
to 1.11) and the preserved `<world name>`.

## Where this sits

* Converter source: [`scripts/world_converter.py`](../../scripts/world_converter.py).
* Backend that needs the injected services:
  [`deepracer_env/sim_control/backends/ros_gz_backend.py`](../../deepracer_env/sim_control/backends/ros_gz_backend.py)
  and [the seam](sim-control-seam.md).
* Per-arena tracks that consume the converted worlds:
  [tiled multi-arena](multi-arena.md).
* Visual / lighting randomization on top of the converted scene:
  [domain randomization](domain-randomization.md).
