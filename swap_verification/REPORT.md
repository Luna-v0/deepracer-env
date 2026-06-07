# `DeepRacerEnv.set_world()` — runtime track swap: verification report

**Date:** 2026-06-07
**Image:** `my-deepracer-project:cpu` (ROS Noetic + Gazebo 11, headless)
**Render mode:** `ENABLE_GUI=False`. The image's `run.sh` always starts an
Xvfb + jwm X server, so the camera sensors render fine headless — the onboard
frame measured mean-brightness ≈ 60–94 (not black), so **no fallback to VNC was
needed**.
**Result:** **main 12/12 + object-avoidance 5/5 checks passed.**

---

## What changed

Pure Python + standard `gazebo_ros` services. No C++ plugin or URDF edits.

| File | Change |
|------|--------|
| `deepracer_env/environments/deepracer_env.py` | New **`set_world(world_name)`** method (the orchestrator). `reset()` now sets a `_has_reset` flag that gates the between-episodes contract. |
| `deepracer_env/environments/world_swap.py` | `WorldSwapper` Gazebo plumbing (delete/spawn/confirm). Pre-existing; fixed `spawn_track` to pass an identity `Pose()` instead of `None` (rospy rejects a `None` pose). |
| `deepracer_env/agent_ctrl/rollout_agent_ctrl.py` | New **`reset_track_data()`** — rebinds the controller's cached `TrackData`, start lane, start-line offset, **rebuilds the reset-rules manager**, and re-registers the agent object on the new track. |
| `deepracer_env/agents/agent.py` | Passthroughs `reset_track_data()` and `drain_sensors()`. |
| `deepracer_env/track_geom/constants.py` | `GET_WORLD_PROPERTIES`, `PAUSE_PHYSICS`, `UNPAUSE_PHYSICS`, `RACETRACK_MODEL_NAME` (pre-existing). |

**dr-gym side** (so multi-world training actually uses the swap): the host
orchestrator now launches **one** container that rotates worlds in-process
(`GYM_DR_ROTATE=1`) instead of one container per `(rotation, world)`.
`Sb3Trainer.fit` walks the chunks, calling `env.set_world()` between them and
re-syncing SB3's `_last_obs`; policy weights + PPO optimizer state stay in
memory across swaps. Stale "cannot switch at runtime" docstrings were corrected.
A `tests/test_smoke.py::test_runtime_world_rotation_swaps_in_process` test was
added (77/77 + 2 skipped pass).

---

## The swap algorithm (`set_world`)

All under `pause_physics`, wrapped in `try/finally` so physics is always
unpaused even on failure:

1. **Validate** target assets (`routes/<w>.npy`, `models/<w>/`) before touching
   Gazebo — a bad name can never leave the world track-less.
2. **Pause physics** (car can't free-fall onto a missing track).
3. **Tear down OA obstacles** *while the old `TrackData` is still live* — deletes
   the obstacle models from Gazebo **and** unregisters them from the correct
   (old) `TrackData`.
4. **Delete** the live `racetrack` model(s) (discovered via
   `get_world_properties`).
5. **Spawn** the new track via an `<include>` wrapper SDF
   (`model://models/<world>`, `<name>racetrack`) so Gazebo resolves every
   mesh/material URI itself — sidestepping the inconsistent SDF filenames.
6. **Rebuild geometry:** `TrackData._instance_ = None`, `set_param WORLD_NAME`,
   `TrackData.get_instance()`; clear `FrustumManager._instance_`; rebind the
   controller + reset rules via `agent.reset_track_data()`.
7. **Rebind the obstacle manager** to the new `TrackData` (reconstruct — no
   setter); the next `reset()` places obstacles on the new track.
8. **Reset the car** onto the new start line (reuses the controller's blocking
   `SetModelState` path — works while paused because blocking calls hit the
   service directly, not the paused sim-clock loop).
9. **Unpause**, **drain** stale sensor buffers, block for one fresh frame.

---

## Gotchas that bit (and how they were handled)

- **Inconsistent SDF filenames.** `reinvent_base` ships `model.sdf`,
  `Vegas_track` ships `Re_Invent_track_model.sdf`. The `<include>`-wrapper spawn
  handles both — **verified live**: both swapped in cleanly (see multi-swap).
- **`WORLD_NAME` ROS param.** The simapp reads `WORLD_NAME` from an *env var* at
  launch and doesn't always set the rosparam. `TrackData.__init__` reads the
  *rosparam*, so it must be set (`rosparam set WORLD_NAME ...`) before the first
  env build; `set_world` sets it thereafter.
- **`points_on_track` wants shapely `Point`s, not tuples** (verification-script
  fix, not a product bug).
- **Cached `TrackData` references.** The controller, the off-track/crash reset
  rules, and the obstacle manager all cached the old singleton. All are refreshed
  — proven by the `reward_geometry_moved` and `oa_manager_rebound` assertions.
- **Sim clock frozen while paused.** `confirm_track_present` / blocking
  car-reset must not depend on `/clock` advancing — they don't (synchronous
  service calls + immediate model-present check).

---

## Assertion values

### Phase 3a–3d (`results_main.json`) — 12/12

| Check | Value |
|-------|-------|
| racetrack count before / after swap | 1 / 1 (old gone, new present) |
| waypoints == `routes/arctic_pro.npy` | **True** — `td=(238,2)` matches route `(238,2)` |
| is_loop / is_ccw (reinvent_base→arctic_pro) | `True→True` / `True→True` (recomputed) |
| **reward geometry moved** | probe point `distance_from_center` `0.000 → 0.886 m`; `on_track True → False` |
| set_world latency | ≈ **0.7 s** |
| 200-step episode on new track | ran 200 steps, no exception |
| drive mp4 | 90 frames, mpeg4 640×480 |
| multi-swap Spain / Tokyo / **Vegas** / reinvent_base | all: waypoints match, racetrack count 1, **no phantom collision** in a 60-step episode |

### Phase 3e — object avoidance (`results_oa.json`) — 5/5

| Check | Value |
|-------|-------|
| obstacles present before swap | `obstacle_0/1/2` in Gazebo **and** registered in `TrackData.object_poses` |
| stale models gone after swap | exactly 3 obstacles in Gazebo (trackA's deleted, trackB's spawned) |
| manager rebound to new `TrackData` | `mgr._track_data is new == True`, waypoints == trackB route |
| **obstacles on the new track** | max distance to trackB centerline = **0.27 m** (on-track, not floating at trackA coords) — this is the assertion that catches the stale-`TrackData` bug |
| deterministic layout (same seed ×2) | identical: `[(6.483,-4.606),(7.028,1.463),(9.628,-3.37)]` |

---

## Media

- `before_reinvent_base.png` / `after_arctic_pro.png` — onboard view; the
  reinvent wall on the horizon disappears and the surface/horizon change, proving
  the onboard camera renders the new world.
- `topdown_reinvent_base.png`, `topdown_arctic_pro.png`,
  `topdown_Spain_track.png`, `topdown_Tokyo_Training_track.png`,
  `topdown_Vegas_track.png` — five **visibly distinct** track layouts (≥4 required,
  incl. the `model.sdf` + `Re_Invent_track_model.sdf` outliers).
- `drive_arctic_pro.mp4` — ~6 s onboard clip of the car driving on the new track.
- `oa_arctic_pro.png` — top-down of arctic_pro with the three obstacles placed on
  the new track.
- `results_main.json`, `results_oa.json` — machine-readable check log.

## How to reproduce

```bash
docker run -d --name swap-verify --shm-size=2g \
  -e WORLD_NAME=reinvent_base -e ENABLE_GUI=False -e RTF_OVERRIDE=3.0 \
  -v /path/to/deepracer-env:/workspace my-deepracer-project:cpu \
  "source /opt/ros/noetic/setup.bash && source /opt/simapp/setup.bash && ./run.sh run deepracer_env.launch"
# wait until /racecar/camera/zed/rgb/image_rect_color publishes, then:
docker exec swap-verify bash -lc 'source /opt/ros/noetic/setup.bash; source /opt/simapp/setup.bash;
  rosparam set WORLD_NAME reinvent_base;
  PYTHONPATH=/workspace:$PYTHONPATH python3 -u /workspace/swap_verification/verify_swap.py main'
docker exec swap-verify bash -lc 'source /opt/ros/noetic/setup.bash; source /opt/simapp/setup.bash;
  rosparam set WORLD_NAME reinvent_base;
  PYTHONPATH=/workspace:$PYTHONPATH python3 -u /workspace/swap_verification/verify_swap.py oa'
```
