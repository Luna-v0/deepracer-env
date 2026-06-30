#!/usr/bin/env python3
"""In-container smoke test: construct DeepRacerEnv and run reset()/step().

Not a deliverable — a throwaway harness run by examples (see func_test.sh) to
prove the ported control plane (seam-backed trackers + ros2_control publish +
reward/reset-rule path) works end-to-end against a live gz Jetty stack. Uses
sensors=[] to isolate the control plane from headless camera rendering.
"""
import os
import signal

import numpy as np

os.environ.setdefault("DR_SIM_BACKEND", "ros_gz")


def _timeout(*_a):
    raise TimeoutError("phase timed out")


signal.signal(signal.SIGALRM, _timeout)

from deepracer_env.environments.deepracer_env import DeepRacerEnv

env = DeepRacerEnv(reward_fn=lambda p: float(p.get("progress", 0.0)), sensors=[])
print("ENV constructed; action_space:", env.action_space, flush=True)

signal.alarm(60)
obs, info = env.reset()
signal.alarm(0)
print("RESET ok; obs keys:", list(obs.keys()),
      "info:", {k: info[k] for k in list(info)[:4]}, flush=True)
dr_keys = {k: info[k] for k in info if k.startswith("dr_")}
print("DR labels in info:", {k: dr_keys[k] for k in
      ("dr_start_ndist", "dr_reverse_dir", "dr_steering_bias_rad", "dr_track_color")
      if k in dr_keys}, flush=True)

samples = []
for i in range(12):
    signal.alarm(30)
    obs, reward, terminated, truncated, info = env.step(np.array([6.0, 2.0], dtype=np.float32))
    signal.alarm(0)
    rp = getattr(getattr(env._agent, "ctrl", None), "reward_params", {}) or {}
    samples.append((round(rp.get("x", 0.0), 3), round(rp.get("y", 0.0), 3),
                    round(rp.get("progress", 0.0), 3), round(float(reward), 3)))
    if terminated:
        print("  terminated at step", i, flush=True)
        break

print("STEP first (x,y,progress,reward):", samples[0], flush=True)
print("STEP last  (x,y,progress,reward):", samples[-1], flush=True)
print("FUNC_OK", flush=True)
env.close()
