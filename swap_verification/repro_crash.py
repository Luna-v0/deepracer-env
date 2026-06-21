#!/usr/bin/env python3
"""Reproduce the reported gzserver segfault during set_world delete_model.

Drives a sequence of swaps and, after every one, checks that the gazebo
process is still alive (the segfault shows up as gzserver dying). Prints a
clear PASS/CRASH line per swap so we can see whether it is a specific track
or an accumulation effect.
"""
import os
import sys
import time
import subprocess

import numpy as np
import rospy

SEQ = os.environ.get(
    "SEQ", "reInvent2019_track,Austin,reInvent2019_track,Spain_track,"
           "reInvent2019_track,Austin,reInvent2019_track,Austin,"
           "reInvent2019_track,Austin").split(",")


def gz_alive():
    out = subprocess.run(["pgrep", "-f", "gzserver"], capture_output=True)
    return out.returncode == 0


def main():
    rospy.init_node("repro", anonymous=True, disable_signals=True)
    from deepracer_env.environments.deepracer_env import DeepRacerEnv

    def reward(p):
        return float(p.get("progress", 0.0))

    print("building env on {} ...".format(rospy.get_param("WORLD_NAME")), flush=True)
    env = DeepRacerEnv(reward_fn=reward, sensors=["FRONT_FACING_CAMERA"])
    env.reset()
    print("gz_alive after first reset:", gz_alive(), flush=True)

    for i, w in enumerate(SEQ):
        print("\n--- swap #{}: -> {} (gz_alive_before={}) ---".format(
            i + 1, w, gz_alive()), flush=True)
        try:
            env.set_world(w)
        except Exception as ex:  # noqa: BLE001
            print("  set_world EXCEPTION:", repr(ex), flush=True)
            print("  gz_alive_after_exception:", gz_alive(), flush=True)
            if not gz_alive():
                print("  >>> GZSERVER DIED on swap #{} -> {}".format(i + 1, w), flush=True)
                return
            continue
        alive = gz_alive()
        print("  swap returned; gz_alive={}".format(alive), flush=True)
        if not alive:
            print("  >>> GZSERVER DIED on swap #{} -> {}".format(i + 1, w), flush=True)
            return
        env.reset()
        for _ in range(20):
            env.step(np.array([0.0, 1.0], dtype=np.float32))
    print("\nALL {} SWAPS SURVIVED".format(len(SEQ)), flush=True)


if __name__ == "__main__":
    main()
