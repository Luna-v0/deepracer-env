"""Multi-agent DeepRacer env — N cars in ONE Gazebo world.

The single-agent ``DeepRacerEnv`` step is *free-running* (publish action → read the
car's latest state → judge), with no explicit Gazebo step barrier. So N cars in the
same world are driven by N independent, namespaced ``Agent``s (``racecar_0`` ..
``racecar_{N-1}``): one ``step`` sends every car's action, then reads every car's
observation/reward — one shared physics context advances all of them. Per-car
episodes are independent: a car that finishes (off-track/lap) is reset on its own
while the others keep driving.

This class is RL-framework-agnostic (lists in / lists out); the SB3 ``VecEnv``
adaptation (batched arrays, per-car obs transforms, DR, auto-reset bookkeeping)
lives in dr-gym ``gym_dr/envs/multi_car.py``. See ``docs/reports/multi-car.md``.
"""
from __future__ import annotations

import math
import os
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

from deepracer_env.environments.deepracer_env import DEFAULT_ACTION_SPACE, build_agent


def grid_offsets(n_cars: int, spacing: float) -> List[Tuple[float, float]]:
    """Square-grid world (dx, dy) offsets for N separated track instances; car 0
    at the origin (reuses the launch's .world track)."""
    cols = max(1, math.ceil(math.sqrt(n_cars)))
    return [((i % cols) * spacing, (i // cols) * spacing) for i in range(n_cars)]


class MultiAgentDeepRacerEnv:
    """N independent agents on **separated track instances** in one Gazebo world.

    Each car gets its own ``racecar_{i}`` ``Agent`` AND its own track instance:
    car 0 drives the launch's origin track; cars 1.. get a copy (or a different
    track, for diversity) spawned ``spacing`` metres away, with a ``TrackData``
    whose waypoints are shifted by the same offset. So each car has a valid start
    on its own track, and the cars never see or collide with each other.

    Args mirror ``DeepRacerEnv`` plus ``n_cars`` / ``worlds`` (per-car track name;
    default all the same = parallel) / ``spacing``.
    """

    def __init__(
        self,
        n_cars: int,
        reward_fn: Callable[[dict], float],
        sensors: List[str],
        world_name: Optional[str] = None,
        worlds: Optional[Sequence[str]] = None,
        spacing: float = 300.0,
        config: Optional[Dict[str, Any]] = None,
        is_training: bool = True,
        extra_ctrl_config: Optional[Dict[str, Any]] = None,
    ) -> None:
        if n_cars < 1:
            raise ValueError(f"n_cars must be >= 1, got {n_cars}")
        self.n_cars = int(n_cars)
        self.car_names = [f"racecar_{i}" for i in range(self.n_cars)]

        base_world = world_name or os.getenv("WORLD_NAME")
        self.worlds = list(worlds) if worlds else [base_world] * self.n_cars
        self.offsets = grid_offsets(self.n_cars, spacing)

        from deepracer_env.track_geom.track_data import TrackData

        # Spawn the extra track instances (car 0 reuses the origin .world track).
        if self.n_cars > 1:
            from deepracer_env.environments.world_swap import WorldSwapper
            swapper = WorldSwapper()
            for i in range(self.n_cars):
                ox, oy = self.offsets[i]
                if i == 0 and ox == 0.0 and oy == 0.0:
                    continue
                swapper.spawn_track_instance(self.worlds[i], f"racetrack_{i}", (ox, oy))

        # Build per-car (offset) TrackData + the agent bound to it.
        self._agents = []
        for i, name in enumerate(self.car_names):
            track_data = TrackData.create(self.worlds[i], offset=self.offsets[i])
            self._agents.append(
                build_agent(name, reward_fn, sensors, config=config,
                            is_training=is_training, extra_ctrl_config=extra_ctrl_config,
                            track_data=track_data))
        # Per-car spaces (identical across cars; the VecEnv exposes these as its
        # single_observation_space / single_action_space).
        self.single_observation_space = self._agents[0].get_observation_space()
        self.single_action_space = DEFAULT_ACTION_SPACE

    # ------------------------------------------------------------------ #
    def reset(self) -> List[dict]:
        """Reset all cars; return the list of N initial observations."""
        return [agent.reset_agent() for agent in self._agents]

    def reset_one(self, i: int) -> dict:
        """Reset just car ``i`` (its episode ended); the others are untouched.
        Returns that car's initial observation (for VecEnv auto-reset)."""
        return self._agents[i].reset_agent()

    def step(self, actions: List[Any]):
        """Send every car's action, then read every car's (obs, reward, done,
        info). One shared physics context advances all cars between the sends and
        the reads. Returns four length-N lists."""
        if len(actions) != self.n_cars:
            raise ValueError(f"expected {self.n_cars} actions, got {len(actions)}")
        # 1. publish all actions (cars advance together in the shared world)
        for agent, action in zip(self._agents, actions):
            agent.send_action(action)
        # 2. read every car's resulting state
        obs_l, rew_l, done_l, info_l = [], [], [], []
        for agent, action in zip(self._agents, actions):
            info_map = agent.update_agent(action)
            obs, reward, done = agent.judge_action(action, info_map)
            obs_l.append(obs)
            rew_l.append(float(reward))
            done_l.append(bool(done))
            info_l.append(self._step_info(agent, info_map))
        return obs_l, rew_l, done_l, info_l

    @staticmethod
    def _step_info(agent, info_map) -> Dict[str, Any]:
        info: Dict[str, Any] = dict(info_map) if isinstance(info_map, dict) else {}
        ctrl = getattr(agent, "ctrl", None)
        rp = getattr(ctrl, "reward_params", None) if ctrl is not None else None
        if rp is not None:
            info["is_crashed"] = bool(rp.get("is_crashed", False))
            info["is_offtrack"] = bool(rp.get("is_offtrack", False))
            # Full params so the dr-gym VecEnv can build feature observations
            # (camera-off path) per car without a separate reward tap.
            info["reward_params"] = dict(rp)
        # Per-arena applied-DR labels (dr_* keys) for the camera->feature dataset.
        if ctrl is not None and hasattr(ctrl, "dr_info"):
            info.update(ctrl.dr_info)
        return info

    def close(self) -> None:
        for agent in self._agents:
            close = getattr(agent, "close", None)
            if callable(close):
                close()
