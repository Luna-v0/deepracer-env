'''Minimal D3 example — PPO-Lagrangian-style training with a cost budget.

This is **illustrative, not benchmark-grade.** It uses Stable-Baselines3 PPO
with a hand-written Lagrangian penalty fed from the cost stream so you can
verify the wiring end-to-end without depending on a particular
constrained-RL library. For real work, plug a proper constrained-RL
algorithm (CPO, PPO-Lag, P3O, ...) into the same env construction.

Prerequisites
-------------
* A running Gazebo + DeepRacer ROS stack (``./run.sh run deepracer_env.launch``).
* Python packages: ``gymnasium``, ``stable-baselines3``.

Usage
-----
    python examples/train_safe.py --level safety-0 --steps 10000
    python examples/train_safe.py --level safety-1 --steps 10000   # needs D1
'''
import argparse
import numpy as np

from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback

from deepracer_env.environments.deepracer_env import DeepRacerEnv
from deepracer_env.object_avoidance import ObjectAvoidanceConfig
from deepracer_env.safety import SafetyDeepRacerEnv, SafetyToGymnasium


def reward_function(p: dict) -> float:
    if not p['all_wheels_on_track']:
        return 1e-3
    return float(max(p['progress'] * p['speed'] / 4.0, 1e-3))


class CostBudgetCallback(BaseCallback):
    '''Subtract ``lambda_ * info["cost"]`` from the reward each step.
    Updates ``lambda_`` once per rollout to drive average cost toward
    ``cost_budget``. PPO-Lagrangian in ~15 lines.'''

    def __init__(self, cost_budget: float = 0.05, lr_lambda: float = 0.01):
        super().__init__()
        self.cost_budget = float(cost_budget)
        self.lr_lambda = float(lr_lambda)
        self.lambda_ = 0.0
        self._rollout_cost_sum = 0.0
        self._rollout_steps = 0

    def _on_step(self) -> bool:
        info = self.locals['infos'][0]
        c = float(info.get('cost', 0.0))
        self._rollout_cost_sum += c
        self._rollout_steps += 1
        # Apply penalty *retroactively* on the buffer we just stored.
        if self.lambda_ != 0.0:
            self.locals['rewards'][0] -= self.lambda_ * c
        return True

    def _on_rollout_end(self) -> None:
        if self._rollout_steps == 0:
            return
        avg_cost = self._rollout_cost_sum / self._rollout_steps
        # Dual ascent: λ ← max(0, λ + lr * (avg_cost − budget))
        self.lambda_ = max(0.0, self.lambda_ + self.lr_lambda
                           * (avg_cost - self.cost_budget))
        print('[safe] avg_cost={:.3f} budget={:.3f} lambda={:.3f}'
              .format(avg_cost, self.cost_budget, self.lambda_), flush=True)
        self._rollout_cost_sum = 0.0
        self._rollout_steps = 0


def make_env(level: str):
    if level == 'safety-0':
        return SafetyDeepRacerEnv(
            DeepRacerEnv(reward_fn=reward_function),
            level='safety-0',
        )
    if level == 'safety-1':
        return SafetyDeepRacerEnv(
            DeepRacerEnv(
                reward_fn=reward_function,
                object_avoidance=ObjectAvoidanceConfig(
                    enabled=True,
                    n_obstacles=2,
                    terminate_on_collision=False,  # keep cost alive
                ),
            ),
            level='safety-1',
            combine='max',
        )
    raise ValueError('unknown --level {!r}'.format(level))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--level', default='safety-0',
                        choices=('safety-0', 'safety-1'))
    parser.add_argument('--steps', type=int, default=10_000)
    parser.add_argument('--cost-budget', type=float, default=0.05)
    parser.add_argument('--seed', type=int, default=0)
    args = parser.parse_args()

    safety_env = make_env(args.level)
    # SB3 wants the 5-tuple shape; the adapter still puts cost in info.
    env = SafetyToGymnasium(safety_env)
    env.reset(seed=args.seed)

    model = PPO('MultiInputPolicy', env, verbose=1,
                n_steps=128, batch_size=64, learning_rate=3e-4)
    model.learn(args.steps, callback=CostBudgetCallback(args.cost_budget))
    model.save('deepracer_safe_ppo_{}'.format(args.level))
    env.close()


if __name__ == '__main__':
    main()
