'''Minimal D1 example: PPO with random static obstacles each episode.

Prerequisites
-------------
* A running Gazebo + DeepRacer ROS stack (e.g. ``./run.sh run deepracer_env.launch``).
* Python packages: ``gymnasium``, ``stable-baselines3``.

Usage
-----
    python examples/train_object_avoidance.py --steps 50000 --n-obstacles 3 --seed 0
'''
import argparse

from stable_baselines3 import PPO

from deepracer_env.environments.deepracer_env import DeepRacerEnv
from deepracer_env.object_avoidance import ObjectAvoidanceConfig


def reward_function(params: dict) -> float:
    '''Progress * speed, with a hard penalty for crashing into an obstacle.'''
    if params['is_crashed']:
        return -10.0
    if not params['all_wheels_on_track']:
        return 1e-3
    # Mild lane-keeping bonus when an obstacle is ahead — encourages the car
    # to commit to one side of the track before reaching the obstacle.
    bonus = 1.0
    if params['closest_objects'][1] >= 0:
        # closest_objects[1] is the next-object index in the sorted-by-projection
        # list; -1 means "no upcoming object".
        bonus = 1.5 if params['distance_from_center'] > 0.1 else 0.7
    return float(max(params['progress'] * params['speed'] * bonus / 4.0, 1e-3))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--steps', type=int, default=50_000)
    parser.add_argument('--n-obstacles', type=int, default=3)
    parser.add_argument('--min-spacing', type=float, default=2.5)
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--no-terminate', action='store_true',
                        help='Set terminate_on_collision=False (safety-style).')
    args = parser.parse_args()

    oa_cfg = ObjectAvoidanceConfig(
        enabled=True,
        n_obstacles=args.n_obstacles,
        placement='random_on_waypoints',
        min_spacing_m=args.min_spacing,
        lane='any',
        terminate_on_collision=not args.no_terminate,
    )

    env = DeepRacerEnv(reward_fn=reward_function, object_avoidance=oa_cfg)
    env.reset(seed=args.seed)

    model = PPO(
        policy='MultiInputPolicy',
        env=env,
        verbose=1,
        n_steps=256,
        batch_size=64,
        learning_rate=3e-4,
        ent_coef=0.01,
        tensorboard_log='./tb_logs/',
    )
    model.learn(total_timesteps=args.steps)
    model.save('deepracer_oa_ppo')
    env.close()


if __name__ == '__main__':
    main()
