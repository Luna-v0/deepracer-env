# Register the Gymnasium env id when gymnasium is available. The registration is
# best-effort so that lightweight, ROS-free submodules — e.g.
# ``deepracer_env.sim_control`` (the simulator seam) and
# ``deepracer_env.agent_ctrl.drive`` (the action->joint math) — can be imported
# in environments that do not have gymnasium installed (in-sim driver scripts,
# unit tests, the bare Gazebo container). Importing those submodules should never
# require the full training-time dependency set.
try:
    from gymnasium.envs.registration import register

    register(
        id='DeepRacer-v0',
        entry_point='deepracer_env.environments.deepracer_env:DeepRacerEnv',
        # reward_fn must be supplied as a keyword argument to gymnasium.make()
        # or by constructing DeepRacerEnv directly.
    )
except ImportError:  # gymnasium not installed — submodules still import fine.
    pass
