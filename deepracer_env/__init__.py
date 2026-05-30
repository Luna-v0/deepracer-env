from gymnasium.envs.registration import register

register(
    id='DeepRacer-v0',
    entry_point='deepracer_env.environments.deepracer_env:DeepRacerEnv',
    # reward_fn must be supplied as a keyword argument to gymnasium.make()
    # or by constructing DeepRacerEnv directly.
)

# Trigger D3 safety-env registration as a side-effect of importing
# ``deepracer_env``. Wrapped in a try/except so a missing optional
# dependency cannot break ``import deepracer_env``.
try:
    from deepracer_env.safety import (  # noqa: F401
        SafetyDeepRacerEnv, SafetyToGymnasium, GymnasiumToSafety,
    )
except Exception:  # pragma: no cover - defensive
    pass
