#################################################################################
#   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#################################################################################
"""The domain-randomization catalog as one declarative spec.

DR knobs used to be scattered across the controller, the URDF/launch, and the
visual randomizer. :class:`RandomizationSpec` gathers the whole catalog into one
place — which knobs are on, and their ranges — so a single per-arena
:class:`~deepracer_env.domain_randomizations.domain_randomizer.DomainRandomizer`
samples and applies them consistently and independently per car.

Every knob defaults **off** (or to a no-op range) so existing runs are
unaffected; :meth:`RandomizationSpec.from_env` turns them on from ``GYM_DR_*``
environment variables (how ``dr-gym`` configures a run).
"""
from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import Tuple


def _b(name: str, default: bool = False) -> bool:
    return os.environ.get(name, "1" if default else "0").strip().lower() in ("1", "true")


def _f(name: str, default: float) -> float:
    try:
        return float(os.environ.get(name, default))
    except (TypeError, ValueError):
        return default


@dataclass
class RandomizationSpec:
    """Which randomizations are active and over what ranges.

    Attributes:
        visual_recolor: Recolour the track surface + background each episode.
        lighting: Recolour / re-aim the scene ``sun`` each episode.
        friction: Sample a per-episode wheel friction μ in ``friction_range``.
        friction_range: ``(low, high)`` absolute wheel μ.
        start_position: Sample a uniform start position along the centre-line.
        direction: Coin-flip the driving direction (CW/CCW) each episode.
        steering_bias: Add a fixed per-episode steering offset (actuator bias).
        steering_bias_deg: Max magnitude of that offset, in degrees.
        motor_delay: Lag the applied action by a per-episode integer step count.
        motor_delay_max_steps: Upper bound on that lag.
        sensor_noise: Add Gaussian noise to camera/LiDAR observations.
        camera_noise_std: Camera noise std (fraction of full scale, 0–1).
        lidar_noise_std: LiDAR range noise std, in metres.
        contrast_threshold: Min squared-RGB distance between track and background
            colours (keeps them visually distinct in the camera frame).
    """

    visual_recolor: bool = False
    lighting: bool = False
    friction: bool = False
    friction_range: Tuple[float, float] = (0.8, 1.6)
    start_position: bool = False
    direction: bool = False
    steering_bias: bool = False
    steering_bias_deg: float = 2.0
    motor_delay: bool = False
    motor_delay_max_steps: int = 2
    sensor_noise: bool = False
    camera_noise_std: float = 0.0
    lidar_noise_std: float = 0.0
    contrast_threshold: float = 0.10

    @staticmethod
    def from_env() -> "RandomizationSpec":
        """Build a spec from ``GYM_DR_*`` environment variables."""
        return RandomizationSpec(
            visual_recolor=_b("GYM_DR_VISUAL_DR"),
            lighting=_b("GYM_DR_LIGHTING"),
            friction=_b("GYM_DR_FRICTION"),
            friction_range=(_f("GYM_DR_FRICTION_MIN", 0.8), _f("GYM_DR_FRICTION_MAX", 1.6)),
            start_position=_b("GYM_DR_RANDOM_START"),
            direction=_b("GYM_DR_RANDOM_DIRECTION"),
            steering_bias=_b("GYM_DR_STEERING_BIAS"),
            steering_bias_deg=_f("GYM_DR_STEERING_BIAS_DEG", 2.0),
            motor_delay=_b("GYM_DR_MOTOR_DELAY"),
            motor_delay_max_steps=int(_f("GYM_DR_MOTOR_DELAY_STEPS", 2)),
            sensor_noise=_b("GYM_DR_SENSOR_NOISE"),
            camera_noise_std=_f("GYM_DR_CAMERA_NOISE_STD", 0.0),
            lidar_noise_std=_f("GYM_DR_LIDAR_NOISE_STD", 0.0),
        )

    @property
    def any_enabled(self) -> bool:
        """True if any randomization is active."""
        return any((self.visual_recolor, self.lighting, self.friction,
                    self.start_position, self.direction, self.steering_bias,
                    self.motor_delay, self.sensor_noise))
