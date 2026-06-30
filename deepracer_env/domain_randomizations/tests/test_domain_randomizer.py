#################################################################################
#   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#################################################################################
"""Host-runnable tests for the per-arena DomainRandomizer sampler + spec."""
import os

import numpy as np
import pytest

from deepracer_env.domain_randomizations.spec import RandomizationSpec
from deepracer_env.domain_randomizations.domain_randomizer import DomainRandomizer


def _rng(seed=0):
    return np.random.default_rng(seed)


def test_spec_defaults_all_off():
    s = RandomizationSpec()
    assert not s.any_enabled
    ep = DomainRandomizer(s, _rng()).sample()
    assert ep.start_ndist is None and ep.reverse_dir is None and ep.friction_mu is None
    assert ep.track_color is None and ep.light_diffuse is None
    assert ep.steering_bias_rad == 0.0 and ep.motor_delay_steps == 0
    assert ep.camera_noise_std == 0.0 and ep.lidar_noise_std == 0.0


def test_spec_from_env(monkeypatch):
    for k in ("GYM_DR_VISUAL_DR", "GYM_DR_RANDOM_START", "GYM_DR_RANDOM_DIRECTION",
              "GYM_DR_FRICTION", "GYM_DR_STEERING_BIAS"):
        monkeypatch.setenv(k, "1")
    monkeypatch.setenv("GYM_DR_FRICTION_MIN", "0.5")
    monkeypatch.setenv("GYM_DR_FRICTION_MAX", "2.0")
    s = RandomizationSpec.from_env()
    assert s.visual_recolor and s.start_position and s.direction and s.friction and s.steering_bias
    assert s.friction_range == (0.5, 2.0)
    assert s.any_enabled


def test_sample_all_on_in_range():
    s = RandomizationSpec(visual_recolor=True, lighting=True, friction=True,
                          friction_range=(0.5, 2.0), start_position=True, direction=True,
                          steering_bias=True, steering_bias_deg=3.0, motor_delay=True,
                          motor_delay_max_steps=4, sensor_noise=True,
                          camera_noise_std=0.05, lidar_noise_std=0.02)
    ep = DomainRandomizer(s, _rng(1)).sample()
    assert 0.0 <= ep.start_ndist < 1.0
    assert isinstance(ep.reverse_dir, bool)
    assert 0.5 <= ep.friction_mu <= 2.0
    assert ep.track_color is not None and ep.background_color is not None
    # track and background colours are contrasting
    d = sum((a - b) ** 2 for a, b in zip(
        (ep.track_color.r, ep.track_color.g, ep.track_color.b),
        (ep.background_color.r, ep.background_color.g, ep.background_color.b)))
    assert d >= s.contrast_threshold
    assert ep.light_diffuse is not None
    assert abs(ep.steering_bias_rad) <= np.radians(3.0)
    assert 0 <= ep.motor_delay_steps <= 4
    assert ep.camera_noise_std == 0.05 and ep.lidar_noise_std == 0.02


def test_per_arena_seeds_differ():
    s = RandomizationSpec(visual_recolor=True, start_position=True)
    a = DomainRandomizer(s, _rng(1)).sample()
    b = DomainRandomizer(s, _rng(2)).sample()
    # different seeds -> different draws (decoupled arenas)
    assert a.start_ndist != b.start_ndist


def test_as_info_is_flat():
    s = RandomizationSpec(start_position=True, visual_recolor=True)
    info = DomainRandomizer(s, _rng(3), track_entity_name="racetrack_2").sample().as_info()
    assert set(["dr_start_ndist", "dr_track_color", "dr_steering_bias_rad"]).issubset(info)
    assert isinstance(info["dr_track_color"], list) and len(info["dr_track_color"]) == 4
