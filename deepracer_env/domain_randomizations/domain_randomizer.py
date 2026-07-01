#################################################################################
#   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#################################################################################
"""One per-arena domain randomizer over the whole DR catalog.

A :class:`DomainRandomizer` owns one arena's randomization: a dedicated RNG
(seeded from the arena's ``dr_seed`` so two arenas running the same track still
differ) and a :class:`~deepracer_env.domain_randomizations.spec.RandomizationSpec`.

Per episode it :meth:`sample`s an :class:`EpisodeRandomization` — every knob in
one immutable bundle — then:

* the **simulator-side** knobs (visual recolour, lighting) are applied with
  :meth:`apply_sim` through the :class:`SimControl` seam (native gz
  ``visual_config`` / ``light_config`` — no custom plugin, Route B), scoped to
  this arena's track entity;
* the **agent-side** knobs (start position, direction, wheel friction, steering
  bias, motor delay, sensor-noise levels) are returned in the bundle for the
  controller / sensors to consume; and
* the whole bundle is exposed via :meth:`EpisodeRandomization.as_info` so the
  applied randomization is recorded in the observation ``info`` — the supervision
  labels for the future camera→feature-vector model.

Decoupling: each car's controller holds its own ``DomainRandomizer`` keyed to its
own arena, so resetting/recolouring one car never touches another.
"""
from __future__ import annotations

import logging
import math
import os
import time
from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple

from deepracer_env.sim_control.interface import Capability, CapabilityNotSupported
from deepracer_env.sim_control.types import ColorRGBA
from deepracer_env.domain_randomizations.spec import RandomizationSpec
from deepracer_env.track_geom.constants import RACETRACK_MODEL_NAME

LOG = logging.getLogger(__name__)

# Default link/visual targets in the shipped track models (see VisualRandomizer
# notes): the track surface and the surrounding backdrop.
_TRACK_LINK = "track"
_BG_LINK = "background"
_VISUAL = "visual"


@dataclass(frozen=True)
class EpisodeRandomization:
    """The randomization sampled for one episode (all knobs in one bundle)."""

    start_ndist: Optional[float] = None        # normalized start along centre-line
    reverse_dir: Optional[bool] = None         # True => CW/CCW flip
    friction_mu: Optional[float] = None        # wheel μ for this episode
    track_color: Optional[ColorRGBA] = None
    background_color: Optional[ColorRGBA] = None
    light_diffuse: Optional[ColorRGBA] = None
    light_direction: Optional[Tuple[float, float, float]] = None
    steering_bias_rad: float = 0.0             # added to every steering command
    motor_delay_steps: int = 0                 # action lag, in steps
    camera_noise_std: float = 0.0
    lidar_noise_std: float = 0.0

    def as_info(self) -> Dict[str, object]:
        """Flatten to plain types for the observation ``info`` (dataset labels)."""
        def _c(c):
            return None if c is None else [c.r, c.g, c.b, c.a]
        return {
            "dr_start_ndist": self.start_ndist,
            "dr_reverse_dir": self.reverse_dir,
            "dr_friction_mu": self.friction_mu,
            "dr_track_color": _c(self.track_color),
            "dr_background_color": _c(self.background_color),
            "dr_light_diffuse": _c(self.light_diffuse),
            "dr_steering_bias_rad": self.steering_bias_rad,
            "dr_motor_delay_steps": self.motor_delay_steps,
            "dr_camera_noise_std": self.camera_noise_std,
            "dr_lidar_noise_std": self.lidar_noise_std,
        }


class DomainRandomizer:
    """Samples + applies the DR catalog for ONE arena."""

    def __init__(
        self,
        spec: RandomizationSpec,
        rng,
        *,
        track_entity_name: str = RACETRACK_MODEL_NAME,
        light_name: str = "sun",
    ) -> None:
        """Create the per-arena randomizer.

        Args:
            spec: The active knobs + ranges.
            rng: A ``numpy.random.Generator`` (seed from the arena's ``dr_seed``).
            track_entity_name: This arena's track entity (e.g. ``"racetrack_2"``)
                — recolour targets are scoped to it.
            light_name: Scene light to randomize (the converted worlds' ``sun``).
        """
        self._spec = spec
        self._rng = rng
        self._track_entity = track_entity_name
        self._light_name = light_name
        # The scene-side DR (recolor + lighting) goes through gz-transport SERVICES
        # (visual_config / light_config) which have no ROS bridge, so they run as
        # `gz` CLI subprocesses. Calling them EVERY episode churns gz-transport
        # connections/ephemeral ports until the server returns "Host unreachable"
        # and the run wedges/segfaults over a long rollout. Throttle them to at
        # most once per _GZ_DR_MIN_INTERVAL_S (wall) — the mesh recolor is visually
        # a no-op on the baked track anyway, and lighting only needs to vary
        # slowly; per-EPISODE visual variation comes from the PHOTOMETRIC DR
        # (brightness/contrast/gamma on the frames, Python-side, unthrottled).
        self._gz_dr_min_interval_s = float(os.getenv("GYM_DR_GZ_DR_INTERVAL_S", "5.0"))
        self._last_gz_dr_t = 0.0

    # -- sampling --------------------------------------------------------------

    def sample(self) -> EpisodeRandomization:
        """Sample one episode's randomization from the spec + RNG."""
        s, rng = self._spec, self._rng

        start_ndist = float(rng.random()) if s.start_position else None
        reverse_dir = bool(rng.random() < 0.5) if s.direction else None
        friction_mu = (float(rng.uniform(*s.friction_range)) if s.friction else None)

        track_color = bg_color = None
        if s.visual_recolor:
            track_color = self._sample_color()
            bg_color = self._sample_contrasting(track_color, s.contrast_threshold)

        light_diffuse = light_dir = None
        if s.lighting:
            # Keep lights fairly bright (0.6–1.0) so the scene stays usable.
            g = float(rng.uniform(0.6, 1.0))
            light_diffuse = ColorRGBA(g * float(rng.uniform(0.85, 1.0)),
                                      g * float(rng.uniform(0.85, 1.0)),
                                      g * float(rng.uniform(0.85, 1.0)), 1.0)
            light_dir = (float(rng.uniform(-0.5, 0.5)),
                         float(rng.uniform(-0.5, 0.5)), -1.0)

        steering_bias_rad = (math.radians(float(rng.uniform(-s.steering_bias_deg, s.steering_bias_deg)))
                             if s.steering_bias else 0.0)
        motor_delay_steps = (int(rng.integers(0, s.motor_delay_max_steps + 1))
                             if s.motor_delay else 0)

        return EpisodeRandomization(
            start_ndist=start_ndist,
            reverse_dir=reverse_dir,
            friction_mu=friction_mu,
            track_color=track_color,
            background_color=bg_color,
            light_diffuse=light_diffuse,
            light_direction=light_dir,
            steering_bias_rad=steering_bias_rad,
            motor_delay_steps=motor_delay_steps,
            camera_noise_std=(s.camera_noise_std if s.sensor_noise else 0.0),
            lidar_noise_std=(s.lidar_noise_std if s.sensor_noise else 0.0),
        )

    def _sample_color(self) -> ColorRGBA:
        r, g, b = (float(x) for x in self._rng.random(3))
        return ColorRGBA(r, g, b, 1.0)

    def _sample_contrasting(self, other: ColorRGBA, threshold: float) -> ColorRGBA:
        """Sample a colour at least ``threshold`` (squared RGB) from *other*."""
        for _ in range(8):
            c = self._sample_color()
            d = (c.r - other.r) ** 2 + (c.g - other.g) ** 2 + (c.b - other.b) ** 2
            if d >= threshold:
                return c
        return c  # last sample if we never cleared the bar

    # -- application (simulator side) ------------------------------------------

    def apply_sim(self, sim, episode: EpisodeRandomization) -> None:
        """Apply the simulator-side knobs (recolour + lighting) for *episode*.

        Best-effort: a backend without a capability, or a target the scene does
        not expose, is logged and skipped — DR must never break a reset.

        The scene-side ops (recolor + lighting) are gz-CLI and are THROTTLED to
        ``_gz_dr_min_interval_s`` to avoid exhausting gz-transport over a long
        rollout (see __init__). Per-episode visual variation is carried by the
        photometric DR applied to the frames elsewhere.
        """
        now = time.monotonic()
        if (now - self._last_gz_dr_t) < self._gz_dr_min_interval_s:
            return  # too soon since the last gz-CLI DR — skip to spare gz-transport
        did_gz = False
        if episode.track_color is not None and sim.supports(Capability.VISUAL_RECOLOR):
            self._recolor(sim, _TRACK_LINK, episode.track_color)
            if episode.background_color is not None:
                self._recolor(sim, _BG_LINK, episode.background_color)
            did_gz = True
        if episode.light_diffuse is not None and sim.supports(Capability.LIGHTING):
            try:
                sim.set_light(self._light_name, diffuse=episode.light_diffuse,
                              direction=episode.light_direction)
                did_gz = True
            except (CapabilityNotSupported, Exception) as ex:  # noqa: BLE001
                LOG.debug("lighting DR skipped: %s", ex)
        if did_gz:
            self._last_gz_dr_t = now

    def _recolor(self, sim, link: str, color: ColorRGBA) -> None:
        ambient = ColorRGBA(color.r * 0.6, color.g * 0.6, color.b * 0.6, color.a)
        try:
            sim.set_visual_color(self._track_entity, link, _VISUAL, color, ambient=ambient)
        except (CapabilityNotSupported, Exception) as ex:  # noqa: BLE001
            LOG.debug("visual recolour of %s::%s skipped: %s", self._track_entity, link, ex)
