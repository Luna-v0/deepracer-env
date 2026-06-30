#################################################################################
#   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#################################################################################
"""The :class:`SimControl` seam — one interface over the whole simulator.

Architecture (ports & adapters / hexagonal)
-------------------------------------------
``SimControl`` is the **port**: a small, intention-revealing interface that the
environment depends on. The concrete **adapters** live in
:mod:`deepracer_env.sim_control.backends`:

* :class:`~deepracer_env.sim_control.backends.simulation_interfaces_backend.SimulationInterfacesBackend`
  — the *primary* path, built on the ROS 2 standard ``simulation_interfaces``
  services (``SpawnEntity``/``GetEntityState``/``StepSimulation``/…). Being a
  cross-vendor standard, this is the most future-proof option.
* :class:`~deepracer_env.sim_control.backends.ros_gz_backend.RosGzBackend`
  — the *fallback*, built on ``ros_gz_sim`` services and the
  ``/world/<world>/control`` topic, for capabilities the standard does not yet
  expose on a given Gazebo release.
* :class:`~deepracer_env.sim_control.backends.visual_shim.VisualShim`
  — the only genuinely DeepRacer-specific capability (visual recolour /
  transparency for domain randomisation), bridged from a tiny gz-sim system
  plugin.

Why this replaces ~1,700 lines of C++ and 99 service call sites
---------------------------------------------------------------
The legacy stack reached the simulator through a custom Gazebo-classic
``SystemPlugin`` plus 14 distinct ``gazebo_msgs``/``deepracer_msgs`` services
scattered across 13 modules. Every one of those calls collapses to one of the
eight verbs below. Concentrating them behind a single seam means the ROS↔sim
coupling is swappable in one place (Strategy pattern) and mockable for tests
(see :class:`NullSimControl`).

Decoupled multi-arena (the load-bearing design choice)
------------------------------------------------------
Every method here is **per-entity**, never world-global. That is what lets one
shared simulator host *N* independent arenas: resetting ``car_3`` teleports only
``car_3`` (``set_entity_state``); it never touches the world or the other cars.
A global ``reset_world`` is deliberately *absent* from the core verbs — see
:meth:`reset` for the narrow, intentionally-discouraged exception.
"""
from __future__ import annotations

import abc
from typing import List, Optional

from deepracer_env.sim_control.types import (
    ColorRGBA,
    EntityState,
    Pose,
    IDENTITY_POSE,
)


class SimControlError(RuntimeError):
    """Base class for every error raised by a :class:`SimControl` backend."""


class SimControlTimeout(SimControlError):
    """A simulator service did not answer within the allotted time."""


class SimControlDead(SimControlError):
    """The simulator process is no longer reachable.

    Mirrors the legacy :class:`~deepracer_env.environments.world_swap.WorldSwapError`
    contract: a looping caller (training rotation) should catch this, checkpoint,
    and restart the sim container rather than spin on a dead backend.
    """


class CapabilityNotSupported(SimControlError):
    """The active backend cannot perform the requested operation.

    Raised, for example, when visual recolour is requested but no
    :class:`VisualShim` is wired in. Callers that treat the capability as
    optional (domain randomisation) should guard with :meth:`SimControl.supports`.
    """


class Capability:
    """String constants for the optional capabilities a backend may advertise.

    Used with :meth:`SimControl.supports` so that optional features (visual DR,
    deterministic stepping) degrade gracefully instead of crashing on a backend
    that lacks them.
    """

    DETERMINISTIC_STEP = "deterministic_step"
    """Backend can advance the world by an exact integer number of steps."""

    VISUAL_RECOLOR = "visual_recolor"
    """Backend can set per-visual material colour / transparency at runtime."""

    LIGHTING = "lighting"
    """Backend can recolour / re-aim scene lights at runtime (lighting DR)."""

    LINK_STATE = "link_state"
    """Backend can read the pose/twist of an individual link, not just a model."""


class SimControl(abc.ABC):
    """Abstract control plane for a running simulation.

    Concrete backends implement the eight verbs below. All poses are in the
    world frame unless a ``reference_frame``/arena-local convention is noted by
    the caller. Implementations must be safe to call from the environment's
    stepping thread; backends that own an executor handle their own locking.
    """

    # -- capability negotiation ------------------------------------------------

    def supports(self, capability: str) -> bool:
        """Return whether this backend advertises *capability*.

        Args:
            capability: One of the :class:`Capability` constants.

        Returns:
            ``True`` if the operation is available; ``False`` otherwise. The
            default implementation reports no optional capabilities.
        """
        return False

    # -- entity lifecycle ------------------------------------------------------

    @abc.abstractmethod
    def spawn_entity(
        self,
        name: str,
        sdf: str,
        pose: Pose = IDENTITY_POSE,
        *,
        allow_renaming: bool = False,
    ) -> str:
        """Insert a new entity described by an SDF/URDF string.

        Replaces ``gazebo_msgs/SpawnModel`` (track meshes, obstacle boxes, cars).

        Args:
            name: Desired unique entity name (e.g. ``"racetrack_0"``).
            sdf: The model description (an ``<sdf>`` document or ``<include>``
                wrapper). The backend resolves ``model://`` URIs through the
                Gazebo resource path exactly as world-load does.
            pose: World pose of the model root. Note that an ``<include>`` block
                carries its own ``<pose>`` which *overrides* this — see
                :class:`~deepracer_env.environments.world_swap` for why offsets
                must be embedded in the include.
            allow_renaming: If ``True``, the backend may append a suffix on name
                collision and return the actual name.

        Returns:
            The name the entity was actually spawned under.

        Raises:
            SimControlDead: The simulator died during the spawn.
            SimControlError: The spawn was rejected.
        """

    @abc.abstractmethod
    def delete_entity(self, name: str) -> bool:
        """Remove a named entity. Replaces ``gazebo_msgs/DeleteModel``.

        Args:
            name: The entity to remove.

        Returns:
            ``True`` if the entity was removed (or was already absent).

        Raises:
            SimControlDead: The simulator died during the delete (the classic
                intermittent segfault on mesh delete).
        """

    @abc.abstractmethod
    def list_entities(self) -> List[str]:
        """Return the names of all top-level entities currently in the world.

        Replaces ``gazebo_msgs/GetWorldProperties.model_names``; used by the
        track swap to confirm a mesh is present/absent.
        """

    # -- state read / write ----------------------------------------------------

    @abc.abstractmethod
    def get_entity_state(
        self, name: str, *, reference_frame: str = "world"
    ) -> EntityState:
        """Read a model's pose and twist. Replaces ``GetModelState(s)``.

        This is the hot path: the car's state is read every step to compute the
        reward parameters and evaluate the reset rules.

        Args:
            name: The entity to query.
            reference_frame: Frame to express the state in (default world).

        Returns:
            The entity's :class:`EntityState`.

        Raises:
            SimControlError: The entity is unknown.
        """

    @abc.abstractmethod
    def set_entity_state(
        self, name: str, state: EntityState, *, blocking: bool = True
    ) -> bool:
        """Teleport an entity to a pose/twist. Replaces ``SetModelState(s)``.

        This is how a car is reset to the start line and how obstacles are
        placed. It is strictly **per-entity** — the cornerstone of decoupled
        multi-arena resets.

        Args:
            name: The entity to move.
            state: Target pose and twist.
            blocking: If ``True``, return only once the write has been applied
                (the legacy ``SetModelState(blocking=True)`` semantics relied on
                by the reset path).

        Returns:
            ``True`` on success.
        """

    def get_link_state(
        self, entity: str, link: str, *, reference_frame: str = "world"
    ) -> EntityState:
        """Read the pose/twist of one link of a model.

        Optional (advertise :attr:`Capability.LINK_STATE`). Replaces
        ``GetLinkState(s)``; used for fine-grained vehicle dynamics. The default
        implementation raises :class:`CapabilityNotSupported`.
        """
        raise CapabilityNotSupported("get_link_state is not supported by this backend")

    # -- time control ----------------------------------------------------------

    def refresh_state(self, force: bool = False) -> None:
        """Refresh any internal pose/state cache (one batched read).

        Backends that batch entity-state reads (the ``ros_gz`` backend snapshots
        ``/world/<w>/pose/info`` once per call) override this; the tracker layer
        calls it from the simulation-clock callback so subsequent
        ``get_entity_state`` reads are cheap and consistent.

        Args:
            force: Bypass any rate-limiting and refresh immediately (used for the
                blocking reads a reset depends on). When ``False`` the backend
                may throttle: the gz ``/clock`` ticks far faster than a CLI
                snapshot can run, so an unthrottled per-tick refresh would spawn
                a subprocess storm.

        The default is a no-op (backends that read per-entity live need no cache).
        """

    @abc.abstractmethod
    def step(self, n: int = 1) -> None:
        """Advance the simulation by *n* steps and block until they complete.

        Where the backend advertises :attr:`Capability.DETERMINISTIC_STEP` (true
        for ``simulation_interfaces/StepSimulation``), this is race-free and
        reproducible across runs given a seed — a strict upgrade over the legacy
        pause/unpause dance, which was a known source of nondeterminism.

        Args:
            n: Number of simulation steps to advance.
        """

    @abc.abstractmethod
    def pause(self) -> None:
        """Pause physics. Replaces ``/gazebo/pause_physics``."""

    @abc.abstractmethod
    def unpause(self) -> None:
        """Resume physics. Replaces ``/gazebo/unpause_physics``."""

    def reset(self) -> None:
        """Reset the *entire* world.

        Intentionally discouraged: in a decoupled multi-arena world this clobbers
        every arena at once. Per-episode resets must use
        :meth:`set_entity_state` on the single car instead. Provided only for
        single-arena bring-up/tests. The default raises so misuse is loud.
        """
        raise CapabilityNotSupported(
            "Global world reset is disabled to protect multi-arena decoupling; "
            "reset a single car with set_entity_state() instead."
        )

    # -- visual domain randomisation (optional) --------------------------------

    def set_visual_color(
        self,
        entity: str,
        link: str,
        visual: str,
        diffuse: ColorRGBA,
        *,
        ambient: Optional[ColorRGBA] = None,
        blocking: bool = True,
    ) -> bool:
        """Recolour one visual of a model (per-arena visual DR).

        Optional (advertise :attr:`Capability.VISUAL_RECOLOR`). Replaces the
        custom ``deepracer_msgs/SetVisualColors`` service that the legacy C++
        plugin provided. The default raises :class:`CapabilityNotSupported`.

        Args:
            entity: Model name (e.g. ``"racetrack_2"``).
            link: Link within the model.
            visual: Visual within the link.
            diffuse: New diffuse colour.
            ambient: New ambient colour; defaults to ``diffuse`` scaled down by
                the backend if omitted.
            blocking: Wait for the change to apply before returning.

        Returns:
            ``True`` on success.
        """
        raise CapabilityNotSupported("visual recolour is not supported by this backend")

    def set_visual_transparency(
        self, entity: str, link: str, visual: str, transparency: float,
        *, blocking: bool = True,
    ) -> bool:
        """Set a visual's transparency in ``[0, 1]`` (0 opaque, 1 invisible).

        Optional (advertise :attr:`Capability.VISUAL_RECOLOR`). The default
        raises :class:`CapabilityNotSupported`.
        """
        raise CapabilityNotSupported("visual transparency is not supported by this backend")

    def set_light(
        self,
        name: str,
        *,
        diffuse: Optional[ColorRGBA] = None,
        specular: Optional[ColorRGBA] = None,
        direction: Optional["tuple"] = None,
        blocking: bool = True,
    ) -> bool:
        """Recolour / re-aim a named scene light (lighting domain randomisation).

        Optional (advertise :attr:`Capability.LIGHTING`). On the ``ros_gz``
        backend this is gz-sim's native ``/world/<w>/light_config`` — no custom
        plugin. The default raises :class:`CapabilityNotSupported`.

        Args:
            name: Light name (e.g. ``"sun"`` in the converted worlds).
            diffuse: New diffuse colour, if given.
            specular: New specular colour, if given.
            direction: New ``(x, y, z)`` direction for a directional light.
            blocking: Wait for the change to apply before returning.

        Returns:
            ``True`` on success.
        """
        raise CapabilityNotSupported("lighting is not supported by this backend")

    # -- lifecycle -------------------------------------------------------------

    def close(self) -> None:
        """Release backend resources (clients, executor threads).

        Idempotent; safe to call even after a partially-failed construction. The
        default is a no-op for backends that hold nothing.
        """


class NullSimControl(SimControl):
    """A no-op :class:`SimControl` that records calls — for tests and dry imports.

    Lets the environment be constructed and exercised with zero ROS/simulator
    dependency (Null Object pattern). Reads return identity/empty values; writes
    are recorded in :attr:`calls` for assertions.
    """

    def __init__(self) -> None:
        self.calls: List[tuple] = []
        self._entities: List[str] = []

    def _record(self, *call: object) -> None:
        self.calls.append(call)

    def supports(self, capability: str) -> bool:  # noqa: D102 (inherited)
        return True

    def spawn_entity(self, name, sdf, pose=IDENTITY_POSE, *, allow_renaming=False):  # noqa: D102
        self._record("spawn_entity", name, pose)
        if name not in self._entities:
            self._entities.append(name)
        return name

    def delete_entity(self, name):  # noqa: D102
        self._record("delete_entity", name)
        if name in self._entities:
            self._entities.remove(name)
        return True

    def list_entities(self):  # noqa: D102
        return list(self._entities)

    def get_entity_state(self, name, *, reference_frame="world"):  # noqa: D102
        self._record("get_entity_state", name)
        return EntityState()

    def set_entity_state(self, name, state, *, blocking=True):  # noqa: D102
        self._record("set_entity_state", name, state)
        return True

    def get_link_state(self, entity, link, *, reference_frame="world"):  # noqa: D102
        self._record("get_link_state", entity, link)
        return EntityState()

    def step(self, n=1):  # noqa: D102
        self._record("step", n)

    def pause(self):  # noqa: D102
        self._record("pause")

    def unpause(self):  # noqa: D102
        self._record("unpause")

    def set_visual_color(self, entity, link, visual, diffuse, *, ambient=None, blocking=True):  # noqa: D102
        self._record("set_visual_color", entity, link, visual, diffuse)
        return True

    def set_visual_transparency(self, entity, link, visual, transparency, *, blocking=True):  # noqa: D102
        self._record("set_visual_transparency", entity, link, visual, transparency)
        return True
