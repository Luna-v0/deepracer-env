#################################################################################
#   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#################################################################################
"""Model-state writer (teleport), re-homed onto the SimControl seam.

Same API as the ROS 1 tracker: ``set_model_state(model_state, blocking)`` where
``model_state`` is a :class:`deepracer_env.sim_control.compat.ModelState`. A
blocking call applies the pose immediately via ``SimControl.set_entity_state``
(per-entity — the basis for decoupled multi-arena resets); a non-blocking call
queues it and :meth:`update_tracker` flushes the queue once per sim tick.
"""
import threading

from deepracer_env.log_handler.deepracer_exceptions import GenericRolloutException
from deepracer_env.gazebo_tracker.abs_tracker import AbstractTracker
import deepracer_env.gazebo_tracker.constants as consts
from deepracer_env.runtime import get_sim_control
from deepracer_env.sim_control.compat import StateResponse, to_entity_state
from deepracer_env.sim_control.interface import SimControlError


class SetModelStateTracker(AbstractTracker):
    """Writes model pose/twist to the shared simulator backend."""

    _instance_ = None

    @staticmethod
    def get_instance():
        """Return the singleton, constructing it on first use."""
        if SetModelStateTracker._instance_ is None:
            SetModelStateTracker()
        return SetModelStateTracker._instance_

    def __init__(self):
        if SetModelStateTracker._instance_ is not None:
            raise GenericRolloutException("Attempting to construct multiple SetModelState Tracker")
        self.lock = threading.RLock()
        self._pending = {}  # model_name -> EntityState
        self._sim = get_sim_control()
        SetModelStateTracker._instance_ = self
        super(SetModelStateTracker, self).__init__(priority=consts.TrackerPriority.LOW)

    def set_model_state(self, model_state, blocking=False):
        """Teleport a model now (blocking) or on the next tick (queued).

        Args:
            model_state (compat.ModelState): target pose/twist + ``model_name``.
            blocking (bool): apply immediately vs. defer to ``update_tracker``.

        Returns:
            StateResponse: ``.success``.
        """
        with self.lock:
            if blocking:
                self._pending.pop(model_state.model_name, None)
                try:
                    ok = self._sim.set_entity_state(
                        model_state.model_name, to_entity_state(model_state))
                    return StateResponse(success=bool(ok))
                except SimControlError as ex:
                    return StateResponse(success=False, status_message=str(ex))
            self._pending[model_state.model_name] = to_entity_state(model_state)
            return StateResponse(success=True)

    def update_tracker(self, delta_time, sim_time):
        """Flush all queued teleports to the simulator."""
        with self.lock:
            pending, self._pending = self._pending, {}
        for name, entity_state in pending.items():
            try:
                self._sim.set_entity_state(name, entity_state)
            except Exception:  # noqa: BLE001 — a dropped queued write retries next set
                pass
