#################################################################################
#   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#################################################################################
"""Model-state reader, re-homed onto the SimControl seam.

Same public API as the ROS 1 tracker (``get_model_state(name, frame, blocking,
auto_sync)`` returning a ``.success`` / ``.pose`` / ``.twist`` response), but the
batched ``deepracer_msgs/GetModelStates`` service is replaced by the shared
:class:`SimControl` backend: :meth:`update_tracker` takes one pose snapshot per
sim tick and :meth:`get_model_state` reads it back. Consumers (the controller,
track geometry) are unchanged.
"""
import threading

from deepracer_env.log_handler.deepracer_exceptions import GenericRolloutException
from deepracer_env.gazebo_tracker.abs_tracker import AbstractTracker
import deepracer_env.gazebo_tracker.constants as consts
from deepracer_env.runtime import get_sim_control
from deepracer_env.sim_control.compat import StateResponse, state_response
from deepracer_env.sim_control.interface import SimControlError


class GetModelStateTracker(AbstractTracker):
    """Reads model pose/twist from the shared simulator backend."""

    _instance_ = None

    @staticmethod
    def get_instance():
        """Return the singleton, constructing it on first use."""
        if GetModelStateTracker._instance_ is None:
            GetModelStateTracker()
        return GetModelStateTracker._instance_

    def __init__(self):
        if GetModelStateTracker._instance_ is not None:
            raise GenericRolloutException("Attempting to construct multiple GetModelState Tracker")
        self.lock = threading.RLock()
        self._tracked = set()
        self._sim = get_sim_control()
        GetModelStateTracker._instance_ = self
        super(GetModelStateTracker, self).__init__(priority=consts.TrackerPriority.HIGH)

    def get_model_state(self, model_name, relative_entity_name, blocking=False, auto_sync=True):
        """Return the model's pose/twist as a ``StateResponse``.

        Args:
            model_name (str): entity to read.
            relative_entity_name (str): reference frame (kept for API parity).
            blocking (bool): force a fresh snapshot before reading.
            auto_sync (bool): keep the model in the per-tick refresh set.

        Returns:
            StateResponse: ``.success`` / ``.pose`` / ``.twist``.
        """
        with self.lock:
            if auto_sync:
                self._tracked.add(model_name)
            if blocking:
                self._sim.refresh_state(force=True)
            try:
                return state_response(self._sim.get_entity_state(model_name), success=True)
            except SimControlError as ex:
                return StateResponse(success=False, status_message=str(ex))

    def update_tracker(self, delta_time, sim_time):
        """Take one batched pose snapshot for the tracked models."""
        if self._tracked:
            try:
                self._sim.refresh_state()
            except Exception:  # noqa: BLE001 — a missed tick is non-fatal
                pass

    def remove(self, model_name, relative_entity_name=""):
        """Stop tracking *model_name*."""
        with self.lock:
            self._tracked.discard(model_name)
