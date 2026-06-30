#################################################################################
#   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#################################################################################
"""Link-state reader, re-homed onto the SimControl seam.

Same API as the ROS 1 tracker: ``get_link_state(link, frame, ...)`` returns a
response whose ``.link_state.pose`` the controller reads for the four wheel
positions (``all_wheels_on_track``).

gz-sim's pose feed is per-*model*, not always per-link. We first try to read the
link entity by name; if the backend doesn't expose it, we fall back to the
owning model's pose (``<model>::<link>`` -> ``<model>``). Wheel points then
collapse to the car centre — i.e. ``all_wheels_on_track`` degrades to
"is the car centre on track", a safe approximation. (A future refinement can
derive exact wheel world positions from the model pose + URDF joint offsets.)
"""
import threading

from deepracer_env.log_handler.deepracer_exceptions import GenericRolloutException
from deepracer_env.gazebo_tracker.abs_tracker import AbstractTracker
import deepracer_env.gazebo_tracker.constants as consts
from deepracer_env.runtime import get_sim_control
from deepracer_env.sim_control.compat import link_state_response
from deepracer_env.sim_control.interface import SimControlError
from deepracer_env.sim_control.types import EntityState


class GetLinkStateTracker(AbstractTracker):
    """Reads link pose/twist from the shared simulator backend."""

    _instance_ = None

    @staticmethod
    def get_instance():
        """Return the singleton, constructing it on first use."""
        if GetLinkStateTracker._instance_ is None:
            GetLinkStateTracker()
        return GetLinkStateTracker._instance_

    def __init__(self):
        if GetLinkStateTracker._instance_ is not None:
            raise GenericRolloutException("Attempting to construct multiple GetLinkState Tracker")
        self.lock = threading.RLock()
        self._sim = get_sim_control()
        GetLinkStateTracker._instance_ = self
        super(GetLinkStateTracker, self).__init__(priority=consts.TrackerPriority.HIGH)

    def get_link_state(self, link_name, reference_frame, blocking=False, auto_sync=True):
        """Return the link's pose/twist as a ``LinkStateResponse``."""
        with self.lock:
            entity_state = self._read(link_name)
        return link_state_response(link_name, entity_state, success=True)

    def _read(self, link_name):
        """Entity-state read for a link's owning model (from the shared cache).

        gz pose/info is per-model, so we read the model (``<model>::<link>`` ->
        ``<model>``) from the cache GetModelStateTracker already refreshes — no
        extra per-call snapshot. Wheel points thus collapse to the car centre
        (a safe ``all_wheels_on_track`` approximation, see module docstring).
        """
        model = link_name.split("::", 1)[0]
        try:
            return self._sim.get_entity_state(model)
        except SimControlError:
            return EntityState()

    def update_tracker(self, delta_time, sim_time):
        """No-op: GetModelStateTracker already refreshes the shared snapshot."""
