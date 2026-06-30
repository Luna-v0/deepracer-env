#################################################################################
#   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#################################################################################
"""Visual transparency tracker, re-homed onto the SimControl seam.

Used by domain randomisation / visual occlusion (gated, default off). Maps the
legacy ``deepracer_msgs/SetVisualTransparencies`` to
``SimControl.set_visual_transparency`` (native gz ``visual_config``). Same API.
"""
import threading
from collections import OrderedDict

from deepracer_env.log_handler.deepracer_exceptions import GenericRolloutException
from deepracer_env.gazebo_tracker.abs_tracker import AbstractTracker
import deepracer_env.gazebo_tracker.constants as consts
from deepracer_env.runtime import get_sim_control
from deepracer_env.sim_control.interface import CapabilityNotSupported
from deepracer_env.track_geom.constants import RACETRACK_MODEL_NAME


class _Response(object):
    def __init__(self, success=True, status_message=""):
        self.success = success
        self.status_message = status_message


class SetVisualTransparencyTracker(AbstractTracker):
    """Sets model-visual transparency via the shared simulator backend."""

    _instance_ = None

    @staticmethod
    def get_instance():
        if SetVisualTransparencyTracker._instance_ is None:
            SetVisualTransparencyTracker()
        return SetVisualTransparencyTracker._instance_

    def __init__(self, model_name=RACETRACK_MODEL_NAME):
        if SetVisualTransparencyTracker._instance_ is not None:
            raise GenericRolloutException("Attempting to construct multiple SetVisualTransparency Tracker")
        self.lock = threading.RLock()
        self._model_name = model_name
        self._pending = OrderedDict()  # (visual, link) -> transparency
        self._sim = get_sim_control()
        SetVisualTransparencyTracker._instance_ = self
        super(SetVisualTransparencyTracker, self).__init__(priority=consts.TrackerPriority.LOW)

    def set_visual_transparency(self, visual_name, link_name, transparency, blocking=False):
        """Set one visual's transparency now (blocking) or on the next tick."""
        if blocking:
            self._apply(visual_name, link_name, float(transparency))
        else:
            with self.lock:
                self._pending[(visual_name, link_name)] = float(transparency)
        return _Response(success=True)

    def _apply(self, visual_name, link_name, transparency):
        try:
            self._sim.set_visual_transparency(self._model_name, link_name, visual_name, transparency)
        except CapabilityNotSupported:
            pass
        except Exception:  # noqa: BLE001 — DR must never break a reset
            pass

    def update_tracker(self, delta_time, sim_time):
        """Flush queued transparency updates."""
        with self.lock:
            pending, self._pending = self._pending, OrderedDict()
        for (visual_name, link_name), transparency in pending.items():
            self._apply(visual_name, link_name, transparency)
