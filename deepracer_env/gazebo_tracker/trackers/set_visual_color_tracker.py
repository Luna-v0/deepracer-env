#################################################################################
#   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#################################################################################
"""Visual recolour tracker, re-homed onto the SimControl seam.

Used only by per-episode visual domain randomisation (gated by
``GYM_DR_VISUAL_DR``, default off). The legacy custom ``deepracer_msgs/
SetVisualColors`` service is replaced by ``SimControl.set_visual_color``, which
the ``ros_gz`` backend serves through gz-sim's native ``/world/<w>/visual_config``
(no custom plugin — Route B). Same API as the ROS 1 tracker.
"""
import threading
from collections import OrderedDict

from deepracer_env.log_handler.deepracer_exceptions import GenericRolloutException
from deepracer_env.gazebo_tracker.abs_tracker import AbstractTracker
import deepracer_env.gazebo_tracker.constants as consts
from deepracer_env.runtime import get_sim_control
from deepracer_env.sim_control.interface import CapabilityNotSupported
from deepracer_env.sim_control.types import ColorRGBA
from deepracer_env.track_geom.constants import RACETRACK_MODEL_NAME


def _to_seam_color(c):
    """Convert a std_msgs/ColorRGBA (or any .r/.g/.b/.a) to a seam ColorRGBA."""
    if c is None:
        return None
    return ColorRGBA(float(c.r), float(c.g), float(c.b), float(getattr(c, "a", 1.0)))


class _Response(object):
    def __init__(self, success=True, status_message=""):
        self.success = success
        self.status_message = status_message


class SetVisualColorTracker(AbstractTracker):
    """Recolours model visuals via the shared simulator backend."""

    _instance_ = None

    @staticmethod
    def get_instance():
        if SetVisualColorTracker._instance_ is None:
            SetVisualColorTracker()
        return SetVisualColorTracker._instance_

    def __init__(self, model_name=RACETRACK_MODEL_NAME):
        if SetVisualColorTracker._instance_ is not None:
            raise GenericRolloutException("Attempting to construct multiple SetVisualColor Tracker")
        self.lock = threading.RLock()
        self._model_name = model_name
        self._pending = OrderedDict()  # (visual, link) -> (diffuse, ambient)
        self._sim = get_sim_control()
        SetVisualColorTracker._instance_ = self
        super(SetVisualColorTracker, self).__init__(priority=consts.TrackerPriority.LOW)

    def set_visual_color(self, visual_name, link_name, ambient, diffuse,
                         specular=None, emissive=None, blocking=False):
        """Recolour one visual now (blocking) or on the next tick (queued)."""
        diffuse_c, ambient_c = _to_seam_color(diffuse), _to_seam_color(ambient)
        if blocking:
            self._apply(visual_name, link_name, diffuse_c, ambient_c)
        else:
            with self.lock:
                self._pending[(visual_name, link_name)] = (diffuse_c, ambient_c)
        return _Response(success=True)

    def _apply(self, visual_name, link_name, diffuse_c, ambient_c):
        try:
            self._sim.set_visual_color(self._model_name, link_name, visual_name,
                                       diffuse_c, ambient=ambient_c)
        except CapabilityNotSupported:
            pass  # backend without visual recolour: DR no-ops gracefully
        except Exception:  # noqa: BLE001 — DR must never break a reset
            pass

    def update_tracker(self, delta_time, sim_time):
        """Flush queued recolours."""
        with self.lock:
            pending, self._pending = self._pending, OrderedDict()
        for (visual_name, link_name), (diffuse_c, ambient_c) in pending.items():
            self._apply(visual_name, link_name, diffuse_c, ambient_c)
