#################################################################################
#   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          #
#                                                                               #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#   You may not use this file except in compliance with the License.            #
#   You may obtain a copy of the License at                                     #
#                                                                               #
#       http://www.apache.org/licenses/LICENSE-2.0                              #
#                                                                               #
#   Unless required by applicable law or agreed to in writing, software         #
#   distributed under the License is distributed on an "AS IS" BASIS,           #
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.    #
#   See the License for the specific language governing permissions and         #
#   limitations under the License.                                              #
#################################################################################
"""Singleton helper for driving Gazebo model pose and visual updates."""

import logging

from std_msgs.msg import ColorRGBA

from deepracer_env.sim_control.compat import ModelState
from deepracer_env.log_handler.logger import Logger

# NOTE (Route B): the legacy visual-introspection services (GetVisualNames /
# GetVisuals / SetVisualColors / SetVisualTransparencies, provided by the deleted
# Gazebo-classic C++ plugin) have no gz-sim equivalent for *discovery*. Visual
# domain randomisation now applies colours through the SimControl seam's native
# gz ``visual_config`` (see the visual trackers). This ModelUpdater is retained
# only for its colour map + pose helper; the discovery methods are stubs pending
# the Phase-7 per-arena DomainRandomizer rework, and the module imports cleanly
# without the removed message packages.

logger = Logger(__name__, logging.INFO).get_logger()

# Colour name → (r, g, b, a) mapping used by update_color()
_COLOUR_MAP = {
    "black":   (0.0,  0.0,  0.0,  1.0),
    "white":   (1.0,  1.0,  1.0,  1.0),
    "grey":    (0.5,  0.5,  0.5,  1.0),
    "gray":    (0.5,  0.5,  0.5,  1.0),
    "red":     (1.0,  0.0,  0.0,  1.0),
    "blue":    (0.0,  0.0,  1.0,  1.0),
    "green":   (0.0,  0.8,  0.0,  1.0),
    "orange":  (1.0,  0.5,  0.0,  1.0),
    "purple":  (0.5,  0.0,  0.5,  1.0),
    "pink":    (1.0,  0.41, 0.71, 1.0),
    "cyan":    (0.0,  1.0,  1.0,  1.0),
    "yellow":  (1.0,  1.0,  0.0,  1.0),
}


def _colour_rgba(colour_name: str) -> ColorRGBA:
    """Return a ColorRGBA for *colour_name*.  Falls back to black if unknown."""
    r, g, b, a = _COLOUR_MAP.get(colour_name.lower(), (0.0, 0.0, 0.0, 1.0))
    return ColorRGBA(r=r, g=g, b=b, a=a)


class ModelUpdater:
    """Singleton that wraps the Gazebo ROS services for pose and visual updates."""

    _instance = None

    def __init__(self):
        from deepracer_env.runtime import get_sim_control
        self._sim = get_sim_control()

    @classmethod
    def get_instance(cls) -> "ModelUpdater":
        """Return (creating if necessary) the singleton instance."""
        if cls._instance is None:
            cls._instance = cls()
        return cls._instance

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def set_model_pose(self, model_name: str, model_pose) -> None:
        """Teleport *model_name* to *model_pose* (geometry_msgs/Pose)."""
        from deepracer_env.sim_control.compat import ros_to_seam_pose
        from deepracer_env.sim_control.types import EntityState
        try:
            self._sim.set_entity_state(model_name, EntityState(pose=ros_to_seam_pose(model_pose)))
        except Exception as ex:  # noqa: BLE001
            logger.warning("set_model_pose failed for %s: %s", model_name, ex)

    def get_model_visuals(self, racecar_name: str):
        """Discovery is unavailable on gz-sim (Route B); returns ``None``.

        Visual DR now recolours via the seam's native ``visual_config`` keyed by
        known link/visual names rather than runtime discovery — see the visual
        trackers and the Phase-7 per-arena DomainRandomizer plan.
        """
        return None

    def hide_visuals(self, visuals, ignore_keywords=None) -> None:
        """No-op stub (see :meth:`get_model_visuals`)."""

    def update_color(self, visuals, color: str) -> None:
        """No-op stub (see :meth:`get_model_visuals`)."""
