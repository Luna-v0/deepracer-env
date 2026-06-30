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

import abc
import logging
import threading

from deepracer_env.log_handler.deepracer_exceptions import GenericRolloutException
from deepracer_env.log_handler.logger import Logger
from deepracer_env.cameras.camera_manager import CameraManager
# ROS 2 port: the ``/gazebo/spawn_sdf_model`` ServiceProxy (ServiceProxyWrapper +
# gazebo_msgs/SpawnModel) is replaced by the shared SimControl seam. These
# cameras are cosmetic spectator/follow cameras, so spawning is best-effort —
# any failure is swallowed so it can never break the controller's reset/step.
from deepracer_env.runtime import get_sim_control
from deepracer_env.sim_control.compat import ros_to_seam_pose

# Python 2 and 3 compatible Abstract class
ABC = abc.ABCMeta('ABC', (object,), {})

LOG = Logger(__name__, logging.INFO).get_logger()


class AbstractCamera(ABC):
    """
    Abstract Camera method
    """
    def __init__(self, name, namespace, model_name):
        if not name or not isinstance(name, str):
            raise GenericRolloutException("Camera name cannot be None or empty string")
        self._name = name
        self._model_name = model_name or self._name
        self._namespace = namespace or 'default'
        self.lock = threading.Lock()
        self.is_reset_called = False
        # Guard so a failed (best-effort) spawn only logs once per camera.
        self._spawn_failed_logged = False
        CameraManager.get_instance().add(self, namespace)

    @property
    def name(self):
        """Return name of Camera

        Returns:
            (str): the name of camera
        """
        return self._name

    @property
    def model_name(self):
        """Return name of gazebo topic

        Returns:
            (str): the name of camera
        """
        return self._model_name

    @property
    def namespace(self):
        """Return namespace of camera in camera manager

        Returns:
            (str): the namespace of camera in camera manager
        """
        return self._namespace

    def reset_pose(self, car_pose):
        """
        Reset the camera pose

        Args:
            car_pose (Pose): Pose of car
        """
        with self.lock:
            self._reset(car_pose)
            self.is_reset_called = True

    def spawn_model(self, car_pose, camera_sdf_path):
        """
        Spawns a sdf model located in the given path

        ROS 2 port: replaces the ``/gazebo/spawn_sdf_model`` ServiceProxy call
        with ``SimControl.spawn_entity`` via the shared backend. The camera pose
        is a ``geometry_msgs/Pose`` (still valid on ROS 2), converted to a seam
        ``Pose`` at the boundary. Wrapped in try/except: a cosmetic spectator
        camera must never break the controller's reset/step path.

        Args:
            car_pose (Pose): Pose of car
            camera_sdf_path (string): full path to the location of sdf file
        """
        try:
            camera_sdf = self._get_sdf_string(camera_sdf_path)
            camera_pose = self._get_initial_camera_pose(car_pose)
            get_sim_control().spawn_entity(
                self.model_name, camera_sdf,
                pose=ros_to_seam_pose(camera_pose),
                allow_renaming=False)
        except Exception as ex:  # noqa: BLE001 — cosmetic camera, spawn is best-effort
            if not self._spawn_failed_logged:
                LOG.info("[AbstractCamera]: spawn of cosmetic camera '%s' "
                         "failed (no-op): %s", self.model_name, ex)
                self._spawn_failed_logged = True

    def update_pose(self, car_pose, delta_time):
        """
        Update the camera pose

        Args:
            car_pose (Pose): Pose of car
            delta_time (float): time delta from last update
        """
        with self.lock:
            if self.is_reset_called:
                self._update(car_pose, delta_time)

    @abc.abstractmethod
    def _get_sdf_string(self, camera_sdf_path):
        """
        Reads the sdf file and converts it to a string in
        memory

        Args:
            camera_sdf_path (string): full path to the location of sdf file
        """
        raise NotImplementedError('Camera must read and convert model sdf file')

    @abc.abstractmethod
    def _reset(self, car_pose):
        """
        Reset the camera pose
        """
        raise NotImplementedError('Camera must be able to reset')

    @abc.abstractmethod
    def _update(self, car_pose, delta_time):
        """Update the camera pose
        """
        raise NotImplementedError('Camera must be able to update')

    @abc.abstractmethod
    def _get_initial_camera_pose(self, car_pose):
        """compuate camera pose
        """
        raise NotImplementedError('Camera must be able to compuate pose')

    def detach(self):
        """Detach camera from manager"""
        CameraManager.get_instance().remove(self, self.namespace)

