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

'''Concrete sensor implementations for the DeepRacer Gazebo environment.

Each sensor subscribes to a ROS topic pub by Gazebo, buffers incoming
messages, and exposes them via the :class:`~markov.sensors.sensor_interface.SensorInterface`
API.  The ``get_observation_space()`` method returns a ``gymnasium.spaces.Dict``
whose single key is the sensor\\'s ``Input.value`` string, ready to be merged
by :class:`~markov.sensors.composite_sensor.CompositeSensor`.
'''
import logging
import os
import numpy as np
from sensor_msgs.msg import Image as sensor_image
from sensor_msgs.msg import LaserScan
from PIL import Image

# ROS 2 port: subscriptions are created on the shared, background-spinning
# rclpy node provided by deepracer_env.runtime instead of rospy.Subscriber.
from deepracer_env.runtime import get_node
from deepracer_env.sensors.utils import get_observation_space
from deepracer_env.sensors.sensor_interface import SensorInterface, LidarInterface
from deepracer_env.sensors.constants import (
    Input,
    LIDAR_360_DEGREE_MIN_RANGE,
    LIDAR_360_DEGREE_MAX_RANGE,
)
from deepracer_env.environments.constants import TRAINING_IMAGE_SIZE
from deepracer_env.log_handler.deepracer_exceptions import GenericRolloutException, GenericError
from deepracer_env import utils
from deepracer_env.log_handler.logger import Logger
from deepracer_env.log_handler.constants import SIMAPP_SIMULATION_WORKER_EXCEPTION


LOGGER = Logger(__name__, logging.INFO).get_logger()


# ---------------------------------------------------------------------------
# Optional domain-randomisation observation noise (OFF by default).
#
# Zero-mean Gaussian noise can be injected at ``get_state()`` time to widen the
# sim-to-real gap during training.  Both knobs default to 0.0, in which case the
# noise path is a strict no-op: no RNG is drawn and the original array object is
# returned untouched, so behaviour is byte-identical to the noise-free code.
#
#   * ``GYM_DR_CAMERA_NOISE_STD`` — std expressed as a *fraction* of the full
#     8-bit scale; the per-pixel std actually applied is ``std * 255`` before
#     the result is clipped back to uint8 ``[0, 255]``.
#   * ``GYM_DR_LIDAR_NOISE_STD``  — std in metres applied to the raw range
#     readings before clipping to the sensor's ``[min, max]`` range.
# ---------------------------------------------------------------------------

def _read_noise_std(env_var):
    '''Read a non-negative Gaussian-noise std from *env_var* (default 0.0).

    Any unparsable or negative value is treated as 0.0 (noise disabled).
    '''
    try:
        std = float(os.environ.get(env_var, 0.0))
    except (TypeError, ValueError):
        LOGGER.warning("Invalid %s value; disabling sensor noise.", env_var)
        return 0.0
    return std if std > 0.0 else 0.0


CAMERA_NOISE_STD = _read_noise_std("GYM_DR_CAMERA_NOISE_STD")
LIDAR_NOISE_STD = _read_noise_std("GYM_DR_LIDAR_NOISE_STD")


def _apply_camera_noise(image_array):
    '''Return *image_array* with optional Gaussian DR noise (uint8, ``[0, 255]``).

    No-op (returns the input unchanged) when ``GYM_DR_CAMERA_NOISE_STD <= 0``.
    '''
    if CAMERA_NOISE_STD <= 0.0:
        return image_array
    noisy = image_array.astype(np.float32) + np.random.normal(
        0.0, CAMERA_NOISE_STD * 255.0, size=image_array.shape)
    return np.clip(noisy, 0, 255).astype(np.uint8)


def _apply_lidar_noise(ranges, low, high):
    '''Return *ranges* with optional Gaussian DR noise clipped to ``[low, high]``.

    A fresh array is always returned (the input/buffered array is never mutated),
    which preserves the ``DoubleBuffer(clear_data_on_get=False)`` semantics where
    the same raw scan may be read repeatedly.  No-op (returns the input
    unchanged) when ``GYM_DR_LIDAR_NOISE_STD <= 0``.
    '''
    if LIDAR_NOISE_STD <= 0.0:
        return ranges
    arr = np.asarray(ranges)
    noisy = arr + np.random.normal(0.0, LIDAR_NOISE_STD, size=arr.shape)
    return np.clip(noisy, low, high).astype(arr.dtype)


class SensorFactory(object):
    '''This class implements a sensot factory and is used to create sensors per
       agent.
    '''
    @staticmethod
    def create_sensor(racecar_name, sensor_type, config_dict):
        '''Factory method for creating sensors
            type - String containing the desired sensor type
            kwargs - Meta data, usually containing the topics to subscribe to, the
                     concrete sensor classes are responsible for checking the topics.
        '''
        if sensor_type == Input.CAMERA.value:
            return Camera(racecar_name)
        elif sensor_type == Input.LEFT_CAMERA.value:
            return LeftCamera(racecar_name)
        elif sensor_type == Input.STEREO.value:
            return DualCamera(racecar_name)
        elif sensor_type == Input.LIDAR.value:
            return Lidar(racecar_name)
        elif sensor_type == Input.SECTOR_LIDAR.value:
            return SectorLidar(racecar_name)
        elif sensor_type == Input.DISCRETIZED_SECTOR_LIDAR.value:
            return DiscretizedSectorLidar(racecar_name, config_dict)
        elif sensor_type == Input.OBSERVATION.value:
            return Observation(racecar_name)
        else:
            raise GenericRolloutException("Unknown sensor")

class Camera(SensorInterface):
    '''Single camera sensor'''
    def __init__(self, racecar_name,  timeout=120.0):
        """Camera sensor constructor

        Args:
            racecar_name (str): racecar name
            timeout (float): time limit for buffer get method until error out.
            The reason why using 120.0 as default value for sensor is
            because virtual event dynamic spawning for sensor can take up to 60.0 seconds to be alive.
        """
        self.image_buffer = utils.DoubleBuffer()
        node = get_node()
        node.create_subscription(sensor_image,
                                 '/{}/camera/zed/rgb/image_rect_color'.format(racecar_name),
                                 self._camera_cb_, 10)
        self.raw_data = None
        self.sensor_type = Input.CAMERA.value
        self.timeout = timeout

    def get_observation_space(self):
        try:
            return get_observation_space(Input.CAMERA.value)
        except GenericError as ex:
            ex.log_except_and_exit(SIMAPP_SIMULATION_WORKER_EXCEPTION)
        except Exception as ex:
            raise GenericRolloutException('{}'.format(ex))

    def get_state(self, block=True):
        try:
            # Make sure the first image is the starting image
            image_data = self.image_buffer.get(block=block, timeout=self.timeout)
            # Read the image and resize to get the state
            image = Image.frombytes('RGB', (image_data.width, image_data.height),
                                    image_data.data, 'raw', 'RGB', 0, 1)
            image = image.resize(TRAINING_IMAGE_SIZE, resample=2)
            self.raw_data = image_data
            return {Input.CAMERA.value: _apply_camera_noise(np.array(image))}
        except utils.DoubleBuffer.Empty:
            return {}
        except Exception as ex:
            raise GenericRolloutException("Unable to set state: {}".format(ex))

    def reset(self):
        self.image_buffer.clear()

    def _camera_cb_(self, data):
        ''' Callback for the single camera, this is triggered by ROS
            data - Image data from the gazebo plugin, it is a sensor message
        '''
        try:
            self.image_buffer.put(data)
        except Exception as ex:
            LOGGER.info("Unable to retrieve frame: %s", ex)

class Observation(SensorInterface):
    '''Single camera sensor that is compatible with simapp v1'''
    def __init__(self, racecar_name, timeout=120.0):
        """Observation sensor constructor

        Args:
            racecar_name (str): racecar name
            timeout (float): time limit for buffer get method until error out.
            The reason why using 120.0 as default value for sensor is
            because virtual event dynamic spawning for sensor can take up to 60.0 seconds to be alive.
        """
        self.image_buffer = utils.DoubleBuffer()
        node = get_node()
        node.create_subscription(sensor_image,
                                 '/{}/camera/zed/rgb/image_rect_color'.format(racecar_name),
                                 self._camera_cb_, 10)
        self.sensor_type = Input.OBSERVATION.value
        self.raw_data = None
        self.timeout = timeout
    
    def get_observation_space(self):
        try:
            return get_observation_space(Input.OBSERVATION.value)
        except GenericError as ex:
            ex.log_except_and_exit(SIMAPP_SIMULATION_WORKER_EXCEPTION)
        except Exception as ex:
            raise GenericRolloutException('{}'.format(ex))

    def get_state(self, block=True):
        try:
            # Make sure the first image is the starting image
            image_data = self.image_buffer.get(block=block, timeout=self.timeout)
            # Read the image and resize to get the state
            image = Image.frombytes('RGB', (image_data.width, image_data.height),
                                    image_data.data, 'raw', 'RGB', 0, 1)
            image = image.resize(TRAINING_IMAGE_SIZE, resample=2)
            self.raw_data = image_data
            return {Input.OBSERVATION.value: _apply_camera_noise(np.array(image))}
        except utils.DoubleBuffer.Empty:
            return {}
        except Exception as ex:
            raise GenericRolloutException("Unable to set state: {}".format(ex))

    def reset(self):
        self.image_buffer.clear()

    def _camera_cb_(self, data):
        ''' Callback for the single camera, this is triggered by ROS
            data - Image data from the gazebo plugin, it is a sensor message
        '''
        try:
            self.image_buffer.put(data)
        except Exception as ex:
            LOGGER.info("Unable to retrieve frame: %s", ex)

class LeftCamera(SensorInterface):
    '''This class is specific to left camera's only, it used the same topic as
       the camera class but has a different observation space. If this changes in
       the future this class should be updated.
    '''
    def __init__(self, racecar_name, timeout=120.0):
        """LeftCamera sensor constructor

        Args:
            racecar_name (str): racecar name
            timeout (float): time limit for buffer get method until error out.
            The reason why using 120.0 as default value for sensor is
            because virtual event dynamic spawning for sensor can take up to 60.0 seconds to be alive.
        """
        self.image_buffer = utils.DoubleBuffer()
        node = get_node()
        node.create_subscription(sensor_image,
                                 '/{}/camera/zed/rgb/image_rect_color'.format(racecar_name),
                                 self._camera_cb_, 10)
        self.sensor_type = Input.LEFT_CAMERA.value
        self.raw_data = None
        self.timeout = timeout

    def get_observation_space(self):
        try:
            return get_observation_space(Input.LEFT_CAMERA.value)
        except GenericError as ex:
            ex.log_except_and_exit(SIMAPP_SIMULATION_WORKER_EXCEPTION)
        except Exception as ex:
            raise GenericRolloutException('{}'.format(ex))

    def get_state(self, block=True):
        try:
            # Make sure the first image is the starting image
            image_data = self.image_buffer.get(block=block, timeout=self.timeout)
            # Read the image and resize to get the state
            image = Image.frombytes('RGB', (image_data.width, image_data.height),
                                    image_data.data, 'raw', 'RGB', 0, 1)
            image = image.resize(TRAINING_IMAGE_SIZE, resample=2)
            self.raw_data = image_data
            return {Input.LEFT_CAMERA.value: _apply_camera_noise(np.array(image))}
        except utils.DoubleBuffer.Empty:
            return {}
        except Exception as ex:
            raise GenericRolloutException("Unable to set state: {}".format(ex))

    def reset(self):
        self.image_buffer.clear()

    def _camera_cb_(self, data):
        ''' Callback for the single camera, this is triggered by ROS
            data - Image data from the gazebo plugin, it is a sensor message
        '''
        try:
            self.image_buffer.put(data)
        except Exception as ex:
            LOGGER.info("Unable to retrieve frame: %s", ex)

class DualCamera(SensorInterface):
    '''This class handles the data for dual cameras'''
    def __init__(self, racecar_name, timeout=120.0):
        """DualCamera sensor constructor

        Args:
            racecar_name (str): racecar name
            timeout (float): time limit for buffer get method until error out.
            The reason why using 120.0 as default value for sensor is
            because virtual event dynamic spawning for sensor can take up to 60.0 seconds to be alive.
        """
        # Queue used to maintain image consumption synchronicity
        self.image_buffer_left = utils.DoubleBuffer()
        self.image_buffer_right = utils.DoubleBuffer()
        # Set up the subscribers
        node = get_node()
        node.create_subscription(sensor_image,
                                 '/{}/camera/zed/rgb/image_rect_color'.format(racecar_name),
                                 self._left_camera_cb_, 10)
        node.create_subscription(sensor_image,
                                 '/{}/camera/zed_right/rgb/image_rect_color_right'.format(racecar_name),
                                 self._right_camera_cb_, 10)
        self.sensor_type = Input.STEREO.value
        self.timeout = timeout

    def get_observation_space(self):
        try:
            return get_observation_space(Input.STEREO.value)
        except GenericError as ex:
            ex.log_except_and_exit(SIMAPP_SIMULATION_WORKER_EXCEPTION)
        except Exception as ex:
            raise GenericRolloutException('{}'.format(ex))

    def get_state(self, block=True):
        try:
            image_data = self.image_buffer_left.get(block=block, timeout=self.timeout)
            left_img = Image.frombytes('RGB', (image_data.width, image_data.height),
                                       image_data.data, 'raw', 'RGB', 0, 1)
            left_img = left_img.resize(TRAINING_IMAGE_SIZE, resample=2).convert('L')

            image_data = self.image_buffer_right.get(block=block, timeout=self.timeout)
            right_img = Image.frombytes('RGB', (image_data.width, image_data.height),
                                        image_data.data, 'raw', 'RGB', 0, 1)
            right_img = right_img.resize(TRAINING_IMAGE_SIZE, resample=2).convert('L')

            return {Input.STEREO.value: _apply_camera_noise(np.array(np.stack((left_img, right_img), axis=2)))}
        except utils.DoubleBuffer.Empty:
            return {}
        except Exception as ex:
            raise GenericRolloutException("Unable to set state: {}".format(ex))

    def reset(self):
        self.image_buffer_left.clear()
        self.image_buffer_right.clear()

    def _left_camera_cb_(self, data):
        ''' Callback for the left camera, this is triggered by ROS
            data - Image data from the gazebo plugin, it is a sensor message
        '''
        try:
            self.image_buffer_left.put(data)
        except Exception as ex:
            LOGGER.info("Unable to retrieve frame: %s", ex)

    def _right_camera_cb_(self, data):
        ''' Callback for the right camera, this is triggered by ROS
            data - Image data from the gazebo plugin, it is a sensor message
        '''
        try:
            self.image_buffer_right.put(data)
        except Exception as ex:
            LOGGER.info("Unable to retrieve frame: %s", ex)


class Lidar(LidarInterface):
    '''This class handles the data collection for lidar'''
    def __init__(self, racecar_name, timeout=120.0):
        """Lidar sensor constructor

        Args:
            racecar_name (str): racecar name
            timeout (float): time limit for buffer get method until error out.
            The reason why using 120.0 as default value for sensor is
            because virtual event dynamic spawning for sensor can take up to 60.0 seconds to be alive.
        """
        self.data_buffer = utils.DoubleBuffer(clear_data_on_get=False)
        node = get_node()
        node.create_subscription(LaserScan, '/{}/scan'.format(racecar_name), self._scan_cb, 10)
        self.sensor_type = Input.LIDAR.value
        self.timeout = timeout

    def get_observation_space(self):
        try:
            return get_observation_space(self.sensor_type)
        except GenericError as ex:
            ex.log_except_and_exit(SIMAPP_SIMULATION_WORKER_EXCEPTION)
        except Exception as ex:
            raise GenericRolloutException('{}'.format(ex))

    def get_state(self, block=True):
        try:
            return {self.sensor_type: _apply_lidar_noise(
                self.data_buffer.get(block=block, timeout=self.timeout),
                LIDAR_360_DEGREE_MIN_RANGE, LIDAR_360_DEGREE_MAX_RANGE)}
        except utils.DoubleBuffer.Empty:
            # For Lidar, we always call non-blocking get_state instead of
            # block-waiting for new state, and the expectation is
            # we always have outdated state data to be read in the worst case as
            # we don't clear the data on get for DoubleBuffer (Refer to __init__).
            # Thus, the expectation is utils.DoubleBuffer.Empty will be never raised.
            # However, there can be an edge case for first call of get_state as DoubleBuffer may be
            # Empty, in such case, this may cause issue for the inference due to
            # incompatible input to NN. Thus, we should get sensor data with blocking if
            # DoubleBuffer.Empty is raised.
            return {self.sensor_type: _apply_lidar_noise(
                self.data_buffer.get(block=True, timeout=self.timeout),
                LIDAR_360_DEGREE_MIN_RANGE, LIDAR_360_DEGREE_MAX_RANGE)}
        except Exception as ex:
            raise GenericRolloutException("Unable to set state: {}".format(ex))

    def reset(self):
        self.data_buffer.clear()

    def _scan_cb(self, data):
        try:
            self.data_buffer.put(np.array(data.ranges))
        except Exception as ex:
            LOGGER.info("Unable to retrieve state: %s", ex)


class SectorLidar(LidarInterface):
    '''This class handles the data collection for sector lidar'''
    def __init__(self, racecar_name, timeout=120.0):
        """SectorLidar sensor constructor

        Args:
            racecar_name (str): racecar name
            timeout (float): time limit for buffer get method until error out.
            The reason why using 120.0 as default value for sensor is
            because virtual event dynamic spawning for sensor can take up to 60.0 seconds to be alive.
        """
        self.data_buffer = utils.DoubleBuffer(clear_data_on_get=False)
        node = get_node()
        node.create_subscription(LaserScan, '/{}/scan'.format(racecar_name), self._scan_cb, 10)
        self.sensor_type = Input.SECTOR_LIDAR.value
        self.timeout = timeout

    def get_observation_space(self):
        try:
            return get_observation_space(self.sensor_type)
        except GenericError as ex:
            ex.log_except_and_exit(SIMAPP_SIMULATION_WORKER_EXCEPTION)
        except Exception as ex:
            raise GenericRolloutException('{}'.format(ex))

    def get_state(self, block=True):
        try:
            return {self.sensor_type: _apply_lidar_noise(
                self.data_buffer.get(block=block, timeout=self.timeout),
                LIDAR_360_DEGREE_MIN_RANGE, LIDAR_360_DEGREE_MAX_RANGE)}
        except utils.DoubleBuffer.Empty:
            # For Lidar, we always call non-blocking get_state instead of
            # block-waiting for new state, and the expectation is
            # we always have outdated state data to be read in the worst case as
            # we don't clear the data on get for DoubleBuffer (Refer to __init__).
            # Thus, the expectation is utils.DoubleBuffer.Empty will be never raised.
            # However, there can be an edge case for first call of get_state as DoubleBuffer may be
            # Empty, in such case, this may cause issue for the inference due to
            # incompatible input to NN. Thus, we should get sensor data with blocking if
            # DoubleBuffer.Empty is raised.
            return {self.sensor_type: _apply_lidar_noise(
                self.data_buffer.get(block=True, timeout=self.timeout),
                LIDAR_360_DEGREE_MIN_RANGE, LIDAR_360_DEGREE_MAX_RANGE)}
        except Exception as ex:
            raise GenericRolloutException("Unable to set state: {}".format(ex))

    def reset(self):
        self.data_buffer.clear()

    def _scan_cb(self, data):
        try:
            self.data_buffer.put(np.array(data.ranges))
        except Exception as ex:
            LOGGER.info("Unable to retrieve state: %s", ex)

class DiscretizedSectorLidar(LidarInterface):
    '''This class handles the data collection for sector lidar'''
    def __init__(self, racecar_name, config, timeout=120.0):
        """DiscretizedSectorLidar sensor constructor

        Args:
            racecar_name (str): racecar name
            timeout (float): time limit for buffer get method until error out.
            The reason why using 120.0 as default value for sensor is
            because virtual event dynamic spawning for sensor can take up to 60.0 seconds to be alive.
        """
        self.data_buffer = utils.DoubleBuffer(clear_data_on_get=False)
        node = get_node()
        node.create_subscription(LaserScan, '/{}/scan'.format(racecar_name), self._scan_cb, 10)
        self.sensor_type = Input.DISCRETIZED_SECTOR_LIDAR.value
        self.model_metadata = config["model_metadata"]
        self.timeout = timeout

    def get_observation_space(self):
        try:
            return get_observation_space(self.sensor_type, self.model_metadata)
        except GenericError as ex:
            ex.log_except_and_exit(SIMAPP_SIMULATION_WORKER_EXCEPTION)
        except Exception as ex:
            raise GenericRolloutException('{}'.format(ex))

    def get_state(self, block=True):
        try:
            return {self.sensor_type: _apply_lidar_noise(
                self.data_buffer.get(block=block, timeout=self.timeout),
                LIDAR_360_DEGREE_MIN_RANGE, LIDAR_360_DEGREE_MAX_RANGE)}
        except utils.DoubleBuffer.Empty:
            # For Lidar, we always call non-blocking get_state instead of
            # block-waiting for new state, and the expectation is
            # we always have outdated state data to be read in the worst case as
            # we don't clear the data on get for DoubleBuffer (Refer to __init__).
            # Thus, the expectation is utils.DoubleBuffer.Empty will be never raised.
            # However, there can be an edge case for first call of get_state as DoubleBuffer may be
            # Empty, in such case, this may cause issue for the inference due to
            # incompatible input to NN. Thus, we should get sensor data with blocking if
            # DoubleBuffer.Empty is raised.
            return {self.sensor_type: _apply_lidar_noise(
                self.data_buffer.get(block=True, timeout=self.timeout),
                LIDAR_360_DEGREE_MIN_RANGE, LIDAR_360_DEGREE_MAX_RANGE)}
        except Exception as ex:
            raise GenericRolloutException("Unable to set state: {}".format(ex))

    def reset(self):
        self.data_buffer.clear()

    def _scan_cb(self, data):
        try:
            self.data_buffer.put(np.array(data.ranges))
        except Exception as ex:
            LOGGER.info("Unable to retrieve state: %s", ex)