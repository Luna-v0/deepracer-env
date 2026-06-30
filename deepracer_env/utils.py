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

import threading
import json
import logging
import os
import subprocess
import io
import re
import signal
import socket
import time
import cProfile
import pstats
import random

from itertools import count
from deepracer_env.log_handler.constants import (SIMAPP_ENVIRONMENT_EXCEPTION,
                                          SIMAPP_EVENT_ERROR_CODE_500,
                                          SIMAPP_SIMULATION_WORKER_EXCEPTION, SIMAPP_DONE_EXIT,
                                          STOP_ROS_NODE_MONITOR_SYNC_FILE)
from deepracer_env.log_handler.logger import Logger
from deepracer_env.log_handler.exception_handler import log_and_exit, simapp_exit_gracefully
from deepracer_env.log_handler.deepracer_exceptions import GenericException
from deepracer_env.constants import (DEEPRACER_JOB_TYPE_ENV,
                              DeepRacerJobType)

logger = Logger(__name__, logging.INFO).get_logger()


def is_int_repr(val):
    '''check whether input is int or not

    Args:
        val (str): str input

    Returns:
        bool: True if string can be convert to int, False otherwise
    '''
    try:
        int(val)
        return True
    except ValueError:
        return False


def force_list(val):
    if type(val) is not list:
        val = [val]
    return val

def str2bool(flag):
    """ bool: convert flag to boolean if it is string and return it else return its initial bool value """
    if not isinstance(flag, bool):
        if flag.lower() == 'false':
            flag = False
        elif flag.lower() == 'true':
            flag = True
    return flag

# stops the execution of ROS Node monitor
def stop_ros_node_monitor():
    try:
        with open(STOP_ROS_NODE_MONITOR_SYNC_FILE, 'w') as f:
            srm = dict()
            srm["stop_ros_node_monitor"] = "True"
            f.write(json.dumps(srm))
        logger.info("[markov_utils] Stored variable for stopping ros node monitor")
    except Exception as ex:
        logger.info("[markov_utils] Not able to store variable for stopping ros node monitor: {}".format(str(ex)))
    
    # Wait for monitoring to close the job
    time.sleep(5)

def str_to_done_condition(done_condition):
    if done_condition == any or done_condition == all:
        return done_condition
    elif done_condition.lower().strip() == 'all':
        return all
    return any

def pos_2d_str_to_list(list_pos_str):
    if list_pos_str and isinstance(list_pos_str[0], str):
        return [tuple(map(float, pos_str.split(","))) for pos_str in list_pos_str]
    return list_pos_str

def get_ip_from_host(timeout=100):
    counter = 0
    ip_address = None

    host_name = socket.gethostname()
    logger.debug("Hostname: %s" % host_name)
    while counter < timeout and not ip_address:
        try:
            ip_address = socket.gethostbyname(host_name)
            break
        except Exception:
            counter += 1
            time.sleep(1)

    if counter == timeout and not ip_address:
        error_string = "Environment Error: Could not retrieve IP address \
        for %s in past %s seconds." % (host_name, timeout)
        log_and_exit(error_string,
                     SIMAPP_ENVIRONMENT_EXCEPTION,
                     SIMAPP_EVENT_ERROR_CODE_500)
    return ip_address


def is_user_error(error):
    ''' Helper method that determines whether a value error is caused by an invalid checkpoint
        or model_metadata by looking for keywords in the exception message
        error - Python exception object, which produces a message by implementing __str__
    '''
    # These are the key words, which if present indicate that the tensorflow saver was unable
    # to restore a checkpoint because it does not match the graph or unable to load
    # model_metadata due to invalid json format
    keys = ['tensor', 'shape', 'checksum', 'checkpoint', 'model_metadata']
    return any(key in str(error).lower() for key in keys)

def get_video_display_name():
    """ Based on the job type display the appropriate name on the mp4.
    For the RACING job type use the racer alias name and for others use the model name.

    Returns:
        list: List of the display name. In head2head there would be two values else one value.
    """
    # Note: under ROS 2 there is no global param server; these job parameters are
    # read from environment variables instead of rospy.get_param.
    video_job_type = os.environ.get("VIDEO_JOB_TYPE", "")
    # TODO: This code should be removed when the cloud service starts providing VIDEO_JOB_TYPE YAML parameter
    if not video_job_type:
        return force_list(os.environ.get("DISPLAY_NAME", ""))
    if video_job_type == "RACING":
        return force_list(os.environ.get("RACER_NAME", ""))
    return force_list(os.environ.get("MODEL_NAME", ""))

def get_racecar_names(racecar_num):
    """Return the racer names based on the number of racecars given.

    Arguments:
        racecar_num (int): The number of race cars
    Return:
        [] - the list of race car names
    """
    racer_names = []
    if racecar_num == 1:
        racer_names.append('racecar')
    else:
        for idx in range(racecar_num):
            racer_names.append('racecar_' + str(idx))
    return racer_names

def get_racecar_idx(racecar_name):
    try:
        racecar_name_list = racecar_name.split("_")
        if len(racecar_name_list) == 1:
            return None
        racecar_num = racecar_name_list[1]
        return int(racecar_num)
    except Exception as ex:
        log_and_exit("racecar name should be in format racecar_x. However, get {}".\
                        format(racecar_name),
                     SIMAPP_SIMULATION_WORKER_EXCEPTION,
                     SIMAPP_EVENT_ERROR_CODE_500)


def check_is_sageonly():
    """Checks if current environment is SageOnly environment

    Returns:
        bool: True if current environment is sageonly, False if not.
    """
    try:
        return DeepRacerJobType(os.environ.get(DEEPRACER_JOB_TYPE_ENV)) == DeepRacerJobType.SAGEONLY
    except Exception as ex:
        logger.info("Exception when checking for DEEPRACER_JOB_TYPE_ENV %s", ex)
        return False


class DoorMan:
    def __init__(self):
        self.terminate_now = False
        logger.info("DoorMan: installing SIGINT, SIGTERM")
        signal.signal(signal.SIGINT, self.exit_gracefully)
        signal.signal(signal.SIGTERM, self.exit_gracefully)

    def exit_gracefully(self, signum, frame):
        self.terminate_now = True
        logger.info("DoorMan: received signal {}".format(signum))
        simapp_exit_gracefully(simapp_exit=SIMAPP_DONE_EXIT)

class DoubleBuffer(object):
    def __init__(self, clear_data_on_get=True):
        self.read_buffer = None
        self.write_buffer = None
        self.clear_data_on_get = clear_data_on_get
        self.cv = threading.Condition()

    def clear(self):
        with self.cv:
            self.read_buffer = None
            self.write_buffer = None

    def put(self, data):
        with self.cv:
            self.write_buffer = data
            self.write_buffer, self.read_buffer = self.read_buffer, self.write_buffer
            self.cv.notify()

    def get(self, block=True, timeout=None):
        """
        Get data from double buffer

        Args:
            block (bool): True for waiting when read buffer is None, False for raising DoubleBuffer.Empty
            timeout (None/float): Wait timeout for condition wait. If None is past in, it will wait
            forever until cv.notify is called

        Returns:
            Any: anything from double buffer
        """
        with self.cv:
            if not block:
                if self.read_buffer is None:
                    raise DoubleBuffer.Empty
            else:
                if self.read_buffer is None:
                    # The return value is True unless a given timeout expired, in which case it is False
                    # Changed in version 3.2: Previously, the method always returned None.
                    self.cv.wait(timeout=timeout)
                    if self.read_buffer is None:
                        log_and_exit("[DoubleBuffer]: wait timeout",
                                     SIMAPP_SIMULATION_WORKER_EXCEPTION,
                                     SIMAPP_EVENT_ERROR_CODE_500)
            data = self.read_buffer
            if self.clear_data_on_get:
                self.read_buffer = None
            return data

    def get_nowait(self):
        return self.get(block=False)

    class Empty(Exception):
        pass

class Profiler(object):
    """ Class to profile the specific code.
    """
    _file_count = count(0)
    _profiler = None
    _profiler_owner = None

    def __init__(self, s3_bucket, s3_prefix, output_local_path, enable_profiling=False):
        self._enable_profiling = enable_profiling
        self.s3_bucket = s3_bucket
        self.s3_prefix = s3_prefix
        self.output_local_path = output_local_path
        self.file_count = next(self._file_count)

    def __enter__(self):
        self.start()

    def __exit__(self, type_val, value, traceback):
        self.stop()

    @property
    def enable_profiling(self):
        """ Property to enable profiling

        Returns:
            (bool): True if profiler is enabled
        """
        return self._enable_profiling

    @enable_profiling.setter
    def enable_profiling(self, val):
        self._enable_profiling = val

    def start(self):
        """ Start the profiler """
        if self.enable_profiling:
            if not self._profiler:
                self._profiler = cProfile.Profile()
                self._profiler.enable()
                self._profiler_owner = self
            else:
                raise GenericException('Profiler is in use!')

    def stop(self):
        """ Stop the profiler and persist the data locally """
        if self._profiler_owner == self:
            if self._profiler:
                self._profiler.disable()
                self._profiler.dump_stats(self.output_local_path)
                s3_file_name = "{}-{}.txt".format(os.path.splitext(os.path.basename(self.output_local_path))[0],
                                                  self.file_count)
                with open(s3_file_name, 'w') as filepointer:
                    pstat_obj = pstats.Stats(self.output_local_path, stream=filepointer)
                    pstat_obj.sort_stats('cumulative')
                    pstat_obj.print_stats()
                self._upload_profile_stats_to_s3(s3_file_name)
            self._profiler = None
            self._profiler_owner = None

    def _upload_profile_stats_to_s3(self, s3_file_name):
        """ Persist the profiler information locally.

        S3 upload has been removed; the profiler stats file produced by ``stop`` is
        simply left in place on the local filesystem instead of being uploaded to a
        bucket. Kept as a method for API compatibility.

        Arguments:
            s3_file_name (str): File name of the local profiler stats file.
        """
        logger.info("Profiler stats written locally to %s (S3 upload disabled).", s3_file_name)
