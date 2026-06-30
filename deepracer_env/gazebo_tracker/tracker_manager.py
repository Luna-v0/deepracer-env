#################################################################################
#   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#################################################################################
"""Clock-driven tracker dispatch, ported to rclpy.

Unchanged design: a singleton that, on every ``/clock`` tick, calls
``update_tracker(delta_time, sim_time)`` on registered trackers in priority order
(HIGH reads before LOW writes). Only the clock source changes — a ``rclpy``
subscription on the shared node (``/clock`` bridged from gz via ros_gz) replaces
the ``rospy.Subscriber``.
"""
import logging
import threading

from rosgraph_msgs.msg import Clock

from deepracer_env.log_handler.deepracer_exceptions import GenericRolloutException
import deepracer_env.gazebo_tracker.constants as consts
from deepracer_env.log_handler.logger import Logger

logger = Logger(__name__, logging.INFO).get_logger()


def _clock_seconds(sim_time) -> float:
    """Seconds from a rosgraph_msgs/Clock, tolerant of ROS1/ROS2 field names."""
    clk = sim_time.clock
    secs = getattr(clk, "sec", None)
    if secs is None:  # ROS 1 names (defensive)
        return clk.secs + 1.0e-9 * clk.nsecs
    return secs + 1.0e-9 * clk.nanosec


class TrackerManager(object):
    """Dispatches ``update_tracker`` to registered trackers on each clock tick."""

    _instance_ = None

    @staticmethod
    def get_instance():
        """Return the singleton, constructing it on first use."""
        if TrackerManager._instance_ is None:
            TrackerManager()
        return TrackerManager._instance_

    def __init__(self):
        if TrackerManager._instance_ is not None:
            raise GenericRolloutException("Attempting to construct multiple TrackerManager")
        self.priority_order = [consts.TrackerPriority.HIGH,
                               consts.TrackerPriority.NORMAL,
                               consts.TrackerPriority.LOW]
        self.tracker_map = {priority: set() for priority in self.priority_order}
        self.lock = threading.RLock()
        self.last_time = 0.0
        # Subscribe on the shared node; /clock is bridged from gz (use_sim_time).
        from deepracer_env.runtime import get_node
        self._node = get_node()
        self._sub = self._node.create_subscription(Clock, "/clock", self._update_sim_time, 10)
        TrackerManager._instance_ = self

    def add(self, tracker, priority=consts.TrackerPriority.NORMAL):
        """Register *tracker* under *priority*."""
        with self.lock:
            self.tracker_map[priority].add(tracker)

    def remove(self, tracker):
        """Unregister *tracker* from all priorities."""
        with self.lock:
            for priority in self.priority_order:
                self.tracker_map[priority].discard(tracker)

    def _update_sim_time(self, sim_time):
        """Clock callback: dispatch update_tracker in priority order."""
        curr_time = _clock_seconds(sim_time)
        if self.last_time is None:
            self.last_time = curr_time
        delta_time = curr_time - self.last_time
        if not self.lock.acquire(False):
            logger.info("TrackerManager: missed an _update_sim_time call")
            return
        try:
            self.last_time = curr_time
            for priority in self.priority_order:
                for tracker in self.tracker_map[priority].copy():
                    # Isolate each tracker: one failure must not starve the
                    # others (e.g. the pose-refresh tracker) on this tick.
                    try:
                        tracker.update_tracker(delta_time, sim_time)
                    except Exception as ex:  # noqa: BLE001
                        logger.info("TrackerManager: %s.update_tracker failed: %s",
                                    type(tracker).__name__, ex)
        finally:
            self.lock.release()
