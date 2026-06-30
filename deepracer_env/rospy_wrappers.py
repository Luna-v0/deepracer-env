#################################################################################
#   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#################################################################################
"""Backwards-compatible service-proxy shim, now backed by rclpy.

The ROS 1 ``ServiceProxyWrapper`` retried a service call then exited the process.
The tracker layer no longer uses it (it goes through the SimControl seam), but a
few legacy call sites may still import this name; this keeps them working by
delegating to :class:`~deepracer_env.sim_control.rclpy_client.ServiceClientWrapper`
on the shared node. Imports are lazy so this module never pulls in ``rclpy`` at
import time (host tools import ``deepracer_env`` freely).
"""
from __future__ import annotations


class ServiceProxyWrapper(object):
    """rclpy-backed stand-in for the ROS 1 ``ServiceProxyWrapper``."""

    def __init__(self, service_name, object_type, persistent=False, max_retry_attempts=5):
        """Create a retrying client for *service_name* of type *object_type*."""
        from deepracer_env.runtime import get_node
        from deepracer_env.sim_control.rclpy_client import ServiceClientWrapper
        self._client = ServiceClientWrapper(
            get_node(), object_type, service_name, max_retry_attempts=max_retry_attempts)

    def __call__(self, *argv):
        """Call the service. Accepts a single request object."""
        request = argv[0] if len(argv) == 1 else argv
        return self._client(request)
