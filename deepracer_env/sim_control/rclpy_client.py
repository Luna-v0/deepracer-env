#################################################################################
#   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#################################################################################
"""rclpy lifecycle plumbing for the simulator seam.

This is the ROS 2 replacement for the legacy ``rospy_wrappers`` /
``rospy.init_node`` machinery. It provides:

* :func:`ensure_rclpy_initialized` — idempotent ``rclpy.init`` so constructing an
  environment twice in one process (common in HPO sweeps) never double-inits.
* :class:`SimNode` — the *single* node the environment owns, with a background
  executor thread so subscriptions (camera, LiDAR, pose) fill buffers while the
  Python training loop runs synchronously.
* :class:`ServiceClientWrapper` — a thin, retry-on-failure wrapper around an
  ``rclpy`` service client that mirrors the call signature and forgiving
  semantics of the legacy
  :class:`~deepracer_env.rospy_wrappers.ServiceProxyWrapper`, so the few code
  paths that still call ROS services port with a one-line type swap.

This module imports ``rclpy`` at import time and therefore only loads inside the
ROS 2 container — the package ``__init__`` keeps it out of the host-importable
surface (types/arena/interface) on purpose.
"""
from __future__ import annotations

import logging
import threading
import time
from typing import Optional

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from deepracer_env.sim_control.interface import SimControlError, SimControlTimeout

LOG = logging.getLogger(__name__)

_INIT_LOCK = threading.Lock()


def ensure_rclpy_initialized(args=None) -> None:
    """Initialise the global rclpy context once, safely from any thread.

    Args:
        args: Optional argv forwarded to ``rclpy.init`` on the first call.
    """
    with _INIT_LOCK:
        if not rclpy.ok():
            rclpy.init(args=args)


class SimNode(Node):
    """The environment's single ROS 2 node, spun on a background thread.

    Owning one node (rather than one per sensor/publisher) keeps the DDS graph
    small and makes multi-car namespacing a matter of per-topic names rather
    than per-node namespaces. The background :class:`SingleThreadedExecutor`
    lets subscription callbacks (which fill the sensor/pose double-buffers) run
    while the gymnasium ``step``/``reset`` calls execute synchronously on the
    main thread — exactly the threading model the legacy ``rospy`` callbacks
    relied on.
    """

    def __init__(self, name: str = "deepracer_env", namespace: str = "") -> None:
        """Create (and start) the node.

        Args:
            name: Node name.
            namespace: Optional ROS namespace for the node itself (per-car topic
                namespacing is done at publisher/subscriber creation instead).
        """
        ensure_rclpy_initialized()
        super().__init__(name, namespace=namespace or None)
        # Single-threaded: set_pose runs on its OWN dedicated node/executor, so
        # this node only fans out subscription callbacks + publishes — no need for
        # concurrency, and one thread tears down predictably (a MultiThreaded pool
        # left worker threads alive under the dynamic_pose flood, so the context
        # shutdown segfaulted at teardown of a multi-car camera run).
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self)
        self._spin_thread: Optional[threading.Thread] = None
        self._spinning = False
        self.start_spinning()

    def start_spinning(self) -> None:
        """Begin spinning the executor on a daemon thread (idempotent)."""
        if self._spinning:
            return
        self._spinning = True
        self._spin_thread = threading.Thread(
            target=self._executor.spin, name="deepracer-env-executor", daemon=True
        )
        self._spin_thread.start()

    def stop_spinning(self) -> None:
        """Stop the executor thread (idempotent).

        shutdown() wakes the spin via its guard condition so spin() returns and
        the daemon thread exits; join with a generous timeout because draining a
        flooded executor (dynamic_pose + N camera subs at unlimited RTF) can take
        a moment. The thread MUST be dead before rclpy tears the context down or
        the finalise segfaults (rc=139)."""
        if not self._spinning:
            return
        self._spinning = False
        try:
            self._executor.shutdown()
        except Exception:  # noqa: BLE001
            pass
        if self._spin_thread is not None:
            self._spin_thread.join(timeout=8.0)
            if self._spin_thread.is_alive():
                LOG.warning("SimNode executor thread did not stop within 8s")
            self._spin_thread = None

    def destroy(self) -> None:
        """Tear down the node and its executor. Safe to call more than once."""
        self.stop_spinning()
        try:
            self.destroy_node()
        except Exception:  # noqa: BLE001 — teardown must never raise
            pass


class ServiceClientWrapper(object):
    """Retry-wrapped ``rclpy`` service client — the ``ServiceProxyWrapper`` heir.

    The legacy wrapper retried a few times then slept and exited the process (a
    RoboMaker quirk). Here we keep the *retry* but raise a typed
    :class:`~deepracer_env.sim_control.interface.SimControlError` instead of
    killing the process, so a looping caller (training rotation) can recover.

    Because :class:`SimNode` spins its executor on a background thread, calls go
    through ``call_async`` and block on the resulting future via an event — never
    via ``spin_until_future_complete`` (which would fight the background spin).
    """

    def __init__(
        self,
        node: Node,
        srv_type,
        service_name: str,
        *,
        max_retry_attempts: int = 5,
        wait_timeout_sec: float = 30.0,
    ) -> None:
        """Create a client for *service_name* and wait for it to appear.

        Args:
            node: The owning :class:`SimNode` (provides the executor).
            srv_type: The ROS 2 service type (e.g.
                ``simulation_interfaces.srv.SetEntityState``).
            service_name: Fully-qualified service name.
            max_retry_attempts: Retries on call failure before raising.
            wait_timeout_sec: How long to wait for the server on first use.
        """
        self._node = node
        self._service_name = service_name
        self._max_retry_attempts = max_retry_attempts
        self._client = node.create_client(srv_type, service_name)
        if not self._client.wait_for_service(timeout_sec=wait_timeout_sec):
            raise SimControlError(
                "service {!r} did not appear within {}s".format(
                    service_name, wait_timeout_sec))

    def __call__(self, request, *, timeout_sec: float = 10.0):
        """Call the service, retrying transient failures.

        Args:
            request: The populated request message.
            timeout_sec: Per-attempt wait for the response.

        Returns:
            The service response message.

        Raises:
            SimControlTimeout: No response within ``timeout_sec`` on the final
                attempt.
            SimControlError: The call raised on the final attempt.
        """
        last_err: Optional[BaseException] = None
        for attempt in range(1, self._max_retry_attempts + 1):
            try:
                future = self._client.call_async(request)
                done = threading.Event()
                future.add_done_callback(lambda _f: done.set())
                if not done.wait(timeout_sec):
                    raise SimControlTimeout(
                        "{} timed out after {}s".format(self._service_name, timeout_sec))
                return future.result()
            except Exception as ex:  # noqa: BLE001
                last_err = ex
                LOG.info("service %s call failed (%d/%d): %s",
                         self._service_name, attempt, self._max_retry_attempts, ex)
                time.sleep(0.2)
        raise SimControlError(
            "service {!r} failed after {} attempts: {}".format(
                self._service_name, self._max_retry_attempts, last_err))
