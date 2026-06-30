#################################################################################
#   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#################################################################################
"""Backend selection for the simulator seam (the Strategy chooser).

:func:`make_sim_control` returns the right :class:`SimControl` adapter for the
running stack. Today that is the ``ros_gz`` backend (Gazebo Jetty serves its
control plane over gz-transport; nothing serves ``simulation_interfaces`` yet),
but the choice is data-driven so the move to the standard later is a config
change, not a code change.

Selection order:
    1. The ``DR_SIM_BACKEND`` environment variable, if set
       (``"ros_gz"`` | ``"simulation_interfaces"`` | ``"null"``).
    2. The ``prefer`` argument.
    3. Auto: ``simulation_interfaces`` if both its messages *and* a live server
       are present; otherwise ``ros_gz``.
"""
from __future__ import annotations

import logging
import os
from typing import Optional

from deepracer_env.sim_control.interface import SimControl, NullSimControl

LOG = logging.getLogger(__name__)

BACKEND_ROS_GZ = "ros_gz"
BACKEND_SIM_IFACE = "simulation_interfaces"
BACKEND_NULL = "null"


def make_sim_control(
    world_name: str,
    *,
    prefer: Optional[str] = None,
    node=None,
    **backend_kwargs,
) -> SimControl:
    """Construct the simulator control backend for *world_name*.

    Args:
        world_name: The gz world name (needed by the ``ros_gz`` backend to build
            its ``/world/<world>/…`` service prefix).
        prefer: Optional explicit backend id; overridden by ``DR_SIM_BACKEND``.
        node: A spinning ``SimNode`` (required by the ``simulation_interfaces``
            backend; created lazily for ``ros_gz`` only if needed).
        **backend_kwargs: Forwarded to the chosen backend constructor.

    Returns:
        A ready :class:`SimControl`.

    Raises:
        ValueError: If an unknown backend id is requested.
    """
    choice = os.environ.get("DR_SIM_BACKEND") or prefer or _auto_select(node)
    LOG.info("sim_control backend: %s (world=%s)", choice, world_name)

    if choice == BACKEND_NULL:
        return NullSimControl()
    if choice == BACKEND_ROS_GZ:
        from deepracer_env.sim_control.backends.ros_gz_backend import RosGzBackend
        return RosGzBackend(world_name, **backend_kwargs)
    if choice == BACKEND_SIM_IFACE:
        if node is None:
            raise ValueError(
                "the simulation_interfaces backend requires a SimNode (node=...)")
        from deepracer_env.sim_control.backends.simulation_interfaces_backend import (
            SimulationInterfacesBackend,
        )
        return SimulationInterfacesBackend(node, **backend_kwargs)
    raise ValueError("unknown sim_control backend {!r}".format(choice))


def _auto_select(node) -> str:
    """Pick a backend automatically (see module docstring for the order)."""
    try:
        from deepracer_env.sim_control.backends.simulation_interfaces_backend import (
            is_available,
        )
    except Exception:  # noqa: BLE001
        return BACKEND_ROS_GZ
    # Messages present AND a server is up (a node is available to probe with).
    if node is not None and is_available() and _has_sim_iface_server(node):
        return BACKEND_SIM_IFACE
    return BACKEND_ROS_GZ


def _has_sim_iface_server(node) -> bool:
    """True iff a ``simulation_interfaces`` service server is currently visible."""
    try:
        services = dict(node.get_service_names_and_types())
    except Exception:  # noqa: BLE001
        return False
    return any(
        "simulation_interfaces/srv/SetEntityState" in types
        for types in services.values()
    )
