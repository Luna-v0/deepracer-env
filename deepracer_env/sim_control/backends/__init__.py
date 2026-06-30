#################################################################################
#   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#################################################################################
"""Concrete :class:`~deepracer_env.sim_control.interface.SimControl` backends.

Each module here imports ``rclpy`` and/or shells out to the ``gz`` CLI, so they
are imported lazily by :mod:`deepracer_env.sim_control.factory` rather than at
package import time (keeping the seam's value types host-importable).
"""
