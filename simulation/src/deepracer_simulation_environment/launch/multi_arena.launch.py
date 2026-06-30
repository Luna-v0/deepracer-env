#################################################################################
#   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#################################################################################
"""ROS 2 launch for *N* decoupled DeepRacer arenas in ONE Gazebo world.

This is the live N-car bring-up: it generalises the single-car
``deepracer_env.launch.py`` to the tiled multi-arena design described in
:class:`deepracer_env.sim_control.arena.ArenaLayout`. One ``gzserver`` process
hosts every arena; each arena gets its own track instance, its own namespaced
car, and its own ``controller_manager`` so the arenas never perturb one another.

How the count is resolved
-------------------------
The number of cars, the world, and the grid spacing are *launch-time* integers
(read from the ``n_cars`` / ``world`` / ``spacing`` launch args), so the same
file serves 1 or 64 cars. Because those values must be read as concrete Python
ints/floats to build the :class:`ArenaLayout`, the body runs inside an
:class:`~launch.actions.OpaqueFunction` (``_launch_setup``) which performs the
launch configurations against the context before emitting actions.

What it emits (mirrors the single-car launch, fanned out per arena)
-------------------------------------------------------------------
1. ``GZ_SIM_RESOURCE_PATH`` (package share + ``/models`` so ``model://models/...``
   and ``model://meshes/...`` resolve) and ``GZ_SIM_SYSTEM_PLUGIN_PATH``
   (``/opt/ros/<distro>/lib`` so ``libgz_ros2_control-system`` is found), then a
   single ``gz sim`` server on ``worlds/<world>.sdf`` — that file already loads
   arena 0's track at the origin. An optional GUI when ``gui:=true``.
2. For each arena ``i >= 1``: a ``ros_gz_sim create`` that spawns a uniquely
   named track instance (``racetrack_i``) at the arena's grid offset. The offset
   is carried inside the ``<include>``'s ``<pose>`` (the include's own pose
   overrides the spawn pose, so it must live there) of a temp ``<include>``
   wrapper SDF, exactly as
   :meth:`deepracer_env.environments.world_swap.WorldSwapper.spawn_track_instance`
   builds it.
3. For each arena ``i``: a namespaced ``robot_state_publisher`` holding the car
   URDF expanded with ``car_name:=car_i namespace:=car_i`` (the ``namespace`` arg
   gives this car a private ``/car_i/controller_manager``); a ``ros_gz_sim
   create`` spawning ``-name car_i`` at the arena origin; and the three
   controller spawners (``joint_state_broadcaster``, ``wheels_velocity_controller``,
   ``steering_position_controller``) targeting ``/car_i/controller_manager``,
   chained off that car's spawn via ``RegisterEventHandler(OnProcessExit)`` so
   they run only once the ``controller_manager`` (created by the in-URDF
   ``gz_ros2_control`` system plugin) exists.
4. ONE ``ros_gz`` bridge for ``/clock`` (the sim clock is shared by every arena).

The gymnasium contract is untouched — this file only shapes the simulator
substrate the Python env drives through the ``SimControl`` seam.
"""
from __future__ import annotations

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

from deepracer_env.sim_control.arena import DEFAULT_ARENA_SPACING_M, ArenaLayout

PKG = "deepracer_simulation_environment"


def _write_track_instance_sdf(world: str, entity_name: str, ox: float, oy: float) -> str:
    """Write an ``<include>``-wrapper SDF for one offset track instance.

    Mirrors
    :meth:`deepracer_env.environments.world_swap.WorldSwapper.spawn_track_instance`:
    the grid offset goes inside the ``<include>``'s ``<pose>`` because the
    include carries the track's own origin pose, which overrides the spawn pose
    (passing the offset to ``create -x/-y`` alone would leave the mesh at 0,0).
    Gazebo resolves the ``model://models/<world>`` mesh/material URIs through
    ``GZ_SIM_RESOURCE_PATH`` exactly as at world-load time.

    Args:
        world: Track name (the ``models/<world>`` key) to instance.
        entity_name: Unique Gazebo entity name for this instance (``racetrack_i``).
        ox: World-frame X offset of the arena origin, in metres.
        oy: World-frame Y offset of the arena origin, in metres.

    Returns:
        Absolute path to the written temp SDF file.
    """
    sdf = (
        '<?xml version="1.0"?>\n'
        '<sdf version="1.6">\n'
        '  <include>\n'
        '    <uri>model://models/{world}</uri>\n'
        '    <name>{name}</name>\n'
        '    <pose>{ox} {oy} 0 0 0 0</pose>\n'
        '  </include>\n'
        '</sdf>\n'
    ).format(world=world, name=entity_name, ox=ox, oy=oy)
    fd, path = tempfile.mkstemp(prefix="{}_".format(entity_name), suffix=".sdf")
    with os.fdopen(fd, "w") as handle:
        handle.write(sdf)
    return path


def _launch_setup(context, *args, **kwargs):
    """Build the per-arena action list once the launch args are concrete.

    Runs inside an :class:`~launch.actions.OpaqueFunction` so the integer
    ``n_cars`` / float ``spacing`` / string ``world`` can be read from the
    context and fed to :class:`ArenaLayout`.

    Args:
        context: The launch context (carries the resolved launch configurations).

    Returns:
        The list of launch actions that bring up every arena.
    """
    share = get_package_share_directory(PKG)

    n_cars = int(LaunchConfiguration("n_cars").perform(context))
    world = LaunchConfiguration("world").perform(context)
    spacing = float(LaunchConfiguration("spacing").perform(context))
    friction_mu = LaunchConfiguration("friction_mu").perform(context)
    gui = LaunchConfiguration("gui")  # left as a substitution for IfCondition

    xacro_file = os.path.join(share, "urdf", "deepracer", "deepracer_gz.urdf.xacro")
    ros2_control_cfg = os.path.join(share, "config", "ros2_control.yaml")
    world_sdf = os.path.join(share, "worlds", "{}.sdf".format(world))

    # Single source of truth for "how many cars, which track, which offset".
    # Use the 'racecar_{i}' / 'racetrack_{i}' naming that MultiAgentDeepRacerEnv
    # and RolloutCtrl publish to, so the Python env commands these controllers.
    layout = ArenaLayout(n_cars, [world], spacing=spacing,
                         car_name_fmt="racecar_{index}",
                         track_entity_fmt="racetrack_{index}")

    # gz lib dir for the gz_ros2_control system plugin discovery.
    ros_lib = "/opt/ros/" + os.environ.get("ROS_DISTRO", "lyrical") + "/lib"

    actions = [
        # 1. Env vars FIRST so the gz server + spawns inherit them.
        SetEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value=share + os.pathsep + os.path.join(share, "models"),
        ),
        SetEnvironmentVariable(name="GZ_SIM_SYSTEM_PLUGIN_PATH", value=ros_lib),
        # 1. ONE gz server on arena-0's world SDF (headless unless gui:=true).
        ExecuteProcess(
            cmd=["gz", "sim", "-s", "-r", "-v", "1", world_sdf], output="screen",
        ),
        ExecuteProcess(
            cmd=["gz", "sim", "-g"], output="screen", condition=IfCondition(gui),
        ),
        # 4. ONE clock bridge (gz->ros); the sim clock is shared by all arenas.
        Node(
            package="ros_gz_bridge", executable="parameter_bridge",
            name="clock_bridge", output="screen",
            arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        ),
    ]

    def spawner(car_name: str, name: str, param_file: bool = False) -> Node:
        """A controller spawner targeting this car's private controller_manager.

        The ``forward_command_controller`` groups need their ``joints`` /
        ``interface_name`` params handed in via ``--param-file`` (the
        ``gz_ros2_control`` plugin gives its ``<parameters>`` file to the
        controller_manager but not down to the controller nodes).
        """
        spawner_args = [
            name,
            "--controller-manager", "/{}/controller_manager".format(car_name),
            # Generous: with N gz_ros2_control controller_managers in one server,
            # each waits on its namespaced /robot_description and they init
            # serially, so a single CM can take tens of seconds to come up.
            "--controller-manager-timeout", "120",
        ]
        if param_file:
            spawner_args += ["--param-file", ros2_control_cfg]
        # Namespaced so the N cars' (otherwise identically named) spawner nodes
        # don't collide; the controller_manager is still targeted absolutely.
        return Node(
            package="controller_manager", executable="spawner",
            namespace=car_name, arguments=spawner_args, output="screen",
        )

    for arena in layout.arenas:
        car_name = arena.car_name
        ox, oy = arena.origin.x, arena.origin.y

        # 2. Arena 0's track ships inside the world SDF at the origin; every
        #    other arena spawns its own offset track instance.
        if arena.index >= 1:
            sdf_path = _write_track_instance_sdf(
                world, arena.track_entity_name, ox, oy)
            actions.append(Node(
                package="ros_gz_sim", executable="create",
                name="spawn_{}".format(arena.track_entity_name), output="screen",
                arguments=[
                    "-world", world, "-file", sdf_path,
                    "-name", arena.track_entity_name,
                    "-x", str(ox), "-y", str(oy),
                ],
            ))

        # 3a. robot_state_publisher (namespaced) holding this car's URDF; the
        #     namespace arg routes its controller_manager under /car_i.
        robot_description = Command([
            "xacro ", xacro_file,
            " car_name:=", car_name,
            " namespace:=", car_name,
            " friction_mu:=", friction_mu,
            " ros2_control_config:=", ros2_control_cfg,
        ])
        actions.append(Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            namespace=car_name, output="screen",
            parameters=[{"robot_description": robot_description, "use_sim_time": True}],
        ))

        # 3b. Spawn the car at its arena origin from its namespaced description.
        spawn = Node(
            package="ros_gz_sim", executable="create",
            namespace=car_name, name="spawn_{}".format(car_name), output="screen",
            arguments=[
                "-world", world, "-topic", "robot_description",
                "-name", car_name,
                "-x", str(ox), "-y", str(oy), "-z", "0.06",
            ],
        )
        actions.append(spawn)

        # 3c. Controllers — chained on this car's spawn exit so the in-URDF
        #     gz_ros2_control controller_manager exists before they run.
        actions.append(RegisterEventHandler(OnProcessExit(
            target_action=spawn,
            on_exit=[
                spawner(car_name, "joint_state_broadcaster"),
                spawner(car_name, "wheels_velocity_controller", param_file=True),
                spawner(car_name, "steering_position_controller", param_file=True),
            ],
        )))

    return actions


def generate_launch_description() -> LaunchDescription:
    """Build the launch description (entry point ROS 2 expects)."""
    declare = [
        DeclareLaunchArgument(
            "n_cars", default_value="2",
            description="number of decoupled arenas/cars in the one gz world"),
        DeclareLaunchArgument(
            "world", default_value=os.environ.get("WORLD_NAME", "reinvent_base"),
            description="track name; loads worlds/<world>.sdf as arena 0's track"),
        DeclareLaunchArgument(
            "spacing", default_value=str(DEFAULT_ARENA_SPACING_M),
            description="grid spacing in metres between adjacent arena origins"),
        DeclareLaunchArgument(
            "gui", default_value="false",
            description="run the Gazebo GUI (false = headless server)"),
        DeclareLaunchArgument(
            "friction_mu", default_value="1.5",
            description="wheel friction coefficient (DR knob), applied to all cars"),
    ]
    return LaunchDescription(declare + [OpaqueFunction(function=_launch_setup)])
