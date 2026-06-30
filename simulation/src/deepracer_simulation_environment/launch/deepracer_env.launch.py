#################################################################################
#   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#################################################################################
"""ROS 2 launch for the DeepRacer Gazebo Jetty environment (single car).

This is the ROS 2 replacement for the legacy XML ``deepracer_env.launch`` chain.
It brings up, in order:

1. **Gazebo Jetty** running the converted world SDF (``-s`` headless unless
   ``gui:=true``), with ``GZ_SIM_RESOURCE_PATH`` pointed at the package share so
   ``model://models/...`` and ``model://meshes/...`` resolve.
2. **robot_state_publisher** holding the car URDF expanded from
   ``urdf/deepracer/deepracer_gz.urdf.xacro`` (carries the ``gz_ros2_control``
   plugin + ``<ros2_control>`` block).
3. **Spawn** the car from ``/robot_description`` via ``ros_gz_sim create``.
4. The **ros_gz bridge** for ``/clock`` and the per-car camera / LiDAR / pose
   topics, so the Python env subscribes to ordinary ROS topics.
5. **Controller spawners**: ``joint_state_broadcaster`` plus the two
   ``forward_command_controller`` groups (wheels velocity, steering position).

The world name comes from the ``world`` launch arg, defaulting to the
``WORLD_NAME`` environment variable (how ``dr-gym`` passes the track in), then to
``reinvent_base``. Multi-arena (N decoupled cars) is layered on top by the Python
env via the :class:`~deepracer_env.sim_control.arena.ArenaLayout`; this file
covers the single-car substrate bring-up.
"""
from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

PKG = "deepracer_simulation_environment"


def generate_launch_description() -> LaunchDescription:
    """Build the launch description (entry point ROS 2 expects)."""
    share = get_package_share_directory(PKG)

    world = LaunchConfiguration("world")
    gui = LaunchConfiguration("gui")
    car_name = LaunchConfiguration("car_name")
    friction_mu = LaunchConfiguration("friction_mu")

    xacro_file = PathJoinSubstitution([FindPackageShare(PKG), "urdf", "deepracer", "deepracer_gz.urdf.xacro"])
    ros2_control_cfg = PathJoinSubstitution([FindPackageShare(PKG), "config", "ros2_control.yaml"])
    world_sdf = PathJoinSubstitution([FindPackageShare(PKG), "worlds", [world, ".sdf"]])

    # model:// resolution: the share dir holds models/ and meshes/.
    resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=share + os.pathsep + os.path.join(share, "models"),
    )
    # gz-sim must find libgz_ros2_control-system.so under the ROS lib dir.
    ros_lib = "/opt/ros/" + os.environ.get("ROS_DISTRO", "lyrical") + "/lib"
    plugin_path = SetEnvironmentVariable(name="GZ_SIM_SYSTEM_PLUGIN_PATH", value=ros_lib)

    declare = [
        DeclareLaunchArgument("world", default_value=os.environ.get("WORLD_NAME", "reinvent_base"),
                              description="track name (loads worlds/<world>.sdf)"),
        DeclareLaunchArgument("gui", default_value="false",
                              description="run the Gazebo GUI (false = headless server)"),
        DeclareLaunchArgument("car_name", default_value="car",
                              description="entity + topic namespace for the car"),
        DeclareLaunchArgument("friction_mu", default_value="1.5",
                              description="wheel friction coefficient (DR knob)"),
    ]

    # 1. Gazebo: headless server, or server+GUI when gui:=true.
    gz_server = ExecuteProcess(
        cmd=["gz", "sim", "-s", "-r", "-v", "1", world_sdf],
        output="screen",
    )
    gz_gui = ExecuteProcess(
        cmd=["gz", "sim", "-g"], output="screen", condition=IfCondition(gui),
    )

    # 2. robot_state_publisher with the xacro-expanded car URDF.
    robot_description = Command([
        "xacro ", xacro_file,
        " car_name:=", car_name,
        " friction_mu:=", friction_mu,
        " ros2_control_config:=", ros2_control_cfg,
    ])
    rsp = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        output="screen",
        # Wrap as a string parameter: a bare Command() substitution is parsed as
        # YAML, and the URDF XML fails that parse ("Unable to parse ... as yaml").
        parameters=[{"robot_description": ParameterValue(robot_description, value_type=str),
                     "use_sim_time": True}],
    )

    # 3. Spawn the car from /robot_description.
    spawn = Node(
        package="ros_gz_sim", executable="create", output="screen",
        arguments=["-topic", "robot_description", "-name", car_name, "-z", "0.06"],
    )

    # 4. ros_gz bridge: clock (gz->ros) + per-car sensors + continuous entity
    #    poses. Bridging dynamic_pose/info lets the SimControl seam read poses
    #    from a live ROS subscription instead of forking a `gz topic` snapshot per
    #    step (that subprocess was ~77% of feature-obs training wall time).
    bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge", output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            ["/world/", world, "/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V"],
            # set_pose as a ROS service -> the seam teleports via a persistent
            # rclpy client instead of forking a `gz service` per reset (~270ms).
            # gz req/rep types are given explicitly (the bridge doesn't auto-map
            # SetEntityPose otherwise).
            ["/world/", world,
             "/set_pose@ros_gz_interfaces/srv/SetEntityPose@gz.msgs.Pose@gz.msgs.Boolean"],
            [car_name, "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"],
        ],
    )
    image_bridge = Node(
        package="ros_gz_image", executable="image_bridge", output="screen",
        arguments=[[car_name, "/camera/zed/rgb/image_rect_color"]],
    )

    # 5. Controllers — spawned after the car (and thus the controller_manager
    #    inside gz_ros2_control) exists. Chained on spawn exit so ordering holds.
    #    The forward_command_controllers need their `joints`/`interface_name`
    #    params passed via --param-file: gz_ros2_control hands its <parameters>
    #    file to the controller_manager, but not down to the controller nodes.
    def spawner(name, param_file=False):
        args = [name, "--controller-manager-timeout", "30"]
        if param_file:
            args += ["--param-file", ros2_control_cfg]
        return Node(package="controller_manager", executable="spawner",
                    arguments=args, output="screen")

    controllers_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn,
            on_exit=[
                spawner("joint_state_broadcaster"),
                spawner("wheels_velocity_controller", param_file=True),
                spawner("steering_position_controller", param_file=True),
            ],
        )
    )

    # 6. Real-time-factor override for training throughput. The world ships
    #    real_time_factor=1.0 (real-time, good for watching); training wants the
    #    sim to run as fast as the CPU allows. RTF_OVERRIDE controls the target
    #    factor — "0" (default) = unlimited; "1" = real-time; "4" = up to 4x.
    #    Applied a few seconds after start via the gz set_physics service (the
    #    world must be up first). dr-gym forwards RTF_OVERRIDE from
    #    TrainingConfig.rtf_override.
    rtf = os.environ.get("RTF_OVERRIDE", "0") or "0"
    set_rtf = TimerAction(period=8.0, actions=[ExecuteProcess(
        cmd=["gz", "service", "-s", ["/world/", world, "/set_physics"],
             "--reqtype", "gz.msgs.Physics", "--reptype", "gz.msgs.Boolean",
             "--timeout", "5000",
             "--req", "real_time_factor: {}, max_step_size: 0.001".format(rtf)],
        output="screen")])

    return LaunchDescription(
        declare + [plugin_path, resource_path, gz_server, gz_gui, rsp, spawn, bridge,
                   image_bridge, controllers_after_spawn, set_rtf]
    )
