#!/usr/bin/env bash
#################################################################################
#   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#################################################################################
# Runtime entrypoint for the ROS 2 Lyrical / Gazebo Jetty image.
#
# Sources ROS + the colcon overlay and exports the two gz search paths the port
# depends on (both discovered empirically during bring-up):
#   * GZ_SIM_SYSTEM_PLUGIN_PATH must include /opt/ros/lyrical/lib so gz-sim can
#     load the gz_ros2_control system plugin (libgz_ros2_control-system.so).
#   * GZ_SIM_RESOURCE_PATH must point at the package share (which holds models/
#     and meshes/) so world and robot model:// URIs resolve.
set -e

source /opt/ros/lyrical/setup.bash
[ -f /opt/simapp/setup.bash ] && source /opt/simapp/setup.bash

SIM_SHARE=/opt/simapp/share/deepracer_simulation_environment
export GZ_SIM_SYSTEM_PLUGIN_PATH="/opt/ros/lyrical/lib${GZ_SIM_SYSTEM_PLUGIN_PATH:+:${GZ_SIM_SYSTEM_PLUGIN_PATH}}"
export GZ_SIM_RESOURCE_PATH="${SIM_SHARE}:${SIM_SHARE}/models${GZ_SIM_RESOURCE_PATH:+:${GZ_SIM_RESOURCE_PATH}}"

exec "$@"
