#!/bin/bash
# Source ROS base and the colcon simapp overlay so every child process
# (roslaunch, python3, etc.) inherits the correct PYTHONPATH and ROS_PACKAGE_PATH.
source /opt/ros/${ROS_DISTRO}/setup.bash
source /opt/simapp/setup.bash

# Tell Gazebo (gzclient/gzserver) where to find world files, meshes,
# materials, and model:// URIs. Without these, gzclient's Ogre instance
# silently fails to load track textures (the track renders as a flat
# polygon) even though gzserver's in-sim camera sensor resolves them via
# a separate path. The simapp ships everything under one share/ tree.
SIMAPP_ENV_SHARE=/opt/simapp/deepracer_simulation_environment/share/deepracer_simulation_environment
# Worlds reference `model://models/<name>` — the literal `models/` prefix means
# GAZEBO_MODEL_PATH must be the parent of `models/`, not `models/` itself.
export GAZEBO_RESOURCE_PATH="${SIMAPP_ENV_SHARE}:${GAZEBO_RESOURCE_PATH:-/usr/share/gazebo-11}"
export GAZEBO_MODEL_PATH="${SIMAPP_ENV_SHARE}:${GAZEBO_MODEL_PATH:-/usr/share/gazebo-11/models:/root/.gazebo/models}"

exec "$@"
