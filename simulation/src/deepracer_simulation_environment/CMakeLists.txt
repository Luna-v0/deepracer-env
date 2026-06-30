#################################################################################
#   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#################################################################################
# ament_cmake port of the DeepRacer simulation-environment package.
#
# This package is now pure data + launch: it installs the car URDF, the
# ros2_control config, the converted Jetty worlds, and the track models/meshes/
# routes into the package share. There is no C++ to build (Route B removed the
# Gazebo-classic plugin) and no message generation (the custom deepracer_msgs
# services were dropped). gz resolves model:// URIs by having
# GZ_SIM_RESOURCE_PATH point at this share dir (set by the launch / entrypoint),
# which holds models/ and meshes/.
cmake_minimum_required(VERSION 3.10)
project(deepracer_simulation_environment NONE)

find_package(ament_cmake REQUIRED)

# Resources under this package directory.
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY fonts/ DESTINATION share/${PROJECT_NAME}/fonts)
install(DIRECTORY track_iconography/ DESTINATION share/${PROJECT_NAME}/track_iconography)

# Shared assets at the simulation/ root (siblings of src/). Mirrors the legacy
# catkin install layout so downstream paths are unchanged.
install(DIRECTORY ../../urdf/    DESTINATION share/${PROJECT_NAME}/urdf)
install(DIRECTORY ../../worlds/  DESTINATION share/${PROJECT_NAME}/worlds)
install(DIRECTORY ../../models/  DESTINATION share/${PROJECT_NAME}/models)
install(DIRECTORY ../../meshes/  DESTINATION share/${PROJECT_NAME}/meshes)
install(DIRECTORY ../../routes/  DESTINATION share/${PROJECT_NAME}/routes)

ament_package()
