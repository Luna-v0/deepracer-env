#!/usr/bin/env bash
#################################################################################
#   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#################################################################################
# End-to-end demo bring-up for the ROS 2 Lyrical / Gazebo Jetty stack.
#
# Brings up gz Jetty on a converted world, spawns the DeepRacer car with
# gz_ros2_control, activates the controllers, then runs examples/drive_demo.py
# to drive the car around the track via the SimControl seam + ros2_control,
# logging the trajectory to ${OUT}/traj.csv.
#
# Expects the repo bind-mounted at /ws and a writable /out. See the comments in
# project memory "lyrical-jetty-substrate-verified" for why each step is shaped
# the way it is (the bring-up gotchas).
set +e
WORLD=${WORLD:-reinvent_base}
source /opt/ros/lyrical/setup.bash
python3 -m pip install -q --break-system-packages numpy >/dev/null 2>&1 || pip3 install -q --break-system-packages numpy >/dev/null 2>&1
export GZ_SIM_RESOURCE_PATH=/ws/simulation
export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/lyrical/lib
export PYTHONPATH=/ws:$PYTHONPATH
CFG=/ws/simulation/src/deepracer_simulation_environment/config/ros2_control.yaml

echo "[bringup] xacro -> urdf"
xacro /ws/simulation/urdf/deepracer/deepracer_gz.urdf.xacro car_name:=car ros2_control_config:=$CFG > /out/car.urdf || { echo XACRO_FAIL; exit 1; }

echo "[bringup] gz server (${WORLD})"
gz sim -s -r -v 1 /ws/simulation/worlds/${WORLD}.sdf >/out/gz.log 2>&1 &
sleep 14

echo "[bringup] robot_state_publisher"
python3 -c "import yaml; yaml.safe_dump({'robot_state_publisher':{'ros__parameters':{'robot_description': open('/out/car.urdf').read(),'use_sim_time': True}}}, open('/out/rsp.yaml','w'))"
ros2 run robot_state_publisher robot_state_publisher --ros-args --params-file /out/rsp.yaml >/out/rsp.log 2>&1 &
sleep 4

echo "[bringup] spawn car"
ros2 run ros_gz_sim create -world ${WORLD} -file /out/car.urdf -name car -z 0.06 >/out/spawn.log 2>&1
sleep 8

echo "[bringup] controllers"
ros2 run controller_manager spawner joint_state_broadcaster --controller-manager-timeout 30 >/out/jsb.log 2>&1
ros2 run controller_manager spawner wheels_velocity_controller --param-file $CFG --controller-manager-timeout 30 >/out/wvc.log 2>&1
ros2 run controller_manager spawner steering_position_controller --param-file $CFG --controller-manager-timeout 30 >/out/spc.log 2>&1
ros2 control list_controllers 2>/dev/null

echo "[bringup] drive demo"
python3 /ws/examples/drive_demo.py --world ${WORLD} --routes /ws/simulation/routes --out /out/traj.csv --duration ${DURATION:-22}

echo "[bringup] done"
pkill -f "gz sim"; pkill -f gz-sim; pkill -f robot_state; true
