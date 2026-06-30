#!/usr/bin/env bash
#################################################################################
#   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#################################################################################
#
# Live decoupled multi-arena demo over VNC (watch it, don't record it).
#
# Run as the workload of the deepracer-env:gui image — the image entrypoint
# (vnc-gui.sh) has already started Xvfb + jwm + x11vnc on :5900:
#
#   docker build -f docker/Dockerfile.gui -t deepracer-env:gui .
#   docker run --rm -p 5900:5900 -v "$PWD:/ws" -e N_CARS=3 \
#       deepracer-env:gui bash /ws/examples/multi_arena_vnc.sh
#
# then connect TigerVNC / RealVNC to localhost:5900 and watch N DeepRacers drive
# on N decoupled tracks in ONE simulator. Drag the gz camera freely. Each car's
# onboard camera is bridged to /racecar_i/camera/... so you can also open
# `rqt_image_view` (or gz's Image Display plugin) to see the agent-eye view.
#
# Env: N_CARS (default 3), WORLD_NAME (default reinvent_base), DURATION (sec, 600).
#
set +e
source /opt/ros/lyrical/setup.bash
export GZ_SIM_RESOURCE_PATH=/ws/simulation
export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/lyrical/lib
export PYTHONPATH=/ws:$PYTHONPATH
export WORLD_NAME=${WORLD_NAME:-reinvent_base}
# Tracks are tiny (~10 m field), so for a LIVE watch pack the arenas close
# enough to see together. (Headless throughput rollouts use a far larger
# spacing; here viewability wins — the arenas are still fully decoupled.)
N=${N_CARS:-3}
SPACING=${SPACING:-13}
DURATION=${DURATION:-600}
CFG=/ws/simulation/src/deepracer_simulation_environment/config/ros2_control.yaml
X=/ws/simulation/urdf/deepracer/deepracer_gz.urdf.xacro

# gz server + GUI on the VNC display (NOT `-s` headless) so it is watchable live.
# -r RUNS on start: a paused sim ticks no physics, renders no sensors, and never
# activates the in-process controller_manager (cars frozen, 0 camera frames).
echo "[vnc-demo] launch gz Jetty (server+GUI, running) on ${DISPLAY} for ${WORLD_NAME}"
gz sim -v2 -r /ws/simulation/worlds/${WORLD_NAME}.sdf >/tmp/gz.log 2>&1 &
sleep 18

python3 -c "from deepracer_env.sim_control.arena import ArenaLayout
print('\n'.join('%g %g'%(x,y) for x,y in ArenaLayout.grid_offsets($N,$SPACING)))" > /tmp/offsets.txt

i=0
while read OX OY; do
  if [ "$i" -gt 0 ]; then
    # NOTE: `ros_gz_sim create` places the model at its -x/-y/-z flags and
    # IGNORES an <include><pose>; pass the arena offset explicitly or every
    # track stacks at the origin.
    printf '<?xml version="1.0"?>\n<sdf version="1.10"><include><uri>model://models/reinvent_base</uri><name>racetrack_%s</name></include></sdf>\n' "$i" > /tmp/track_$i.sdf
    ros2 run ros_gz_sim create -world ${WORLD_NAME} -file /tmp/track_$i.sdf -x $OX -y $OY >/dev/null 2>&1
  fi
  C=racecar_$i
  # include_camera:=true -> the onboard "camera real view" is available to bridge.
  xacro $X car_name:=$C namespace:=$C include_camera:=true include_lidar:=false ros2_control_config:=$CFG > /tmp/$C.urdf 2>/dev/null
  python3 -c "import yaml;yaml.safe_dump({'/**':{'ros__parameters':{'robot_description':open('/tmp/'+'$C'+'.urdf').read(),'use_sim_time':True}}},open('/tmp/'+'$C'+'.yaml','w'))"
  ros2 run robot_state_publisher robot_state_publisher --ros-args -r __ns:=/$C --params-file /tmp/$C.yaml >/tmp/${C}_rsp.log 2>&1 &
  sleep 2
  YY=$(python3 -c "print($OY+0.6)")
  ros2 run ros_gz_sim create -world ${WORLD_NAME} -file /tmp/$C.urdf -name $C -x $OX -y $YY -z 0.06 >/dev/null 2>&1
  i=$((i+1))
done < /tmp/offsets.txt

echo "[vnc-demo] waiting for controller_managers"; sleep 30
for i in $(seq 0 $((N-1))); do
  CM=/racecar_$i/controller_manager
  ros2 run controller_manager spawner joint_state_broadcaster --controller-manager $CM --controller-manager-timeout 120 >/dev/null 2>&1 &
  ros2 run controller_manager spawner wheels_velocity_controller --param-file $CFG --controller-manager $CM --controller-manager-timeout 120 >/dev/null 2>&1 &
  ros2 run controller_manager spawner steering_position_controller --param-file $CFG --controller-manager $CM --controller-manager-timeout 120 >/dev/null 2>&1 &
done
sleep 35

# Bridge the onboard car cameras so the agent-eye view is viewable (rqt_image_view).
CAMS=""; for i in $(seq 0 $((N-1))); do CAMS="$CAMS /racecar_$i/camera/zed/rgb/image_rect_color"; done
echo "[vnc-demo] bridge onboard cameras:$CAMS"
ros2 run ros_gz_image image_bridge $CAMS >/tmp/bridge.log 2>&1 &
sleep 3

# Auto-frame the GUI camera on ALL arenas so a connecting viewer sees every
# track at once (gz's default camera sits at the origin = arena 0 only).
read CX CY CZ QX QY QZ QW < <(python3 -c "
import math
offs=[tuple(map(float,l.split())) for l in open('/tmp/offsets.txt')]
xs=[o[0]+4.0 for o in offs]; ys=[o[1]+2.6 for o in offs]
cx=sum(xs)/len(xs); cy=sum(ys)/len(ys)
spread=max(max(xs)-min(xs), max(ys)-min(ys), 8.0)
H=spread*1.4+14.0; back=spread*0.7+10.0
yaw=math.pi/2; pitch=math.atan2(H,back)
cp,sp,cy2,sy2=math.cos(pitch/2),math.sin(pitch/2),math.cos(yaw/2),math.sin(yaw/2)
print(cx, cy-back, H, -sp*sy2, sp*cy2, cp*sy2, cp*cy2)")
gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera --reptype gz.msgs.Boolean \
  --timeout 4000 --req "pose: {position: {x: $CX, y: $CY, z: $CZ}, orientation: {x: $QX, y: $QY, z: $QZ, w: $QW}}" >/dev/null 2>&1 || true

echo "[vnc-demo] >>> connect a VNC client to localhost:5900 now <<< driving for ${DURATION}s"
N_CARS=$N python3 /ws/examples/multi_arena_record.py --n_cars $N --spacing $SPACING --routes /ws/simulation/routes --out /tmp --duration ${DURATION}
echo "[vnc-demo] done"
