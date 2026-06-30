#!/usr/bin/env bash
#################################################################################
#   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#################################################################################
#
# VNC GUI bootstrap (entrypoint for deepracer-env:gui).
#
# Brings up a virtual X display + a lightweight window manager + a VNC server,
# then execs the workload ("$@") against that display. Connect a desktop VNC
# client (TigerVNC, RealVNC) to localhost:${VNC_PORT:-5900} to watch the gz
# Jetty GUI live and drag the camera — the same live-view workflow the classic
# ROS 1 / Gazebo 11 stack served on :5900, restored for Gazebo Jetty.
#
# Env:
#   DISPLAY       virtual display id          (default :0)
#   VNC_PORT      raw VNC (RFB) port          (default 5900)
#   VNC_GEOMETRY  Xvfb screen WxHxDEPTH       (default 1600x900x24)
#   VNC_PASSWORD  if set, require this password (default: none / open)
#
set -e

export DISPLAY="${DISPLAY:-:0}"
PORT="${VNC_PORT:-5900}"
GEOM="${VNC_GEOMETRY:-1600x900x24}"

echo "[vnc-gui] Xvfb on ${DISPLAY} (${GEOM})"
Xvfb "${DISPLAY}" -screen 0 "${GEOM}" >/tmp/xvfb.log 2>&1 &
# wait for the display socket
for _ in $(seq 1 30); do [ -S "/tmp/.X11-unix/X${DISPLAY#:}" ] && break; sleep 0.2; done

echo "[vnc-gui] window manager (jwm)"
jwm >/tmp/jwm.log 2>&1 &
sleep 1

if [ -n "${VNC_PASSWORD:-}" ]; then
    x11vnc -storepasswd "${VNC_PASSWORD}" /tmp/.vncpw >/dev/null 2>&1
    AUTH="-rfbauth /tmp/.vncpw"
else
    AUTH="-nopw"
fi
echo "[vnc-gui] x11vnc on :${PORT}  ->  connect a VNC client to localhost:${PORT}"
x11vnc -bg -forever -shared ${AUTH} -rfbport "${PORT}" -display "${DISPLAY}" \
    >/tmp/x11vnc.log 2>&1

# Source ROS so the workload can launch gz / ros2 directly.
# shellcheck disable=SC1091
source /opt/ros/lyrical/setup.bash
[ -f /opt/simapp/setup.bash ] && source /opt/simapp/setup.bash 2>/dev/null || true

if [ "$#" -eq 0 ]; then
    echo "[vnc-gui] no command given; opening gz Jetty GUI for WORLD_NAME=${WORLD_NAME:-reinvent_base}"
    WORLD="${WORLD_NAME:-reinvent_base}"
    # prefer a mounted dev tree, fall back to the baked share dir
    for d in /ws/simulation/worlds /opt/simapp/share/deepracer_simulation_environment/worlds; do
        [ -f "${d}/${WORLD}.sdf" ] && { export GZ_SIM_RESOURCE_PATH="$(dirname "$d")${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"; exec gz sim -v2 "${d}/${WORLD}.sdf"; }
    done
    echo "[vnc-gui] world ${WORLD}.sdf not found; dropping to a shell"
    exec bash
fi

exec "$@"
