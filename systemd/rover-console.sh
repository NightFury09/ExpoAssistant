#!/bin/bash
# Launcher for the rover console under systemd.
#
# systemd starts processes with an almost empty environment, so every overlay
# the console needs has to be sourced here -- it cannot inherit them from an
# interactive shell that is never going to run.
# No `set -u`: ROS's own setup.bash reads AMENT_TRACE_SETUP_FILES before
# setting it, so nounset makes sourcing ROS fail outright.
set -e
source /opt/ros/humble/setup.bash
# The micro-ROS agent, built in this repo's uros_ws. Sourced explicitly: until
# now it was found only because ros2_ws had happened to be built with uros_ws
# already sourced. Built any other way, every stack failed to find
# micro_ros_agent. Tolerated if missing so the console still comes up and can
# say what is wrong.
source /home/rptech/AGX_Orin_Backup/rover_project/uros_ws/install/local_setup.bash 2>/dev/null || true
source /home/rptech/AGX_Orin_Backup/rover_project/ros2_ws/install/setup.bash

# The console supervises `ros2 launch` children; they inherit this environment,
# so anything the stacks need belongs here too.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export PYTHONUNBUFFERED=1

exec ros2 run rover_core rover_dashboard "$@"
