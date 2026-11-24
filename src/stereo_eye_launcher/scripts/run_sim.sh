#!/usr/bin/env bash
# run_sim.sh - start the stereo_eye simulation in this terminal.
#
# Usage (from /workspaces/ros2-dev):
#   ./src/stereo_eye_launcher/scripts/run_sim.sh [launch args...]
#   ./src/stereo_eye_launcher/scripts/run_sim.sh rviz:=false steering:=false
#   ./src/stereo_eye_launcher/scripts/run_sim.sh explore:=true   # explore by itself, save the map
#
# What it does:
#   1. Sources the workspace overlay (/workspaces/ros2-dev/install/setup.bash),
#      so you do not have to do it yourself in every new terminal.
#   2. Runs "ros2 launch stereo_eye_launcher launch.py", passing your launch
#      args through. All output is printed in this terminal only; nothing is
#      written to log files by this script. Press Ctrl+C to stop.

set -eo pipefail

# ROS setup scripts read unset variables, so source them before any "set -u".
source /workspaces/ros2-dev/install/setup.bash

ros2 launch stereo_eye_launcher launch.py "$@"
