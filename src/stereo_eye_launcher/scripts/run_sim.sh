#!/usr/bin/env bash
# run_sim.sh - start the stereo_eye simulation and save its output to a log file.
#
# Usage (from /workspaces/ros2-dev):
#   ./src/stereo_eye_launcher/scripts/run_sim.sh [launch args...]
#   ./src/stereo_eye_launcher/scripts/run_sim.sh rviz:=false steering:=false
#   ./src/stereo_eye_launcher/scripts/run_sim.sh explore:=true   # explore by itself, save the map
#
# What it does:
#   1. Sources the workspace overlay (/workspaces/ros2-dev/install/setup.bash).
#   2. Creates the package's logs/ folder if it is missing.
#   3. Points ROS_LOG_DIR at logs/, so every node's own ROS log file lands there.
#   4. Runs "ros2 launch stereo_eye_launcher launch.py", passing your launch args
#      through, and copies all output (stdout and stderr) to
#      logs/launch_<YYYY-mm-dd_HH-MM-SS>.log while still showing it on screen.

set -eo pipefail

# Folder this script lives in, and the package folder one level up.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(dirname "${SCRIPT_DIR}")"
LOG_DIR="${PKG_DIR}/logs"

# ROS setup scripts read unset variables, so source them before any "set -u".
source /workspaces/ros2-dev/install/setup.bash

mkdir -p "${LOG_DIR}"
export ROS_LOG_DIR="${LOG_DIR}"

LOG_FILE="${LOG_DIR}/launch_$(date +%Y-%m-%d_%H-%M-%S).log"
echo "Saving launch output to ${LOG_FILE}"
echo "Per-node ROS logs go to ${ROS_LOG_DIR}"

ros2 launch stereo_eye_launcher launch.py "$@" 2>&1 | tee "${LOG_FILE}"
