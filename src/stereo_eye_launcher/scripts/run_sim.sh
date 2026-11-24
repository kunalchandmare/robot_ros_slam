#!/usr/bin/env bash
# run_sim.sh - start the stereo_eye simulation in this terminal.
#
# Usage (from /workspaces/ros2-dev):
#   ./src/stereo_eye_launcher/scripts/run_sim.sh [launch args...]
#   ./src/stereo_eye_launcher/scripts/run_sim.sh rviz:=false steering:=false
#   ./src/stereo_eye_launcher/scripts/run_sim.sh explore:=true   # explore by itself, save the map
#   ./src/stereo_eye_launcher/scripts/run_sim.sh gz_gui:=false   # no Gazebo window (saves GPU memory)
#
# What it does:
#   1. Sources the workspace overlay (/workspaces/ros2-dev/install/setup.bash),
#      so you do not have to do it yourself in every new terminal.
#   2. Runs "ros2 launch stereo_eye_launcher launch.py", passing your launch
#      args through. Everything is printed in this terminal as usual (with
#      colours), AND saved as plain text (no colour codes) to
#      <repo>/logs/latest.log. The run before that is kept as
#      <repo>/logs/previous.log. Only these two files are ever written.
#      Press Ctrl+C to stop.
#   3. Exits with the exit code of "ros2 launch".

set -eo pipefail

# ROS setup scripts read unset variables, so source them before any "set -u".
source /workspaces/ros2-dev/install/setup.bash

# --- Where the log goes ------------------------------------------------------
# <repo> is the git repository root. Ask git first (works from the source
# folder and from an installed copy inside the workspace); if git is missing
# or this is not a git checkout, use the folder three levels above this
# script (src/stereo_eye_launcher/scripts -> repo root).
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(git -C "$SCRIPT_DIR" rev-parse --show-toplevel 2>/dev/null || true)"
if [ -z "$REPO_ROOT" ]; then
  REPO_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"
fi
LOG_DIR="$REPO_ROOT/logs"          # ignored by git (see the repo's .gitignore)
LOG_FILE="$LOG_DIR/latest.log"
mkdir -p "$LOG_DIR"
# Keep exactly one older run: latest.log -> previous.log, then start fresh.
if [ -f "$LOG_FILE" ]; then
  mv -f "$LOG_FILE" "$LOG_DIR/previous.log"
fi
echo "Saving this run's output to $LOG_FILE"

# --- Terminal output stays as before ------------------------------------------
# Output now goes through a pipe, so ROS would normally switch colours off and
# Python would buffer its output in big blocks. Keep both like in a terminal:
export RCUTILS_COLORIZED_OUTPUT=1  # coloured [WARN]/[ERROR] lines, as before
export PYTHONUNBUFFERED=1          # ros2 launch prints each line at once

# --- Run --------------------------------------------------------------------
# ros2 launch -> tee -> terminal (with colours)
#                    -> sed (removes colour codes) -> latest.log
#
# Ctrl+C: the terminal sends SIGINT to every process of this command at once:
# ros2 launch, tee and sed. ros2 launch needs it (it then shuts all nodes down
# and prints their last lines). tee and sed must NOT die from it, or those
# last lines are lost, and ros2 launch could even get a broken pipe while
# shutting down. So the right-hand side runs in a subshell with
# "trap '' INT": SIGINT is ignored there, and tee and sed inherit that.
# They stop by themselves when ros2 launch exits and closes the pipe.
# (This also covers sed; "tee -i" alone would only protect tee.)
# tee writes the terminal copy to file descriptor 3, a copy of this script's
# stdout, and passes the stream on to sed. sed -u writes each line at once.
exec 3>&1
set +e  # read the exit code ourselves instead of stopping here
ros2 launch stereo_eye_launcher launch.py "$@" 2>&1 \
  | ( trap '' INT; tee /dev/fd/3 | sed -u 's/\x1b\[[0-9;]*[A-Za-z]//g' > "$LOG_FILE" )
status=${PIPESTATUS[0]}  # exit code of ros2 launch, not of tee/sed
set -e
exec 3>&-
echo "Output saved to $LOG_FILE"
exit "$status"
