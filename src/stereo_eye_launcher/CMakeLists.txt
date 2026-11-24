cmake_minimum_required(VERSION 3.8)
project(stereo_eye_launcher)

find_package(ament_cmake REQUIRED)

# config/ holds the Nav2 and explore_lite parameter files for explore:=true,
# behavior_trees/ the Nav2 behaviour tree used for exploration goals.
install(DIRECTORY launch rviz config behavior_trees DESTINATION share/${PROJECT_NAME})

# Helper nodes for explore:=true, run with e.g. "ros2 run stereo_eye_launcher
# explore_map_saver.py". Maps are written to the source tree's maps/ folder
# (launch arg maps_dir), so maps/ itself is not installed.
install(PROGRAMS
  scripts/explore_map_saver.py   # saves the map when exploration is done
  scripts/stuck_recovery.py      # backs up and turns when the robot is stuck
  scripts/frontier_explorer.py   # our own frontier explorer (explorer:=own)
  scripts/frontier_core.py       # its frontier selection (NumPy), imported by it
  DESTINATION lib/${PROJECT_NAME})

ament_package()
