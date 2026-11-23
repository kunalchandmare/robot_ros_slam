cmake_minimum_required(VERSION 3.8)
project(stereo_eye_launcher)

find_package(ament_cmake REQUIRED)

# config/ holds the Nav2 and explore_lite parameter files for explore:=true.
install(DIRECTORY launch rviz config DESTINATION share/${PROJECT_NAME})

# Map saver for explore:=true, run with "ros2 run stereo_eye_launcher
# explore_map_saver.py". Maps are written to the source tree's maps/ folder
# (launch arg maps_dir), so maps/ itself is not installed.
install(PROGRAMS scripts/explore_map_saver.py DESTINATION lib/${PROJECT_NAME})

ament_package()
