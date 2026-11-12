cmake_minimum_required(VERSION 3.8)
project(stereo_eye)

find_package(ament_cmake REQUIRED)

# Model-only description package: install the robot model so that
# package://stereo_eye/... and $(find stereo_eye) resolve.
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})

ament_package()
