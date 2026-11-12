cmake_minimum_required(VERSION 3.8)
project(apartment_gazebo)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY worlds models
  DESTINATION share/${PROJECT_NAME}
)

# After sourcing the workspace, model:// URIs resolve to
# share/apartment_gazebo/models. ${prefix} is expanded at source time;
# @PROJECT_NAME@ is substituted when this .dsv.in is configured.
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/apartment_gazebo.dsv.in")

ament_package()
