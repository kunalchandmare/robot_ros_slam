# robot_ros_slam

This project aims to build a simple robot using ROS 2 with Simultaneous Localization and Mapping (SLAM) capability. The main goal is to bring up a robot, integrate its sensor data, and use SLAM to create a map while estimating the robot's position.

## Practice project: `battery_node_demo`

`battery_node_demo/` is a small practice project for learning ROS 2 package structure, node execution, and topic-based communication before developing the robot and SLAM system. It is an example, not the SLAM implementation.

It contains two independent ROS 2 packages:

- `battery_python`: Python publisher and processor.
- `battery_cpp`: C++ listener.

The repository root is the ROS 2 workspace root. The `battery_node_demo/` directory groups these packages; it is not itself a ROS package.

For the practice project, see [its README](battery_node_demo/README.md) for build and run instructions and [its learning notes](battery_node_demo/LEARNING_NOTES.md) for explanations. See [documents/ROS_Build_Guide.md](documents/ROS_Build_Guide.md) for the repository's package/build overview and [documents/ROS_Setup_Cariad.md](documents/ROS_Setup_Cariad.md) for the ROS 2 Humble development environment setup.
