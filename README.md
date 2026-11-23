# robot_ros_slam

This project aims to build a simple robot using ROS 2 with Simultaneous Localization and Mapping (SLAM) capability. The main goal is to bring up a robot, integrate its sensor data, and use SLAM to create a map while estimating the robot's position.

Today the project simulates a small two-wheeled robot with a stereo camera
(`stereo_eye`) driving around an apartment in Gazebo Fortress, and builds a map
with RTAB-Map stereo SLAM on ROS 2 Humble.

## Packages

| Package | What it is | Owner | Docs |
|---|---|---|---|
| `src/stereo_eye` | Robot description (URDF/xacro): body, wheels, stereo cameras, IMU, Gazebo plugins | MechaX (robot design) | [README](src/stereo_eye/README.md) |
| `src/apartment_gazebo` | Gazebo world: the apartment, its models and system plugins | Isekai (world design) | [README](src/apartment_gazebo/README.md) |
| `src/stereo_eye_launcher` | Launch package: Gazebo, bridge, stereo odometry, RTAB-Map, RViz, driving, log script | Bazooka (launch files) | [README](src/stereo_eye_launcher/README.md) |
| `src/camera_brightness` | Small node that reports the left camera image brightness | Kunal | none yet |
| `.devcontainer/` | Docker development container with ROS 2 Humble and all dependencies | Robocop (CTO) | [setup notes](documents/ROS_Docker_Container_Setup.md) |
| `battery_node_demo/` | Practice project, see below | Kunal | [README](battery_node_demo/README.md) |

## Quick start

Inside the dev container, the repository root is the workspace
`/workspaces/ros2-dev`:

```bash
cd /workspaces/ros2-dev
git pull
colcon build --packages-up-to stereo_eye_launcher
source install/setup.bash
./src/stereo_eye_launcher/scripts/run_sim.sh
```

Gazebo, RViz and a Robot Steering window open. Drive slowly with the sliders.
Each run's output is saved in `src/stereo_eye_launcher/logs/`. The
[launcher README](src/stereo_eye_launcher/README.md) explains every step, the
TF tree, driving and troubleshooting.

## Practice project: `battery_node_demo`

`battery_node_demo/` is a small practice project for learning ROS 2 package structure, node execution, and topic-based communication before developing the robot and SLAM system. It is an example, not the SLAM implementation.

It contains two independent ROS 2 packages:

- `battery_python`: Python publisher and processor.
- `battery_cpp`: C++ listener.

The repository root is the ROS 2 workspace root. The `battery_node_demo/` directory groups these packages; it is not itself a ROS package.

For the practice project, see [its README](battery_node_demo/README.md) for build and run instructions and [its learning notes](battery_node_demo/LEARNING_NOTES.md) for explanations. See [documents/ROS_Build_Guide.md](documents/ROS_Build_Guide.md) for the repository's package/build overview and [documents/ROS_Setup_Cariad.md](documents/ROS_Setup_Cariad.md) for the ROS 2 Humble development environment setup.
