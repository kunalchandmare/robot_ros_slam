# robot_ros_slam

This project aims to build a simple robot using ROS 2 with Simultaneous Localization and Mapping (SLAM) capability. The main goal is to bring up a robot, integrate its sensor data, and use SLAM to create a map while estimating the robot's position.

Today the project simulates a small two-wheeled robot with a stereo camera
(`stereo_eye`) driving around an apartment in Gazebo Fortress, and builds a map
with RTAB-Map stereo SLAM on ROS 2 Humble. With `explore:=true` the robot also
drives itself: explore_lite picks unexplored frontiers and Nav2 follows a path
to them. How that stack is wired, and which file and parameter control planning
and navigation, is in [documents/ARCHITECTURE.md](documents/ARCHITECTURE.md).

## Current status

**What works.** The robot maps the apartment and, after the tf2 upgrade
described below, drives autonomously with
`explore:=true`. Completed maps are written under
`src/stereo_eye_launcher/maps/`. Each `run_sim.sh` run is also saved as
`logs/latest.log` (the previous run is kept as `logs/previous.log`).

**Known issues.**

- Humble **tf2 0.25.23** has a deadlock (ros2/geometry2 #990, fixed in 0.25.24)
  that freezes Nav2's position tracking: the planner may still print a path,
  but the robot never drives. The Dockerfile upgrades tf2 on a container
  rebuild. In an already-built container, run
  `sudo apt update && sudo apt install --only-upgrade 'ros-humble-tf2*'`
  and check that `dpkg -l ros-humble-tf2` shows **0.25.24 or later**.
  `run_sim.sh` warns if it still finds 0.25.23.
- The robot can still **get stuck mid-navigation** sometimes (a plan is
  visible, but it does not move). That is being investigated.
- Frontier **goal choice** is simple (nearest / largest open edge). Making it
  smarter is planned, but not the current priority.

**Roadmap.** The current goal is reliable autonomous exploration of this
apartment, including getting unstuck. The next goal after that is a learned
local navigation policy (Stable-Baselines3, trained in Gazebo, ToF + stereo
depth → `/cmd_vel`) that would replace only Nav2's controller. The planner,
explore_lite and the ToF safety stop would stay. That work has not started
yet. Details are in the [architecture document](documents/ARCHITECTURE.md#10-current-state-known-issues-and-roadmap).

## Packages

| Package | What it is | Owner | Docs |
|---|---|---|---|
| `src/stereo_eye` | Robot description (URDF/xacro): body, wheels, stereo cameras, IMU, Gazebo plugins | MechaX (robot design) | [README](src/stereo_eye/README.md) |
| `src/apartment_gazebo` | Gazebo world: the apartment, its models and system plugins | Isekai (world design) | [README](src/apartment_gazebo/README.md) |
| `src/stereo_eye_launcher` | Launch package: Gazebo, bridge, stereo odometry, RTAB-Map, Nav2, explore_lite, RViz, driving, log script | Bazooka (launch files) | [README](src/stereo_eye_launcher/README.md) |
| `src/third_party/m-explore-ros2` | Git submodule: `explore_lite` (frontier exploration) and `explore_lite_msgs`, pinned at commit `326cf8a` | upstream, pinned here | [README](src/third_party/m-explore-ros2/README.md) |
| `src/camera_brightness` | Small node that reports the left camera image brightness | Kunal | none yet |
| `.devcontainer/` | Docker development container with ROS 2 Humble and all dependencies | Robocop (CTO) | [setup notes](documents/ROS_Docker_Container_Setup.md) |
| `documents/` | Project documentation, including the system architecture | Robocop | [ARCHITECTURE.md](documents/ARCHITECTURE.md) |
| `battery_node_demo/` | Practice project, see below | Kunal | [README](battery_node_demo/README.md) |

## Quick start

Work inside the **ROS 2 Humble dev container** (`.devcontainer/`). On Windows
this is Docker Desktop with WSLg. The repository root is mounted as the
workspace `/workspaces/ros2-dev`.

First time only, on Windows in the repo folder, download the
`m-explore-ros2` submodule (frontier exploration source). Do this on the
host: git inside the container may be blocked by the work network.

```bash
git submodule update --init
```

Then, inside the container:

```bash
cd /workspaces/ros2-dev
git pull
colcon build --packages-up-to stereo_eye_launcher
source install/setup.bash
./src/stereo_eye_launcher/scripts/run_sim.sh explore:=true gz_gui:=false
```

`run_sim.sh` first applies our patches to third-party code (`patches/`, see
[patches/apply_patches.sh](patches/apply_patches.sh)) and, if a patch is newer
than the build, rebuilds `explore_lite` by itself. To do it by hand before a
`colcon build`: `bash patches/apply_patches.sh` (safe to run again; it prints
`applied now` or `already applied`, or an `ERROR`).

`explore:=true` starts Nav2 and explore_lite, so the robot drives itself.
`gz_gui:=false` starts only the Gazebo server (no Gazebo window). That avoids
the WSL GPU running out of memory and taking the whole launch down. RViz still
opens and shows the robot, the cameras and the map.

To look at Gazebo as well, attach a GUI from a **second** terminal while the
server is running:

```bash
ign gazebo -g
```

If that window later crashes, close it; the server and the rest of the launch
should keep running.

Without `explore:=true`, Gazebo, RViz and a Robot Steering window open and
you drive with the sliders (keep about 0.15 m/s and 0.4 rad/s).

Output is printed in the terminal and also saved as plain text to
`logs/latest.log` at the repo root (`/workspaces/ros2-dev/logs/latest.log`
in the container). The previous run is kept as `logs/previous.log`.

The [launcher README](src/stereo_eye_launcher/README.md) explains every
launch argument, the TF tree, driving, exploration and troubleshooting.
The [architecture document](documents/ARCHITECTURE.md) explains the node
graph and how to tune planning and navigation.

## Practice project: `battery_node_demo`

`battery_node_demo/` is a small practice project for learning ROS 2 package structure, node execution, and topic-based communication before developing the robot and SLAM system. It is an example, not the SLAM implementation.

It contains two independent ROS 2 packages:

- `battery_python`: Python publisher and processor.
- `battery_cpp`: C++ listener.

The repository root is the ROS 2 workspace root. The `battery_node_demo/` directory groups these packages; it is not itself a ROS package.

For the practice project, see [its README](battery_node_demo/README.md) for build and run instructions and [its learning notes](battery_node_demo/LEARNING_NOTES.md) for explanations. See [documents/ROS_Build_Guide.md](documents/ROS_Build_Guide.md) for the repository's package/build overview and [documents/ROS_Setup_Cariad.md](documents/ROS_Setup_Cariad.md) for the ROS 2 Humble development environment setup.
