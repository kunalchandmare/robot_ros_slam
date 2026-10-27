# ROS 2 Build Guide for `robot_ros_slam`

This guide describes how this repository builds and runs its battery-node example. The project uses ROS 2 Humble in the Dev Container configured under `.devcontainer/`.

## Project layout

The repository root is the colcon workspace root. `battery_node_demo/` is a grouping directory, not a ROS package; it contains two independent packages with different build systems:

```text
robot_ros_slam/                 # Workspace root
├── battery_node_demo/          # Groups the example packages
│   ├── battery_python/          # ament_python package
│   │   ├── package.xml
│   │   ├── setup.py
│   │   ├── setup.cfg
│   │   ├── resource/battery_python
│   │   └── battery_python/
│   │       ├── battery_publisher.py
│   │       └── battery_processor.py
│   └── battery_cpp/             # ament_cmake package
│       ├── package.xml
│       ├── CMakeLists.txt
│       └── src/battery_listener.cpp
├── build/                       # Generated; do not commit
├── install/                     # Generated; do not commit
└── log/                          # Generated; do not commit
```

The Dev Container mounts the opened repository at `/workspaces/ros2-dev`. Run the commands below in a terminal inside that container, from the workspace root.

## What registers each executable?

A ROS package must install its runnable executable into the location ROS 2 expects. In this project, each language uses its own package and build configuration:

| Package | Build type | Executable registration |
|---|---|---|
| `battery_python` | `ament_python` | `setup.py` declares the `battery_publisher` and `battery_processor` console scripts; `setup.cfg` places their wrappers under `lib/battery_python`. |
| `battery_cpp` | `ament_cmake` | `CMakeLists.txt` creates `battery_listener` and installs it under `lib/battery_cpp`. |

Each package's `package.xml` declares its build type and ROS dependencies. The Python package depends on `rclpy` and `std_msgs`; the C++ package depends on `rclcpp` and `std_msgs`. The `resource/battery_python` marker and Python package directory support package discovery and module installation.

The packages are separate, not a single mixed-language package. Colcon discovers and builds both from the workspace root.

## Build

In the Dev Container terminal:

```bash
cd /workspaces/ros2-dev
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select battery_python battery_cpp
```

`--symlink-install` is useful during development: Python source files are linked into the install space instead of copied. The C++ executable is still compiled during the build. `--packages-select` builds the two packages in this example.

After a successful build, source the workspace overlay in that terminal:

```bash
source install/setup.bash
```

For every new terminal, source both the ROS distribution and the workspace overlay before using the packages:

```bash
source /opt/ros/humble/setup.bash
source /workspaces/ros2-dev/install/setup.bash
```

Sourcing makes installed packages discoverable in the current shell; it does not build packages or start nodes.

## Run the nodes

Run each executable in its own sourced terminal:

```bash
ros2 run battery_python battery_publisher
```

```bash
ros2 run battery_python battery_processor
```

```bash
ros2 run battery_cpp battery_listener
```

The publisher sends `std_msgs/msg/Int32` messages on `/battery_level`. The processor and C++ listener subscribe to that topic; the processor publishes low-battery warnings as `std_msgs/msg/String` on `/battery_warning` when the level is 20 or lower. See [`battery_node_demo/README.md`](../battery_node_demo/README.md) for the full communication diagram and inspection commands.

Press Ctrl+C in each node terminal to stop it.

## Build and runtime flow

```text
battery_python/package.xml + setup.py + setup.cfg
    -> install Python modules and console-script wrappers
    -> ros2 run battery_python <executable>

battery_cpp/package.xml + CMakeLists.txt + src/battery_listener.cpp
    -> compile and install the C++ executable
    -> ros2 run battery_cpp battery_listener

colcon build
    -> workspace install/ space
    -> source install/setup.bash
    -> ROS 2 can discover the installed packages and executables
```

## When project files change

| Change | Action |
|---|---|
| Edit Python node logic | Restart that node. With `--symlink-install`, ordinary source edits usually do not require another build. |
| Add or rename a Python executable | Update the `console_scripts` list in `battery_python/setup.py`, rebuild, and use the registered executable name with `ros2 run`. |
| Edit C++ node logic | Rebuild `battery_cpp` and restart the node. |
| Add or rename a C++ executable | Add/update its target and install rule in `battery_cpp/CMakeLists.txt`, rebuild, and use the installed target name with `ros2 run`. |
| Add a ROS dependency | Declare it in the package's `package.xml` and ensure it is available in the development environment. |
| Change package installation metadata | Rebuild the affected package. |

Build artifacts in `build/`, `install/`, and `log/` are generated and are excluded from Git by the repository's `.gitignore`.
