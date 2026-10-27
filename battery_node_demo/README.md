# Battery Nodes Demo

A ROS 2 example with separate Python and C++ packages in one workspace.

## Structure

- `battery_python`: `ament_python` package containing `battery_publisher` and `battery_processor`.
- `battery_cpp`: `ament_cmake` package containing `battery_listener`.

The repository root is the workspace root. `battery_node_demo` is a grouping directory, not a ROS package.

## Communication

```text
battery_publisher (Python)
    |
    | /battery_level — std_msgs/msg/Int32
    +--------------------------+
    |                          |
    v                          v
battery_processor (Python)  battery_listener (C++)
    |
    | /battery_warning — std_msgs/msg/String
    v
Low-battery warnings
```

## Build

Inside the development container:

```bash
cd /workspaces/ros2-dev
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select battery_python battery_cpp
```

## Run

In each new terminal:

```bash
source /opt/ros/humble/setup.bash
source /workspaces/ros2-dev/install/setup.bash
```

Run these in three separate terminals:

```bash
ros2 run battery_python battery_publisher
```

```bash
ros2 run battery_python battery_processor
```

```bash
ros2 run battery_cpp battery_listener
```

## Inspect

In another sourced terminal:

```bash
ros2 node list
ros2 topic list
ros2 topic echo /battery_level
```

To inspect warnings:

```bash
ros2 topic echo /battery_warning
```

Warnings are published when the battery level reaches 20 or lower.

## Stop

Press Ctrl+C in each running node terminal.

## Learning notes

See LEARNING_NOTES.md for the package build and execution flow.
