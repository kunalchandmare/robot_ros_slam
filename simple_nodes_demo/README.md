# Simple ROS 2 Nodes Demo

A small educational example showing how ROS 2 nodes communicate using **topics**.

## What this demonstrates
- Creating nodes in **Python** and **C++**
- Publishers and Subscribers
- Message passing between nodes
- Basic logging
- How to build and run a mixed-language package

## Nodes

| Node | Language | Role |
|------|----------|------|
| `battery_publisher` | Python | Publishes a decreasing battery level (integer) on `/battery_level` |
| `battery_processor` | Python | Subscribes to `/battery_level`, logs it, and publishes a warning string on `/battery_warning` when low |
| `battery_listener` | C++ | Subscribes to `/battery_level` and prints the value |

## Topics
- `/battery_level` (`std_msgs/msg/Int32`) — current battery percentage
- `/battery_warning` (`std_msgs/msg/String`) — warning message when battery is low

## Prerequisites
- ROS 2 (Humble or Jazzy recommended)
- `colcon` and `rosdep`
- Python 3 and a C++ compiler

## Build
```bash
cd ~/your_ros2_ws
source /opt/ros/<distro>/setup.bash
colcon build --packages-select simple_nodes_demo
source install/setup.bash
```

## Run
Open **three terminals** (or use a launch file later):

**Terminal 1 – Publisher**
```bash
source install/setup.bash
ros2 run simple_nodes_demo battery_publisher
```

**Terminal 2 – Processor**
```bash
source install/setup.bash
ros2 run simple_nodes_demo battery_processor
```

**Terminal 3 – C++ Listener**
```bash
source install/setup.bash
ros2 run simple_nodes_demo battery_listener
```

## Useful commands while running
```bash
ros2 topic list
ros2 topic echo /battery_level
ros2 topic echo /battery_warning
ros2 node list
```

## Learning notes
See `LEARNING_NOTES.md` for a line-by-line explanation of every important concept.
