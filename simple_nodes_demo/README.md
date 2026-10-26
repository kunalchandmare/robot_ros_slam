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

---

## Running inside the Dev Container (ROS 2 Humble)

Your project uses a **VS Code Dev Container** (`.devcontainer`). Follow these steps:

### 1. Rebuild the container (only needed once after Dockerfile change)
In VS Code:
- Open the Command Palette (`Ctrl+Shift+P`)
- Run: **Dev Containers: Rebuild Container**

(This installs the extra packages needed for the demo.)

### 2. Open a terminal inside the container
The workspace is mounted at `/workspaces/ros2-dev`.

```bash
cd /workspaces/ros2-dev
source /opt/ros/humble/setup.bash
```

### 3. Build the package
```bash
colcon build --packages-select simple_nodes_demo
source install/setup.bash
```

### 4. Run the nodes
Open **three terminals** inside the container (VS Code terminal → + button, or split):

**Terminal 1 – Publisher**
```bash
source /workspaces/ros2-dev/install/setup.bash
ros2 run simple_nodes_demo battery_publisher
```

**Terminal 2 – Processor**
```bash
source /workspaces/ros2-dev/install/setup.bash
ros2 run simple_nodes_demo battery_processor
```

**Terminal 3 – C++ Listener**
```bash
source /workspaces/ros2-dev/install/setup.bash
ros2 run simple_nodes_demo battery_listener
```

### 5. Inspect topics (optional fourth terminal)
```bash
source /workspaces/ros2-dev/install/setup.bash
ros2 topic list
ros2 topic echo /battery_level
ros2 topic echo /battery_warning
ros2 node list
```

---

## Learning notes
See `LEARNING_NOTES.md` for a clear explanation of every important concept.
