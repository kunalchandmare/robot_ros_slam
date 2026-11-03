# ROS Visualization & Simulation Tools Summary

A practical overview of **Gazebo**, **RViz**, and tools for visualizing **node/topic connections**.

---

## 1. Gazebo

### What it is
Physics-based robot **simulator**. It creates a virtual 3D world with gravity, collisions, and realistic physics.

### Main use cases
- Test robot algorithms **before** deploying on real hardware
- Simulate sensors (LiDAR, cameras, IMU, GPS, etc.)
- Test navigation, manipulation, and control in a safe virtual environment
- Generate synthetic sensor data for development and testing
- Train and validate systems when real robots are unavailable

### Key characteristics
- **Produces data** (sensor readings, odometry, joint states, etc.)
- Has its own 3D visualization of the simulated world
- Integrates with ROS via bridges (e.g., `ros_gz_bridge`)
- Independent of ROS (can run standalone)
- Current recommended version for ROS 2: **Gazebo Sim** (formerly Ignition), e.g. Harmonic

### Typical output
Publishes ROS topics such as:
- `/scan` (LiDAR)
- `/camera/image` or `/camera/image_raw`
- `/odom`
- `/joint_states`
- `/clock` (simulation time)

---

## 2. RViz / RViz2

### What it is
ROS **visualization tool**. It displays data that already exists on ROS topics.

### Main use cases
- Visualize robot model (from URDF)
- Display sensor data (laser scans, point clouds, camera images)
- Show maps, costmaps, and planned paths (especially with Nav2)
- Visualize TF frames (coordinate systems)
- Debug navigation, localization, and perception
- Interact with the system (set 2D Pose Estimate, 2D Nav Goal)
- Monitor robot state on real hardware or in simulation

### Key characteristics
- **Consumes data** (subscribes to topics)
- No physics simulation
- Runs as a ROS node
- Excellent interactive markers and plugin ecosystem
- Free and open source
- Requires a ROS environment

### Common displays
- RobotModel
- TF
- LaserScan / PointCloud2
- Map / OccupancyGrid
- Path
- Image
- Marker / MarkerArray

---

## 3. How Gazebo and RViz Work Together

They are **complementary**, not alternatives.

### Typical workflow

```
Gazebo (Simulation)
    ↓ publishes sensor data, odometry, TF
ROS Topics
    ↓ subscribed by
RViz (Visualization)
    ↓ shows robot, sensors, maps, paths
Developer / Algorithm
```

### Practical example (Navigation)
1. Launch Gazebo with a robot in a world (simulates physics + sensors).
2. Launch Nav2 (or other algorithms).
3. Launch RViz.
4. In RViz:
   - Add Map display (`/map`)
   - Add LaserScan (`/scan`)
   - Add Path (`/plan`)
   - Use **2D Nav Goal** tool to send a goal
5. Watch the robot navigate in both Gazebo (physical movement) and RViz (sensor data + planned path).

### Summary table

| Aspect              | Gazebo                          | RViz / RViz2                     |
|---------------------|---------------------------------|----------------------------------|
| Role                | Simulator (data producer)       | Visualizer (data consumer)       |
| Physics             | Yes                             | No                               |
| Sensors             | Simulates them                  | Visualizes their data            |
| Needs real robot    | No                              | Works with real or simulated     |
| Best for            | Testing before real hardware    | Debugging and understanding data |
| Cost                | Free (open source)              | Free (open source)               |

---

## 4. Node & Topic Connection Visualization Tools

These tools show **which nodes exist** and **how they communicate** via topics (and sometimes services/actions).

### 4.1 rqt_graph (Classic Standard)

| Item          | Details                                      |
|---------------|----------------------------------------------|
| Type          | Graphical ROS graph viewer                   |
| Shows         | Nodes (ovals), Topics (rectangles), arrows   |
| Cost          | Free (part of ROS / rqt)                     |
| Platform      | Desktop (requires ROS environment)           |
| Best for      | Quick view of live node-topic connections    |
| How to run    | `ros2 run rqt_graph rqt_graph` or `rqt_graph`|

**Use case**: Instantly see which nodes are publishing and subscribing to which topics while the system is running. Excellent for debugging communication issues.

**Features**:
- Highlight related nodes/topics on hover
- Filter modes (active only, all possible, nodes only)
- Hide unused topics (`/rosout`, `/parameter_events`, etc.)

---

### 4.2 Foxglove

| Item          | Details                                              |
|---------------|------------------------------------------------------|
| Type          | Modern multimodal visualization & data platform      |
| Shows         | Topic Graph panel + 3D, images, plots, raw messages, etc. |
| Cost          | **Free tier** + paid plans                           |
| Platform      | Desktop app + Web browser                           |
| Best for      | Live + recorded data, remote viewing, team collaboration |
| Connection    | Foxglove Bridge (WebSocket) or open bag/MCAP files   |

**Use case**: Modern alternative that combines node/topic graph with rich visualization panels in one place. Strong for bag playback and remote debugging.

#### Foxglove Cost Structure (2026)

| Plan         | Price              | Key limits                                      |
|--------------|--------------------|-------------------------------------------------|
| **Free**     | $0                 | 3 Developer Seats, 10 GB storage, 5 devices     |
| **Pro**      | Starts at $20/mo   | 1 TB storage + 3 Developer Seats included       |
| **Academic** | Free               | For `.edu` / `.ac` emails                       |
| **Enterprise**| Custom            | Advanced security, on-prem, support             |

- Free plan includes full visualization (including Topic Graph).
- Pro includes unlimited free **Basic Seats** (view-only).
- Usage-based pricing beyond included limits (storage, query, bandwidth).

---

### 4.3 Other Tools

| Tool                  | Type                    | Cost     | Brief use case                              |
|-----------------------|-------------------------|----------|---------------------------------------------|
| **ros2_graph**        | Mermaid diagram generator | Free   | Generate documentation diagrams (topics + services + actions) |
| **PlotJuggler**       | Time-series plotter     | Free     | Advanced plotting of topic data over time   |
| **rqt_topic**         | Topic monitor           | Free     | See active topics and message rates         |
| **rqt_console**       | Log viewer              | Free     | View ROS log messages                       |
| **Static analyzers**  | Code analysis           | Free     | Graph nodes/topics from source without running |

---

## 5. Recommended Tool Combinations

| Scenario                          | Recommended tools                          |
|-----------------------------------|--------------------------------------------|
| Local development & debugging     | Gazebo + RViz + rqt_graph                  |
| Navigation / SLAM testing         | Gazebo + RViz (with Nav2 config)           |
| Real robot debugging              | RViz + rqt_graph                           |
| Modern team workflow / remote     | Foxglove (live + bags) + RViz (local interactive) |
| Documentation                     | rqt_graph screenshots or ros2_graph        |
| Time-series analysis              | PlotJuggler or Foxglove plots              |

---

## 6. Quick Decision Guide

| I want to...                              | Use                        |
|-------------------------------------------|----------------------------|
| Simulate a robot in a virtual world       | **Gazebo**                 |
| See sensor data, maps, and paths in 3D    | **RViz**                   |
| See which nodes talk to which topics      | **rqt_graph**              |
| Modern all-in-one visualization + playback| **Foxglove**               |
| Work without a real robot                 | **Gazebo + RViz**          |
| Debug a real robot                        | **RViz + rqt_graph**       |
| Share views with teammates remotely       | **Foxglove**               |

---

## 7. Summary

- **Gazebo** = Simulator that creates a virtual world and generates data.
- **RViz** = Visualizer that displays ROS data in 3D.
- They work together: Gazebo produces data → RViz consumes and displays it.
- **rqt_graph** = Simple free tool for live node/topic connections.
- **Foxglove** = Modern platform with Topic Graph + many other panels; free tier available, Pro starts at $20/month.
- Most professional teams use **Gazebo + RViz + rqt_graph** for development, and increasingly add **Foxglove** for better playback, remote, and team workflows.

---

*Document generated for practical ROS development reference.*
