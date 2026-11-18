# stereo_eye_launcher

A launch-only ROS 2 package. It starts a **Gazebo Sim (Fortress)** simulation of the
`stereo_eye` robot (from MechaX) inside the `apartment_gazebo` world (from Isekai),
then runs stereo visual odometry and RTAB-Map stereo SLAM.

The repo's `.devcontainer/Dockerfile` is `FROM ros:humble-ros-base` and installs
`ros-humble-ros-gz`, which is built against **Gazebo Fortress** (ign-gazebo 6).
`gz_sim.launch.py` therefore runs `ign gazebo`. Comments and docs in earlier
revisions said "Harmonic"; that was incorrect for this container. The same
`gz_args`, `gz.msgs.*` bridge types, and `gz-sim-*` plugin tags also work on
Harmonic (`ros-humble-ros-gzharmonic`), but Fortress is what this project
actually installs.

```
stereo_eye_launcher/
├── CMakeLists.txt     # ament_cmake, installs launch/ and rviz/ into share/
├── package.xml        # runtime dependencies
├── README.md
├── launch/
│   └── launch.py      # the simulation + RTAB-Map + RViz launch file
└── rviz/
    └── stereo_slam.rviz
```

## Requirements

- ROS 2 Humble with Gazebo Fortress (the default `ros-humble-ros-gz` pairing)
- `ros_gz_sim`, `ros_gz_bridge`, `robot_state_publisher`, `xacro`
- `rtabmap_odom` and `rtabmap_slam` (from `ros-humble-rtabmap-ros`, already in the Dockerfile)
- The `stereo_eye`, `apartment_gazebo`, and `camera_brightness` packages in the same workspace

## Build and run

The repo root is the container workspace `/workspaces/ros2-dev`, and the packages
are already under `src/`. Build and launch from there:

```bash
cd /workspaces/ros2-dev
colcon build --packages-up-to stereo_eye_launcher
source /workspaces/ros2-dev/install/setup.bash
ros2 launch stereo_eye_launcher launch.py
```

Sourcing `/workspaces/ros2-dev/install/setup.bash` is required so
`FindPackageShare` can find the robot and world packages.

## Launch arguments

| Argument       | Default                                         | Meaning                                  |
|----------------|-------------------------------------------------|------------------------------------------|
| `use_sim_time` | `true`                                          | Nodes use the Gazebo `/clock`            |
| `world`        | `<apartment_gazebo share>/worlds/apartment.sdf` | Full path to the world SDF               |
| `x`            | `4.00`                                          | Spawn x position (m)                     |
| `y`            | `5.50`                                          | Spawn y position (m)                     |
| `z`            | `0.05`                                          | Spawn height (m), small drop onto floor  |
| `yaw`          | `0.0`                                           | Spawn heading (rad), faces +X            |
| `rviz`         | `true`                                          | Start RViz2 with `stereo_slam.rviz`      |

Example: `ros2 launch stereo_eye_launcher launch.py x:=3.0 y:=4.5 yaw:=1.57`

## What launch.py does, step by step

1. **Resolves paths** with `FindPackageShare`: the world
   (`apartment_gazebo/worlds/apartment.sdf`), its models folder, the robot xacro
   (`stereo_eye/urdf/stereo_eye.urdf.xacro`), and `ros_gz_sim`'s `gz_sim.launch.py`.

2. **Sets model paths.** Appends `apartment_gazebo/models` to
   `IGN_GAZEBO_RESOURCE_PATH` (Fortress / `ign gazebo`), `GZ_SIM_RESOURCE_PATH`
   (Harmonic), and `GAZEBO_MODEL_PATH` so the world's `model://` URIs
   (walls, sofa, bed, coffee table, kitchen counter, vanity) resolve.

3. **Starts Gazebo** by including `gz_sim.launch.py` with
   `gz_args = "-r -v 1 <world>"` (`-r` runs immediately, `-v 1` keeps logs quiet)
   and `on_exit_shutdown = true`, so closing Gazebo stops the whole launch.
   On Humble/Fortress this starts `ign gazebo`.

4. **Builds the robot description** by running
   `xacro stereo_eye.urdf.xacro use_gazebo:=true`. The `use_gazebo` flag pulls in
   `stereo_eye.gazebo.xacro`, which adds the gz-sim DiffDrive and
   JointStatePublisher plugins plus the stereo cameras and IMU.

5. **robot_state_publisher** publishes the robot's TF tree from that description,
   using sim time: `base_footprint` → `base_link` → … → `camera_*_optical_frame`.

6. **Spawns the robot** with `ros_gz_sim create`, reading the `robot_description`
   topic, as entity `stereo_eye` at (`x`, `y`, `z`, `yaw`). The default pose is in
   the living room, inside free space x 0.15 to 5.93 m and y 3.58 to 7.85 m, clear of
   the sofa (at 2.4, 7.2) and coffee table (at 2.4, 6.0).

7. **Bridges topics** between Gazebo and ROS with `ros_gz_bridge parameter_bridge`.
   `[` means Gazebo to ROS, `]` means ROS to Gazebo. Every node is launched with
   `use_sim_time` true. DiffDrive `/tf` is **not** bridged (see TF ownership below).
   DiffDrive `/odom` is remapped to `/wheel_odom` so it does not collide with
   stereo visual odometry:

   | ROS topic                    | Type                        | Direction     |
   |------------------------------|-----------------------------|---------------|
   | `/clock`                     | `rosgraph_msgs/Clock`       | Gazebo to ROS |
   | `/stereo/left/image_raw`     | `sensor_msgs/Image`         | Gazebo to ROS |
   | `/stereo/left/camera_info`   | `sensor_msgs/CameraInfo`    | Gazebo to ROS |
   | `/stereo/right/image_raw`    | `sensor_msgs/Image`         | Gazebo to ROS |
   | `/stereo/right/camera_info`  | `sensor_msgs/CameraInfo`    | Gazebo to ROS |
   | `/imu`                       | `sensor_msgs/Imu`           | Gazebo to ROS |
   | `/wheel_odom`                | `nav_msgs/Odometry`         | Gazebo to ROS (DiffDrive `/odom`) |
   | `/joint_states`              | `sensor_msgs/JointState`    | Gazebo to ROS |
   | `/cmd_vel`                   | `geometry_msgs/Twist`       | ROS to Gazebo |

8. **stereo_odometry** (`rtabmap_odom`) and **rtabmap** (`rtabmap_slam`) subscribe
   to the stereo pair with `subscribe_stereo` true, `subscribe_depth` false,
   `frame_id` `base_footprint`, and `Reg/Force3DoF` true. The sim images are
   distortion-free, so they are already rectified: `left/image_rect` and
   `right/image_rect` (and the matching `camera_info` topics) are remapped to
   `/stereo/{left,right}/image_raw` and `/stereo/{left,right}/camera_info`.
   Exact sync is used (`approx_sync` false) because both cameras share the same
   update rate and are rendered in one Sensors-system step, so their stamps
   match. rtabmap takes `/odom` from stereo_odometry (plus `odom_info`) and
   waits up to 0.2 s for TF.

9. **camera_brightness** is remapped from `/image` to `/stereo/left/image_raw`.

## TF ownership

Only one source may publish `odom` → `base_footprint`. This launch gives that
transform to **stereo_odometry** (`publish_tf` true) and lets **rtabmap** publish
`map` → `odom`. DiffDrive still publishes wheel odometry on Gazebo `/odom`, but
the bridge remaps it to `/wheel_odom` and does not bridge `/tf`.
`robot_state_publisher` remains the source of `base_footprint` → camera frames.

## RViz

Do **not** run a bare `rviz2`. A plain RViz session uses wall time, while every
TF, image, map, and odometry message here is stamped in sim time (`/clock`).
Its default Fixed Frame is also `map`, which does not exist until `rtabmap`
publishes the `map` → `odom` transform, so the view stays empty.

RViz starts automatically with this launch file, using
`rviz/stereo_slam.rviz` and `use_sim_time:=true`. Disable it with
`rviz:=false`:

```bash
ros2 launch stereo_eye_launcher launch.py rviz:=false
```

If you start it manually, load the config and enable sim time:

```bash
rviz2 -d $(ros2 pkg prefix stereo_eye_launcher)/share/stereo_eye_launcher/rviz/stereo_slam.rviz --ros-args -p use_sim_time:=true
```

The config uses Fixed Frame `odom` (available as soon as stereo odometry
publishes TF) and these displays:

| Display     | Topic                      | Notes |
|-------------|----------------------------|-------|
| TF          | —                          | Full tree: `odom` → `base_footprint` → cameras; `map` → `odom` once SLAM starts |
| RobotModel  | `/robot_description`       | Description Source is Topic, durability Transient Local (matches `robot_state_publisher`) |
| Image       | `/stereo/left/image_raw`   | Reliability Best Effort |
| Image       | `/stereo/right/image_raw`  | Reliability Best Effort |
| Map         | `/map`                     | OccupancyGrid from `rtabmap`; durability Transient Local |
| PointCloud2 | `/cloud_map`               | Assembled 3D map from `rtabmap` |
| Odometry    | `/odom`                    | Stereo visual odometry (not `/wheel_odom`) |

`/map` and `/cloud_map` are used (not `/rtabmap/map` or `/rtabmap/cloud_map`)
because `launch.py` starts the `rtabmap` node with no namespace and no remaps
for those outputs. On Humble `rtabmap_ros` 0.21+ the node publishes the
relative names `map` (OccupancyGrid) and `cloud_map` (PointCloud2), which
resolve to `/map` and `/cloud_map`. Older ROS 1 names such as `/rtabmap/grid_map`
or `proj_map` do not apply here.

## Quick checks after launch

```bash
ros2 topic list
ros2 topic hz /stereo/left/image_raw
ros2 topic hz /odom
ros2 topic echo --once /stereo/right/camera_info --field p   # expect P[3] = -54.96
ros2 run tf2_ros tf2_echo map base_footprint
ros2 run teleop_twist_keyboard teleop_twist_keyboard   # drive with /cmd_vel
```

Do not start a bare `rviz2` from this list. Use the launch file's RViz node, or
see the RViz section below if you need to start it by hand.

## Known limitations

- **Sensors may not publish yet.** `apartment.sdf` has no gz-sim system plugins. The
  cameras and IMU need the `gz-sim-sensors-system` (render engine `ogre2`) and
  `gz-sim-imu-system` plugins in the world, plus Physics, UserCommands and
  SceneBroadcaster (declaring any world plugin disables the default set). If
  `/stereo/*` or `/imu` are silent, add those to the `<world>` in `apartment.sdf`.
  On Fortress the `gz-sim-*` filenames map to `ignition-gazebo-*`.
- **Topic prefixes.** Topic names in the robot xacro are relative. If Gazebo publishes
  them under a prefix (for example `/model/stereo_eye/...`), check with
  `gz topic -l` (or `ign topic -l` on Fortress) and update the bridge arguments
  in `launch.py`.
- **Plain walls.** Untextured surfaces give stereo matching few features, so visual
  odometry and loop closures can be weak until the world has more visual texture.
- Not built or run on real ROS in the environment that last edited this package.
  The launch file compiles and `package.xml` parses; a full `colcon` build and
  Gazebo launch was not verified here.
