# stereo_eye_launcher

A launch-only ROS 2 package. It starts a **Gazebo Sim (Harmonic)** simulation of the
`stereo_eye` robot (from MechaX) inside the `apartment_gazebo` world (from Isekai).

```
stereo_eye_launcher/
├── CMakeLists.txt     # ament_cmake, installs launch/ into share/
├── package.xml        # runtime dependencies
├── README.md
└── launch/
    └── launch.py      # the simulation launch file
```

## Requirements

- ROS 2 (Humble / Iron / Jazzy) with Gazebo Harmonic (gz-sim 8)
- `ros_gz_sim`, `ros_gz_bridge`, `robot_state_publisher`, `xacro`
- The `stereo_eye` and `apartment_gazebo` packages in the same workspace

## Build and run

```bash
mkdir -p ~/ws/src
cp -r stereo_eye apartment_gazebo stereo_eye_launcher ~/ws/src/
cd ~/ws
colcon build --packages-up-to stereo_eye_launcher
source install/setup.bash
ros2 launch stereo_eye_launcher launch.py
```

Sourcing `install/setup.bash` is required so `FindPackageShare` can find the robot
and world packages.

## Launch arguments

| Argument       | Default                                         | Meaning                                  |
|----------------|-------------------------------------------------|------------------------------------------|
| `use_sim_time` | `true`                                          | Nodes use the Gazebo `/clock`            |
| `world`        | `<apartment_gazebo share>/worlds/apartment.sdf` | Full path to the world SDF               |
| `x`            | `4.00`                                          | Spawn x position (m)                     |
| `y`            | `5.50`                                          | Spawn y position (m)                     |
| `z`            | `0.05`                                          | Spawn height (m), small drop onto floor  |
| `yaw`          | `0.0`                                           | Spawn heading (rad), faces +X            |

Example: `ros2 launch stereo_eye_launcher launch.py x:=3.0 y:=4.5 yaw:=1.57`

## What launch.py does, step by step

1. **Resolves paths** with `FindPackageShare`: the world
   (`apartment_gazebo/worlds/apartment.sdf`), its models folder, the robot xacro
   (`stereo_eye/urdf/stereo_eye.urdf.xacro`), and `ros_gz_sim`'s `gz_sim.launch.py`.

2. **Sets model paths.** Appends `apartment_gazebo/models` to `GZ_SIM_RESOURCE_PATH`
   (and `GAZEBO_MODEL_PATH` for compatibility) so the world's `model://` URIs
   (walls, sofa, bed, coffee table, kitchen counter, vanity) resolve.

3. **Starts Gazebo** by including `gz_sim.launch.py` with
   `gz_args = "-r -v 1 <world>"` (`-r` runs immediately, `-v 1` keeps logs quiet)
   and `on_exit_shutdown = true`, so closing Gazebo stops the whole launch.

4. **Builds the robot description** by running
   `xacro stereo_eye.urdf.xacro use_gazebo:=true`. The `use_gazebo` flag pulls in
   `stereo_eye.gazebo.xacro`, which adds the gz-sim DiffDrive and
   JointStatePublisher plugins plus the stereo cameras and IMU.

5. **robot_state_publisher** publishes the robot's TF tree from that description,
   using sim time.

6. **Spawns the robot** with `ros_gz_sim create`, reading the `robot_description`
   topic, as entity `stereo_eye` at (`x`, `y`, `z`, `yaw`). The default pose is in
   the living room, inside free space x 0.15 to 5.93 m and y 3.58 to 7.85 m, clear of
   the sofa (at 2.4, 7.2) and coffee table (at 2.4, 6.0).

7. **Bridges topics** between Gazebo and ROS with `ros_gz_bridge parameter_bridge`.
   `[` means Gazebo to ROS, `]` means ROS to Gazebo:

   | ROS topic                    | Type                        | Direction     |
   |------------------------------|-----------------------------|---------------|
   | `/clock`                     | `rosgraph_msgs/Clock`       | Gazebo to ROS |
   | `/stereo/left/image_raw`     | `sensor_msgs/Image`         | Gazebo to ROS |
   | `/stereo/left/camera_info`   | `sensor_msgs/CameraInfo`    | Gazebo to ROS |
   | `/stereo/right/image_raw`    | `sensor_msgs/Image`         | Gazebo to ROS |
   | `/stereo/right/camera_info`  | `sensor_msgs/CameraInfo`    | Gazebo to ROS |
   | `/imu`                       | `sensor_msgs/Imu`           | Gazebo to ROS |
   | `/odom`                      | `nav_msgs/Odometry`         | Gazebo to ROS |
   | `/tf`                        | `tf2_msgs/TFMessage`        | Gazebo to ROS |
   | `/joint_states`              | `sensor_msgs/JointState`    | Gazebo to ROS |
   | `/cmd_vel`                   | `geometry_msgs/Twist`       | ROS to Gazebo |

## Quick checks after launch

```bash
ros2 topic list
ros2 topic hz /stereo/left/image_raw
ros2 run teleop_twist_keyboard teleop_twist_keyboard   # drive with /cmd_vel
rviz2                                                   # view TF and camera images
```

## Known limitations

- **Sensors may not publish yet.** `apartment.sdf` has no gz-sim system plugins. The
  cameras and IMU need the `gz-sim-sensors-system` (render engine `ogre2`) and
  `gz-sim-imu-system` plugins in the world. If `/stereo/*` or `/imu` are silent, add
  those to the `<world>` in `apartment.sdf`.
- **Topic prefixes.** Topic names in the robot xacro are relative. If Gazebo publishes
  them under a prefix (for example `/model/stereo_eye/...`), check with
  `gz topic -l` and update the bridge arguments in `launch.py`.
- Not built or run on real ROS yet. The launch file compiles, the xacro expands, and
  the world SDF parses.
