# Stereo Eye: stereo camera pair design (RTAB-Map stereo SLAM and navigation)

This document covers the **2-camera stereo pair** that forms Stereo Eye's single "eye": geometry, camera specs, topics,
how the baseline reaches `camera_info`, the IMU, and what the simulation and launch packages need to do to use it.
Only `src/stereo_eye` was changed. `src/stereo_eye_launcher` and `src/apartment_gazebo` are **not** edited. Section 8 lists
what they may need.

## 1. Target platform (detected from the repo)

| Evidence | Finding |
|---|---|
| `.devcontainer/Dockerfile`: `FROM ros:humble-ros-base`, `ros-humble-*` packages; `devcontainer.json` name "ROS 2 Humble" | **ROS 2 Humble** |
| Dockerfile installs `ros-humble-ros-gz` (apt) and **no** `ros-humble-ros-gzharmonic` / `gz-harmonic` | The Humble `ros_gz` binaries are built against **Gazebo Fortress** (ign-gazebo 6) by default (`ros_gz_sim/CMakeLists.txt`: "Default to Fortress"). `gz_sim.launch.py` then runs `ign gazebo` (`gz_version` 6) |
| `stereo_eye_launcher/launch/launch.py` uses `ros_gz_sim` `gz_sim.launch.py` + `create` + `ros_gz_bridge parameter_bridge` with `gz.msgs.*` types; comments say "Gazebo Sim (Harmonic)" | It's **gz sim (new Gazebo), not Classic**. The "Harmonic" wording conflicts with the Dockerfile (see open question below) |
| `apartment_gazebo`: SDF 1.7 world, `ros_gz_sim` exec dependency, no world system plugins | Works on Fortress and Harmonic. Sensors need world systems (section 8) |
| `stereo_eye.gazebo.xacro` (before the stereo change) | `gz-sim-*` plugin filenames, `gz::sim::systems::*` names, `<gz_frame_id>` only (now both frame tags, see below) |

**Decision: write tags that work on both Fortress (what the container actually installs) and Harmonic.** Upstream source confirms:

* **Plugin filenames.** Fortress's `SystemLoader` rewrites the prefix `gz-sim` → `ignition-gazebo`, and Fortress registers the
  `gz::sim::systems::DiffDrive` / `JointStatePublisher` aliases. So the existing `gz-sim-*` / `gz::sim::*` plugins load on both.
* **Frame IDs: version-dependent, so every sensor sets both `<ignition_frame_id>` and `<gz_frame_id>`** (same value).
  Checked against the gz-sensors release tags:

  | gz-sensors version | Tag(s) read for the sensor frame | Source |
  |---|---|---|
  | ign-sensors 6.0.0 – 6.1.0 (Ubuntu jammy *universe* ships 6.1.0) | **none**: image/camera_info frame_id = sensor name (`stereo_left`) | `src/Sensor.cc` L131 "todo how to use frame?"; `src/CameraSensor.cc` L403, L609 `add_value(this->Name())` @ `ignition-sensors6_6.1.0` |
  | ign-sensors 6.2.0 – 6.7.1 | `<ignition_frame_id>` only | `src/Sensor.cc` L148-150 @ `ignition-sensors6_6.6.0` |
  | ign-sensors 6.8.0+ (packages.ros.org ships **6.9.0** for Humble today) | `<ignition_frame_id>` preferred, `<gz_frame_id>` fallback; warns if both are present | `src/Sensor.cc` L148-166 @ `ignition-sensors6_6.9.0` ([link](https://github.com/gazebosim/gz-sensors/blob/ignition-sensors6_6.9.0/src/Sensor.cc#L148-L166)); added by commit ff5bbb1 "Allow specifying gz_frame_id as an alternative to ignition_frame_id (#409)", 2024-01-12 |
  | gz-sensors 8 (Harmonic) | `<gz_frame_id>` only | `src/Sensor.cc` L188-190 @ `gz-sensors8` ([link](https://github.com/gazebosim/gz-sensors/blob/gz-sensors8/src/Sensor.cc#L188-L190)) |

  gz-sim's Sensors system (`ign-gazebo6`) doesn't handle frame ids. It passes the sensor SDF to gz-sensors.
  With both tags, ign-sensors 6.8+ logs one harmless warning per sensor ("Found both `ignition_frame_id` and `gz_frame_id` ...
  Only `ignition_frame_id` will be used"). Both carry the same frame, so the result is the same.
  **Minimum: ign-sensors 6.2.0.** Below that, image and camera_info frame_ids are the sensor names and RTAB-Map's TF lookups fail.
  The repo's container (`ros:humble` + packages.ros.org) gets 6.9.0. Make sure it isn't pinned to Ubuntu's 6.1.0.
* **Camera info.** `<camera_info_topic>` and `<lens><intrinsics>/<projection>` are SDFormat 1.7+ (sdformat12 for Fortress, sdformat14 for Harmonic).
* **Bridge.** `ros_gz_bridge` (humble branch) accepts both `gz.msgs.*` and `ignition.msgs.*` type strings, so the launcher's bridge arguments are valid.
* **Not Classic.** Gazebo Classic tags (`libgazebo_ros_camera.so`, `multicamera`, `hack_baseline`) are **not** used. They would be ignored by gz sim.

> **Open question:** the launcher README says "Gazebo Sim (Harmonic)", but the container installs Fortress. Either install
> Harmonic for Humble (`ros-humble-ros-gzharmonic` from packages.osrfoundation.org and `gz-harmonic`), or update the
> launcher wording. The robot description works either way.

## 2. Frames

```
base_footprint                                   (ground, under the axle centre; RTAB-Map frame_id)
└── base_link                                    (axle height, z = 0.065)
    ├── left_wheel_link / right_wheel_link       (continuous; DiffDrive)
    ├── caster_link                              (rear ball)
    ├── imu_link                                 (z = +0.01 on the axle line)
    └── head_link                                (z = +0.1925)
        └── stereo_bar_link                      (fixed, x = +0.061, z = +0.005: RIGID STEREO BAR, lens front plane)
            ├── camera_link                      (identity; stereo-pair centre, x forward, massless)
            ├── camera_left_link                 (y = +0.040, x forward; gz sensor "stereo_left")
            │   └── camera_left_optical_frame    (rpy -pi/2 0 -pi/2: z fwd, x right, y down, REP-103)
            └── camera_right_link                (y = -0.040, x forward; gz sensor "stereo_right")
                └── camera_right_optical_frame   (rpy -pi/2 0 -pi/2)
```
At runtime, `odom → base_footprint` comes from the DiffDrive plugin (bridged `/tf`), and `map → odom` comes from RTAB-Map.

* **The bar is the only parent of both cameras.** There is one rigid body and no joint between the cameras, so the
  relative pose (the stereo extrinsics) is fixed by construction. The bar carries the 0.06 kg camera mass (box
  0.025 × 0.11 × 0.03 m) and the two lens visuals.
* **Lens front height:** both optical centres are **0.2625 m above the ground** and 0.061 m ahead of the axle, level (0° pitch).
* **Validated optical frame axes:** in `base_footprint`, x = (0, −1, 0) (right), y = (0, 0, −1) (down), z = (1, 0, 0) (forward).
  The right camera sits at **(+0.080, 0, 0)** in the left optical frame. That's the standard ROS stereo layout: right camera at +x of the left camera.
* **Unchanged:** `base_footprint` / `base_link`, wheels, DiffDrive, caster, IMU, masses (3.16 kg total) and the other links.
  The old `head_link → camera_link → camera_*` chain became `head_link → stereo_bar_link → {camera_link, camera_left_link, camera_right_link}`.

## 3. Camera specification

| Parameter | Value | Notes |
|---|---|---|
| Sensors | 2 × gz `camera` (`stereo_left`, `stereo_right`) | gz sim has no `multicamera` sensor. Sync is explained below |
| Baseline B | **0.080 m** | Justification below |
| Resolution | **1280 × 720 per eye** (R8G8B8 → `rgb8`) | Meets the ≥ 720p requirement |
| Frame rate | **15 Hz**, both cameras | See the FPS rationale below |
| HFOV / VFOV | 1.5 rad (85.9°) / 0.965 rad (55.3°) | Wide enough for features and loop closures, but narrower than before to raise fx |
| Intrinsics (both) | fx = fy = 640 / tan(0.75) = **686.99 px**, cx = 640, cy = 360, skew 0 | Identical K and P for left and right |
| Projection Tx | left 0, right **−fx·B = −54.96** | `camera_info.P[3]` (section 5) |
| Distortion / rectification | none / R = identity | Sim images are ideal pinhole, i.e. **already rectified** |
| Clip | 0.05 m to 15 m | The apartment is 10 × 8 m |
| Noise | Gaussian σ = 0.005 (both) | |
| Image frame_id | `camera_left_optical_frame` / `camera_right_optical_frame` (via `<ignition_frame_id>` + `<gz_frame_id>`) | Also used by camera_info (ign-sensors 6.2.0+) |

**Why an 80 mm baseline (allowed range 6-12 cm) for an indoor apartment.** Depth comes from disparity d = fx·B / Z.
The depth error for a matching error δd ≈ 0.25 px is δZ = Z²·δd / (fx·B). The closest measurable depth for a disparity
search of D px is Z_min = fx·B / D. With fx = 686.99:

| B | Z_min (D=128, RTAB-Map default) | Z_min (D=160) | δZ @ 1 m | δZ @ 3 m | δZ @ 5 m | disparity @ 5 m |
|---|---|---|---|---|---|---|
| 0.060 | 0.32 m | 0.26 m | 0.6 cm | 5.5 cm | 15.2 cm | 8.2 px |
| 0.063 (old) | 0.34 m | 0.27 m | 0.6 cm | 5.2 cm | 14.4 cm | 8.7 px |
| **0.080** | **0.43 m** | **0.34 m** | **0.5 cm** | **4.1 cm** | **11.4 cm** | **11.0 px** |
| 0.100 | 0.54 m | 0.43 m | 0.4 cm | 3.3 cm | 9.1 cm | 13.7 px |
| 0.120 | 0.64 m | 0.52 m | 0.3 cm | 2.7 cm | 7.6 cm | 16.5 px |

The rooms are at most about 5.8 m across (living room 5.77 × 4.27 m) and doorways are 0.9-1.6 m wide, so the robot
regularly sees obstacles at 0.4-1 m (furniture, door frames, corridor walls) and walls at 3-5 m.

* **80 mm cuts the far-range error by about 21% compared with 63 mm**: 11 cm instead of 14 cm at 5 m, with 11 px of disparity at the far wall.
* **The near blind zone stays small:** 0.34 m with `Stereo/MaxDisparity=160`, 0.43 m with the default 128.
* **A 10-12 cm baseline** would push Z_min to about 0.5-0.65 m. That's bad for 1 m doorways and close furniture. It would
  also reduce left/right overlap near the robot and not fit the head (flat face ±0.075 m) inside a single eye.
* **80 mm is between ZED Mini (63 mm) and OAK-D (75 mm) / ZED 2 (120 mm),** so a real camera with similar geometry exists.

**FPS rationale (15 Hz).** Two 1280×720 render passes per step are the largest load in the sim, and the devcontainer runs
under WSLg with limited GPU access. At 15 Hz and the DiffDrive limits (0.6 m/s, 2 rad/s), the robot moves at most 4 cm
and turns at most 7.6° between frames. That's well inside what stereo odometry and feature matching handle with an 86° HFOV.
RTAB-Map's SLAM loop runs at 1 Hz (`Rtabmap/DetectionRate`). Raise `cam_fps` to 30 if the machine keeps a real-time factor near 1.0.

**Synchronisation.** Both sensors have the same `<update_rate>` and are rendered in the same simulation step by the
`Sensors` system, so left and right images **carry identical header stamps (sim time)**. This is the gz sim equivalent of
Classic's single `multicamera` sensor. Exact sync can be used (`approx_sync:=false`). `approx_sync:=true` with a small
`approx_sync_max_interval` (e.g. 0.01 s) is a safe default if the bridge reorders messages under load.

## 4. Topics (gz → ROS, laid out for RTAB-Map stereo)

| gz / ROS topic | Type | frame_id |
|---|---|---|
| `/stereo/left/image_raw` | sensor_msgs/Image (rgb8, 1280×720) | `camera_left_optical_frame` |
| `/stereo/left/camera_info` | sensor_msgs/CameraInfo (P[3] = 0) | `camera_left_optical_frame` |
| `/stereo/right/image_raw` | sensor_msgs/Image | `camera_right_optical_frame` |
| `/stereo/right/camera_info` | sensor_msgs/CameraInfo (P[3] = −54.96) | `camera_right_optical_frame` |
| `/imu` | sensor_msgs/Imu, 100 Hz | `imu_link` |
| `/odom`, `/tf` (odom → base_footprint), `/joint_states` | DiffDrive / JointStatePublisher | |

These are the same names the launcher already bridges, so no topic renames are needed.

**`image_rect`.** RTAB-Map's stereo inputs are named `left/image_rect`, `right/image_rect`, `left/camera_info` and `right/camera_info`.
The sim images have no distortion (R = identity, P = K plus Tx), so they are already rectified. **Remap `image_raw` to
`image_rect`.** No `stereo_image_proc` is needed in sim. On real hardware, use the driver's rectified topics or run
`stereo_image_proc` / `image_proc` first.

## 5. Baseline in the right `camera_info` (Tx = −fx·B)

* **gz sim (Fortress and Harmonic): handled by the simulator plus the robot description, through two independent paths:**
  1. **gz-sim auto-baseline (works on every Fortress and Harmonic version).** The Sensors system notices when a second
     camera is created on a link that already has a camera. It uses the topic names ("left"/"right") to decide which is the
     right camera, computes `baseline = |y_right − y_left|` from the sensor poses, and calls `CameraSensor::SetBaseline()`, which sets
     `P[3] = −fx · baseline`.
     Sources: `src/systems/sensors/Sensors.cc` L831-866 @ `ignition-gazebo6_6.18.0` ([link](https://github.com/gazebosim/gz-sim/blob/ignition-gazebo6_6.18.0/src/systems/sensors/Sensors.cc#L831-L866)),
     present since `ignition-gazebo6_6.0.0` (L624-626); `gz-sim8` L1217-1219.
     `SetBaseline` writes P[3]: gz-sensors `src/CameraSensor.cc` L805-814 @ `ignition-sensors6_6.9.0`, L616-625 @ `ignition-sensors6_6.1.0`.
     **What this needs from our model:** after URDF → SDF fixed-joint lumping, both camera sensors end up on `base_link` at
     y = ±0.04, and their topics contain `left` / `right`. Our layout meets both conditions, so the auto-baseline is 0.080 m.
     Don't add a third camera sensor to the robot without re-checking this.
  2. **`<lens><projection><tx>` in SDF (gz-sensors version-dependent).** `CameraSensor::PopulateInfo` copies `<tx>` into
     `P[3]` on **ign-sensors 6.7.0+** (`src/CameraSensor.cc` L722 / L758 `proj->add_p(_cameraSdf->LensProjectionTx())` @ `ignition-sensors6_6.9.0`,
     [link](https://github.com/gazebosim/gz-sensors/blob/ignition-sensors6_6.9.0/src/CameraSensor.cc#L722-L770); commit d090f91 "Camera: configure projection matrix from SDFormat (#249)")
     and on gz-sensors 8 (L774). **ign-sensors ≤ 6.6.0 ignores `<tx>`.** There, `PopulateInfo` writes `−fx · baseline` from the member set by
     path 1 (`// TODO(anyone) Get tx and ty from SDF`, L571/L578 @ 6.1.0, L645/L652 @ 6.6.0).
     The xacro sets the right camera's `<tx>` = `−cam_fx·stereo_baseline` = **−54.96** (left 0), the same value path 1 computes, so
     the two paths agree on every version. Path 1 runs after `PopulateInfo` and has the last word.
  `ros_gz_bridge` copies P unchanged. Validated in the URDF: −Tx/fx = 0.0800 m = the TF baseline.
  (gz sim has no `hack_baseline`. It exists only in the Classic `libgazebo_ros_camera` plugin.)
* **Launcher-side check (required once):** `ros2 topic echo --once /stereo/right/camera_info --field p` should show
  `[686.99, 0, 640, -54.96, 0, 686.99, 360, 0, 0, 0, 1, 0]`. If it shows `p[3] = 0`, path 1 didn't trigger (e.g. the sensors
  ended up on different links, or the topics lost `left`/`right`) and the build is ign-sensors ≤ 6.6. RTAB-Map then fails with an invalid stereo baseline. The fallbacks below are launcher-side:
  1. **A small camera_info relay** (about 20 lines of rclpy) that subscribes to `/stereo/right/camera_info`, sets `p[3] = -p[0]*0.080`,
     and republishes on `/stereo/right/camera_info_fixed`. Remap RTAB-Map's `right/camera_info` to it.
  2. **Publish both camera_infos from a calibration YAML** with `camera_info_manager` (as a real stereo driver does) and ignore the gz ones.
  3. **rtabmap_odom / rtabmap_slam** read the baseline only from the right camera_info P (no parameter override). They also
     need the `camera_left_optical_frame → camera_right_optical_frame` TF to be consistent, and it is: 0.08 m along x.
* **Gazebo Classic (not used here):** this would be one `type="multicamera"` sensor with `libgazebo_ros_camera.so` and
  `<hack_baseline>0.08</hack_baseline>`, which writes Tx into the right camera_info.

## 6. IMU: does it help?

The repo's model already has `imu_link` (100 Hz, gyro σ 0.0009 rad/s, accel σ 0.017 m/s²) on the axle centreline, and
the launcher already bridges `/imu`. It's kept as is.

* **Wheel odometry mode (recommended for this robot):** fuse `/odom` + `/imu` with `robot_localization` (EKF; yaw rate from the
  gyro) to cut yaw drift from wheel slip, especially with a frictionless caster. RTAB-Map then consumes the filtered odom.
* **Visual odometry mode:** `rtabmap_odom/stereo_odometry` can take `/imu` (`subscribe_imu:=true`, `wait_imu_to_init:=true`) for a
  rotation prior during fast turns and low-texture moments. RTAB-Map can also add gravity constraints
  (`Optimizer/GravitySigma`, useful if `Reg/Force3DoF` is false).
* **For pure 2D mapping with `Reg/Force3DoF=true` and good wheel odometry,** the IMU is optional but cheap insurance.
  The world must load the `Imu` system for it to publish (section 8).

## 7. Real-hardware equivalence

The same frames and topic layout fit a real 80 mm stereo module (or e.g. OAK-D at 75 mm: change `stereo_baseline`).
Point the driver's TF frames at `camera_left_optical_frame` / `camera_right_optical_frame` (or publish a static TF from the
driver's frames to `stereo_bar_link`), and feed RTAB-Map the driver's **rectified** images.

## 8. What `stereo_eye_launcher` and `apartment_gazebo` may need (not edited here)

**apartment_gazebo/worlds/apartment.sdf**
* **Add world systems, or the cameras and IMU never publish.** The launcher README already lists this as a known
  limitation. Inside `<world>`:
  ```xml
  <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
  <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
  <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
  <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
  <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
  ```
  (On Fortress these `gz-sim-*` names map to `ignition-gazebo-*`. Declaring any world plugin disables the default set, so list Physics, UserCommands and SceneBroadcaster too.)
* **Visual texture.** Plain-colour walls give stereo matching and loop closure few features. Add textured materials, posters or
  pictures on walls, or more varied furniture. Otherwise expect sparse disparity on walls and weak loop closures.

**stereo_eye_launcher/launch/launch.py**
* **Bridge:** the existing entries already match (`/stereo/{left,right}/image_raw`, `/stereo/{left,right}/camera_info`, `/imu`,
  `/odom`, `/tf`, `/joint_states`, `/clock`, `/cmd_vel`). No change is needed. Optionally, move the image topics to `ros_gz_image image_bridge` for
  lower overhead, and check sensor QoS (best effort) on subscribers.
* **Wording / version:** "Harmonic" in the comments vs Fortress in the container (section 1).
* **Odom TF conflict:** if stereo visual odometry is used, stop bridging the DiffDrive `/tf` (or set `publish_tf:=false` on
  `stereo_odometry`), so `base_footprint` has only one parent.
* **Add RTAB-Map** (a new node block or a separate launch). Key parameters:
  ```python
  rtabmap_params = {
      'use_sim_time': True,
      'frame_id': 'base_footprint',
      'subscribe_stereo': True, 'subscribe_depth': False, 'subscribe_rgb': False,
      'approx_sync': False,            # identical stamps in sim; True + approx_sync_max_interval 0.01 is also fine
      'queue_size': 20,
      'odom_frame_id': '',             # use the /odom topic (wheel odom), or 'odom' to read TF
      'map_frame_id': 'map', 'publish_tf': True,
      'Reg/Force3DoF': 'true',         # planar robot
      'Stereo/MaxDisparity': '160',    # min depth about 0.34 m with B = 0.08, fx = 687
      'Grid/FromDepth': 'true',        # occupancy grid from stereo point cloud (for Nav2)
      'Grid/RangeMax': '4.0',
      'Grid/MaxGroundHeight': '0.05', 'Grid/MaxObstacleHeight': '0.40',   # robot is 0.34 m tall
      'Grid/RayTracing': 'true',
      'RGBD/LinearUpdate': '0.05', 'RGBD/AngularUpdate': '0.05',
  }
  remaps = [('left/image_rect', '/stereo/left/image_raw'),   ('left/camera_info', '/stereo/left/camera_info'),
            ('right/image_rect', '/stereo/right/image_raw'), ('right/camera_info', '/stereo/right/camera_info'),
            ('odom', '/odom')]
  # Node(package='rtabmap_slam', executable='rtabmap', parameters=[rtabmap_params], remappings=remaps, arguments=['-d'])
  # optional VO: Node(package='rtabmap_odom', executable='stereo_odometry', ...same remaps..., publish_tf=True)
  ```
  Or, with the stock launch file (`ros-humble-rtabmap-ros` is already in the Dockerfile):
  ```bash
  ros2 launch rtabmap_launch rtabmap.launch.py use_sim_time:=true stereo:=true frame_id:=base_footprint \
    left_image_topic:=/stereo/left/image_raw right_image_topic:=/stereo/right/image_raw \
    left_camera_info_topic:=/stereo/left/camera_info right_camera_info_topic:=/stereo/right/camera_info \
    visual_odometry:=false odom_topic:=/odom approx_sync:=false \
    rtabmap_args:="--delete_db_on_start --Reg/Force3DoF true --Stereo/MaxDisparity 160 --Grid/FromDepth true"
  ```
* **Navigation (Nav2):** use RTAB-Map's `/map` (occupancy grid from stereo) as the global map, `base_footprint` as the robot
  frame and `/cmd_vel` for the DiffDrive. A local costmap can use RTAB-Map's `/cloud_obstacles` or a `/stereo` point cloud.
* **camera_brightness:** that node subscribes to `/image`, which nothing publishes. Remap it to `/stereo/left/image_raw` if it should
  monitor the eye.

## 9. Validation (this change)

* `xacro stereo_eye.urdf.xacro` (with and without `use_gazebo:=false`) expands without errors. `stereo_eye.urdf` was regenerated.
* `urdf_parser_py` check: single tree rooted at `base_footprint`. **13 links / 12 joints** (2 continuous wheel joints).
  `camera_left_link` and `camera_right_link` both have parent `stereo_bar_link`.
* Total mass 3.160 kg, COM (−0.0212, 0, 0.0963) m. All inertia tensors are positive and satisfy the triangle inequality.
* Optical frames: z forward, x right, y down. The right camera is at (+0.080, 0, 0) in the left optical frame.
  TF baseline 0.080 m = −Tx/fx.
* Both camera sensors have identical intrinsics, rate and resolution: 1280×720, 15 Hz, HFOV 1.5, fx = fy = 686.99, cx = 640, cy = 360.
  Left Tx = 0, right Tx = −54.96.
* Both camera sensors and the IMU carry `<ignition_frame_id>` and `<gz_frame_id>` with the same frame (checked in the generated URDF).
* No `<mesh>` / `package://` references (primitives only). `check_urdf` wasn't available in the validation environment.
* **Not yet run in Gazebo.** The first sim run should confirm the `camera_info` P[3] value and the identical stamps (section 5).
