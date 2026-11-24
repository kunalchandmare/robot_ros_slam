"""Launch stereo_eye in the apartment_gazebo world under Gazebo Sim (Fortress).

The Dockerfile installs ROS 2 Humble with ros-humble-ros-gz, which is built
against Gazebo Fortress (ign-gazebo 6). gz_sim.launch.py therefore runs
``ign gazebo``. The same launch arguments and gz.msgs.* bridge types also
work on Harmonic (ros-humble-ros-gzharmonic), but this file follows the
container that is actually in the repo.
"""

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

# RewrittenYaml (from Nav2) is only needed for explore:=true. Import it softly
# so the normal launch still works on a machine without Nav2.
try:
    from nav2_common.launch import RewrittenYaml
except ImportError:
    RewrittenYaml = None


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    explore = LaunchConfiguration('explore')

    default_world = PathJoinSubstitution([
        FindPackageShare('apartment_gazebo'),
        'worlds',
        'apartment.sdf',
    ])
    apartment_models = PathJoinSubstitution([
        FindPackageShare('apartment_gazebo'),
        'models',
    ])
    xacro_file = PathJoinSubstitution([
        FindPackageShare('stereo_eye'),
        'urdf',
        'stereo_eye.urdf.xacro',
    ])
    gz_sim_launch = PathJoinSubstitution([
        FindPackageShare('ros_gz_sim'),
        'launch',
        'gz_sim.launch.py',
    ])
    nav2_navigation_launch = PathJoinSubstitution([
        FindPackageShare('nav2_bringup'),
        'launch',
        'navigation_launch.py',
    ])
    nav2_params = PathJoinSubstitution([
        FindPackageShare('stereo_eye_launcher'),
        'config',
        'nav2_params.yaml',
    ])
    explore_bt_xml = PathJoinSubstitution([
        FindPackageShare('stereo_eye_launcher'),
        'behavior_trees',
        'explore_navigate_to_pose.xml',
    ])
    explore_params = PathJoinSubstitution([
        FindPackageShare('stereo_eye_launcher'),
        'config',
        'explore_params.yaml',
    ])
    # explorer:=own -> our frontier_explorer.py instead of explore_lite
    explorer = LaunchConfiguration('explorer')
    frontier_explorer_params = PathJoinSubstitution([
        FindPackageShare('stereo_eye_launcher'),
        'config',
        'frontier_explorer_params.yaml',
    ])
    explore_with_lite = PythonExpression(
        ["'", explore, "'.lower() in ('true', '1') and '", explorer, "' == 'explore_lite'"])
    explore_with_own = PythonExpression(
        ["'", explore, "'.lower() in ('true', '1') and '", explorer, "' == 'own'"])

    # Living-room pose recommended by apartment_gazebo: clear of the sofa
    # (x=2.4,y=7.2, 2.2x0.85 m) and coffee table (x=2.4,y=6.0, 1.2x0.6 m),
    # inside free space x=[0.150, 5.925], y=[3.575, 7.850], facing +X
    # toward the kitchen opening.
    declared_args = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo simulation clock if true',
        ),
        DeclareLaunchArgument(
            'world',
            default_value=default_world,
            description='Full path to the Gazebo world SDF',
        ),
        DeclareLaunchArgument(
            'x', default_value='4.00',
            description='Spawn x position (m) in the apartment world',
        ),
        DeclareLaunchArgument(
            'y', default_value='5.50',
            description='Spawn y position (m) in the apartment world',
        ),
        DeclareLaunchArgument(
            'z', default_value='0.05',
            description='Spawn z position (m); small offset so the robot drops onto the floor',
        ),
        DeclareLaunchArgument(
            'yaw', default_value='0.0',
            description='Spawn yaw (rad); 0 faces +X toward the kitchen opening',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Start RViz2 with the stereo SLAM config if true',
        ),
        DeclareLaunchArgument(
            'steering',
            default_value='true',
            description='Start rqt_robot_steering (mouse sliders on /cmd_vel) if true',
        ),
        DeclareLaunchArgument(
            'teleop',
            default_value='false',
            description='Start teleop_twist_keyboard in an xterm if true',
        ),
        # Ogre2 crashes on the WSL D3D12 GPU driver (Ogre UnimplementedException
        # in GL3PlusTextureGpu::copyTo), so the default is the older Ogre 1.x
        # engine, which works there. Use render_engine:=ogre2 on a native
        # Linux GPU for better-looking images.
        DeclareLaunchArgument(
            'render_engine',
            default_value='ogre',
            choices=['ogre', 'ogre2'],
            description='Gazebo render engine for server (camera sensors) and GUI',
        ),
        # The Gazebo GUI window is optional. On the WSL D3D12 GPU it can run out
        # of GPU memory ("D3D12: Removing Device", then Ogre "Vertex Buffer:
        # Out of memory" while creating the GUI render window), which shuts the
        # whole launch down. gz_gui:=false starts only the Gazebo server: the
        # world, physics and the camera/ToF sensors still run and render, and
        # RViz still shows everything; there is just no Gazebo window.
        DeclareLaunchArgument(
            'gz_gui',
            default_value='true',
            choices=['true', 'false'],
            description='Open the Gazebo GUI window if true; false = server only (-s)',
        ),
        # Autonomous frontier exploration: Nav2 + explore_lite drive the robot
        # and explore_map_saver.py saves the 2D map when no frontiers are left.
        DeclareLaunchArgument(
            'explore',
            default_value='false',
            description='Explore autonomously (Nav2 + explore_lite) and save the map if true; '
                        'turns off steering and teleop',
        ),
        # Which node picks the frontiers when explore:=true. explore_lite =
        # as before (default). own = scripts/frontier_explorer.py (reachable
        # frontiers only, see config/frontier_explorer_params.yaml).
        DeclareLaunchArgument(
            'explorer',
            default_value='explore_lite',
            choices=['explore_lite', 'own'],
            description='Frontier explorer for explore:=true: explore_lite (default) or own '
                        '(frontier_explorer.py)',
        ),
        # Maps are written to the package source folder, not to install/,
        # so they survive a rebuild and are easy to find.
        DeclareLaunchArgument(
            'maps_dir',
            default_value='/workspaces/ros2-dev/src/stereo_eye_launcher/maps',
            description='Folder where explore:=true saves <map_name>_<date>.pgm/.yaml',
        ),
        DeclareLaunchArgument(
            'map_name',
            default_value='apartment',
            description='File name prefix for the saved map',
        ),
    ]

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' use_gazebo:=true']),
        value_type=str,
    )

    # Fortress (ign gazebo) reads IGN_GAZEBO_RESOURCE_PATH. GZ_SIM_RESOURCE_PATH
    # is the Garden/Harmonic name; keep both so model:// URIs resolve either way.
    set_ign_resource_path = AppendEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH', apartment_models
    )
    set_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', apartment_models
    )
    set_model_path = AppendEnvironmentVariable(
        'GAZEBO_MODEL_PATH', apartment_models
    )

    # ros_gz_sim's gz_sim.launch.py accepts gz_args on Humble/Fortress (and
    # then execs "ign gazebo") as well as on Harmonic ("gz sim").
    # --render-engine sets the engine for both the server (which renders the
    # camera sensors) and the GUI; see the render_engine argument above.
    # In ign gazebo 6 it is the same as giving --render-engine-server and
    # --render-engine-gui; with -s the GUI part is simply unused, so the
    # sensors still render with the chosen engine.
    #
    # gz_gui:=false adds "-s" (server only). gz_sim.launch.py runs one
    # process either way ("ign gazebo <gz_args>"), and on_exit_shutdown
    # shuts the whole launch down when that process exits: with the GUI that
    # is when you close the Gazebo window, with -s it is when the server
    # stops (Ctrl+C or a crash).
    gz_server_only_flag = PythonExpression(
        ["'' if '", LaunchConfiguration('gz_gui'), "'.lower() in ('true', '1') else '-s '"]
    )
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={
            'gz_args': [
                gz_server_only_flag,
                '-r -v 1 --render-engine ',
                LaunchConfiguration('render_engine'),
                ' ',
                world,
            ],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description,
        }],
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_stereo_eye',
        output='screen',
        arguments=[
            '-name', 'stereo_eye',
            '-topic', 'robot_description',
            '-x', x,
            '-y', y,
            '-z', z,
            '-Y', yaw,
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Topics match stereo_eye.gazebo.xacro. Humble ros_gz_bridge accepts
    # gz.msgs.* (and ignition.msgs.*) type strings on Fortress and Harmonic.
    # '[' = Gazebo -> ROS, ']' = ROS -> Gazebo.
    #
    # TF ownership (tree: map -> vo -> odom -> base_footprint):
    #   - DiffDrive (wheels) publishes odom -> base_footprint on the gz topic
    #     /wheel_tf, bridged here onto ROS /tf. It is the only parent of
    #     base_footprint.
    #   - stereo_odometry publishes vo -> odom (visual correction on top of
    #     the wheels), rtabmap publishes map -> vo.
    #   - robot_state_publisher provides base_footprint -> camera_* frames
    #     (and the rest of the model) from the URDF plus bridged /joint_states.
    # The DiffDrive /odom topic is remapped to /wheel_odom so it does not
    # collide with stereo_odometry's /odom.
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/stereo/left/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/stereo/left/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/stereo/right/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/stereo/right/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/wheel_tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            # Front time-of-flight range sensor (gpu_lidar in
            # stereo_eye.gazebo.xacro, MechaX), 0.30 m range, covering the
            # stereo blind zone. Read by the local costmap (obstacles 0.04 to
            # 0.30 m away) and stuck_recovery.py (closer than 0.04 m).
            # Harmless if the sensor is not in the model yet (no messages).
            '/front_tof@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
        remappings=[
            ('/odom', '/wheel_odom'),
            ('/wheel_tf', '/tf'),
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Sim images have no distortion (R = identity), so they are already
    # rectified. Remap image_raw onto RTAB-Map's image_rect inputs; do not
    # rename the bridge topics.
    #
    # Approximate sync (approx_sync true): in theory both gz cameras render in
    # one Sensors-system step and share a stamp, but under load (two
    # cameras, first at 1280x720 @ 15 fps, now 640x360 @ 24 fps)
    # the sim slows down and the left/right (and /odom) stamps stop matching
    # exactly. Exact sync then dropped nearly every frame ("Did not receive
    # data since 5 seconds"). Approx sync pairs messages whose stamps are at
    # most approx_sync_max_interval (20 ms) apart. rtabmap also syncs /odom
    # with the images, so it needs approx sync too.
    #
    # Queue sizes: sync_queue_size is the synchronizer's buffer (replaces the
    # deprecated queue_size); topic_queue_size is each subscriber's buffer.
    stereo_remaps = [
        ('left/image_rect', '/stereo/left/image_raw'),
        ('left/camera_info', '/stereo/left/camera_info'),
        ('right/image_rect', '/stereo/right/image_raw'),
        ('right/camera_info', '/stereo/right/camera_info'),
        ('odom', '/odom'),
    ]
    stereo_common_params = {
        'use_sim_time': use_sim_time,
        'frame_id': 'base_footprint',
        'approx_sync': True,
        'approx_sync_max_interval': 0.02,
        'wait_for_transform': 0.2,
        'sync_queue_size': 20,
        'topic_queue_size': 10,
        'Reg/Force3DoF': 'true',
        'Stereo/MaxDisparity': '160',
    }

    # Wheel odometry as a guess: guess_frame_id 'odom' makes stereo_odometry
    # read the wheel motion (odom -> base_footprint from DiffDrive, via TF) as
    # its starting guess for every frame. It then publishes only the visual
    # correction, vo -> odom, instead of a second parent for base_footprint.
    # With a good guess the feature matcher searches a smaller area and is
    # much less likely to get lost on turns or plain walls.
    #
    # Odom/ResetCountdown 1: when visual odometry still gets lost ("Not
    # enough inliers", quality 0), reset it on the very next frame instead of
    # staying lost forever. Without it, rtabmap keeps printing "no odometry is
    # provided, image ignored" and the map freezes. After a reset, rtabmap
    # starts a new map session and links it to the old one through loop
    # closure once the robot sees a known place again.
    #
    # Modest visual odometry tuning (rtabmap parameters are strings):
    #   Odom/Strategy 0     Frame-to-Map (F2M): match each frame against a
    #                       small local map of recent features, not only the
    #                       last frame. This is the default, set explicitly.
    #   Vis/FeatureType 8   GFTT corners with ORB descriptors (the default).
    #                       GFTT finds corners even on low-texture surfaces.
    #   Vis/MaxFeatures     Up to 1000 features per frame, so there are more
    #                       candidates to match on plain walls.
    #   GFTT/MinDistance 5  Allow corners 5 px apart (default 7), which suits
    #                       the 640x360 images.
    #   Odom/KeyFrameThr    Add a new keyframe to the local map when inliers
    #                       drop below 40 % (default 30 %), so the map refreshes
    #                       a little earlier during turns.
    #
    # publish_null_when_lost false: keeps ONE map across odometry resets.
    # With the default (true), the first frame after an automatic reset is
    # published on /odom with covariance 9999, which is how rtabmap_odom tells
    # rtabmap "odometry restarted". rtabmap then starts a NEW map session
    # ("Odometry is reset ... Increment map id!") that is not connected to the
    # old one, and /map shows only the new, small session until a loop
    # closure joins them: the explored map seems to vanish in RViz, Nav2's
    # global costmap shrinks with it and explore_lite loses its frontiers.
    # With false and a guess frame (guess_frame_id odom), rtabmap_odom
    # restarts from the latest pose plus the wheel motion and publishes that
    # first frame as a normal continuation, so rtabmap keeps extending the
    # same map. While odometry is lost nothing is published on /odom (rtabmap
    # just skips those frames), and the vo -> odom TF is still sent from the
    # wheel guess, so the TF tree never breaks.
    #
    # "Dropping image/scan data with stamp ... clock difference ... is much
    # lower than the expected one" warnings are harmless here. They come from
    # rtabmap_odom's always_process_most_recent_frame (default true): when
    # two camera frames arrive almost together in wall time (Gazebo running
    # slower than real time delivers frames in small bursts), it skips the
    # older one so odometry stays current. Kunal's log: 8 drops in about 27 s
    # of 24 Hz images (~1 %), with odometry quality steady at 420-620. It is
    # not a sync problem, so approx_sync_max_interval and the queue sizes stay
    # as they are. Setting always_process_most_recent_frame false would
    # process every frame, but odometry takes 70-110 ms per frame (longer
    # than the 42 ms between frames), so a backlog and growing delay would
    # build up instead.
    stereo_odometry = Node(
        package='rtabmap_odom',
        executable='stereo_odometry',
        name='stereo_odometry',
        output='screen',
        parameters=[{
            **stereo_common_params,
            'odom_frame_id': 'vo',
            'guess_frame_id': 'odom',
            'publish_tf': True,
            'publish_null_when_lost': False,
            'Odom/ResetCountdown': '1',
            'Odom/Strategy': '0',
            'Vis/FeatureType': '8',
            'Vis/MaxFeatures': '1000',
            'GFTT/MinDistance': '5',
            'Odom/KeyFrameThr': '0.4',
        }],
        remappings=stereo_remaps,
    )

    # odom_frame_id is left unset so rtabmap subscribes to the /odom topic
    # from stereo_odometry (setting it would read TF only and drop the topic).
    # That topic's frame is vo, so rtabmap publishes only map -> vo.
    #
    # Map settings (rtabmap parameters are strings). The 2D grid on /map is
    # what Nav2 and explore_lite use, so it should be flat and well filled:
    #   Reg/Force3DoF true (in stereo_common_params): x, y, yaw only, so the
    #                       map cannot tilt or float on a flat floor.
    #   RGBD/ProximityBySpace true: also look for loop closures with nearby
    #                       old nodes (by position, not only by image
    #                       similarity), which closes small drifts and stops
    #                       walls from doubling. This is rtabmap's default;
    #                       set explicitly because the map depends on it.
    #   Grid/Sensor 1       Build the grid from stereo depth (no laser).
    #   Grid/3D false       2D grid cells only (no OctoMap), which is lighter
    #                       and makes /map come straight from 2D local grids.
    #   Grid/CellSize 0.025 2.5 cm cells (was 0.05; same as the Nav2
    #                       costmaps). With 5 cm cells a door only 10 cm
    #                       wider than the robot usually shows up 5 cm too
    #                       narrow and Nav2 will not drive through it; see
    #                       config/nav2_params.yaml. Saved maps are finer too.
    #   Grid/RangeMax 4.0   Use depth up to 4 m. Stereo depth error grows with
    #                       distance squared: with fx 343.5 px and a 0.08 m
    #                       baseline one pixel of disparity is about 0.58 m at
    #                       4 m and 0.15 m at 2 m. Lower this to 3.0 if walls
    #                       look thick or blurry.
    #   Grid/RayTracing true Mark the cells between the camera and each
    #                       obstacle as free, so rooms fill in as free space
    #                       instead of staying unknown (frontiers need that).
    #   Grid/NormalsSegmentation false Split ground from obstacles by height
    #                       only (below MaxGroundHeight = floor, between that
    #                       and MaxObstacleHeight = obstacle). Simple and
    #                       robust on the flat sim floor; normals from noisy
    #                       stereo points are not.
    #   Grid/MaxGroundHeight 0.05 Points up to 5 cm high are floor.
    #   Grid/MaxObstacleHeight 0.40 Ignore points above 40 cm (table tops,
    #                       ceiling) that the low robot drives under or never
    #                       reaches.
    #   Grid/NoiseFilteringRadius 0.05 and Grid/NoiseFilteringMinNeighbors 5:
    #                       drop obstacle points with fewer than 5 neighbours
    #                       within 5 cm (isolated stereo speckles), which
    #                       would otherwise show up as fake obstacles.
    #   RGBD/LinearUpdate / AngularUpdate 0.05: add a map node every 5 cm or
    #                       0.05 rad of motion, so the map grows smoothly.
    rtabmap = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        arguments=['-d'],
        parameters=[{
            **stereo_common_params,
            'map_frame_id': 'map',
            'publish_tf': True,
            'subscribe_stereo': True,
            'subscribe_depth': False,
            'subscribe_rgb': False,
            'subscribe_rgbd': False,
            'subscribe_odom_info': True,
            'RGBD/ProximityBySpace': 'true',
            'Grid/Sensor': '1',  # 1 = depth (from stereo); was Grid/FromDepth true
            'Grid/3D': 'false',
            'Grid/CellSize': '0.025',  # was 0.05, see above
            'Grid/RangeMax': '4.0',
            'Grid/RayTracing': 'true',
            'Grid/NormalsSegmentation': 'false',
            'Grid/MaxGroundHeight': '0.05',
            'Grid/MaxObstacleHeight': '0.40',
            'Grid/NoiseFilteringRadius': '0.05',
            'Grid/NoiseFilteringMinNeighbors': '5',
            'RGBD/LinearUpdate': '0.05',
            'RGBD/AngularUpdate': '0.05',
        }],
        remappings=stereo_remaps,
    )

    camera_brightness = Node(
        package='camera_brightness',
        executable='camera_brightness',
        name='camera_brightness',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[('/image', '/stereo/left/image_raw')],
    )

    # Fixed Frame is odom (exists as soon as the wheel TF is bridged).
    # use_sim_time must be true so Image/TF/Map stamps match /clock. A bare
    # rviz2 uses wall time and defaults Fixed Frame to map, which is empty
    # until rtabmap publishes map -> odom.
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[
            '-d',
            PathJoinSubstitution([
                FindPackageShare('stereo_eye_launcher'),
                'rviz',
                'stereo_slam.rviz',
            ]),
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    # Driving. Recommended speeds for visual odometry: 0.15 m/s and 0.4 rad/s.
    #
    # Manual driving is only started when explore is false: rqt_robot_steering
    # publishes /cmd_vel all the time (zero when the sliders are centred), so
    # it would fight Nav2 for the robot. With explore:=true, steering:=true and
    # teleop:=true are ignored. To take over, stop the launch and start again
    # without explore.
    #
    # Launch booleans may be written true/True/1, hence the lower() checks.
    explore_off = ["'", explore, "'.lower() not in ('true', '1')"]
    steering_on = PythonExpression(
        ["'", LaunchConfiguration('steering'), "'.lower() in ('true', '1') and "] + explore_off
    )
    teleop_on = PythonExpression(
        ["'", LaunchConfiguration('teleop'), "'.lower() in ('true', '1') and "] + explore_off
    )
    #
    # rqt_robot_steering (default): a small window with mouse sliders that
    # publishes geometry_msgs/Twist on /cmd_vel, which the bridge forwards to
    # Gazebo. Unlike the keyboard teleop it does not need keyboard focus, so it
    # keeps working after you click RViz or Gazebo. The slider ranges below are
    # capped at the recommended speeds; raise them in the window if needed.
    # No node name is set, because rqt picks its own and the parameters are
    # then applied to it through the "/**" wildcard.
    steering = Node(
        package='rqt_robot_steering',
        executable='rqt_robot_steering',
        output='screen',
        parameters=[{
            'default_topic': '/cmd_vel',
            'default_vx_max': 0.15,
            'default_vx_min': -0.15,
            'default_vw_max': 0.4,
            'default_vw_min': -0.4,
        }],
        condition=IfCondition(steering_on),
    )

    # teleop_twist_keyboard (optional, teleop:=true): only reads keys while its
    # xterm window has keyboard focus. Clicking RViz or Gazebo takes focus away,
    # so click the xterm again before pressing keys. Its start speeds are the
    # recommended 0.15 m/s ("speed") and 0.4 rad/s ("turn").
    teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        prefix='xterm -e',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'speed': 0.15,
            'turn': 0.4,
        }],
        condition=IfCondition(teleop_on),
    )

    # ---- Autonomous exploration (explore:=true) ----
    #
    # Nav2 navigation only (controller, planner, behaviours, bt_navigator,
    # velocity_smoother). No localization_launch.py: there is no AMCL or
    # map_server, because rtabmap already provides map -> vo -> odom and the
    # map itself on /map.
    #
    # cmd_vel path: controller_server publishes cmd_vel_nav, velocity_smoother
    # limits it to 0.15 m/s / 0.4 rad/s and publishes /cmd_vel (both remaps
    # are inside navigation_launch.py), and ros_gz_bridge forwards /cmd_vel to
    # the Gazebo DiffDrive. Recovery behaviours (spin, back up) publish
    # /cmd_vel directly.
    #
    # The behaviour tree path cannot be written into nav2_params.yaml (it
    # depends on where the package is installed), so RewrittenYaml fills in
    # default_nav_to_pose_bt_xml with the installed
    # behavior_trees/explore_navigate_to_pose.xml (a temporary copy of the
    # params file is passed to Nav2).
    if RewrittenYaml is not None:
        nav2_params_file = RewrittenYaml(
            source_file=nav2_params,
            param_rewrites={'default_nav_to_pose_bt_xml': explore_bt_xml},
            convert_types=True,
        )
    else:
        nav2_params_file = nav2_params  # Nav2 missing: explore:=true cannot run anyway
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_navigation_launch),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'autostart': 'true',
        }.items(),
        condition=IfCondition(explore),
    )

    # explore_lite (m-explore-ros2): finds frontiers on Nav2's global costmap
    # and sends them as navigate_to_pose goals. It waits by itself until the
    # costmap and the Nav2 action server are up.
    explore_lite = Node(
        package='explore_lite',
        executable='explore',
        name='explore_node',
        output='screen',
        parameters=[explore_params, {'use_sim_time': use_sim_time}],
        condition=IfCondition(explore_with_lite),
    )

    # Our own frontier explorer (explore:=true explorer:=own): same topics and
    # Nav2 action as explore_lite, so stuck_recovery.py and
    # explore_map_saver.py work unchanged. See scripts/frontier_explorer.py.
    frontier_explorer = Node(
        package='stereo_eye_launcher',
        executable='frontier_explorer.py',
        name='frontier_explorer',
        output='screen',
        parameters=[frontier_explorer_params, {'use_sim_time': use_sim_time}],
        condition=IfCondition(explore_with_own),
    )

    # Live obstacle cloud for Nav2's obstacle layers (explore:=true only).
    # rtabmap_util's point_cloud_xyzrgb takes the raw stereo pair, computes a
    # disparity image with OpenCV block matching (rtabmap's StereoBM
    # parameters) and publishes a point cloud in the left camera's optical
    # frame on /stereo/obstacle_cloud. No extra package is needed (rtabmap_util
    # is part of ros-humble-rtabmap-ros) and no separate disparity node.
    #   decimation 4        use every 4th pixel in x and y for the cloud
    #                       (16x fewer points; the disparity itself is still
    #                       computed at full 640x360 for accuracy)
    #   max_depth 3.0       drop points further than 3 m (too noisy)
    #   voxel_size 0.05     one point per 5 cm cube (2 costmap cells of
    #                       2.5 cm; kept at 5 cm so the noise filter below
    #                       behaves as before. Inflation closes the 2.5 cm
    #                       gaps between marked cells.)
    #   noise_filter_radius 0.1 / min_neighbors 5: drop lonely points (stereo
    #                       speckles) that would show up as fake obstacles
    #   StereoBM/NumDisparities 128 (rtabmap's default, set explicitly): the
    #                       closest depth it can measure is
    #                       343.5 * 0.08 / 128 = 0.21 m. Plain walls fail
    #                       earlier, at about 0.35 m: that is the blind zone.
    # It runs at the camera rate (24 Hz) because the node has no rate limit;
    # block matching at 640x360 is cheap compared with visual odometry.
    stereo_obstacle_cloud = Node(
        package='rtabmap_util',
        executable='point_cloud_xyzrgb',
        name='stereo_obstacle_cloud',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'approx_sync': True,
            'approx_sync_max_interval': 0.02,
            'decimation': 4,
            'max_depth': 3.0,
            'voxel_size': 0.05,
            'noise_filter_radius': 0.1,
            'noise_filter_min_neighbors': 5,
            'StereoBM/NumDisparities': '128',
        }],
        remappings=[
            ('left/image', '/stereo/left/image_raw'),
            ('left/camera_info', '/stereo/left/camera_info'),
            ('right/image', '/stereo/right/image_raw'),
            ('right/camera_info', '/stereo/right/camera_info'),
            ('cloud', '/stereo/obstacle_cloud'),
        ],
        condition=IfCondition(explore),
    )

    # Stuck watchdog (explore:=true only, and only active while explore_lite
    # is exploring): notices the front ToF sensor seeing something closer than
    # 4 cm while driving forward (the main trigger), /cmd_vel asking to drive
    # forward for 5 s while visual odometry (/odom) does not move, or the
    # robot standing too close to a wall for Nav2 to plan. It then makes
    # explore_lite give up that frontier (not for the last case), backs up
    # 0.15 m (no turn any more; Nav2 turns towards its next path itself) and
    # lets exploration continue. It is the only thing that reverses the robot.
    # All thresholds are parameters of the node (see the header of
    # scripts/stuck_recovery.py). /front_tof comes from the bridge above.
    stuck_recovery = Node(
        package='stereo_eye_launcher',
        executable='stuck_recovery.py',
        name='stuck_recovery',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'tof_topic': '/front_tof',
            # (m) recovery only below 4 cm, and only while driving forward;
            # the costmap steers away from anything 0.04-0.30 m ahead.
            # Readings of exactly range_min (0.01 m, was 0.02) are ignored as a Gazebo
            # clamp artefact (tof_min_margin, see the script).
            'tof_too_close_m': 0.04,
        }],
        condition=IfCondition(explore),
    )

    # Saves /map with nav2_map_server's map_saver_cli as soon as explore_lite
    # publishes "exploration_complete" on /explore/status. A "complete" in the
    # first 60 s or with less than 4 m2 of known map is treated as a start-up
    # false alarm instead: the node turns the robot once on the spot and
    # restarts explore_lite (up to 3 times). Defaults live in the script.
    map_saver = Node(
        package='stereo_eye_launcher',
        executable='explore_map_saver.py',
        name='explore_map_saver',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'maps_dir': LaunchConfiguration('maps_dir'),
            'map_name': LaunchConfiguration('map_name'),
        }],
        condition=IfCondition(explore),
    )

    return LaunchDescription(declared_args + [
        set_ign_resource_path,
        set_resource_path,
        set_model_path,
        gazebo,
        robot_state_publisher,
        spawn_robot,
        ros_gz_bridge,
        stereo_odometry,
        rtabmap,
        camera_brightness,
        rviz,
        steering,
        teleop,
        nav2,
        stereo_obstacle_cloud,
        explore_lite,
        frontier_explorer,
        stuck_recovery,
        map_saver,
    ])
