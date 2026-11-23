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
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')

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
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={
            'gz_args': [
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
            'Grid/Sensor': '1',  # 1 = depth (from stereo); was Grid/FromDepth true
            'Grid/RangeMax': '4.0',
            'Grid/MaxGroundHeight': '0.05',
            'Grid/MaxObstacleHeight': '0.40',
            'Grid/RayTracing': 'true',
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
        condition=IfCondition(LaunchConfiguration('steering')),
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
        condition=IfCondition(LaunchConfiguration('teleop')),
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
    ])
