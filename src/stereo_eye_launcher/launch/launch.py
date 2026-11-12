"""Launch stereo_eye in the apartment_gazebo world under Gazebo Sim (Harmonic)."""

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
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
    ]

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' use_gazebo:=true']),
        value_type=str,
    )

    set_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', apartment_models
    )
    set_model_path = AppendEnvironmentVariable(
        'GAZEBO_MODEL_PATH', apartment_models
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={
            'gz_args': ['-r -v 1 ', world],
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

    # Topics match stereo_eye.gazebo.xacro (Gazebo Harmonic / gz-sim 8).
    # '[' = Gazebo -> ROS, ']' = ROS -> Gazebo.
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
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription(declared_args + [
        set_resource_path,
        set_model_path,
        gazebo,
        robot_state_publisher,
        spawn_robot,
        ros_gz_bridge,
    ])
