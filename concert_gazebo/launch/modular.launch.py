from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, TextSubstitution
from launch.conditions import UnlessCondition
from launch_ros.actions import Node
import os
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare launch arguments
    arg_launch_arguments = [
        DeclareLaunchArgument('gazebo', default_value='true'),
        DeclareLaunchArgument('xbot2', default_value='true'),
        DeclareLaunchArgument('xbot2_gui', default_value='true'),
        DeclareLaunchArgument('xbot2_config', default_value=os.path.join(get_package_share_directory('concert_xbot2'), 'modular.yaml')),
        DeclareLaunchArgument('modular_description', default_value=os.path.join(get_package_share_directory('concert_examples'), 'concert_example.py')),
        DeclareLaunchArgument('realsense', default_value='false'),
        DeclareLaunchArgument('velodyne', default_value='false'),
        DeclareLaunchArgument('ultrasound', default_value='false'),
        DeclareLaunchArgument('use_gpu_ray', default_value='false'),
        DeclareLaunchArgument('paused', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('debug', default_value='false'),
        DeclareLaunchArgument('physics', default_value='ode'),
        DeclareLaunchArgument('verbose', default_value='false'),
        DeclareLaunchArgument('extra_gazebo_args', default_value=''),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('world_file', default_value=os.path.join(get_package_share_directory('concert_gazebo'), 'world/empty_world.sdf'))
    ]


    # Construct `gz_args` with conditional '-r' based on `paused`
    gz_args = [
        LaunchConfiguration('world_file'),
        TextSubstitution(text=' '),
        TextSubstitution(text='-v ') if LaunchConfiguration('verbose') == 'true' else TextSubstitution(text=''),
        TextSubstitution(text='-s ') if LaunchConfiguration('gui') == 'false' else TextSubstitution(text=''),
        TextSubstitution(text='-r') if UnlessCondition(LaunchConfiguration('paused')) else TextSubstitution(text='')
    ]

    # Robot description commands
    robot_description_gz = Command([
        'python3', ' ', LaunchConfiguration('modular_description'),
        ' -o urdf -a gazebo_urdf:=true floating_base:=true',
        ' realsense:=', LaunchConfiguration('realsense'),
        ' velodyne:=', LaunchConfiguration('velodyne'),
        ' ultrasound:=', LaunchConfiguration('ultrasound'),
        ' use_gpu_ray:=', LaunchConfiguration('use_gpu_ray'),
        ' -r modularbot_gz'
    ],
    on_stderr='ignore'
    )

    robot_description_xbot = Command([
        'python3', ' ', LaunchConfiguration('modular_description'),
        ' -o urdf -a gazebo_urdf:=false floating_base:=true',
        ' realsense:=', LaunchConfiguration('realsense'),
        ' velodyne:=', LaunchConfiguration('velodyne'),
        ' ultrasound:=false',
        ' use_gpu_ray:=', LaunchConfiguration('use_gpu_ray'),
        ' -r modularbot'
    ],
    on_stderr='ignore'
    )

    robot_description_semantic = Command([
        'python3', ' ', LaunchConfiguration('modular_description'),
        ' -o srdf -a gazebo_urdf:=false',
        ' realsense:=', LaunchConfiguration('realsense'),
        ' velodyne:=', LaunchConfiguration('velodyne'),
        ' ultrasound:=false',
        ' use_gpu_ray:=', LaunchConfiguration('use_gpu_ray'),
        ' -r modularbot'
    ],
    on_stderr='ignore'
    )

    # Robot description publisher node
    description_publisher_node = Node(
        package='concert_xbot2',  # Replace with your package name
        executable='robot_description_publisher',  # Replace with your node executable
        name='robot_description_publisher',
        parameters=[
            {'robot_description': robot_description_xbot},
            {'robot_description_semantic': robot_description_semantic}
        ],
        output='screen'
    )

    # Gazebo group
    gazebo_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': gz_args}.items()
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            name='urdf_spawner',
            parameters=[{'string': robot_description_gz, 'z': 1.0}]
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge',
            arguments=[
                '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                '/VLP16_lidar_back/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                '/VLP16_lidar_front/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'
            ],
        )
    ])

    # Xbot2 process
    xbot2_process = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('xbot2')),
        cmd=[
            'xbot2-core', '-V', '--hw', 'sim', '--simtime',
            '--config', LaunchConfiguration('xbot2_config'), '--'
        ],
        output='screen'
    )

    # Xbot2 GUI server and client
    xbot2_gui_server = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('xbot2_gui')),
        cmd=['xbot2_gui_server', os.path.join(get_package_share_directory('concert_xbot2'), 'gui_config.yaml')],
        output='log'
    )

    xbot2_gui_client = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('xbot2_gui')),
        cmd=['xbot2_gui'],
        output='log'
    )

    # RViz node
    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration('rviz')),
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('concert_gazebo'), 'rviz/concert_sensors.rviz')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Create and return launch description
    return LaunchDescription(arg_launch_arguments + [
        description_publisher_node,
        gazebo_group,
        xbot2_process,
        xbot2_gui_server,
        xbot2_gui_client,
        rviz_node
    ])