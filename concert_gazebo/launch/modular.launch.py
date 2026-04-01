from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, GroupAction, OpaqueFunction, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, TextSubstitution
from launch.conditions import UnlessCondition
from launch_ros.actions import Node
import os
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ultrasound_sensor_names = [
        'ultrasound_fl_sag',
        'ultrasound_fr_sag',
        'ultrasound_rl_sag',
        'ultrasound_rr_sag',
        'ultrasound_fl_lat',
        'ultrasound_fr_lat',
        'ultrasound_rl_lat',
        'ultrasound_rr_lat'
    ]

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
        DeclareLaunchArgument('imu', default_value='false'),
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

    def _build_gz_args(context, *args, **kwargs):
        world_file = LaunchConfiguration('world_file').perform(context)
        verbose = LaunchConfiguration('verbose').perform(context).lower() == 'true'
        gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
        paused = LaunchConfiguration('paused').perform(context).lower() == 'true'
        extra_gazebo_args = LaunchConfiguration('extra_gazebo_args').perform(context).strip()

        parts = [world_file]
        if verbose:
            parts.append('-v')
        if not gui:
            parts.append('-s')
        if not paused:
            parts.append('-r')
        if extra_gazebo_args:
            parts.append(extra_gazebo_args)

        resolved_gz_args = ' '.join(parts)
        if verbose:
            print(f"[modular.launch.py] Resolved gz_args: {resolved_gz_args!r}")

        return [SetLaunchConfiguration('gz_args_resolved', resolved_gz_args)]

    def _create_dynamic_bridge_node(context, *args, **kwargs):
        bridge_topics = []

        if LaunchConfiguration('imu').perform(context).strip().lower() == 'true':
            bridge_topics.append('/imu@sensor_msgs/msg/Imu[gz.msgs.IMU')

        if LaunchConfiguration('velodyne').perform(context).strip().lower() == 'true':
            bridge_topics.extend([
                '/VLP16_lidar_back/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                '/VLP16_lidar_front/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                '/VLP16_lidar_back@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '/VLP16_lidar_front@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
            ])

        if LaunchConfiguration('ultrasound').perform(context).strip().lower() == 'true':
            bridge_topics.extend([
                f'/bosch_uss5/{sensor_name}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
                for sensor_name in ultrasound_sensor_names
            ])

        if LaunchConfiguration('realsense').perform(context).strip().lower() == 'true':
            bridge_topics.extend([
                '/D435i_camera_front/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
                '/D435i_camera_back/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
                '/D435i_camera_front/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                '/D435i_camera_back/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                '/D435i_camera_front/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                '/D435i_camera_back/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            ])

        # Keep simulation time synchronized independently from enabled sensors.
        bridge_topics.append('/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock')

        return [
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='ros_gz_bridge',
                arguments=bridge_topics,
            )
        ]

    set_gz_args_action = OpaqueFunction(function=_build_gz_args)

    # Robot description commands
    robot_description_gz = Command([
        'python3', ' ', LaunchConfiguration('modular_description'),
        ' -o urdf -a gazebo_urdf:=true floating_base:=true',
        ' realsense:=', LaunchConfiguration('realsense'),
        ' velodyne:=', LaunchConfiguration('velodyne'),
        ' ultrasound:=', LaunchConfiguration('ultrasound'),
        ' imu:=', LaunchConfiguration('imu'),
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
        ' ultrasound:=', LaunchConfiguration('ultrasound'),
        ' imu:=false',
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
        ' imu:=false',
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
            launch_arguments={'gz_args': LaunchConfiguration('gz_args_resolved')}.items()
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            name='urdf_spawner',
            parameters=[{'string': robot_description_gz, 'z': 1.0}]
        ),
        OpaqueFunction(function=_create_dynamic_bridge_node),
        # RealSense RGB bridges (Gazebo -> ROS Image)
        Node(
            condition=IfCondition(LaunchConfiguration('realsense')),
            package='ros_gz_image',
            executable='image_bridge',
            name='d435i_front_color_bridge',
            arguments=['/D435i_camera_front/image'],
            remappings=[
                ('/D435i_camera_front/image', '/D435i_camera_front/color/image_raw')
            ],
        ),
        Node(
            condition=IfCondition(LaunchConfiguration('realsense')),
            package='ros_gz_image',
            executable='image_bridge',
            name='d435i_back_color_bridge',
            arguments=['/D435i_camera_back/image'],
            remappings=[
                ('/D435i_camera_back/image', '/D435i_camera_back/color/image_raw')
            ],
        ),
        # RealSense depth / camera_info / point cloud bridges are handled
        # by the single dynamic ros_gz_bridge node above.
    ])

    ultrasound_scan_to_range_node = Node(
        condition=IfCondition(LaunchConfiguration('ultrasound')),
        package='concert_gazebo',
        executable='ultrasound_scan_to_range.py',
        name='ultrasound_scan_to_range',
        output='screen',
        parameters=[{
            'input_topics': [f'/bosch_uss5/{sensor_name}/scan' for sensor_name in ultrasound_sensor_names]
        }]
    )

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
        set_gz_args_action,
        gazebo_group,
        ultrasound_scan_to_range_node,
        xbot2_process,
        xbot2_gui_server,
        xbot2_gui_client,
        rviz_node
    ])