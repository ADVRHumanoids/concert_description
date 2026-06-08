from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, GroupAction, OpaqueFunction, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, TextSubstitution
from launch.conditions import UnlessCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os
import yaml
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory


def _load_yaml(raw_content: str):
    if not raw_content:
        return {}

    try:
        data = yaml.safe_load(raw_content) or {}
    except yaml.YAMLError:
        return {}

    return data if isinstance(data, dict) else {}


def generate_launch_description():
    default_camera_names = [
        'D435i_camera_front',
        'D435i_camera_back',
    ]

    default_velodyne_names = [
        'VLP16_lidar_front',
        'VLP16_lidar_back',
    ]

    default_ultrasound_names = [
        'ultrasound_fl_sag',
        'ultrasound_fr_sag',
        'ultrasound_rl_sag',
        'ultrasound_rr_sag',
        'ultrasound_fl_lat',
        'ultrasound_fr_lat',
        'ultrasound_rl_lat',
        'ultrasound_rr_lat'
    ]

    def _get_sensors_config(context):
        return _load_yaml(sensor_config_gz.perform(context))

    def _resolve_sensor_names(sensors_config, sensor_type, default_names):
        names = list(default_names)
        sensor_names = sensors_config.get('sensor_names', {})
        node = sensor_names

        if isinstance(node, dict):
            for level in sensor_type.split('/'):
                if not isinstance(node, dict):
                    node = None
                    break
                node = node.get(level)

        if isinstance(node, dict):
            file_names = node.get('generic', [])
        elif isinstance(node, list):
            file_names = node
        else:
            file_names = []

        if isinstance(file_names, list) and file_names:
            names = [str(name).strip() for name in file_names if str(name).strip()]
        return names

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
        bridge_remappings = []
        dynamic_nodes = []

        # Sensor enable/disable is controlled only by launch args.
        # Sensor names are obtained from the generator dedicated sensors output.
        sensors_config = _get_sensors_config(context)

        imu_enabled = LaunchConfiguration('imu').perform(context).strip().lower() == 'true'
        velodyne_enabled = LaunchConfiguration('velodyne').perform(context).strip().lower() == 'true'
        ultrasound_enabled = LaunchConfiguration('ultrasound').perform(context).strip().lower() == 'true'
        realsense_enabled = LaunchConfiguration('realsense').perform(context).strip().lower() == 'true'

        #################
        # NOTE: To resolve the names we use the /tmp{robot_name}_gz.sensors.yaml file generated by running the modular_description script with -o sensors argument.
        # In the file, sensor are assigned classes and subclasses (e.g. camera/realsense).
        # Sensors must have the subclasses defined at the same level of the hierarchy and for the one that it has not
        # been provided, 'generic' is used as fallback. For this reason the _resolve_name_function needs to retrieve the exact 
        # subclass hierarchy and 'generic' must be inserted.
        #
        # A good rule would be to check the temp file hierarchy before creating the dynamics bridges
        #################

        if imu_enabled:
            bridge_topics.append('/imu@sensor_msgs/msg/Imu[gz.msgs.IMU')

        if velodyne_enabled:
            velodyne_names = _resolve_sensor_names(sensors_config, 'lidar/velodyne', default_velodyne_names)
            for velodyne_name in velodyne_names:
                bridge_topics.extend([
                    f'/{velodyne_name}/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                    f'/{velodyne_name}@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                ])

        if ultrasound_enabled:
            ultrasound_names = _resolve_sensor_names(sensors_config, 'ultrasound/bosch_uss5', default_ultrasound_names)
            
            bridge_topics.extend([
                f'/bosch_uss5/{sensor_name}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
                for sensor_name in ultrasound_names
            ])

        if realsense_enabled:
            aligned_depth_rs_names = _resolve_sensor_names(sensors_config, 'camera/realsense_depth_aligned', [])
            rs_names = _resolve_sensor_names(sensors_config, 'camera/realsense', [])
            camera_names = _resolve_sensor_names(sensors_config, 'camera/generic', default_camera_names)

            # Keep lists disjoint: aligned cameras must not be processed also as non-aligned.
            aligned_depth_rs_set = set(aligned_depth_rs_names)
            rs_names = [name for name in rs_names if name not in aligned_depth_rs_set]
            
            # Realsense cameras with non-aligned depth
            for camera_name in rs_names:
                bridge_topics.extend([
                    f'/{camera_name}/color/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                    f'/{camera_name}/depth/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                    f'/{camera_name}/depth/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                    f'/{camera_name}/depth/image_raw/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                ])
                bridge_remappings.extend([
                    (f'/{camera_name}/depth/image_raw/points', f'/{camera_name}/depth/color/points'),
                    (f'/{camera_name}/depth/image_raw', f'/{camera_name}/depth/image_rect_raw')
                ])

                # RGB image uses ros_gz_image bridge and can be remapped per camera name.
                dynamic_nodes.append(
                    Node(
                        package='ros_gz_image',
                        executable='image_bridge',
                        name=f'{camera_name}_color_bridge',
                        arguments=[f'/{camera_name}/color/image_raw'],
                    )
                )

            # Realsense cameras with aligned depth (different topic names)
            for camera_name in aligned_depth_rs_names:
                bridge_topics.extend([
                    f'/{camera_name}/color/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                    f'/{camera_name}/aligned_depth_to_color/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                    f'/{camera_name}/aligned_depth_to_color/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                    f'/{camera_name}/aligned_depth_to_color/image_raw/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'
                ])
                bridge_remappings.append(
                    # Remapping to have the same name as the real hardware topics
                    (
                        f'/{camera_name}/aligned_depth_to_color/image_raw/points',
                        f'/{camera_name}/depth/color/points',
                    )
                )

                dynamic_nodes.append(
                    Node(
                        package='ros_gz_image',
                        executable='image_bridge',
                        name=f'{camera_name}_color_bridge',
                        arguments=[f'/{camera_name}/color/image_raw'],
                    )
                )

            # Generic cameras
            for camera_name in camera_names:
                bridge_topics.extend([
                    f'/{camera_name}/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
                    f'/{camera_name}/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                    f'/{camera_name}/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                ])
                bridge_remappings.extend([
                    (f'/{camera_name}/points', f'/{camera_name}/depth/color/points'),
                    (f'/{camera_name}/depth_image', f'/{camera_name}/depth/image_rect_raw'),
                    (f'/{camera_name}/camera_info', f'/{camera_name}/color/camera_info'),
                    (f'/{camera_name}/image', f'/{camera_name}/color/image_raw')
                    # /{camera_name}/depth/camera_info is not available in this mode as we use a single Gazebo rgbd sensor and the camera info would be the same for both depth and color, while in real hardware these two differ (bigger fov for depth).
                ])
                dynamic_nodes.append(
                    Node(
                        package='ros_gz_image',
                        executable='image_bridge',
                        name=f'{camera_name}_color_bridge',
                        arguments=[f'/{camera_name}/color/image_raw'],
                    )
                )

        # Keep simulation time synchronized independently from enabled sensors.
        bridge_topics.append('/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock')

        dynamic_nodes.append(
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='ros_gz_bridge',
                arguments=bridge_topics,
                remappings=bridge_remappings,
            )
        )

        return dynamic_nodes

    def _create_ultrasound_scan_to_range_node(context, *args, **kwargs):
        ultrasound_enabled = LaunchConfiguration('ultrasound').perform(context).strip().lower() == 'true'
        if not ultrasound_enabled:
            return []

        sensors_config = _get_sensors_config(context)
        ultrasound_names = _resolve_sensor_names(sensors_config, 'ultrasound', default_ultrasound_names)

        return [
            Node(
                package='concert_gazebo',
                executable='ultrasound_scan_to_range.py',
                name='ultrasound_scan_to_range',
                output='screen',
                parameters=[{
                    'input_topics': [f'/bosch_uss5/{sensor_name}/scan' for sensor_name in ultrasound_names]
                }]
            )
        ]

    set_gz_args_action = OpaqueFunction(function=_build_gz_args)

    # Robot description commands
    robot_description_gz = Command([
        'python3', ' ', LaunchConfiguration('modular_description'),
        ' -o urdf --quiet -a gazebo_urdf:=true',
        ' realsense:=', LaunchConfiguration('realsense'),
        ' velodyne:=', LaunchConfiguration('velodyne'),
        ' ultrasound:=', LaunchConfiguration('ultrasound'),
        ' imu:=', LaunchConfiguration('imu'),
        ' use_gpu_ray:=', LaunchConfiguration('use_gpu_ray'),
        ' -r modularbot_gz'
    ],
    on_stderr='ignore'
    )

    sensor_config_gz = Command([
        'python3', ' ', LaunchConfiguration('modular_description'),
        ' -o sensors --quiet -a gazebo_urdf:=true',
        ' realsense:=', LaunchConfiguration('realsense'),
        ' velodyne:=', LaunchConfiguration('velodyne'),
        ' ultrasound:=', LaunchConfiguration('ultrasound'),
        ' imu:=', LaunchConfiguration('imu'),
        ' use_gpu_ray:=', LaunchConfiguration('use_gpu_ray'),
        ' -r modularbot_gz'
    ],
    on_stderr='ignore'
    )

    # NOTE:In the XBot urdf realsense and velodyne args should be 'false' otherwise the Gazebo plugin will be included.
    # We keep it like this so to have also in simulation all the frames from cameras and lidars.
    # To be able to remove the gazebo plugins from the XBot urdf we should modify (fork) the repos of realsense and velodyne

    robot_description_xbot = Command([
        'python3', ' ', LaunchConfiguration('modular_description'),
        ' -o urdf --quiet -a gazebo_urdf:=false',
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
        ' -o srdf --quiet -a gazebo_urdf:=false',
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
            {'robot_description': ParameterValue(robot_description_xbot, value_type=str)},
            {'robot_description_semantic': ParameterValue(robot_description_semantic, value_type=str)}
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
            parameters=[{'string': ParameterValue(robot_description_gz, value_type=str)}, {'z': ParameterValue(1.0, value_type=float)}]
        ),
        OpaqueFunction(function=_create_dynamic_bridge_node),
        # Camera depth / camera_info / point cloud bridges and RGB image bridges
        # are generated dynamically in _create_dynamic_bridge_node().
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
        set_gz_args_action,
        gazebo_group,
        OpaqueFunction(function=_create_ultrasound_scan_to_range_node),
        xbot2_process,
        xbot2_gui_server,
        xbot2_gui_client,
        rviz_node
    ])
