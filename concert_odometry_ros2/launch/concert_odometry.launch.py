from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    

    # Define the configuration paths
    concert_odom_dir = get_package_share_directory('concert_odometry_ros2')
    config_dir = os.path.join(concert_odom_dir, 'config')
    base_estimation_config = os.path.join(config_dir, 'base_estimation_params.yaml')
    odometry_stack_config = os.path.join(config_dir, 'concert_odometry_stack.yaml')
    odometry_config = os.path.join(config_dir, 'concert_odometry.yaml')
    rviz_config = os.path.join(concert_odom_dir, 'rviz', 'concert_odometry.rviz')


    with open(base_estimation_config, 'r') as f:
        parsed_base_estimation = yaml.safe_load(f)['base_estimation']['ros__parameters']
        # parsed_base_estimation['base_estimation']['ros__parameters']

    with open(odometry_config, 'r') as f:
        parsed_odometry = yaml.safe_load(f)['concert_odometry']['ros__parameters']
        # parsed_odometry['concert_odometry']['ros__parameters']


    # Launch arguments
    publish_ground_truth_arg  = DeclareLaunchArgument(
        'publish_ground_truth',
        default_value='false',
        description='Enable publishing of ground truth from Gazebo to TF'
    )

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='false',
        description='Enable GUI visualization with RViz'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time (useful for Gazebo simulations)'
    )

    publish_ground_truth = LaunchConfiguration("publish_ground_truth")
    gui = LaunchConfiguration("gui")
    use_sim_time = LaunchConfiguration('use_sim_time')


    # Load the file contents as a string
    with open(odometry_stack_config, 'r') as file:
        ik_problem_content = file.read()

    # Main base estimation node with external YAML configuration file
    base_estimation_node = Node(
        package='base_estimation',
        executable='base_estimation_node',
        name='concert_odometry',
        output='screen',
        parameters=[
                {'ik_problem': ik_problem_content},
                parsed_base_estimation,  # Loading parameters from YAML file
                {'world_from_tf': 'world' if publish_ground_truth == 'true' else {}},
                parsed_odometry,  # Loading parameters from YAML fileV
                {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/robot_description', '/xbotcore/robot_description'),
            ('/robot_description_semantic', '/xbotcore/robot_description_semantic')
        ]
    )

    # Publish ground truth from Gazebo to TF
    gazebo_tf_publisher_node = Node(
        condition=IfCondition(publish_ground_truth),
        package='base_estimation',
        executable='gazebo_tf_publisher',
        name='gz_tf_publisher',
        output='screen',
        parameters=[
            {'gz_link': 'ModularBot::base_link'},
            {'tf_link': 'base_link'},
            {'world': 'world'},
            {'use_sim_time': use_sim_time}
        ]
    )

    # Connect ground truth world to odometry world
    static_transform_publisher_node = Node(
        condition=IfCondition(publish_ground_truth),
        package='tf2_ros',
        executable='static_transform_publisher',
        name='connect_worlds',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'odometry/world', '10']
    )

    # RViz Node
    rviz2_node = Node(
        condition=IfCondition(gui),
        package='rviz2',
        executable='rviz2',
        name='odom_rviz',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', rviz_config])  

    return LaunchDescription([
        publish_ground_truth_arg,
        gui_arg,
        use_sim_time_arg,
        base_estimation_node,
        gazebo_tf_publisher_node,
        static_transform_publisher_node,
        rviz2_node
    ])
