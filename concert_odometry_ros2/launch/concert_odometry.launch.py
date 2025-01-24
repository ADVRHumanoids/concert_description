from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():

    # --- Get package directory and config paths ---
    concert_odom_dir = get_package_share_directory('concert_odometry_ros2')
    config_dir = os.path.join(concert_odom_dir, 'config')
    base_estimation_config = os.path.join(config_dir, 'base_estimation_params.yaml')
    odometry_config = os.path.join(config_dir, 'concert_odometry.yaml')
    odometry_stack_config = os.path.join(config_dir, 'concert_odometry_stack.yaml')
    rviz_config = os.path.join(concert_odom_dir, 'rviz', 'concert_odometry.rviz')

    # --- Load YAML files ---
    with open(base_estimation_config, 'r') as f:
        parsed_base_estimation = yaml.safe_load(f)['base_estimation']['ros__parameters']

    with open(odometry_config, 'r') as f:
        parsed_odometry = yaml.safe_load(f)['concert_odometry']['ros__parameters']

    with open(odometry_stack_config, 'r') as f:
        ik_problem_content = f.read()

    # --- Declare Launch Arguments ---
    publish_ground_truth_arg = DeclareLaunchArgument(
        'publish_ground_truth',
        default_value='false',
        description='Enable publishing of ground truth from Gazebo to TF'
    )
    
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='false',
        description='Enable GUI visualization with RViz'
    )
    
    # 1) Pull the environment variable 'USE_SIM_TIME', default to 'true' if not set
    #    So if the user doesn't do export USE_SIM_TIME=..., it will assume 'true'.
    use_sim_time_env = EnvironmentVariable(name='USE_SIM_TIME', default_value='true')

    # 2) Use this environment variable as the default for the 'use_sim_time' LaunchConfiguration
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value=use_sim_time_env,
        description='Use simulation time (useful for Gazebo simulations)'
    )

    # --- Retrieve LaunchConfiguration objects ---
    publish_ground_truth = LaunchConfiguration('publish_ground_truth')
    gui = LaunchConfiguration('gui')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # --- Main base estimation node ---
    base_estimation_node = Node(
        package='base_estimation',
        executable='base_estimation_node',
        name='concert_odometry',
        output='screen',
        parameters=[
            {'ik_problem': ik_problem_content},
            parsed_base_estimation,  
            parsed_odometry,
            {
            'world_from_tf': PythonExpression([
                '"world" if "', publish_ground_truth, '" == "true" else ""'
            ])
            },
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/robot_description', '/xbotcore/robot_description'),
            ('/robot_description_semantic', '/xbotcore/robot_description_semantic')
        ]
    )

    # --- Publish ground truth from Gazebo to TF, only if publish_ground_truth==true ---
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

    # --- Connect ground truth world to odometry world, only if publish_ground_truth==true ---
    static_transform_publisher_node = Node(
        condition=IfCondition(publish_ground_truth),
        package='tf2_ros',
        executable='static_transform_publisher',
        name='connect_worlds',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'odometry/world', '10']
    )

    # --- RViz Node, only if gui==true ---
    rviz2_node = Node(
        condition=IfCondition(gui),
        package='rviz2',
        executable='rviz2',
        name='odom_rviz',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', rviz_config]
    )

    return LaunchDescription([
        publish_ground_truth_arg,
        gui_arg,
        use_sim_time_arg,
        base_estimation_node,
        gazebo_tf_publisher_node,
        static_transform_publisher_node,
        rviz2_node
    ])
