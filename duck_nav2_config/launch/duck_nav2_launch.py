# duck_nav2_config/launch/duck_nav2_launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('duck_nav2_config')

    # Declare arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    map_yaml_file = LaunchConfiguration('map')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all nav2 nodes')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(bringup_dir, 'maps', 'small_house.yaml'),
        description='Full path to map yaml file to load')

    # --- Nodes to Launch ---

    # 1. Include the standard Nav2 bringup launch file
    # This is the simplest and most reliable way to launch Nav2
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'map': map_yaml_file
        }.items()
    )

    # 2. Our custom bridge node
    # This node will subscribe to '/cmd_vel' and publish to '/duck_control/cmd_vel'
    twist_republisher_node = Node(
        package='duck_nav2_config',
        executable='twist_to_twist_stamped_republisher.py',
        name='twist_republisher',
        output='screen',
        parameters=[{
            'input_topic': '/cmd_vel',
            'output_topic': '/duck_control/cmd_vel'
        }]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_map_yaml_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(navigation_launch)
    ld.add_action(twist_republisher_node)

    return ld