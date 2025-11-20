from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Default paths
    default_param_file = '/home/troublemaker/comp3431/turtlebot_ws/src/wall_follower/config/waypoint_nav_params.yaml'
    default_map_file = '/home/troublemaker/map.yaml'
    default_waypoints_file = '/home/troublemaker/comp3431/turtlebot_ws/landmarks.csv'
    default_use_sim_time = 'false'

    # Path to nav2 launch file
    nav2_launch_file = os.path.join(
        get_package_share_directory('turtlebot3_navigation2'),
        'launch',
        'navigation2.launch.py'
    )

    params_file = LaunchConfiguration('params_file')
    map_file = LaunchConfiguration('map')
    waypoints_file = LaunchConfiguration('waypoints_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument('params_file', default_value=default_param_file),
        DeclareLaunchArgument('map', default_value=default_map_file),
        DeclareLaunchArgument('waypoints_file', default_value=default_waypoints_file),
        DeclareLaunchArgument('use_sim_time', default_value=default_use_sim_time),

        # Include Nav2 launch file with parameters
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_file),
            launch_arguments={
                'use_sim_time' : use_sim_time,
                'params_file': params_file,
                'map': map_file
            }.items()
        ),

        # Launch waypoint navigator node
        Node(
            package='wall_follower',
            executable='waypoint_navigator.py',
            name='waypoint_navigator',
            output='screen',
            parameters=[{
                'waypoints_file': waypoints_file,
                'use_sim_time' : use_sim_time
            }]
        )
    ])

