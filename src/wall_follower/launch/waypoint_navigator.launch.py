from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    map_file = LaunchConfiguration('map')
    waypoints_file = LaunchConfiguration('waypoints_file')

    return LaunchDescription([
        # Declare each launch argument
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value='',
            description='Full path to the ROS2 parameters YAML file'
        ),
        DeclareLaunchArgument(
            'map',
            default_value='',
            description='Full path to the map YAML file'
        ),
        DeclareLaunchArgument(
            'waypoints_file',
            default_value='',
            description='CSV file of waypoint coordinates'
        ),

        # Launch the Python waypoint navigator node
        Node(
            package='wall_follower',          
            executable='waypoint_navigator.py',  
            name='waypoint_navigator',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'map': map_file},
                {'csv_file': waypoints_file},
                params_file  
            ]
        )
    ])