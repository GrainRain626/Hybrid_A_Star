from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import ThisLaunchFileDir
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import os

def generate_launch_description():
    map_loader_pkg = FindPackageShare("map_loader").find("map_loader")
    rviz_config_path = os.path.join(map_loader_pkg, "rviz", "map_display.rviz")

    map_path = PathJoinSubstitution([
        FindPackageShare('map_loader'),
        'maps',
        'map.yaml'
    ])

    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_path}]
        ),
        Node(
            package="map_loader",
            executable="map_loader_node",
            name="map_loader",
            output='screen',
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config_path],
            output="screen"
        )
    ])
