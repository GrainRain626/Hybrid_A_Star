from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import ThisLaunchFileDir
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    map_loader_pkg = FindPackageShare("map_loader").find("map_loader")
    rviz_config_path = os.path.join(map_loader_pkg, "rviz", "map_display.rviz")
    map_txt_path = os.path.join(map_loader_pkg, "config", "map.txt")

    return LaunchDescription([
        Node(
            package="map_loader",
            executable="map_loader_node",
            name="map_loader",
            parameters=[{"map_file": map_txt_path}]
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config_path],
            output="screen"
        )
    ])
