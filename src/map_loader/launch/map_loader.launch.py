from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='map_loader',
            executable='map_loader_node',
            name='map_loader',
            parameters=[{'map_file': 'src/map_loader/config/map.txt'}],
            output='screen'
        )
    ])
