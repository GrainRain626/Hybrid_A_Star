from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
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
        # 启动 lifecycle_manager 来激活 map_server
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[
                {'autostart': True},
                {'node_names': ['map_server']}
            ]
        ),
        Node(
            package='map_loader',
            executable='map_loader_node',
            name='map_loader',
            output='screen'
        )
    ])
