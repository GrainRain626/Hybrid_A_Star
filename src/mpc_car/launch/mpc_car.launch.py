import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    mpc_car_share = get_package_share_directory('mpc_car')
    return LaunchDescription([
        Node(
            package='mpc_car',
            executable='mpc_car_node',
            name='mpc_car',
            output='screen',
            parameters=[os.path.join(mpc_car_share, 'config', 'mpc_car.yaml')]
        ),
    ])
