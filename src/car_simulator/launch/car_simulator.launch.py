from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory("car_simulator")
    param_file = os.path.join(pkg_share, "config", "car_simulator.yaml")

    return LaunchDescription([
        Node(
            package="car_simulator",
            executable="car_simulator_node",
            name="car_simulator_node",
            output="screen",
            parameters=[param_file]
        )
    ])
