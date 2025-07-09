from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare("path_smoother").find("path_smoother")

    return LaunchDescription([
        # 混合A*规划器节点
        Node(
            package="path_smoother",
            executable="smoother_node",
            name="path_smoother",
            output="screen",
            parameters=[os.path.join(pkg_share, "config", "planner_params.yaml")],
        )
    ])
