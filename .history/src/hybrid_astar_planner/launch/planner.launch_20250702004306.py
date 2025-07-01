from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare("hybrid_astar_planner").find("hybrid_astar_planner")

    return LaunchDescription([
        # 起点桥接节点
        Node(
            package="hybrid_astar_planner",
            executable="pose_bridge_node",
            name="pose_bridge",
            output="screen",
        ),
        Node(
            package="hybrid_astar_planner",
            executable="planner_node",
            name="hybrid_astar_planner",
            output="screen",
        ),

        # 可选：自动加载 RViz2 可视化配置（如存在）
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=[
                "-d", os.path.join(pkg_share, "rviz", "planner_view.rviz")
            ],
            output="screen"
        )
    ])
