from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    planner_share = get_package_share_directory('hybrid_astar_planner')
    smoother_share = get_package_share_directory('path_smoother')
    map_tools_share = get_package_share_directory('map_tools')
    mpc_car_share = get_package_share_directory('mpc_car')
    car_simulator_share = get_package_share_directory("car_simulator")
    rviz_cfg = os.path.join(planner_share, 'rviz', 'planner_view.rviz')

    return LaunchDescription([
        # nav2地图服务节点
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': os.path.join(map_tools_share, 'maps', 'map.yaml')}]
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
        # 地图加载节点
        Node(
            package='map_tools',
            executable='map_loader_node',
            name='map_loader',
            output='screen'
        ),
        # 起点桥接节点
        Node(
            package="map_tools",
            executable="pose_bridge_node",
            name="pose_bridge",
            output="screen",
        ),
        # 启动车辆仿真节点
        Node(
            package="car_simulator",
            executable="car_simulator_node",
            name="car_simulator_node",
            output="screen",
            parameters=[os.path.join(car_simulator_share, "config", "car_simulator.yaml")]
        ),
        # 启动MPC车辆节点
        Node(
            package='mpc_car',
            executable='mpc_car_node',
            name='mpc_car',
            output='screen',
            parameters=[os.path.join(mpc_car_share, 'config', 'mpc_car.yaml')]
        ),
        # 订阅processed_map的节点，延时3秒启动
        TimerAction(
            period=3.0,
            actions=[
                # 混合A*规划器节点
                Node(
                    package='hybrid_astar_planner',
                    executable='planner_node',
                    name='hybrid_astar_planner',
                    output='screen',
                    parameters=[os.path.join(planner_share, 'config', 'planner_params.yaml')]
                ),
                # 路径平滑器节点
                Node(
                    package='path_smoother',
                    executable='smoother_node',
                    name='path_smoother',
                    output='screen',
                    parameters=[os.path.join(smoother_share, "config", "planner_params.yaml")],
                )
            ]
        ),
        # 自动加载 RViz2 可视化配置
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_cfg],
            output='screen'
        ),
    ])
