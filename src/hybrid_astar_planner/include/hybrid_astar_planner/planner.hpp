#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "hybrid_astar_algorithm/hybrid_a_star.h"
#include "hybrid_astar_algorithm/dynamicvoronoi.h"

class HybridAStarPlanner : public rclcpp::Node {
public:
    HybridAStarPlanner();

private:
    // 回调函数
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void start_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    bool isSynchronized(
        const nav_msgs::msg::OccupancyGrid &map,
        const geometry_msgs::msg::PoseStamped &start,
        const geometry_msgs::msg::PoseStamped &goal) const; // 检查地图、起点和目标点是否同步
    void try_plan();  // 在条件满足时触发规划
    void hybridAStarInit(); // 初始化 Hybrid A* 规划器

    // 规划相关
    std::shared_ptr<HybridAStar> kinodynamic_astar_searcher_ptr_;

    // The voronoi diagram
    DVORONOI::DynamicVoronoi voronoiDiagram; //Voroni Diagram

    // ROS 接口
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr start_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    // 数据缓存
    nav_msgs::msg::OccupancyGrid::SharedPtr current_map_ptr_;
    geometry_msgs::msg::PoseStamped::SharedPtr start_pose_ptr_;
    geometry_msgs::msg::PoseStamped::SharedPtr goal_pose_ptr_;

    // 规划结果
    nav_msgs::msg::Path planned_path_;

    // 状态标志
    bool has_map_ = false;
    bool has_start_ = false;
    bool has_goal_ = false;
};
