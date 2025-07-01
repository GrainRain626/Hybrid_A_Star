#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"

class HybridAStarPlanner : public rclcpp::Node {
public:
    HybridAStarPlanner();

private:
    // 回调函数
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void start_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    void try_plan();  // 在条件满足时触发规划

    // ROS 接口
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr start_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    // 数据缓存
    nav_msgs::msg::OccupancyGrid current_map_;
    // geometry_msgs::msg::PoseStamped start_pose_;
    geometry_msgs::msg::PoseWithCovarianceStamped start_pose_;
    geometry_msgs::msg::PoseStamped goal_pose_;

    bool has_map_ = false;
    bool has_start_ = false;
    bool has_goal_ = false;
};
