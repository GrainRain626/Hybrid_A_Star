#include "hybrid_astar_planner/planner.hpp"

HybridAStarPlanner::HybridAStarPlanner() : Node("hybrid_astar_planner")
{
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/processed_map", 10,
        std::bind(&HybridAStarPlanner::map_callback, this, std::placeholders::_1));

    start_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStampedPtr>(
        "/start_point", 10,
        std::bind(&HybridAStarPlanner::start_callback, this, std::placeholders::_1));

    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/end_point", 10,
        std::bind(&HybridAStarPlanner::goal_callback, this, std::placeholders::_1));

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);

    RCLCPP_INFO(this->get_logger(), "Hybrid A* Planner Node started.");
}

void HybridAStarPlanner::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    current_map_ = *msg;
    RCLCPP_INFO(this->get_logger(), "load map successfully, resolution: %f",
                current_map_.info.resolution);
    has_map_ = true;
    try_plan();
}

void HybridAStarPlanner::start_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    start_pose_ = msg;
    RCLCPP_INFO(this->get_logger(), "load start point successfully, position: (%f, %f)",
                start_pose_.pose.position.x, start_pose_.pose.position.y);
    has_start_ = true;
    try_plan();
}

void HybridAStarPlanner::goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    goal_pose_ = *msg;
    RCLCPP_INFO(this->get_logger(), "load goal point successfully, position: (%f, %f)",
                goal_pose_.pose.position.x, goal_pose_.pose.position.y);
    has_goal_ = true;
    try_plan();
}

void HybridAStarPlanner::try_plan()
{
    if (!(has_map_ && has_start_ && has_goal_))
        return;

    RCLCPP_INFO(this->get_logger(), "Planning path using Hybrid A*...");

    // 这里你将接入 Hybrid A* 算法：
    // 1. 将地图转换为 costmap/gridmap
    // 2. 调用你的 Hybrid A* 规划器接口
    // 3. 生成 nav_msgs::msg::Path

    nav_msgs::msg::Path dummy_path;
    dummy_path.header.frame_id = "map";
    dummy_path.header.stamp = this->get_clock()->now();

    dummy_path.poses.push_back(start_pose_);
    dummy_path.poses.push_back(goal_pose_);

    path_pub_->publish(dummy_path);
    RCLCPP_INFO(this->get_logger(), "Path published.");
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HybridAStarPlanner>());
    rclcpp::shutdown();
    return 0;
}

