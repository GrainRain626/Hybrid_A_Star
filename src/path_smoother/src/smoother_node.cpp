#include "path_smother/smoother_node.hpp"

SmootherNode::SmootherNode() : Node("smoother_node") {
    // 创建订阅器
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/planned_path",
        10,
        std::bind(&SmootherNode::pathCallback, this, std::placeholders::_1)
    );

    // 创建发布器
    smooth_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/smoothed_path", 10);

    // 初始化Smoother类
    smoother_ = std::make_shared<Smoother>();

    RCLCPP_INFO(this->get_logger(), "Smoother node started, waiting for /planned_path...");
}

void SmootherNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
    if (msg->poses.empty()) {
        RCLCPP_WARN(this->get_logger(), "Received empty path. Skipping smoothing.");
        return;
    }
    // 路径平滑处理
    nav_msgs::msg::Path smoothed_path = smoother_->smoothPath(*msg);

    // 设置发布路径的header
    smoothed_path.header.stamp = this->now();
    smoothed_path.header.frame_id = msg->header.frame_id;

    // 发布平滑后的路径
    smooth_path_pub_->publish(smoothed_path);

    RCLCPP_INFO(this->get_logger(), "Smoothed path published, poses count: %zu", smoothed_path.poses.size());
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SmootherNode>>());
    rclcpp::shutdown();
    return 0;
}
