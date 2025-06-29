// main_node.cpp
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"

class TrajectoryNode : public rclcpp::Node
{
public:
    TrajectoryNode() : Node("trajectory_node")
    {
        RCLCPP_INFO(this->get_logger(), "TrajectoryNode started.");

        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/planned_path", 10,
            std::bind(&TrajectoryNode::path_callback, this, std::placeholders::_1));

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&TrajectoryNode::control_loop, this));
    }

private:
    void path_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received path with %ld poses.", msg->poses.size());
        latest_path_ = *msg;
    }

    void control_loop()
    {
        // 伪控制逻辑：这里只是发布一个恒定速度作为示例
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = 1.0;
        cmd.angular.z = 0.0;
        cmd_pub_->publish(cmd);
    }

    nav_msgs::msg::Path latest_path_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryNode>());
    rclcpp::shutdown();
    return 0;
}
