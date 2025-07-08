#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include "smoother/smoother.hpp"  // 假设你的Smoother类在smoother/include/smoother/smoother.hpp

class class SmootherNode : public rclcpp::Node {
public:
    SmootherNode();

private:
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    // 成员变量
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr smooth_path_pub_;
    std::shared_ptr<Smoother> smoother_;
};
