#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class PoseBridgeNode : public rclcpp::Node {
public:
    PoseBridgeNode() : Node("pose_bridge_node") {
        sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10,
            std::bind(&PoseBridgeNode::callback, this, std::placeholders::_1));

        pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/start_point", 10);

        RCLCPP_INFO(this->get_logger(), "PoseBridgeNode started.");
    }

private:
    void callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        geometry_msgs::msg::PoseStamped out_msg;
        out_msg.header = msg->header;
        out_msg.pose = msg->pose.pose;

        pub_->publish(out_msg);
        RCLCPP_INFO(this->get_logger(), "Forwarded initialpose to /start_point");
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PoseBridgeNode>());
    rclcpp::shutdown();
    return 0;
}
