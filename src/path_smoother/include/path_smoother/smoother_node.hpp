#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "smoother.hpp"

class class SmootherNode : public rclcpp::Node {
public:
    SmootherNode();

private:
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void publishPath(const VectorVec4d &path); // 发布平滑后的路径

    // 成员变量
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr smooth_path_pub_;

    DVORONOI::DynamicVoronoi voronoiDiagram; //Voroni Diagram
    std::shared_ptr<Smoother> smoother_;
    nav_msgs::msg::Path smoothed_path_; // 规划结果
};
