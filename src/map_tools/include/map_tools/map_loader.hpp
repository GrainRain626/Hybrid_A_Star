#pragma once

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

class MapLoader : public rclcpp::Node {
public:
    MapLoader();

private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
};
