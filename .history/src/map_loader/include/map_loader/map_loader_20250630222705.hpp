#pragma once

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

class MapLoader : public rclcpp::Node {
public:
    MapLoader();

private:
    void load_map_from_txt(const std::string &file_path);
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
};
