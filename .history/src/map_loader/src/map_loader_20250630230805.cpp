#include "map_loader/map_loader.hpp"
#include <fstream>
#include <sstream>

MapLoader::MapLoader() : Node("map_loader")
{
    this->declare_parameter<std::string>("map_file", "config/map.txt");
    std::string map_file_path = this->get_parameter("map_file").as_string();

    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

    load_map_from_txt(map_file_path);
}

void MapLoader::load_map_from_txt(const std::string &file_path)
{
    std::ifstream file(file_path);
    if (!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open map file: %s", file_path.c_str());
        return;
    }

    int width, height;
    file >> width >> height;

    nav_msgs::msg::OccupancyGrid grid;
    grid.header.frame_id = "map";
    grid.info.resolution = 1.0;  // 单位尺寸，可修改
    grid.info.width = width;
    grid.info.height = height;
    grid.info.origin.position.x = 0.0;
    grid.info.origin.position.y = 0.0;
    grid.info.origin.orientation.w = 1.0;

    grid.data.resize(width * height);

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int value;
            file >> value;
            grid.data[y * width + x] = (value == 1) ? 100 : 0;  // ROS 中 100 为障碍物
            RCLCPP_INFO(this->get_logger(), "Loaded map: %dx%d:%d", y, x, grid.data[y * width + x]);
        }
    }

    RCLCPP_INFO(this->get_logger(), "Loaded map: %dx%d", width, height);
    map_pub_->publish(grid);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapLoader>());
    rclcpp::shutdown();
    return 0;
}
