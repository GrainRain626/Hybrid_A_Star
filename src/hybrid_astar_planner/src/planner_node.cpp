#include "hybrid_astar_planner/planner.hpp"


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HybridAStarPlanner>());
    rclcpp::shutdown();
    return 0;
}
