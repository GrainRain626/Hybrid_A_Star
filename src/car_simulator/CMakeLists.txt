find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(car_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(car_simulator_node src/car_simulator_node.cpp)
ament_target_dependencies(car_simulator_node
  rclcpp tf2 tf2_ros nav_msgs geometry_msgs car_msgs
)
target_link_libraries(car_simulator_node Eigen3::Eigen)

install(TARGETS car_simulator_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/)
