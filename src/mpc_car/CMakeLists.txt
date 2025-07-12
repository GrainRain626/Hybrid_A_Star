cmake_minimum_required(VERSION 3.8)
project(car_mpc)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(car_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(osqp REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(mpc_car_node src/mpc_car_node.cpp)
ament_target_dependencies(mpc_car_node
  rclcpp
  nav_msgs
  car_msgs
)
target_link_libraries(mpc_car_node Eigen3::Eigen osqp::osqp)

install(TARGETS mpc_car_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/)
