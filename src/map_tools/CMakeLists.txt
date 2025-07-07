cmake_minimum_required(VERSION 3.5)
project(map_tools)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# 头文件目录
include_directories(include)

# map_loader_node
add_executable(map_loader_node src/map_loader.cpp)
ament_target_dependencies(map_loader_node rclcpp nav_msgs)

# pose_bridge_node
add_executable(pose_bridge_node src/initialpose_to_startpoint.cpp)
ament_target_dependencies(pose_bridge_node rclcpp geometry_msgs)

# 安装可执行文件
install(TARGETS pose_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS map_loader_node
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY launch maps
  DESTINATION share/${PROJECT_NAME}
)
  

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
