cmake_minimum_required(VERSION 3.5)
project(hybrid_astar_planner)

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
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(tf2 REQUIRED)
pkg_check_modules(GLOG REQUIRED libglog)
# 查找planning_utils包
find_package(planning_utils REQUIRED)

# 头文件目录
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# 源文件列表
set(SOURCES
  src/bucketedqueue.cpp
  src/dubins.cpp
  src/dynamicvoronoi.cpp
  src/hybrid_a_star.cpp
  src/planner.cpp
  src/planner_node.cpp
  src/rs_path.cpp
)

# 添加源文件和生成可执行文件
add_executable(planner_node ${SOURCES})

# 链接依赖库
ament_target_dependencies(planner_node 
  rclcpp 
  nav_msgs 
  geometry_msgs
  tf2
  planning_utils
)

target_link_libraries(planner_node
  planning_utils
  Eigen3::Eigen
  ${OpenCV_LIBS}
  ${GLOG_LIBRARIES}
)
target_include_directories(planner_node PRIVATE
  ${GLOG_INCLUDE_DIRS}
)

# 安装可执行文件
install(TARGETS planner_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件（如果后续封装 Hybrid A* 类）
install(DIRECTORY include/
  DESTINATION include/
)

# 安装 launch 文件（如果有）
install(DIRECTORY launch config
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
