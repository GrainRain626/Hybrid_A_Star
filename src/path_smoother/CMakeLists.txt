cmake_minimum_required(VERSION 3.5)
project(path_smoother)

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
find_package(tf2 REQUIRED)
find_package(planning_utils REQUIRED)

# 源文件列表
set(SOURCES
  src/smoother.cpp
  src/smoother_node.cpp
)

# 添加源文件和生成可执行文件
add_executable(smoother_node ${SOURCES})

# 链接依赖库
ament_target_dependencies(smoother_node
  rclcpp 
  tf2
  planning_utils
)

# 安装可执行文件
install(TARGETS smoother_node
  DESTINATION lib/${PROJECT_NAME}
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
