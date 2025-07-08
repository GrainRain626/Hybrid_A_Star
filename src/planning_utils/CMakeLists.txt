cmake_minimum_required(VERSION 3.8)
project(planning_utils)

set(CMAKE_CXX_STANDARD 14)

# 依赖
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

# 头文件目录
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# 源文件
set(UTILS_SOURCES
  src/bucketedqueue.cpp
  src/dynamicvoronoi.cpp
)

# 构建静态库（也可以add_library(planning_utils SHARED ...)）
add_library(planning_utils STATIC ${UTILS_SOURCES})

target_link_libraries(planning_utils
  Eigen3::Eigen
  ${OpenCV_LIBS}
)

# 安装头文件和库
install(TARGETS planning_utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
