# ROS2多包工程CMake实用总结（以hybrid_astar_planner和planning_utils为例）

## 一、共用工具库包（如 planning_utils）的 CMake 要点

1. **库目标创建与安装**

   - 使用 `add_library(planning_utils STATIC ...)` 或 `add_library(planning_utils SHARED ...)` 创建目标。
   - 推荐以包名为库名，便于管理和自动依赖。

2. **头文件管理**

   - 共用头文件放在 `include/planning_utils/xxx.hpp`。
   - `install(DIRECTORY include/ DESTINATION include/)` 安装头文件。

3. **依赖导出与库导出**

   - `ament_export_include_directories(include)` 导出头文件目录。
   - `ament_export_libraries(planning_utils)` 导出库目标，供依赖包自动链接。

4. **目标链接外部依赖**

   - 例如：

     ```cmake
     target_link_libraries(planning_utils Eigen3::Eigen ${OpenCV_LIBS})
     ```

   - `find_package(Eigen3 REQUIRED)`、`find_package(OpenCV REQUIRED)` 等提前声明依赖。

5. **最后调用 ament_package()**

   - 一定要放在最后，**且在此之前完成所有导出和install相关设置。**

**示例片段：**

```cmake
add_library(planning_utils STATIC ${UTILS_SOURCES})
target_link_libraries(planning_utils Eigen3::Eigen ${OpenCV_LIBS})
install(TARGETS planning_utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include/)
ament_export_include_directories(include)
ament_export_libraries(planning_utils)
ament_package()
```

------

## 二、功能包（如 hybrid_astar_planner）的 CMake 要点

1. **依赖库声明**

   - 用 `find_package(planning_utils REQUIRED)`。
   - 其他依赖如 `rclcpp`、`nav_msgs`、`tf2`等同理。

2. **目标依赖管理**

   - 推荐只用

     ```cmake
     ament_target_dependencies(planner_node
       rclcpp nav_msgs geometry_msgs tf2 planning_utils ...)
     ```

     这样自动加好头文件和链接依赖。

   - **不需要也不要**手动 `target_link_libraries(planner_node planning_utils)`，会导致重复查找或链接报错。

3. **头文件引用**

   - 只需 `#include <planning_utils/xxx.hpp>`，头文件搜索路径会自动处理。

4. **本包的头文件安装**

   - 如果有公共头文件，也应用

     ```cmake
     install(DIRECTORY include/ DESTINATION include/)
     ```

5. **其他第三方库链接**

   - 比如 GLOG、Eigen、OpenCV，若需要可写

     ```cmake
     target_link_libraries(planner_node
       Eigen3::Eigen
       ${OpenCV_LIBS}
       ${GLOG_LIBRARIES}
     )
     ```

6. **最终 install 可执行文件**

   - 一般如下

     ```cmake
     install(TARGETS planner_node
       DESTINATION lib/${PROJECT_NAME}
     )
     ```

**示例片段：**

```cmake
find_package(planning_utils REQUIRED)
add_executable(planner_node ${SOURCES})
ament_target_dependencies(planner_node
  rclcpp nav_msgs geometry_msgs tf2 planning_utils)
target_link_libraries(planner_node
  Eigen3::Eigen
  ${OpenCV_LIBS}
  ${GLOG_LIBRARIES}
)
install(TARGETS planner_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/)
ament_package()
```

------

## 三、常见易错点与建议

- **只用 ament_target_dependencies 来管理ROS包依赖即可，不要重复用 target_link_libraries 链接本地包目标名，否则容易出现找不到库/重复链接/路径不一致的问题。**
- **必须 ament_export_include_directories/include ament_export_libraries**，否则依赖包无法自动找到头文件和库。
- **install 头文件和库**是被依赖包能找到的前提，注意目录结构一致（如 include/planning_utils/xxx.hpp）。
- **全 workspace clean build**，如遇路径/链接问题，优先 `colcon build --cmake-clean-cache`。
- **优先用标准cmake语法与ament推荐写法，避免自定义或手写特殊路径。**
- **调试时可查看 build/\<pkg\>/CMakeFiles/target.dir/flags.make 或 compile_commands.json 检查实际 -I 和 -L 路径**。

------

## 四、结论

- **推荐架构**：“基础共用工具库包 + 业务功能包 + ament标准依赖集成”
- **只要工具库包 CMake 标准导出，功能包只需 find_package 和 ament_target_dependencies 即可自动集成所有头文件和库，无需特殊配置。**
- **遵循上述方法，可以高效地支持多包、多团队协作开发、后期维护与复用！**