**2025.07.07**

1. `map_loader`包改名为`map_tools`
2. 将`hybrid_astar_planner`包中的`initialpose_to_startpoint`节点移动到`map_tools`包下
   * 需要重新生成一个包：`ros2 pkg create --build-type ament_cmake map_loader`
   * 然后将相关文件拷贝，并修改`map_tools`和`hybrid_astar_planner`的`cmake`和`package`文件
3. 修改`doc`相关文件
4. 重新理一下`hybrid_astar_planner`的执行逻辑
5. TODO：
   * 将混合A*代码中涉及ROS1的部分去除（目前来看是`display_tools.h`这一块）
   * 将混合A*代码中涉及轨迹优化的部分去除
   * 已经大体移植完毕，需要检查



