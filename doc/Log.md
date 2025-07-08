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
6. [安装Glog库](https://www.cnblogs.com/tdyizhen1314/p/18047566)

**2025.07.08**

1. 完善了`planner`的路径发布逻辑
2. 路径优化节点和规划节点有共用的头文件，GPT推荐单独开一个包来构建一个共用库包，构建共用包`planning_utils`，需要修改`planner`的文件结构和一些代码，解决了此处关于CMake的问题，添加经验文档
3. 新建`path_smoother`包及节点代码，待完善



********

GPT输入样例

```text
1. 包名：hybrid_astar_planner
2. 源码结构：
   - src/bucketedqueue.cpp
        #include "hybrid_astar_algorithm/bucketedqueue.h"
        #include "limits.h"
        #include <stdio.h>
        #include <stdlib.h>
   - src/dubins.cpp
        #include "hybrid_astar_algorithm/dubins.h"
        #include <math.h>
        #include <assert.h>
   - src/dynamicvoronoi.cpp
        #include "hybrid_a_star/dynamicvoronoi.h"
        #include <math.h>
        #include <iostream>
        #include <opencv4/opencv2/core.hpp>
   - src/hybrid_a_star.cpp
        #include "hybrid_astar_algorithm/hybrid_a_star.h"
        #include "hybrid_astar_algorithm/display_tools.h"
        #include "hybrid_astar_algorithm/timer.h"
        #include "hybrid_astar_algorithm/trajectory_optimizer.h"
        #include <iostream>
   - src/planner_node.cpp
		#include "hybrid_astar_planner/planner.hpp"
   - src/planner.cpp
		#include "hybrid_astar_planner/planner.hpp"
   - src/rs_path.cpp
        #include "hybrid_a_star/rs_path.h"
        #include <glog/logging.h>
        #include <iostream>
   - include/hybrid_astar_planner/planner.hpp
		#include "rclcpp/rclcpp.hpp"
        #include "geometry_msgs/msg/pose_stamped.hpp"
        #include "nav_msgs/msg/occupancy_grid.hpp"
        #include "nav_msgs/msg/path.hpp"
        #include "hybrid_astar_algorithm/hybrid_a_star.h"
        #include "hybrid_astar_algorithm/dynamicvoronoi.h"
   - include/hybrid_astar_algorithm/bucketedqueue.hpp
        #include <vector>
        #include <set>
        #include <queue>
        #include <assert.h>
        #include "hybrid_astar_algorithm/point.h"
   - include/hybrid_astar_algorithm/dubins.hpp
   - include/hybrid_astar_algorithm/dynamicvoronoi.hpp
        #include <stdlib.h>
        #include <stdio.h>
        #include <limits.h>
        #include <queue>
        #include "bucketedqueue.h"
   - include/hybrid_astar_algorithm/hybrid_a_star.hpp
        #include "rs_path.h"
        #include "state_node.h"
        #include "dubins.h"
        #include <glog/logging.h>
        #include <map>
        #include <memory>
   - include/hybrid_astar_algorithm/point.hpp
   - include/hybrid_astar_algorithm/rs_path.hpp
        #include "type.h"
        #include <cmath>
        #include <limits>
   - include/hybrid_astar_algorithm/state_node.hpp
        #include "type.h"
        #include <Eigen/Dense>
   - include/hybrid_astar_algorithm/timer.hpp
        #include <string>
        #include <iostream>
        #include <chrono>
   - include/hybrid_astar_algorithm/type.hpp
        #include <vector>
        #include <Eigen/Core>
3. 可执行文件：
   - planner_node（由src/planner_node.cpp编译）
4. 依赖包：
   - rclcpp
   - geometry_msgs
   - nav_msgs
   - eigen
   - opencv
5. 其他需要安装的资源：
   - config/planner_params.yaml
   - launch/planner.launch.py
6. 只需可执行文件，不需要生成库/插件

```

```txt
1. 包名：planning_utils
2. 源码结构：
   - src/bucketedqueue.cpp
   - src/dynamicvoronoi.cpp
   - include/planning_utils/bucketedqueue.hpp
   - include/planning_utils/dynamicvoronoi.hpp
   - include/planning_utils/point.hpp
   - include/planning_utils/state_node.hpp
   - include/planning_utils/type.hpp
3. 可执行文件：
   - 无可执行文件
4. 依赖包：
   - eigen
   - opencv
5. 其他需要安装的资源：
   无
```





