# 基于Hybrid A Star的车辆路径规划

## 0. 项目描述

This project uses C++ and ROS2 to implement vehicle trajectory planning and control, combining Hybrid A* and Model Predictive Control (MPC) for autonomous path generation and dynamic tracking.

**技术栈**：

* 开发平台：UBuntu 20.04

* 工具类：

  * C++
  * ROS2-foxy
  * git + github

* 算法类：

  * Hybrid A Star：[zm0612/Hybrid_A_Star: Hybrid A Star algorithm C++ implementation](https://github.com/zm0612/Hybrid_A_Star)
  * MPC

**参考**：

* [qimao7213/Hybrid_A_Star-and-mpc_controller](https://github.com/qimao7213/Hybrid_A_Star-and-mpc_controller)
* [Hybrid A+MPC controller+倒车路径跟踪_哔哩哔哩_bilibili](https://www.bilibili.com/video/BV14y411a77H/?spm_id_from=333.1387.upload.video_card.click&vd_source=62a3fea4dfcaedbff9492ac5c593f178)
* [0_展示视频_哔哩哔哩_bilibili](https://www.bilibili.com/video/BV1T9TvzUEmF/?spm_id_from=333.1007.top_right_bar_window_history.content.click&vd_source=62a3fea4dfcaedbff9492ac5c593f178)

## 1. 项目规划

**核心功能**：

1. 基于混合A\*的路径规划
2. 基于MPC的车辆控制

**基础功能**：

1. 地图导入与建图
   * 0.1：只考虑静态地图，不考虑动态
2. 轨迹优化

![image-20250630160332100](https://picgo-img-lyx.oss-cn-hangzhou.aliyuncs.com/img/image-20250630160332100.png)

****

**拟加入功能**

1. 利用Carla-ROS-Bridge，将模块应用于Carla仿真

****

**进阶功能**

能够接入神经网络模型



## 2. 模块功能描述及接口设计

### 2.1 地图导入与建图

**功能描述**：

加载静态地图，为路径规划提供障碍物信息。

* 进阶：动态构建环境地图（SLAM）。

* 建议：使用 `nav2_map_server` 提供地图服务

**包名**：`map_loader`

**节点名**：`map_loader`

**输入**：

|      数据名      |                数据描述                |            数据类型            |    订阅    |
| :--------------: | :------------------------------------: | :----------------------------: | :--------: |
|      `msg`       | 静态地图文件，由`nav2_map_server`提供  | `nav_msgs::msg::OccupancyGrid` |   `/map`   |
| `currentMapData` | 动态地图信息（如果有的话，初期不考虑） |                                | `/cur_map` |

**输出**：	

|     数据名      | 数据描述 |            数据类型            |       发布       |
| :-------------: | :------: | :----------------------------: | :--------------: |
| `processed_map` | 地图文件 | `nav_msgs::msg::OccupancyGrid` | `/processed_map` |

### 2.2 路径规划

**功能描述**：

基于当前起点、终点和障碍物信息，生成可行的车辆路径。

* 使用车辆运动模型生成路径；

* 支持前进与倒车路径搜索；

* 可使用开源项目如 [zm0612/Hybrid_A_Star](https://github.com/zm0612/Hybrid_A_Star) 进行改造。

**输入**：

|    数据名    | 数据描述 |             数据类型              |       订阅       |
| :----------: | :------: | :-------------------------------: | :--------------: |
|  `mapData`   | 地图文件 |  `nav_msgs::msg::OccupancyGrid`   | `/processed_map` |
| `startPoint` |   起点   | `geometry_msgs::msg::PoseStamped` |  `/start_point`  |
|  `endPoint`  |   终点   | `geometry_msgs::msg::PoseStamped` |   `/goal_pose`   |

**输出**：

|    数据名     |         数据描述         |       数据类型        |      发布       |
| :-----------: | :----------------------: | :-------------------: | :-------------: |
| `plannedPath` | 路径，包含姿态与方向信息 | `nav_msgs::msg::Path` | `/planned_path` |

### 2.3 路径优化

**功能描述**：

对 A* 输出的折线路径进行平滑或再优化，提高可控性和轨迹平稳性。

* 优化方式：Cubic Splines、Bezier 曲线、CHOMP、Ceres 等

**输入**：

|    数据名     |          数据描述          |       数据类型        |      订阅       |
| :-----------: | :------------------------: | :-------------------: | :-------------: |
| `plannedPath` | 来自路径规划模块的原始路径 | `nav_msgs::msg::Path` | `/planned_path` |

**输出**：

|     数据名     |   数据描述   |       数据类型        |       发布       |
| :------------: | :----------: | :-------------------: | :--------------: |
| `smoothedPath` | 平滑后的路径 | `nav_msgs::msg::Path` | `/smoothed_path` |

### 2.4 MPC控制

**功能描述**：

使用模型预测控制根据当前状态和目标轨迹计算控制指令（速度 + 转向）。

* 控制频率：建议 10~50Hz

* MPC 实现建议：
  * 使用 CppAD + Ipopt
  * 考虑车辆动力学（Kinematic / Dynamic Bicycle Model）

**输入**：

|     数据名     | 数据描述 |         数据类型          |      订阅       |
| :------------: | :------: | :-----------------------: | :-------------: |
| `smoothedPath` | 平滑轨迹 |   `nav_msgs::msg::Path`   | `/planned_path` |
| `currentState` | 当前状态 | `nav_msgs::msg::Odometry` |     `/odom`     |

**输出**：

|    数据名    | 数据描述 |                   数据类型                   |    发布    |
| :----------: | :------: | :------------------------------------------: | :--------: |
| `cnlCommand` | 控制命令 | `geometry_msgs::msg::Twist` 或自定义控制消息 | `/cmd_vel` |

### 2.5 仿真器

**功能描述**：

提供车辆模型与环境仿真，测试路径规划与控制器。

**输入**：

|    数据名    | 数据描述 |                   数据类型                   |    订阅    |
| :----------: | :------: | :------------------------------------------: | :--------: |
| `cnlCommand` | 控制命令 | `geometry_msgs::msg::Twist` 或自定义控制消息 | `/cmd_vel` |

**输出**：

|      数据名      |            数据描述            |         数据类型          |    发布    |
| :--------------: | :----------------------------: | :-----------------------: | :--------: |
|  `currentState`  |          车辆状态反馈          | `nav_msgs::msg::Odometry` |  `/odom`   |
| `currentMapData` | 实时地图信息（当前阶段不考虑） |                           | `/cur_map` |



### 2.6 Rviz可视化





