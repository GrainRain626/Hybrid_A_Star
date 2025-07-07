#include "hybrid_astar_planner/planner.hpp"

double vehicle_length_;
double vehicle_width_;
double vehicle_rear_dis_;
double wheel_base_;
double steering_angle_;

HybridAStarPlanner::HybridAStarPlanner() : Node("hybrid_astar_planner")
{
    /*
    TODO：参数获取，构造 Hybrid A* 规划器
    */
    // 获取参数
    double steering_angle = this->get_parameter("planner.steering_angle").as_double();
    int steering_angle_discrete_num = this->get_parameter("planner.steering_angle_discrete_num").as_int();
    double wheel_base = this->get_parameter("planner.wheel_base").as_double();
    double segment_length = this->get_parameter("planner.segment_length").as_double();
    int segment_length_discrete_num = this->get_parameter("planner.segment_length_discrete_num").as_int();
    double steering_penalty = this->get_parameter("planner.steering_penalty").as_double();
    double steering_change_penalty = this->get_parameter("planner.steering_change_penalty").as_double();
    double reversing_penalty = this->get_parameter("planner.reversing_penalty").as_double();
    double shot_distance = this->get_parameter("planner.shot_distance").as_double();
    bool reverse_enable = this->get_parameter("planner.reverse_enable").as_bool();

    vehicle_length_ = this->get_parameter("planner.vehicle_length").as_double();
    vehicle_width_ = this->get_parameter("planner.vehicle_width").as_double();
    vehicle_rear_dis_ = this->get_parameter("planner.vehicle_rear_dis").as_double();
    wheel_base_ = wheel_base;
    steering_angle_ = steering_angle;
    // 构造 Hybrid A* 规划器
    kinodynamic_astar_searcher_ptr_ = std::make_shared<HybridAStar>(
            steering_angle, steering_angle_discrete_num, segment_length, segment_length_discrete_num, wheel_base_,
            steering_penalty, reversing_penalty, steering_change_penalty, shot_distance, 72, reverse_enable
    );

    // 设置 QoS Profile 为 transient local
    rclcpp::QoS qos_profile(rclcpp::KeepLast(1));  // 历史策略：保留最后1条消息
    qos_profile.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);  // 明确设置耐久性策略
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);  // 明确设置可靠性策略

    // 订阅地图、起点和目标点
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/processed_map", qos_profile,
        std::bind(&HybridAStarPlanner::map_callback, this, std::placeholders::_1));

    start_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/start_point", 10,
        std::bind(&HybridAStarPlanner::start_callback, this, std::placeholders::_1));

    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 10,
        std::bind(&HybridAStarPlanner::goal_callback, this, std::placeholders::_1));
    
    // 发布规划路径
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);

    RCLCPP_INFO(this->get_logger(), "Hybrid A* Planner Node started.");
}

void HybridAStarPlanner::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    current_map_ptr_ = msg;
    RCLCPP_INFO(this->get_logger(), "load map successfully, resolution: %f, timestamp: %f",
                current_map_ptr_->info.resolution, current_map_ptr_->header.stamp.sec + current_map_ptr_->header.stamp.nanosec * 1e-9);
    has_map_ = true;
    hybridAStarInit();
    try_plan();
}

void HybridAStarPlanner::start_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    start_pose_ptr_ = msg;
    RCLCPP_INFO(this->get_logger(), "load start point successfully, position: (%f, %f)",
                start_pose_ptr_->pose.position.x, start_pose_ptr_->pose.position.y);
    has_start_ = true;
    try_plan();
}

void HybridAStarPlanner::goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    goal_pose_ptr_ = msg;
    RCLCPP_INFO(this->get_logger(), "load goal point successfully, position: (%f, %f)",
                goal_pose_ptr_->pose.position.x, goal_pose_ptr_->pose.position.y);
    has_goal_ = true;
    try_plan();
}

void HybridAStarPlanner::hybridAStarInit()
{
    /*
    ***************处理地图数据，初始化 Hybrid A* 规划器***************
    */
    //current_costmap_ptr_->info.width，这里的width不是米制的，而是地图的格子数目的宽度
    const double map_resolution = 0.2;
    kinodynamic_astar_searcher_ptr_->Init(
            current_costmap_ptr_->info.origin.position.x,
            1.0 * current_costmap_ptr_->info.width * current_costmap_ptr_->info.resolution,
            current_costmap_ptr_->info.origin.position.y,
            1.0 * current_costmap_ptr_->info.height * current_costmap_ptr_->info.resolution,
            vehicle_length_, vehicle_width_, vehicle_rear_dis_,
            current_costmap_ptr_->info.resolution,
            map_resolution
    );
    //map_w，这里的map应该就不是米制的了，而是和状态分辨率有关
    unsigned int map_w = std::floor(current_costmap_ptr_->info.width / map_resolution * current_costmap_ptr_->info.resolution);
    unsigned int map_h = std::floor(current_costmap_ptr_->info.height / map_resolution * current_costmap_ptr_->info.resolution);
    for (unsigned int w = 0; w < map_w; ++w) {
        for (unsigned int h = 0; h < map_h; ++h) {
            //这里的x, y应该是米制下的
            auto x = static_cast<unsigned int> ((w + 0.5) * map_resolution
                                                / current_costmap_ptr_->info.resolution);
            auto y = static_cast<unsigned int> ((h + 0.5) * map_resolution
                                                / current_costmap_ptr_->info.resolution);

            if (current_costmap_ptr_->data[y * current_costmap_ptr_->info.width + x]) {
                kinodynamic_astar_searcher_ptr_->SetObstacle(w, h);
            }
        }
    }
    //这里生成维诺图, 和地图一样的尺寸（width * height），不是米制
    bool** binMap;//二维数组，
    binMap = new bool*[current_costmap_ptr_->info.width];

    for (int x = 0; x < current_costmap_ptr_->info.width; x++) { binMap[x] = new bool[current_costmap_ptr_->info.width]; }//这里可简化为一次申请

    for (int x = 0; x < current_costmap_ptr_->info.width; ++x) {
        for (int y = 0; y < current_costmap_ptr_->info.width; ++y) {
        binMap[x][y] = current_costmap_ptr_->data[y * current_costmap_ptr_->info.width + x] ? true : false;
        }
    }//转化为二值地图

    voronoiDiagram.initializeMap(current_costmap_ptr_->info.width, current_costmap_ptr_->info.width,
                                    current_costmap_ptr_->info.resolution, binMap);//注意这里传入到DynamicVoronoi里并进行保存，所以没有delete
    voronoiDiagram.update();
    voronoiDiagram.visualize("");//将Voronoi Diagram初始化、更新并显示

    RCLCPP_INFO(this->get_logger(), "Hybrid A* Planner initialized with map resolution: %f, width: %d, height: %d",
                current_costmap_ptr_->info.resolution, current_costmap_ptr_->info.width, current_costmap_ptr_->info.height);
}

// 检查地图、起点和目标点是否同步
bool HybridAStarPlanner::isSynchronized(
    const nav_msgs::msg::OccupancyGrid &map,
    const geometry_msgs::msg::PoseStamped &start,
    const geometry_msgs::msg::PoseStamped &goal)
{
    // 检查地图、起点和目标点的时间戳是否同步
    if (map.header.stamp.sec == 0 || start.header.stamp.sec == 0 || goal.header.stamp.sec == 0) {
        return false; // 如果有任意一个时间戳为0，认为数据不同步
    }
    
    // 检查时间戳差异是否在允许范围内
    double map_time = map.header.stamp.sec + map.header.stamp.nanosec * 1e-9;
    double start_time = start.header.stamp.sec + start.header.stamp.nanosec * 1e-9;
    double goal_time = goal.header.stamp.sec + goal.header.stamp.nanosec * 1e-9;

    double time_diff_start = std::abs(map_time - start_time);
    double time_diff_goal = std::abs(map_time - goal_time);

    return (time_diff_start < 0.5 && time_diff_goal < 0.5); // 允许0.5秒的时间差
}

void HybridAStarPlanner::try_plan()
{
    // 检查是否有足够的数据进行规划
    if (!(has_map_ && has_start_ && has_goal_)) {
        RCLCPP_WARN(this->get_logger(), "Waiting for sufficient data: map, start, and goal.");
        return;
    }
        
    // 检查数据是否同步，静态地图不需要判断
    /*
    if (isSynchronized(current_map_, start_pose_, goal_pose_)) {
        RCLCPP_INFO(this->get_logger(), "All data synchronized, starting planning...");
        plan();
    } else {
        RCLCPP_WARN(this->get_logger(), "Data not synchronized, waiting for more data...");
        return;
    }*/
    // 规划
    RCLCPP_INFO(this->get_logger(), "Planning started...");
    double start_yaw = tf::getYaw(current_init_pose_ptr_->pose.pose.orientation);
    double goal_yaw = tf::getYaw(current_goal_pose_ptr_->pose.orientation);
    Vec3d start_state = Vec3d(
            current_init_pose_ptr_->pose.pose.position.x,
            current_init_pose_ptr_->pose.pose.position.y,
            start_yaw
    );
    Vec3d goal_state = Vec3d(
            current_goal_pose_ptr_->pose.position.x,
            current_goal_pose_ptr_->pose.position.y,
            goal_yaw
    );

    if (kinodynamic_astar_searcher_ptr_->Search(start_state, goal_state)) {
        auto path = kinodynamic_astar_searcher_ptr_->GetPath();
        //这里要对第一个点的前向和后向进行修正
        {
            double path_yaw = atan2((path[1] -path[0])(1), (path[1] -path[0])(0));
            if(abs(start_yaw - path_yaw) > 0.5 * M_PI)  path[0](3) = 0;
        }
        path_pub_->publish(planned_path_);
        RCLCPP_INFO(this->get_logger(), "Path published.");
        /*
        // 路径平滑过程
        smoother.tracePath(path);
        PublishPath(path);
        
        PublishVehiclePath(path, vehicle_length_, vehicle_width_, 5u);
        PublishSearchedTree(kinodynamic_astar_searcher_ptr_->GetSearchedTree());
        PublishCurrentStartAndGoal();
        Timer smooth_used_time;
        smoother.smoothPath(voronoiDiagram);
        std::cout << "Soomthen the path time(ms): " << smooth_used_time.End() << "\n" << std::endl;
        auto path_smoothed = smoother.getPath();
        PublishPathSmoothed(path_smoothed);
        */
    }
    else {
        RCLCPP_ERROR(this->get_logger(), "Planning failed, no valid path found.");
    }
    kinodynamic_astar_searcher_ptr_->Reset();
    

    /*
    // 发布虚拟路径（仅用于测试）
    nav_msgs::msg::Path dummy_path;
    dummy_path.header.frame_id = "map";
    dummy_path.header.stamp = this->get_clock()->now();

    dummy_path.poses.push_back(start_pose_);
    dummy_path.poses.push_back(goal_pose_);

    path_pub_->publish(planned_path_);
    RCLCPP_INFO(this->get_logger(), "Path published.");
    */
}

