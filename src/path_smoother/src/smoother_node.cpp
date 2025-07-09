#include "path_smoother/smoother_node.hpp"
#include "planning_utils/timer.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

VectorVec4d pathMsgToVectorVec4d(const nav_msgs::msg::Path &path_msg)
{
    VectorVec4d result;
    result.reserve(path_msg.poses.size());

    for (const auto & pose_stamped : path_msg.poses) {
        double x = pose_stamped.pose.position.x;
        double y = pose_stamped.pose.position.y;
        double z = pose_stamped.pose.position.z;  // 你如果用不上，可以写0.0
        const auto & q = pose_stamped.pose.orientation;

        // 四元数转yaw
        tf2::Quaternion quat(q.x, q.y, q.z, q.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

        result.emplace_back(Eigen::Vector4d(x, y, yaw, 0.0));
    }
    return result;
}

SmootherNode::SmootherNode() : Node("smoother_node") {
    /*
    TODO：从参数文件读取相关参数
    */
    // 声明参数（声明后才可以get_parameter，建议全部在构造函数声明）
    this->declare_parameter<double>("planner.steering_angle", 30.0);
    this->declare_parameter<double>("planner.wheel_base", 0.8);
    this->declare_parameter<double>("planner.vehicle_width", 1.0);
    // 获取参数
    double steering_angle = this->get_parameter("planner.steering_angle").as_double();
    double wheel_base = this->get_parameter("planner.wheel_base").as_double();
    double vehicle_width = this->get_parameter("planner.vehicle_width").as_double();

    // maximum possible curvature of the non-holonomic vehicle
    float kappaMax = 1.f / (wheel_base * 57.3 / steering_angle * 1.1);
    // maximum distance to obstacles that is penalized
    float obsDMax = vehicle_width * 0.5 + 0.5;// = Constants::minRoadWidth;
    // maximum distance for obstacles to influence the voronoi field
    float vorObsDMax = vehicle_width * 0.5 + 0.5;// = Constants::minRoadWidth;


    // 设置 QoS Profile 为 transient local
    rclcpp::QoS qos_profile(rclcpp::KeepLast(1));  // 历史策略：保留最后1条消息
    qos_profile.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);  // 明确设置耐久性策略
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);  // 明确设置可靠性策略

    // 订阅地图、规划路径
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/processed_map", qos_profile,
        std::bind(&SmootherNode::map_callback, this, std::placeholders::_1));
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/planned_path", qos_profile,
        std::bind(&SmootherNode::pathCallback, this, std::placeholders::_1)
    );

    // 创建发布器
    smooth_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/smoothed_path", qos_profile);

    // 初始化Smoother类
    smoother_ = std::make_shared<Smoother>();
    smoother_->init(kappaMax, obsDMax, vorObsDMax);

    RCLCPP_INFO(this->get_logger(), "Smoother node started, waiting for /planned_path...");
}

void SmootherNode::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    auto current_costmap_ptr_ = msg;
    RCLCPP_INFO(this->get_logger(), "load map successfully, resolution: %f, timestamp: %f",
                current_costmap_ptr_->info.resolution, current_costmap_ptr_->header.stamp.sec + current_costmap_ptr_->header.stamp.nanosec * 1e-9);
    
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
}

void SmootherNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
    if (msg->poses.empty()) {
        RCLCPP_WARN(this->get_logger(), "Received empty path. Skipping smoothing.");
        return;
    }
    auto path = pathMsgToVectorVec4d(*msg);
    // 路径平滑处理
    smoother_->tracePath(path);
    Timer smooth_used_time;
    smoother_->smoothPath(voronoiDiagram);
    std::cout << "Soomthen the path time(ms): " << smooth_used_time.End() << "\n" << std::endl;
    auto smoothed_path = smoother_->getPath();

    // 发布平滑后的路径
    publishPath(smoothed_path);

    RCLCPP_INFO(this->get_logger(), "Smoothed path published, poses count: %zu", smoothed_path.size());
}

// 发布优化路径
void SmootherNode::publishPath(const VectorVec4d &path) 
{
    nav_msgs::msg::Path smoothed_path;  // 不要直接用成员，避免旧数据堆积
    smoothed_path.header.frame_id = "map";
    smoothed_path.header.stamp = this->now();

    for (const auto &pose: path) {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = "map";
        pose_stamped.header.stamp = this->now();
        pose_stamped.pose.position.x = pose.x();
        pose_stamped.pose.position.y = pose.y();
        pose_stamped.pose.position.z = 0.0;

        // 用 tf2::Quaternion 生成四元数
        tf2::Quaternion q;
        q.setRPY(0, 0, pose.z());  // 只绕Z轴旋转
        pose_stamped.pose.orientation.x = q.x();
        pose_stamped.pose.orientation.y = q.y();
        pose_stamped.pose.orientation.z = q.z();
        pose_stamped.pose.orientation.w = q.w();

        smoothed_path.poses.emplace_back(pose_stamped);
    }

    smoothed_path_ = smoothed_path;  // 更新成员变量
    smooth_path_pub_->publish(smoothed_path);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SmootherNode>());
    rclcpp::shutdown();
    return 0;
}
