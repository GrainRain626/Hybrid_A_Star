#include "car_simulator/car_simulator_node.hpp"

CarSimulatorNode::CarSimulatorNode() : Node("car_simulator_node") {
    this->declare_parameter<double>("l", 2.5);
    this->declare_parameter<double>("x", 0.0);
    this->declare_parameter<double>("y", 0.0);
    this->declare_parameter<double>("phi", 0.0);
    this->declare_parameter<double>("v", 0.0);
    this->declare_parameter<double>("delay", 0.0);

    car_.l = this->get_parameter("l").as_double();
    Eigen::Vector4d initS;
    initS(0) = this->get_parameter("x").as_double();
    initS(1) = this->get_parameter("y").as_double();
    initS(2) = this->get_parameter("phi").as_double();
    initS(3) = this->get_parameter("v").as_double();
    car_.setInitialState(initS);
    delay_ = this->get_parameter("delay").as_double();
    input_.setZero();

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom_car", 1);
    cmd_sub_ = this->create_subscription<car_msgs::msg::CarCmd>(
        "car_cmd", 1, std::bind(&CarSimulatorNode::cmd_callback, this, std::placeholders::_1));
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/hybrid_a_star_zm0612/searched_path_smoothed_with_d", 1, std::bind(&CarSimulatorNode::path_callback, this, std::placeholders::_1));
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    sim_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / 400),
        std::bind(&CarSimulatorNode::timer_callback, this));
}

// ----- Car 结构体成员函数实现 -----
void Car::setInitialState(const Eigen::Vector4d& s) {
  state = s;
}

Eigen::Vector4d Car::diff(const Eigen::Vector4d& s, const Eigen::Vector2d& input) const {
  Eigen::Vector4d ds;
  double phi = s(2), v = s(3), a = input(0), delta = input(1);
  ds(0) = v * cos(phi);
  ds(1) = v * sin(phi);
  ds(2) = v / l * tan(delta);
  ds(3) = a;
  return ds;
}

void Car::step(const Eigen::Vector2d& input, double dt) {
  Eigen::Vector4d k1 = diff(state, input);
  Eigen::Vector4d k2 = diff(state + k1 * dt / 2, input);
  Eigen::Vector4d k3 = diff(state + k2 * dt / 2, input);
  Eigen::Vector4d k4 = diff(state + k3 * dt, input);
  state = state + (k1 + k2 * 2 + k3 * 2 + k4) * dt / 6;
}

// ----- CarSimulatorNode 成员函数实现 -----

void CarSimulatorNode::cmd_callback(const car_msgs::msg::CarCmd::SharedPtr msg) {
  delayedMsgs_.emplace_back(this->now(), msg->a, msg->delta);
}

void CarSimulatorNode::path_callback(const nav_msgs::msg::Path::SharedPtr pathMsg) {
  has_path_ = true;
  Eigen::Vector4d initS;
  initS(0) = pathMsg->poses[0].pose.position.x;
  initS(1) = pathMsg->poses[0].pose.position.y;
  auto &q = pathMsg->poses[0].pose.orientation;
  tf2::Quaternion quat(q.x, q.y, q.z, q.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  initS(2) = yaw;
  initS(3) = 0; // 初始速度
  car_.setInitialState(initS);
  RCLCPP_INFO(this->get_logger(), "Car Simulator Init Success!");
}

void CarSimulatorNode::timer_callback() {
  if (!has_path_) return;
  if (!delayedMsgs_.empty()) {
    auto &msg = delayedMsgs_.front();
    if ((this->now() - msg.t).seconds() > delay_) {
      input_(0) = msg.a;
      input_(1) = msg.delta;
      delayedMsgs_.pop_front();
    }
  }
  car_.step(input_, 1.0 / 400);

  // 发布Odometry
  auto odom_msg = nav_msgs::msg::Odometry();
  odom_msg.header.stamp = this->now();
  odom_msg.header.frame_id = "world";
  odom_msg.pose.pose.position.x = car_.state(0);
  odom_msg.pose.pose.position.y = car_.state(1);
  odom_msg.pose.pose.position.z = 0.0;
  double phi = car_.state(2);
  double v = car_.state(3);

  // 构造四元数
  tf2::Quaternion q;
  q.setRPY(0, 0, phi);
  odom_msg.pose.pose.orientation.x = q.x();
  odom_msg.pose.pose.orientation.y = q.y();
  odom_msg.pose.pose.orientation.z = q.z();
  odom_msg.pose.pose.orientation.w = q.w();

  odom_msg.twist.twist.linear.x = v * cos(phi);
  odom_msg.twist.twist.linear.y = v * sin(phi);
  odom_msg.twist.twist.linear.z = 0.0;

  odom_pub_->publish(odom_msg);

  // 发布TF
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = this->now();
  tf_msg.header.frame_id = "world";
  tf_msg.child_frame_id = "ground_link";
  tf_msg.transform.translation.x = car_.state(0);
  tf_msg.transform.translation.y = car_.state(1);
  tf_msg.transform.translation.z = 0.0;
  tf_msg.transform.rotation = odom_msg.pose.pose.orientation;

  tf_broadcaster_->sendTransform(tf_msg);
}

// main函数
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CarSimulatorNode>());
  rclcpp::shutdown();
  return 0;
}
