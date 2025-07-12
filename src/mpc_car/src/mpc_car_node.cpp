#include "car_mpc/mpc_car_node.hpp"
#include <cmath>
#include <iostream>

MpcCarNode::MpcCarNode() : Node("mpc_car_node") {
  // 参数声明与读取
  this->declare_parameter<double>("dt", 0.02);
  this->declare_parameter<double>("delay", 0.0);

  double dt = this->get_parameter("dt").as_double();
  delay_ = this->get_parameter("delay").as_double();

  mpcPtr_ = std::make_shared<MpcCar>(this); // 如你的MpcCar构造需传node或参数，自行适配

  plan_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(dt),
      std::bind(&MpcCarNode::plan_timer_callback, this));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/car_simulator/odom_car", 1, std::bind(&MpcCarNode::odom_callback, this, std::placeholders::_1));
  cmd_pub_ = this->create_publisher<car_msgs::msg::CarCmd>("car_cmd", 1);
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/hybrid_a_star_zm0612/searched_path_smoothed_with_d", 1, std::bind(&MpcCarNode::path_callback, this, std::placeholders::_1));
}

double MpcCarNode::Mod2Pi(const double &x) {
  double v = fmod(x, 2 * M_PI);
  if (v < -M_PI) v += 2.0 * M_PI;
  else if (v > M_PI) v -= 2.0 * M_PI;
  return v;
}

void MpcCarNode::plan_timer_callback() {
  if(init_path_all_ && !init_path_seg_) {
    if(path_segs_.empty()) return;
    if(path_segs_[path_seg_index_].size() > 2) {
      mpcPtr_->setPath(path_segs_[path_seg_index_], path_direction_[path_seg_index_]);
    } else if (path_segs_[path_seg_index_].size() == 2) {
      std::vector<Eigen::Vector2d> interpPoint(3);
      interpPoint[0] = path_segs_[path_seg_index_][0];
      interpPoint[2] = path_segs_[path_seg_index_][1];
      interpPoint[1] = 0.5 * (path_segs_[path_seg_index_][0] + path_segs_[path_seg_index_][1]);
      mpcPtr_->setPath(interpPoint, path_direction_[path_seg_index_]);
      RCLCPP_WARN(this->get_logger(), "Get an interpolated traj!!");
    }
    init_path_seg_ = true;
  }
  if(init_odom_ && init_path_seg_ && !arrive_goal_) {
    int ret = mpcPtr_->solveQP(state_);
    if(ret == 11) {
      if(path_seg_index_ < path_segs_.size() - 1) {
        path_seg_index_++;
        if(path_segs_[path_seg_index_].size() > 2) {
          mpcPtr_->setPath(path_segs_[path_seg_index_], path_direction_[path_seg_index_]);
        } else if (path_segs_[path_seg_index_].size() == 2) {
          std::vector<Eigen::Vector2d> interpPoint(3);
          interpPoint[0] = path_segs_[path_seg_index_][0];
          interpPoint[2] = path_segs_[path_seg_index_][1];
          interpPoint[1] = 0.5 * (path_segs_[path_seg_index_][0] + path_segs_[path_seg_index_][1]);
          mpcPtr_->setPath(interpPoint, path_direction_[path_seg_index_]);
          RCLCPP_WARN(this->get_logger(), "Get an interpolated traj!!");
        }
        car_msgs::msg::CarCmd msg;
        msg.header.frame_id = "world";
        msg.header.stamp = this->now();
        msg.a = 0;
        msg.delta = 0;
        cmd_pub_->publish(msg);
      } else {
        car_msgs::msg::CarCmd msg;
        msg.header.frame_id = "world";
        msg.header.stamp = this->now();
        msg.a = 0;
        msg.delta = 0;
        cmd_pub_->publish(msg);
        arrive_goal_ = true;
        RCLCPP_INFO(this->get_logger(), "Arrived at the goal!");
        return;
      }
    }
    car_msgs::msg::CarCmd msg;
    msg.header.frame_id = "world";
    msg.header.stamp = this->now();

    Eigen::VectorXd x, u;
    mpcPtr_->getPredictXU(0, x, u);
    msg.a = u(0);
    msg.delta = u(1);
    cmd_pub_->publish(msg);
    mpcPtr_->visualization();
  }
}

void MpcCarNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;
  Eigen::Quaterniond q(msg->pose.pose.orientation.w,
                       msg->pose.pose.orientation.x,
                       msg->pose.pose.orientation.y,
                       msg->pose.pose.orientation.z);
  Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
  Eigen::Vector2d v(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
  state_.resize(4);
  state_ << x, y, euler.z(), v.norm();
  double vel_angle = atan2(msg->twist.twist.linear.y, msg->twist.twist.linear.x);
  double angle_diff = vel_angle - euler.z();
  int forward_dir = path_direction_[path_seg_index_];

  angle_diff = Mod2Pi(angle_diff);
  if(forward_dir == 1 && abs(angle_diff) > 0.75 * M_PI) {
    state_(3) *= -1;
  }
  if(forward_dir == 0) state_(3) *= -1;
  if(forward_dir == 0 && abs(angle_diff) < 0.25 * M_PI) {
    state_(3) *= -1;
  }

  init_odom_ = true;
}

void MpcCarNode::path_callback(const nav_msgs::msg::Path::SharedPtr pathMsg) {
  path_direction_.clear();
  path_segs_.clear();
  std::vector<Eigen::Vector2d> path_seg;
  int initDirection = pathMsg->poses[0].pose.position.z;
  path_direction_.emplace_back(initDirection);
  for(int i = 0; i < pathMsg->poses.size(); ++i) {
    if(pathMsg->poses[i].pose.position.z == path_direction_.back()) {
      path_seg.emplace_back(Eigen::Vector2d(pathMsg->poses[i].pose.position.x, pathMsg->poses[i].pose.position.y));
    } else {
      path_direction_.emplace_back(pathMsg->poses[i].pose.position.z);
      path_segs_.emplace_back(path_seg);
      path_seg.clear();
      path_seg.emplace_back(Eigen::Vector2d(pathMsg->poses[i].pose.position.x, pathMsg->poses[i].pose.position.y));
    }
    if(i == pathMsg->poses.size() - 1) {
      path_segs_.emplace_back(path_seg);
    }
  }
  init_path_all_ = true;
  init_path_seg_ = false;
  arrive_goal_ = false;
  path_seg_index_ = 0;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MpcCarNode>());
  rclcpp::shutdown();
  return 0;
}
