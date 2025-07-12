#pragma once

#include <rclcpp/rclcpp.hpp>
#include <car_msgs/msg/car_cmd.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Core>
#include <deque>

struct Car {
  double l;
  Eigen::Vector4d state; // x, y, phi, v

  void setInitialState(const Eigen::Vector4d& s);
  Eigen::Vector4d diff(const Eigen::Vector4d& s, const Eigen::Vector2d& input) const;
  void step(const Eigen::Vector2d& input, double dt);
};

class CarSimulatorNode : public rclcpp::Node {
public:
  CarSimulatorNode();

private:
  Car car_;
  double delay_ = 0.0;
  Eigen::Vector2d input_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<car_msgs::msg::CarCmd>::SharedPtr cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::TimerBase::SharedPtr sim_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  bool has_path_ = false;

  struct DelayedMsg {
    rclcpp::Time t;
    double a, delta;
    DelayedMsg() = default;
    DelayedMsg(const rclcpp::Time& _t, double _a, double _delta) : t(_t), a(_a), delta(_delta) {}
  };
  std::deque<DelayedMsg> delayedMsgs_;

  void cmd_callback(const car_msgs::msg::CarCmd::SharedPtr msg);
  void path_callback(const nav_msgs::msg::Path::SharedPtr pathMsg);
  void timer_callback();
};
