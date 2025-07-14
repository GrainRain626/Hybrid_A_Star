#pragma once

#include <rclcpp/rclcpp.hpp>
#include <car_msgs/msg/car_cmd.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <Eigen/Geometry>
#include <deque>
#include <mpc_car/mpc_car.hpp> // 你自己的MPC算法头文件

class MpcCarNode : public rclcpp::Node {
public:
  	MpcCarNode();

private:
	std::shared_ptr<MpcCar> mpcPtr_;
	rclcpp::TimerBase::SharedPtr plan_timer_;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
	rclcpp::Publisher<car_msgs::msg::CarCmd>::SharedPtr cmd_pub_;
	rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
	// 发布相关
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr ref_pub_;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr traj_pub_;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr traj_delay_pub_;
	
	Eigen::VectorXd state_; // 和VectorX用法兼容
	bool init_odom_ = false;
	bool init_path_seg_ = false;
	bool init_path_all_ = false;
	bool arrive_goal_ = false;
	double delay_ = 0.0;
	int path_seg_index_ = 0;

	std::vector<std::vector<Eigen::Vector2d>> path_segs_;
	std::vector<int> path_direction_;
	void plan_timer_callback();
	void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
	void path_callback(const nav_msgs::msg::Path::SharedPtr msg);
	void vis_publish(nav_msgs::msg::Path &msg1,
					 nav_msgs::msg::Path &msg2,
					 nav_msgs::msg::Path &msg3);

	static double Mod2Pi(const double &x);
};
