#include "mpc_car/mpc_car_node.hpp"
#include <cmath>
#include <iostream>

double MpcCarNode::Mod2Pi(const double &x) {
	double v = fmod(x, 2 * M_PI);
	if (v < -M_PI) v += 2.0 * M_PI;
	else if (v > M_PI) v -= 2.0 * M_PI;
	return v;
}

MpcCarNode::MpcCarNode() : Node("mpc_car_node") {
	// 声明并读取参数
	this->declare_parameter<std::vector<double>>("track_points_x", {});
	this->declare_parameter<std::vector<double>>("track_points_y", {});
	this->declare_parameter<double>("desired_v", 1.0);
	this->declare_parameter<double>("ll", 1.282);
	this->declare_parameter<double>("dt", 0.04);
	this->declare_parameter<double>("rho", 1.0);
	this->declare_parameter<int>("N", 80);
	this->declare_parameter<double>("rhoN", 2.0);
	this->declare_parameter<double>("v_max", 2.0);
	this->declare_parameter<double>("a_max", 1.0);
	this->declare_parameter<double>("delta_max", 2.0);
	this->declare_parameter<double>("ddelta_max", 2.0);
	this->declare_parameter<double>("delay", 0.0);

	// 读取参数
	std::vector<double> track_points_x = this->get_parameter("track_points_x").as_double_array();
	std::vector<double> track_points_y = this->get_parameter("track_points_y").as_double_array();
	double desired_v = this->get_parameter("desired_v").as_double();
	double ll = this->get_parameter("ll").as_double();
	double dt = this->get_parameter("dt").as_double();
	double rho = this->get_parameter("rho").as_double();
	int N = this->get_parameter("N").as_int();
	double rhoN = this->get_parameter("rhoN").as_double();
	double v_max = this->get_parameter("v_max").as_double();
	double a_max = this->get_parameter("a_max").as_double();
	double delta_max = this->get_parameter("delta_max").as_double();
	double ddelta_max = this->get_parameter("ddelta_max").as_double();
	double delay = this->get_parameter("delay").as_double();

	delay_ = delay;

	RCLCPP_INFO(this->get_logger(), "%d track points loaded.", track_points_x.size());
	RCLCPP_INFO(this->get_logger(), "MPC parameters: desired_v=%.2f, ll=%.2f, dt=%.2f, rho=%.2f, N=%d, rhoN=%.2f, v_max=%.2f, a_max=%.2f, delta_max=%.2f, ddelta_max=%.2f, delay=%.2f",
		desired_v, ll, dt, rho, N, rhoN, v_max, a_max, delta_max, ddelta_max, delay);

	mpcPtr_ = std::make_shared<MpcCar>(track_points_x, track_points_y,
               			desired_v, ll, dt, rho, N, rhoN, v_max, 
						a_max, delta_max, ddelta_max, delay); // 如你的MpcCar构造需传node或参数，自行适配

	plan_timer_ = this->create_wall_timer(
		std::chrono::duration<double>(dt),
		std::bind(&MpcCarNode::plan_timer_callback, this));

	odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
		"/odom_car", 1, std::bind(&MpcCarNode::odom_callback, this, std::placeholders::_1));
	cmd_pub_ = this->create_publisher<car_msgs::msg::CarCmd>("/car_cmd", 1);
	path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
		"/smoothed_path", 1, std::bind(&MpcCarNode::path_callback, this, std::placeholders::_1));

	// 发布相关
	ref_pub_        = this->create_publisher<nav_msgs::msg::Path>("path_ref",        rclcpp::QoS(1));
	traj_pub_       = this->create_publisher<nav_msgs::msg::Path>("traj_mpc",        rclcpp::QoS(1));
	traj_delay_pub_ = this->create_publisher<nav_msgs::msg::Path>("traj_mpc_delay",  rclcpp::QoS(1));
}

void MpcCarNode::vis_publish(nav_msgs::msg::Path &msg1,
						   nav_msgs::msg::Path &msg2,
						   nav_msgs::msg::Path &msg3) {
	ref_pub_->publish(msg1);
	traj_pub_->publish(msg2);
	traj_delay_pub_->publish(msg3);						
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
		RCLCPP_INFO(this->get_logger(), "Start solveQP  ------------------------------------------");
		int ret = mpcPtr_->solveQP(state_);
		RCLCPP_INFO(this->get_logger(), "Finish solveQP ------------------------------------------");
		
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
				msg.header.frame_id = "map";
				msg.header.stamp = this->now();
				msg.a = 0;
				msg.delta = 0;
				cmd_pub_->publish(msg);
			} else {
				car_msgs::msg::CarCmd msg;
				msg.header.frame_id = "maps";
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
		msg.header.frame_id = "map";
		msg.header.stamp = this->now();

		VectorX x;
    	VectorU u;
		mpcPtr_->getPredictXU(0, x, u);
		msg.a = u(0);
		msg.delta = u(1);
		cmd_pub_->publish(msg);

		nav_msgs::msg::Path msg1;
		nav_msgs::msg::Path msg2;
		nav_msgs::msg::Path msg3;
		mpcPtr_->visualization(msg1, msg2, msg3);
		vis_publish(msg1, msg2, msg3);
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
	RCLCPP_INFO(this->get_logger(), "MPC_CarNode: Received a new path!");
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
