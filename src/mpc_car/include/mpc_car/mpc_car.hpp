#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <iostream>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <arc_spline/arc_spline.hpp>
#include <deque>
#include <iosqp/iosqp.hpp>

static constexpr int n = 4;  // state [x y phi v]，状态的维度
static constexpr int m = 2;  // input [a delta]，输入的维度
typedef Eigen::Matrix<double, n, n> MatrixA;//线性化时的A
typedef Eigen::Matrix<double, n, m> MatrixB;//线性化时的B
typedef Eigen::Vector4d VectorG;//线性化时的G
typedef Eigen::Vector4d VectorX;//状态向量
typedef Eigen::Vector2d VectorU;//输入向量

class MpcCar {
public:
    // 构造时可传入MPC参数，也可由Node注入参数
    explicit MpcCar(std::vector<double>& track_points_x,
					std::vector<double>& track_points_y,
					double desired_v, double ll, double dt, double rho, int N,
					double rhoN, double v_max, double a_max, double delta_max,
					double ddelta_max, double delay);

    bool check_goal(const VectorX& x0);

    // 设置要跟踪的路径段与方向
    void setPath(const nav_msgs::msg::Path::SharedPtr pathMsg);
    void setPath(const std::vector<Eigen::Vector2d> &path_seg, int path_direction);

    // 解决当前时刻的最优控制（返回0/1/11等表示不同结果，如到达终点等）
    int solveQP(const Eigen::VectorXd& x0_observe);

    // 获取当前步长的预测状态和控制输入
    void getPredictXU(double t, VectorX& state, VectorU& input) const;

    // 可选：可视化接口，由Node调用（例如画预测轨迹/参考轨迹等）
    void visualization(nav_msgs::msg::Path &msg1, nav_msgs::msg::Path &msg2, nav_msgs::msg::Path &msg3);

private:
	void linearization(const double& phi, const double& v, const double& delta);
	void calLinPoint(const double& s0, double& phi, double& v, double& delta);
	VectorX diff(const VectorX& state, const VectorU& input) const;
	void step(VectorX& state, const VectorU& input, const double dt) const;
	VectorX compensateDelay0(const VectorX& x0);
	VectorX compensateDelay1(const VectorX& x0);
	VectorX compensateDelay2(const VectorX &x0);

    // 路径存储
    std::vector<Eigen::Vector2d> ref_path_;

    // 其他MPC参数、状态、QP变量等成员
    double ll_;//车长
    double dt_;//预测周期的时长
    double rho_;//过程的权重？
    int N_;//预测的horizon
    double rhoN_;//终端状态的权重？

    double v_max_, a_max_, delta_max_, ddelta_max_;//bound约束
    double delay_;//延迟

    bool path_direction_ = 1;//1表示前进

    arc_spline::ArcSpline s_;//参考轨迹
    double desired_v_;//期待的速度？预估的一个速度？？

    osqp::IOSQP qpSolver_;

    std::vector<VectorX> predictState_;//预测出来的state
    std::vector<VectorU> predictInput_;//预测出来的input
    std::deque<VectorU> historyInput_;//先前的input，用于有delay的场景
    int history_length_;//由于delay造成的延迟的预测周期数
    VectorX x0_observe_;//预测时的状态，由car_simulator直接龙格库塔积分得到
    std::vector<VectorX> compensateDelayX0_;
    bool useCompensate1 = 0, useCompensate2 = 0;
    
    // 线性化时用到的几个矩阵
	MatrixA Ad_;//4*4
	MatrixB Bd_;//4*2
	VectorG gd_;//4*1
	Eigen::SparseMatrix<double> P_, q_, A_, l_, u_;
	Eigen::SparseMatrix<double> Cx_, lx_, ux_;
	Eigen::SparseMatrix<double> Cu_, lu_, uu_;
  	Eigen::SparseMatrix<double> Qx_;
};
