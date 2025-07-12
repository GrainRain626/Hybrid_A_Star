#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <iostream>

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
  explicit MpcCar(/* 你可加参数，如车辆参数/配置文件等 */);

  // 设置要跟踪的路径段与方向
  void setPath(const std::vector<Eigen::Vector2d>& path_points, int direction);

  // 解决当前时刻的最优控制（返回0/1/11等表示不同结果，如到达终点等）
  int solveQP(const Eigen::VectorXd& state);

  // 获取当前步长的预测状态和控制输入
  void getPredictXU(int index, Eigen::VectorXd& x, Eigen::VectorXd& u) const;

  // 可选：可视化接口，由Node调用（例如画预测轨迹/参考轨迹等）
  void visualization() const;

private:
  // 路径存储
  std::vector<Eigen::Vector2d> ref_path_;
  int direction_ = 1;

  // 其他MPC参数、状态、QP变量等成员
  // ...
};
