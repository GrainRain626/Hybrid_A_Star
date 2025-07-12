#include "car_mpc/mpc_car.hpp"

MpcCar::MpcCar(/*参数*/) {
  // 可初始化MPC参数
}

void MpcCar::setPath(const std::vector<Eigen::Vector2d>& path_points, int direction) {
  ref_path_ = path_points;
  direction_ = direction;
}

int MpcCar::solveQP(const Eigen::VectorXd& state) {
  // 这里填入你的MPC QP求解流程
  // 结果存到成员变量中
  // 返回控制结果
  return 1;
}

void MpcCar::getPredictXU(int index, Eigen::VectorXd& x, Eigen::VectorXd& u) const {
  // 填写当前预测结果
}

void MpcCar::visualization() const {
  // 可打印或将数据交给Node做ROS2话题可视化
}
