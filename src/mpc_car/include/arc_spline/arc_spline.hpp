#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cassert>
#include <cstdio>
#include <vector>

namespace arc_spline {

// band matrix solver
class band_matrix {
 private:
  std::vector<std::vector<double> > m_upper;  // upper band
  std::vector<std::vector<double> > m_lower;  // lower band
 public:
  band_matrix(){};                         // constructor
  band_matrix(int dim, int n_u, int n_l);  // constructor
  ~band_matrix(){};                        // destructor
  void resize(int dim, int n_u, int n_l);  // init with dim,n_u,n_l
  int dim() const;                         // matrix dimension
  int num_upper() const {
    return m_upper.size() - 1;
  }
  int num_lower() const {
    return m_lower.size() - 1;
  }
  // access operator
  double& operator()(int i, int j);       // write
  double operator()(int i, int j) const;  // read
  // we can store an additional diogonal (in m_lower)
  double& saved_diag(int i);
  double saved_diag(int i) const;
  void lu_decompose();
  std::vector<double> r_solve(const std::vector<double>& b) const;
  std::vector<double> l_solve(const std::vector<double>& b) const;
  std::vector<double> lu_solve(const std::vector<double>& b,
                               bool is_lu_decomposed = false);
};

class spline {
 public:
  enum bd_type {
    first_deriv = 1,
    second_deriv = 2
  };

 public:
  std::vector<double> m_x, m_y;  // x,y coordinates of points
  // interpolation parameters
  // f(x) = a*(x-x_i)^3 + b*(x-x_i)^2 + c*(x-x_i) + y_i
  std::vector<double> m_a, m_b, m_c;  // spline coefficients
  double m_b0, m_c0;                  // for left extrapol
  bd_type m_left, m_right;
  double m_left_value, m_right_value;
  bool m_force_linear_extrapolation;

 public:
  // set default boundary condition to be zero curvature at both ends
  spline() : m_left(second_deriv), m_right(second_deriv), m_left_value(0.0), m_right_value(0.0), m_force_linear_extrapolation(false) {
    ;
  }

  // optional, but if called it has to come be before set_points()
  void set_boundary(bd_type left, double left_value,
                    bd_type right, double right_value,
                    bool force_linear_extrapolation = false);
  void set_points(const std::vector<double>& x,
                  const std::vector<double>& y, bool cubic_spline = true);
  // double operator() (double x) const;
  double operator()(double x, int dd = 0) const;
};


class ArcSpline {
 private:
  double arcL_;
  spline xs_, ys_;
  std::vector<double> sL_;
  std::vector<double> xL_;
  std::vector<double> yL_;

 public:
  ArcSpline(){};
  ~ArcSpline(){};
  void setWayPoints(const std::vector<double>& x_waypoints,
                    const std::vector<double>& y_waypoints);
  Eigen::Vector2d operator()(double s, int n = 0);
  double findS(const Eigen::Vector2d& p);
  inline double arcL(){
    return arcL_;
  };
};

inline double ArcSpline::findS(const Eigen::Vector2d& p) {
  // TODO a more efficient and accurate method
  double min_dist = (operator()(sL_.front()) - p).norm();
  double min_s = sL_.front();
  for (double s = sL_.front(); s < sL_.back(); s += 0.01) {
    double dist = (operator()(s) - p).norm();
    if (dist < min_dist) {
      min_s = s;
      min_dist = dist;
    }
  }
  return min_s;
}

// inline double ArcSpline::findS(const Eigen::Vector2d& p) {
//   double l = sL_.front();
//   double r = sL_.back();
//   while (abs(r - l) > 1e-3) {
//     double dist_l = (operator()(l) - p).norm();
//     double dist_r = (operator()(r) - p).norm();
//     if (dist_l < dist_r) {
//       r = (l + r) / 2;
//     } else {
//       l = (l + r) / 2;
//     }
//   }
//   return l;
// }

}  // namespace arc_spline
