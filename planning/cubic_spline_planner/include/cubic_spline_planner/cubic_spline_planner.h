#pragma once

#include <array>
#include <eigen3/Eigen/Core>
#include <vector>

namespace planning {
class Spline {
public:
  Spline() = default;
  Spline(const std::vector<double> &x_, const std::vector<double> &y_);

  std::vector<double> x;
  std::vector<double> y;
  int nx{0};
  std::vector<double> h;
  std::vector<double> a;
  std::vector<double> b;
  std::vector<double> c;
  std::vector<double> d;

  double calc(double t);

  double calc_d(double t);

  double calc_dd(double t);

  Eigen::MatrixXd calc_A();
  Eigen::VectorXd calc_B();

  int bisect(double t, int start, int end);
};

class Spline2D {
public:
  // TODO(lucabonamini): manage with friendship
  Spline sx;
  Spline sy;
  std::vector<double> s;

  Spline2D(const std::vector<double> &x, const std::vector<double> &y);

  std::array<double, 2> calc_position(double s_t);

  double calc_curvature(double s_t);

  double calc_yaw(double s_t);

  std::array<double, 2> calcCartesianCoordinates(const double &s,
                                                 const double &d);

private:
  static std::vector<double> calc_s(const std::vector<double> &x,
                                    const std::vector<double> &y);
};
} // namespace planning
