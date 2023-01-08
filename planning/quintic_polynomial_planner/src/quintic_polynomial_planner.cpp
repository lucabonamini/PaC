#include "quintic_polynomial_planner/quintic_polynomial_planner.h"
#include <eigen3/unsupported/Eigen/MatrixFunctions>

namespace QuinticPolynomialPlanner {
std::optional<Output> plan(const Input &input) {
  double vxs = input.sv * std::cos(input.syaw);
  double vys = input.sv * std::sin(input.syaw);
  double vxg = input.gv * std::cos(input.gyaw);
  double vyg = input.gv * std::sin(input.gyaw);
  double axs = input.sa * std::cos(input.syaw);
  double ays = input.sa * std::sin(input.syaw);
  double axg = input.ga * std::cos(input.gyaw);
  double ayg = input.ga * std::sin(input.gyaw);

  for (int T = input.min_t; T < input.max_t; T += input.min_t) {
    QuinticPolynomial xqp(input.sx, vxs, axs, input.gx, vxg, axg, T);
    QuinticPolynomial yqp(input.sy, vys, ays, input.gy, vyg, ayg, T);
    std::optional<Output> output = std::make_optional<Output>();

    for (auto t = 0.0; t < T + input.dt; t += input.dt) {
      output->time.push_back(t);
      output->rx.push_back(xqp.calcPoint(t));
      output->ry.push_back(yqp.calcPoint(t));

      double vx = xqp.calcFirstDerivative(t);
      double vy = yqp.calcFirstDerivative(t);
      double v = sqrt(pow(vx, 2) + pow(vy, 2));
      double yaw = atan2(vy, vx);
      output->rv.push_back(v);
      output->ryaw.push_back(yaw);

      double ax = xqp.calcSecondDerivative(t);
      double ay = yqp.calcSecondDerivative(t);
      double a = sqrt(pow(ax, 2) + pow(ay, 2));
      int v_size = output->rv.size();
      if (v_size >= 2 && output->rv.back() - output->rv.at(v_size - 1) < 0.0) {
        a *= -1;
      }
      output->ra.push_back(a);

      double jx = xqp.calcThirdDerivative(t);
      double jy = yqp.calcThirdDerivative(t);
      double j = sqrt(pow(jx, 2) + pow(jy, 2));
      int a_size = output->rv.size();
      if (a_size >= 2 && output->rv.back() - output->rv.at(a_size - 1) < 0.0) {
        j *= -1;
      }
      output->rj.push_back(j);
    }
    if (*std::max_element(output->ra.begin(), output->ra.end()) <=
            input.max_accel &&
        *std::max_element(output->rj.begin(), output->rj.end()) <=
            input.max_jerk) {
      return output;
    }
  }
  return std::nullopt;
}
} // namespace QuinticPolynomialPlanner

QuinticPolynomial::QuinticPolynomial(double xs,
                                     double vxs,
                                     double axs,
                                     double xe,
                                     double vxe,
                                     double axe,
                                     double t)
    : xs_(xs)
    , vxs_(vxs)
    , axs_(axs)
    , xe_(xe)
    , vxe_(vxe)
    , axe_(axe)
    , a0_(xs)
    , a1_(vxs)
    , a2_(axs / 2.0) {
  Eigen::Matrix3d A;
  A(0, 0) = std::pow(t, 3);
  A(0, 1) = std::pow(t, 4);
  A(0, 2) = std::pow(t, 5);
  A(1, 0) = 3 * std::pow(t, 2);
  A(1, 1) = 4 * std::pow(t, 3);
  A(1, 2) = 5 * std::pow(t, 4);
  A(2, 0) = 6 * t;
  A(2, 1) = 12 * std::pow(t, 2);
  A(2, 2) = 20 * std::pow(t, 3);

  Eigen::Vector3d B;
  B(0) = xe - a0_ - a1_ * t - a2_ * std::pow(t, 2);
  B(1) = vxe - a1_ - 2 * a2_ * t;
  B(2) = axe - 2 * a2_;

  Eigen::Vector3d c_eigen = A.colPivHouseholderQr().solve(B);
  a3_ = c_eigen[0];
  a4_ = c_eigen[1];
  a5_ = c_eigen[2];
}
