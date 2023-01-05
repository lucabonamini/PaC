#pragma once

#include <cmath>

/**
 * @brief Class for Quartic Polynomial calculation
 * @param xs Start position
 * @param vxs Start velocity
 * @param axs Start acceleration
 * @param vxe End desired velocity
 * @param axe End desired acceleration
 * @param t Time
 */
class QuarticPolynomial {
public:
  QuarticPolynomial(
      double xs, double vxs, double axs, double vxe, double axe, double t);

  double calcPoint(double t) const {
    return a0_ + a1_ * t + a2_ * std::pow(t, 2) + a3_ * std::pow(t, 3) +
           a4_ * std::pow(t, 4);
  };
  double calcFirstDerivative(double t) const {
    return a1_ + 2 * a2_ * t + 3 * a3_ * std::pow(t, 2) + 4 * a4_ * std::pow(t, 3);
  };
  double calcSecondDerivative(double t) const {
    return 2 * a2_ + 6 * a3_ * t + 12 * a4_ * std::pow(t, 2);
  };
  double calcThirdDerivative(double t) const { return 6 * a3_ + 24 * a4_ * t; };
private:
  // current parameter at t=0
  double xs_ = 0.0;
  double vxs_ = 0.0;
  double axs_ = 0.0;
  // parameters at target t=t_j
  double vxe_ = 0.0;
  double axe_ = 0.0;
  // function parameters
  double a0_ = 0.0;
  double a1_ = 0.0;
  double a2_ = 0.0;
  double a3_ = 0.0;
  double a4_ = 0.0;
};