#pragma once

#include <eigen3/Eigen/Core>
#include <optional>
#include <vector>

namespace QuinticPolynomialPlanner {
/**
 * @brief Input structure needed by plan function
 * @param sx Start x position [m]
 * @param sy Start y position [m]
 * @param syaw Start yaw angle [rad]
 * @param sv Start velocity [m/s]
 * @param sa Start acceleration [m/ss]
 * @param gx Goal x position [m]
 * @param gy Goal y position [m]
 * @param gyaw Goal yaw angle [rad]
 * @param gv Goal velocity [m/s]
 * @param ga Goal acceleration [m/ss]
 * @param min_t Minimum time to goal [s]
 * @param max_t Maximum time to goal [s]
 * @param dt Time tick [s]
 * @param max_accel Maximum acceleration [m/ss]
 * @param max_jerk Maximum jerk [m/sss]
 */
struct Input {
  double sx = 0.0;
  double sy = 0.0;
  double syaw = 0.0;
  double sv = 0.0;
  double sa = 0.0;
  double gx = 0.0;
  double gy = 0.0;
  double gyaw = 0.0;
  double gv = 0.0;
  double ga = 0.0;
  double min_t = 0.0;
  double max_t = 0.0;
  double dt = 0.0;
  double max_accel = 0.0;
  double max_jerk = 0.0;
};
/**
 * @brief Output structure of plan function
 * @param time Time result
 * @param rx x position result vector
 * @param ry y position result vector
 * @param ryaw yaw angle result vector
 * @param rv velocity result vector
 * @param ra acceleration result vector
 * @param rj jerk result vector
 */
struct Output {
  std::vector<double> time;
  std::vector<double> rx;
  std::vector<double> ry;
  std::vector<double> ryaw;
  std::vector<double> rv;
  std::vector<double> ra;
  std::vector<double> rj;
};
/**
 * @brief Calc quintic polynomial path
 * @param input List of start and end conditions
 * @return Quinti polynomial path
 */
std::optional<Output> plan(const Input &input);
} // namespace QuinticPolynomialPlanner

/**
 * @brief Class for Quintic Polynomial calculation
 * @param xs Start position
 * @param vxs Start velocity
 * @param axs Start acceleration
 * @param xe End desired position
 * @param vxe End desired velocity
 * @param axe End desired acceleration
 * @param t Time
 */
class QuinticPolynomial {
public:
  QuinticPolynomial(double xs,
                    double vxs,
                    double axs,
                    double xe,
                    double vxe,
                    double axe,
                    double t);

  double calcPoint(double t) const {
    return a0_ + a1_ * t + a2_ * std::pow(t, 2) + a3_ * std::pow(t, 3) +
           a4_ * std::pow(t, 4) + a5_ * std::pow(t, 5);
  };
  double calcFirstDerivative(double t) const {
    return a1_ + 2 * a2_ * t + 3 * a3_ * std::pow(t, 2) +
           4 * a4_ * std::pow(t, 3) + 5 * a5_ * std::pow(t, 4);
  };
  double calcSecondDerivative(double t) const {
    return 2 * a2_ + 6 * a3_ * t + 12 * a4_ * std::pow(t, 2) +
           20 * a5_ * std::pow(t, 3);
  };
  double calcThirdDerivative(double t) const {
    return 6 * a3_ + 24 * a4_ * t + 60 * a5_ * std::pow(t, 2);
  };

private:
  // current parameter at t=0
  double xs_ = 0.0;
  double vxs_ = 0.0;
  double axs_ = 0.0;
  // parameters at target t=t_j
  double xe_ = 0.0;
  double vxe_ = 0.0;
  double axe_ = 0.0;
  // function parameters
  double a0_ = 0.0;
  double a1_ = 0.0;
  double a2_ = 0.0;
  double a3_ = 0.0;
  double a4_ = 0.0;
  double a5_ = 0.0;
};
