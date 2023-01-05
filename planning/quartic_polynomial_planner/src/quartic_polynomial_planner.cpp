#include "quartic_polynomial_planner/quartic_polynomial_planner.h"
#include <eigen3/unsupported/Eigen/MatrixFunctions>

QuarticPolynomial::QuarticPolynomial(
    double xs, double vxs, double axs, double vxe, double axe, double t)
    : xs_(xs)
    , vxs_(vxs)
    , axs_(axs)
    , vxe_(vxe)
    , axe_(axe)
    , a0_(xs)
    , a1_(vxs)
    , a2_(axs / 2.0) {
  Eigen::Matrix2d A;
  A(0, 0) = 3 * std::pow(t, 2);
  A(0, 1) = 4 * std::pow(t, 3);
  A(1, 0) = 6 * t;
  A(1, 1) = 12 * std::pow(t, 2);
  Eigen::Vector2d B;
  B(0) = vxe - a1_ - 2 * a2_ * t;
  B(1) = axe - 2 * a2_;

  Eigen::Vector2d c_eigen = A.colPivHouseholderQr().solve(B);
  a3_ = c_eigen[0];
  a4_ = c_eigen[1];
}
