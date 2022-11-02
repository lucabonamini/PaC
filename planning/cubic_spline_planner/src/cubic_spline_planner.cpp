#include "cubic_spline_planner/cubic_spline_planner.h"

#include <eigen3/unsupported/Eigen/MatrixFunctions>

constexpr double coeff_a = 3.0;
constexpr double coeff_b = 6.0;
constexpr double coeff_c = 2.0;

namespace planning {
Spline::Spline(const std::vector<double> &x, const std::vector<double> &y)
    : x(x), y(y), nx(x.size()), h(utilities::math::vecDiff(x)), a(y) {
  Eigen::MatrixXd A = calc_A();
  Eigen::VectorXd B = calc_B();
  Eigen::VectorXd c_eigen = A.colPivHouseholderQr().solve(B); // NOLINT
  c.resize(c_eigen.size());
  Eigen::VectorXd::Map(&c[0], c_eigen.size()) = c_eigen;

  for (int i = 0; i < nx - 1; i++) {
    d.push_back((c[i + 1] - c[i]) / (coeff_a * h[i]));
    b.push_back((a[i + 1] - a[i]) / h[i] -
                h[i] * (c[i + 1] + coeff_c * c[i]) / coeff_a);
  }
}

double Spline::calc(double t) {
  if (t < x.front() || t > x.back()) {
    throw std::invalid_argument("received value out of the pre-defined range");
  }
  int seg_id = bisect(t, 0, nx);
  double dx = t - x[seg_id];
  return a[seg_id] + b[seg_id] * dx + c[seg_id] * dx * dx +
         d[seg_id] * dx * dx * dx;
}

double Spline::calc_d(double t) {
  if (t < x.front() || t > x.back()) {
    throw std::invalid_argument("received value out of the pre-defined range");
  }
  int seg_id = bisect(t, 0, nx - 1);
  double dx = t - x[seg_id];
  return b[seg_id] + coeff_c * c[seg_id] * dx + coeff_a * d[seg_id] * dx * dx;
}

double Spline::calc_dd(double t) {
  if (t < x.front() || t > x.back()) {
    throw std::invalid_argument("received value out of the pre-defined range");
  }
  int seg_id = bisect(t, 0, nx);
  double dx = t - x[seg_id];
  return coeff_c * c[seg_id] + coeff_b * d[seg_id] * dx;
}

Eigen::MatrixXd Spline::calc_A() {
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(nx, nx);
  A(0, 0) = 1;
  for (int i = 0; i < nx - 1; i++) {
    if (i != nx - 2) {
      A(i + 1, i + 1) = coeff_c * (h[i] + h[i + 1]);
    }
    A(i + 1, i) = h[i];
    A(i, i + 1) = h[i];
  }
  A(0, 1) = 0.0;
  A(nx - 1, nx - 2) = 0.0;
  A(nx - 1, nx - 1) = 1.0;
  return A;
}

Eigen::VectorXd Spline::calc_B() {
  Eigen::VectorXd B = Eigen::VectorXd::Zero(nx);
  for (int i = 0; i < nx - 2; i++) {
    B(i + 1) = coeff_a * (a[i + 2] - a[i + 1]) / h[i + 1] -
               coeff_a * (a[i + 1] - a[i]) / h[i];
  }
  return B;
}

int Spline::bisect(double t, int start, int end) {
  int mid = (start + end) / static_cast<int>(coeff_c);
  if (t == x[mid] || end - start <= 1) {
    return mid;
  }
  if (t > x[mid]) {
    return bisect(t, mid, end);
  }
  return bisect(t, start, mid);
}

Spline2D::Spline2D(const std::vector<double> &x, const std::vector<double> &y)
    : s{calc_s(x, y)} {
  sx = Spline(s, x); // NOLINT
  sy = Spline(s, y);
}

std::array<double, 2> Spline2D::calc_position(double s_t) {
  double x = sx.calc(s_t);
  double y = sy.calc(s_t);
  return {{x, y}};
}

double Spline2D::calc_curvature(double s_t) {
  double dx = sx.calc_d(s_t);
  double ddx = sx.calc_dd(s_t);
  double dy = sy.calc_d(s_t);
  double ddy = sy.calc_dd(s_t);
  return (ddy * dx - ddx * dy) /
         std::pow((dx * dx + dy * dy), coeff_a / coeff_c);
}

double Spline2D::calc_yaw(double s_t) {
  double dx = sx.calc_d(s_t);
  double dy = sy.calc_d(s_t);
  return std::atan2(dy, dx);
}

std::vector<double> Spline2D::calc_s(const std::vector<double> &x,
                                     const std::vector<double> &y) {
  std::vector<double> ds;
  std::vector<double> out_s{0};
  std::vector<double> dx = utilities::math::vecDiff(x);
  std::vector<double> dy = utilities::math::vecDiff(y);

  for (unsigned int i = 0; i < dx.size(); i++) {
    ds.push_back(std::sqrt(dx.at(i) * dx.at(i) + dy.at(i) * dy.at(i)));
  }

  std::vector<double> cum_ds = utilities::math::cumSum(ds);
  out_s.insert(out_s.end(), cum_ds.begin(), cum_ds.end());
  return out_s;
}
} // namespace planning
