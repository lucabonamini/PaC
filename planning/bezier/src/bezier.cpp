#include "bezier/bezier.h"

#include <algorithm>

namespace bezier {
double bernsteinPoly(const size_t &n, const size_t &i, const double &t) {
  return static_cast<double>(nCr(n, i)) *
         static_cast<double>(pow(t, static_cast<double>(i))) *
         pow((1 - t), (static_cast<double>(n) - static_cast<double>(i)));
}

std::array<double, 2>
bezier(const double &t,
       const std::vector<std::array<double, 2>> &control_points) {
  size_t n = control_points.size() - 1;
  double x = 0.0;
  double y = 0.0;
  for (size_t i = 0; i < n + 1; i++) {
    auto d = control_points.at(i) * bernsteinPoly(n, i, t);
    x += d.at(0);
    y += d.at(1);
  }
  std::array<double, 2> res = {{x, y}};
  return res;
}

std::vector<std::array<double, 2>>
calcBezierPath(const std::vector<std::array<double, 2>> &control_points,
               const int &n_points) {
  std::vector<std::array<double, 2>> trajectory;
  trajectory.reserve(control_points.size());
  auto linspace_vec = linspace(0.0, 1.0, static_cast<double>(n_points));
  std::transform(
      linspace_vec.begin(),
      linspace_vec.end(),
      std::back_inserter(trajectory),
      [&control_points](const auto &t) { return bezier(t, control_points); });
  return trajectory;
}
} // namespace bezier