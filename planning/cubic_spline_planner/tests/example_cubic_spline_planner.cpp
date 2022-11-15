#include "cubic_spline_planner/cubic_spline_planner.h"
#include "speed_profile/speed_profile.h"

#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

constexpr double path_resolution = 10.0;

int main() {

  std::vector<double> wx{-2.5, 0.0, 2.5, 5.0, 7.5, 3.0, -1.0};
  std::vector<double> wy{0.7, -6.0, 5.0, 6.5, 0.0, 5.0, -2.0};

  std::vector<double> res_x, res_y, res_s, res_k;

  planning::Spline2D csp(wx, wy);
  ::types::Path path;

  for (size_t count = 0; count < csp.s.back() * path_resolution; ++count) {
    auto i = count / path_resolution;
    auto p = csp.calc_position(i);
    res_x.push_back(p.at(0));
    res_y.push_back(p.at(1));
    res_s.push_back(i);
    res_k.push_back(csp.calc_curvature(i));

    path.push_back(::types::Position{
        .point = ::types::Point{.x = p.at(0), .y = p.at(1)}, .yaw = 0.0});
  }

  auto traj_v = ::planning::computeSpeedProfile(res_s, res_k, 1.0, 0.5);
  std::vector<double> traj_x, traj_y;

  for (size_t i = 0; i < traj_v.size(); i++) {
    traj_x.push_back(res_x.at(i));
    traj_y.push_back(res_y.at(i));
  }

  plt::figure();
  plt::plot(wx, wy, "or");
  plt::plot(res_x, res_y);
  plt::figure();
  plt::plot(traj_x, traj_y);
  plt::figure();
  plt::plot(traj_v);
  plt::show();

  return 0;
}
