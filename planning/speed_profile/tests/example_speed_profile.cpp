#include "cubic_spline_planner/cubic_spline_planner.h"
#include "matplotlibcpp.h"
#include "speed_profile/speed_profile.h"
#include "utilities/types.h"

#include <thread>

namespace plt = matplotlibcpp;

int main() {

  std::vector<double> wx{-2.5, 0.0, 2.5, 5.0, 7.5, 3.0, -1.0};
  std::vector<double> wy{0.7, -6.0, 2.0, -4.0, 0.0, 5.0, -2.0};

  std::vector<double> rx, ry, ryaw, rs, rk;
  planning::Spline2D csp(wx, wy);
  for (double i = 0; i < csp.s.back(); i += 0.1) {
    auto p = csp.calc_position(i);
    rx.push_back(p.at(0));
    ry.push_back(p.at(1));
    ryaw.push_back(csp.calc_yaw(i));
    rs.push_back(i);
    rk.push_back(csp.calc_curvature(i));
  }

  auto rv = ::planning::computeSpeedProfile(rs, rk, 1.0, 0.5);

  plt::figure();
  plt::plot(rk, "*");
  plt::plot(rv, "*");
  plt::show();

  return 0;
}
