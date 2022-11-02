#include "cubic_spline_planner/cubic_spline_planner.h"
#include "speed_profile/speed_profile.h"

#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

constexpr double path_resolution = 10.0;

int main() {

  std::vector<double> wx{-2.5, 0.0, 2.5, 5.0, 7.5, 3.0, -1.0};
  std::vector<double> wy{0.7, -6.0, 5.0, 6.5, 0.0, 5.0, -2.0};

  std::vector<double> res_x;
  std::vector<double> res_y;

  planning::Spline2D csp(wx, wy);
  ::types::Path path;

  for (size_t count = 0; count < csp.s.back() * path_resolution; ++count) {
    auto i = count / path_resolution;
    auto p = csp.calc_position(i);
    res_x.push_back(p.at(0));
    res_y.push_back(p.at(1));

    path.push_back(::types::Position{
      .point=::types::Point{
        .x=p.at(0),
        .y=p.at(1)
      },
      .yaw=0.0
    });
  }


  auto traj = ::planning::computeSpeedProfile(path,0.0,2.0);
  std::vector<double> traj_x,traj_y,traj_v;

  for (size_t i = 0; i < traj.size(); i++) {
    traj_x.push_back(traj.at(i).x);
    traj_y.push_back(traj.at(i).y);
    traj_v.push_back(traj.at(i).v);
  }

  plt::figure();
  plt::plot(wx, wy, "or");
  plt::plot(res_x, res_y);
  plt::figure();
  plt::plot(traj_x,traj_y);
  plt::figure();
  plt::plot(traj_v);
  plt::show();

  return 0;
}
