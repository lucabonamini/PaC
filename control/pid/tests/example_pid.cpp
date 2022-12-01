#include "cubic_spline_planner/cubic_spline_planner.h"
#include "matplotlibcpp.h"
#include "models/dynamic_unicycle.h"
#include "pid/pid.h"
#include "speed_profile/speed_profile.h"
#include "utilities/math.h"
#include "utilities/planning.h"
#include "utilities/types.h"

namespace plt = matplotlibcpp;

constexpr int MAX_TIME = 5000;

int main() {

  std::vector<double> wx{-2.5, 0.0, 2.5, 5.0, 7.5, 3.0, -1.0};
  std::vector<double> wy{0.7, -6.0, 2.0, -4.0, 0.0, 5.0, -2.0};
  int time = 0;
  std::vector<::types::State> states;

  std::vector<double> rx, ry, ryaw, rs, rk;
  planning::Spline2D csp(wx, wy);
  ::types::Traj path;
  for (double i = 0; i < csp.s.back(); i += 0.1) {
    auto p = csp.calc_position(i);
    rx.push_back(p.at(0));
    ry.push_back(p.at(1));
    ryaw.push_back(csp.calc_yaw(i));
    rs.push_back(i);
    rk.push_back(csp.calc_curvature(i));
    path.push_back(::types::State{.x = p.at(0),
                                  .y = p.at(1),
                                  .yaw = csp.calc_yaw(i),
                                  .v = 0.0,
                                  .w = 0.0});
  }

  // Initial conditions
  ::types::State state;
  state.x = rx.front();
  state.y = ry.front();
  state.yaw = std::atan2((ry.at(1) - ry.at(0)), (rx.at(1) - rx.at(0)));
  state.v = 0.0;
  model::DynamicUnicycle unicycle(10.0);
  control::Pid pid(20.0, 0.0, 0.0);
  control::Pid v_pid(1.0, 0.0, 0.0);

  auto rv = ::planning::computeSpeedProfile(rs, rk, 1.0, 0.5);

  for (size_t i = 0; i < rv.size(); i++) {
    path.at(i).v = rv.at(i);
  }

  size_t closest_index = 0;
  while (time < MAX_TIME) {
    ::utilities::planning::findClosestIndex(
        closest_index, {state.x, state.y}, path);

    auto steer = pid.calcCommand(utilities::math::normalizeAngle(
        path.at(closest_index).yaw - state.yaw));
    auto speed = v_pid.calcCommand(rv.at(closest_index) - state.v);

    ::types::Controls controls{.steer = steer, .v = speed, .a = 0.0};

    unicycle.updateState(state, controls);

    auto dist_from_goal =
        std::sqrt((state.x - rx.back()) * (state.x - rx.back()) +
                  (state.y - ry.back()) * (state.y - ry.back()));

    if (dist_from_goal < 0.3) {
      std::cout << "==== GOAL ====" << std::endl;
      break;
    }
    time++;
    states.push_back(state);
  }

  // Visualization
  std::vector<double> state_x, state_y, state_v;
  for (auto s : states) {
    state_x.push_back(s.x);
    state_y.push_back(s.y);
    state_v.push_back(s.v);
  }
  plt::figure();
  plt::plot(rx, ry, "-k");
  plt::plot(wx, wy, "ob");
  plt::plot(state_x, state_y, "xr");
  plt::title("PID Controller");
  plt::show();

  return 0;
}
