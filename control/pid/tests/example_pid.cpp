#include "cubic_spline_planner/cubic_spline_planner.h"
#include "matplotlibcpp.h"
#include "models/unicycle.h"
#include "pid/pid.h"
#include "utilities/math.h"
#include "utilities/types.h"

namespace plt = matplotlibcpp;

constexpr int MAX_TIME = 5000;

int main() {

  std::vector<double> wx{-2.5, 0.0, 2.5, 5.0, 7.5, 3.0, -1.0};
  std::vector<double> wy{0.7, -6.0, 2.0, -4.0, 0.0, 5.0, -2.0};
  int time = 0;
  std::vector<types::State> states;

  std::vector<double> rx, ry, ryaw;
  planning::Spline2D csp(wx, wy);
  types::Path path;
  for (double i = 0; i < csp.s.back(); i += 0.1) {
    auto p = csp.calc_position(i);
    rx.push_back(p.at(0));
    ry.push_back(p.at(1));
    ryaw.push_back(csp.calc_yaw(i));
    path.push_back(types::Position{.point = {.x = p.at(0), .y = p.at(1)},
                                   .yaw = csp.calc_yaw(i)});
  }

  // Initial conditions
  types::State state;
  state.x = rx.front();
  state.y = ry.front();
  state.yaw = std::atan2((ry.at(1) - ry.at(0)), (rx.at(1) - rx.at(0)));
  state.v = 2.0;
  model::Unicycle unicycle(10.0);
  control::Pid pid(10.0, 5.37114e-07, 1.26836);

  int closest_index = 0;
  while (time < MAX_TIME) {
    utilities::math::findClosestIndex(closest_index, {state.x, state.y}, path);

    auto steer = pid.calcCommand(utilities::math::normalizeAngle(
        path.at(closest_index).yaw - state.yaw));
    // if (steer < -30*M_PI/180) {
    //     steer = -30*M_PI/180;
    // } else if (steer > 30*M_PI/180) {
    //     steer = 30*M_PI/180;
    // }

    types::Controls controls{.steer = steer, .v = 0.1, .a = 0.0};

    unicycle.updateState(state, controls);

    auto dist_from_goal =
        std::sqrt((state.x - rx.back()) * (state.x - rx.back()) +
                  (state.y - ry.back()) * (state.y - ry.back()));

    std::cout << "dist_from_goal: " << dist_from_goal << std::endl;

    if (dist_from_goal < 0.1) {
      std::cout << "==== GOAL ====" << std::endl;
      break;
    }
    time++;
    states.push_back(state);
  }

  std::vector<double> state_x, state_y;
  for (auto s : states) {
    state_x.push_back(s.x);
    state_y.push_back(s.y);
  }
  plt::figure();
  plt::plot(rx, ry, "-k");
  plt::plot(wx, wy, "ob");
  plt::plot(state_x, state_y, "xr");
  plt::title("PID Controller");
  plt::show();

  return 0;
}
