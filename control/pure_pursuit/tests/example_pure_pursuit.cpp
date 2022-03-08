#include "cubic_spline_planner/cubic_spline_planner.h"
#include "matplotlibcpp.h"
#include "models/unicycle.h"
#include "pure_pursuit/pure_pursuit.h"
#include "utilities/math.h"
#include "utilities/types.h"

constexpr int MAX_TIME = 5000;
constexpr double FREQUENCY = 10.0;
constexpr double TARGET_VELOCITY = 1.0; // [m/s]
constexpr double LAT = 2;               // lookahead time [s]
constexpr double PATH_RESOLUTION = 0.1;

int main() {

  std::vector<double> wx{-2.5, 0.0, 2.5, 5.0, 7.5, 3.0, -1.0};
  std::vector<double> wy{0.7, -6.0, 2.0, -4.0, 0.0, 5.0, -2.0};
  int time = 0;
  std::vector<types::State> states;

  std::vector<double> rx, ry, ryaw;
  planning::Spline2D csp(wx, wy);
  types::Path path;
  for (double i = 0; i < csp.s.back(); i += PATH_RESOLUTION) {
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
  state.v = 0.1;

  model::Unicycle uni(FREQUENCY);
  control::PurePursuit pp(
      control::PurePursuit::Config{.lookahead_time = LAT,
                                   .target_velocity = TARGET_VELOCITY,
                                   .path_resolution = PATH_RESOLUTION,
                                   .path = std::move(path),
                                   .type = "Classic"});

  while (time < MAX_TIME) {
    auto controls = pp.computeCommands(state);

    uni.updateState(state, controls);

    auto dist_from_goal =
        std::sqrt((state.x - rx.back()) * (state.x - rx.back()) +
                  (state.y - ry.back()) * (state.y - ry.back()));

    if (dist_from_goal < 0.1) {
      std::cout << "==== GOAL ====" << std::endl;
      break;
    }
    time++;
    states.push_back(state);
  }

  std::vector<double> state_x, state_y, state_v;
  for (auto s : states) {
    state_x.push_back(s.x);
    state_y.push_back(s.y);
    state_v.push_back(s.v);
  }
  matplotlibcpp::figure();
  matplotlibcpp::plot(rx, ry, "-k");
  matplotlibcpp::plot(wx, wy, "ob");
  matplotlibcpp::plot(state_x, state_y, "xr");
  matplotlibcpp::title("Pure Pursuit Controller");
  matplotlibcpp::figure();
  matplotlibcpp::plot(state_v);
  matplotlibcpp::show();

  return 0;
}