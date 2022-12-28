#include "cubic_spline_planner/cubic_spline_planner.h"
#include "models/dynamic_unicycle.h"
#include "pid/pid.h"
#include "speed_profile/speed_profile.h"
#include "utilities/math.h"
#include "utilities/planning.h"
#include "utilities/types.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

constexpr int MAX_TIME = 5000;

cv::Point2i cv_offset(float x, float y, int image_height = 2000) {
  cv::Point2i output;
  output.x = int(x * 100) + 600;
  output.y = image_height - int(y * 100) - image_height / 2;
  return output;
}

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
  control::Pid pid(15.0, 0.0, 0.0);
  control::Pid v_pid(1.0, 0.0, 0.0);

  auto rv = ::planning::computeSpeedProfile(rs, rk, 1.0, 0.5);

  for (size_t i = 0; i < rv.size(); i++) {
    path.at(i).v = rv.at(i);
  }

  size_t closest_index = 0;
  while (time < MAX_TIME) {
    ::utilities::planning::findClosestIndex(
        closest_index, {state.x, state.y}, path);

    auto steer = pid.calculateValue(utilities::math::normalizeAngle(
        path.at(closest_index).yaw - state.yaw));
    auto speed = v_pid.calculateValue(rv.at(closest_index) - state.v);

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

    cv::Mat bg(2000, 2000, CV_8UC3, cv::Scalar(255, 255, 255));
    for (size_t i = 1; i < rx.size(); i++) {
      cv::line(bg,
               cv_offset(rx.at(i - 1), ry.at(i - 1), bg.rows),
               cv_offset(rx.at(i), ry.at(i), bg.rows),
               cv::Scalar(0, 0, 0),
               10);
    }
    for (size_t i = 0; i < states.size(); i++) {
      cv::circle(bg,
                 cv_offset(states.at(i).x, states.at(i).y, bg.rows),
                 10,
                 cv::Scalar(0, 255, 0),
                 -1);
    }
    cv::circle(bg,
               cv_offset(states.back().x, states.back().y, bg.rows),
               30,
               cv::Scalar(0, 0, 255),
               -1);
    cv::circle(bg,
               cv_offset(rx.back(), ry.back(), bg.rows),
               30,
               cv::Scalar(255, 0, 0),
               -1);
    cv::arrowedLine(bg,
                    cv_offset(states.back().x, states.back().y, bg.rows),
                    cv_offset(states.back().x + std::cos(states.back().yaw),
                              states.back().y + std::sin(states.back().yaw),
                              bg.rows),
                    cv::Scalar(255, 0, 255),
                    7);

    decltype(bg) outImg;
    cv::resize(bg, outImg, cv::Size(), 0.2, 0.2);
    cv::imshow("pid", outImg);
    cv::waitKey(5);
  }

  return 0;
}
