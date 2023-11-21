#include "cubic_spline_planner/cubic_spline_planner.h"
#include "matplotlibcpp.h"
#include "mpc/mpc.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

using namespace matplotlibcpp;

constexpr int MAX_TIME = 5000;

cv::Point2i
cv_offset(float x, float y, int image_width = 2000, int image_height = 2000) {
  cv::Point2i output;
  output.x = int(x * 20) + 300;
  output.y = image_height - int(y * 20) - image_height / 5;
  return output;
};

void smoothYaw(std::vector<double> &cyaw) {
  for (size_t i = 0; i < cyaw.size() - 1; i++) {
    double dyaw = cyaw[i + 1] - cyaw[i];

    while (dyaw > M_PI / 2.0) {
      cyaw[i + 1] -= M_PI * 2.0;
      dyaw = cyaw[i + 1] - cyaw[i];
    }
    while (dyaw < -M_PI / 2.0) {
      cyaw[i + 1] += M_PI * 2.0;
      dyaw = cyaw[i + 1] - cyaw[i];
    }
  }
}

int main() {

  // std::vector<double> wx {-2.5,0.0,2.5,5.0,7.5,3.0,-1.0};
  // std::vector<double> wy {0.7,-6.0,2.0,-4.0,0.0,5.0,-2.0};
  std::vector<double> wx({0.0, 60.0, 125.0, 50.0, 75.0, 35.0, -10.0});
  std::vector<double> wy({0.0, 0.0, 50.0, 65.0, 30.0, 50.0, -20.0});

  // std::vector<State> states;

  std::vector<double> rx, ry, ryaw, rk, rs;

  planning::Spline2D csp(wx, wy);
  for (double i = 0; i < csp.s.back(); i += 0.1) {
    auto p = csp.calc_position(i);
    rx.push_back(p.at(0));
    ry.push_back(p.at(1));
    ryaw.push_back(csp.calc_yaw(i));
    rk.push_back(csp.calc_curvature(i));
    rs.push_back(i);
  }

  auto speed_profile = calcSpeedProfile(rs, rk);
  State initial_state{.x = rx.at(0),
                      .y = ry.at(0),
                      .yaw = ryaw.at(0),
                      .v = speed_profile.at(0)};

  smoothYaw(ryaw);

  cv::namedWindow("mpc", cv::WINDOW_NORMAL);

  MPC mpc(initial_state, rx, ry, ryaw, rk, rs, speed_profile);
  mpc.run();

  for (size_t ii = 0; ii < mpc.states_.size(); ii++) {
    // visualization
    cv::Mat bg(2000, 3000, CV_8UC3, cv::Scalar(255, 255, 255));
    for (unsigned int i = 1; i < rx.size(); i++) {
      cv::line(bg,
               cv_offset(rx[i - 1], ry[i - 1], bg.cols, bg.rows),
               cv_offset(rx[i], ry[i], bg.cols, bg.rows),
               cv::Scalar(0, 0, 0),
               10);
    }

    // for(unsigned int j=0; j< T; j++){
    // 	cv::circle(
    // 		bg,
    // 		cv_offset(output[x_start+j], output[y_start+j], bg.cols,
    // bg.rows), 		10, cv::Scalar(0, 0, 255), -1);
    // }

    for (unsigned int k = 0; k < mpc.states_.size(); k++) {
      cv::circle(
          bg,
          cv_offset(mpc.states_.at(k).x, mpc.states_.at(k).y, bg.cols, bg.rows),
          8,
          cv::Scalar(255, 0, 0),
          -1);
    }

    // cv::line(bg,
    //          cv_offset(state.x, state.y, bg.cols, bg.rows),
    //          cv_offset(state.x + std::cos(state.yaw) * WB * 2,
    //                    state.y + std::sin(state.yaw) * WB * 2,
    //                    bg.cols,
    //                    bg.rows),
    //          cv::Scalar(255, 0, 255),
    //          15);

    // cv::line(bg,
    //          cv_offset(state.x + std::cos(state.yaw) * 0.5,
    //                    state.y + std::sin(state.yaw) * 0.5,
    //                    bg.cols,
    //                    bg.rows),
    //          cv_offset(state.x - std::cos(state.yaw) * 0.5,
    //                    state.y - std::sin(state.yaw) * 0.5,
    //                    bg.cols,
    //                    bg.rows),
    //          cv::Scalar(255, 0, 127),
    //          30);

    // cv::line(bg,
    //          cv_offset(state.x + std::cos(state.yaw) * WB * 2 +
    //                        std::cos(state.yaw + steer) * 0.5,
    //                    state.y + std::sin(state.yaw) * WB * 2 +
    //                        std::sin(state.yaw + steer) * 0.5,
    //                    bg.cols,
    //                    bg.rows),
    //          cv_offset(state.x + std::cos(state.yaw) * WB * 2 -
    //                        std::cos(state.yaw + steer) * 0.5,
    //                    state.y + std::sin(state.yaw) * WB * 2 -
    //                        std::sin(state.yaw + steer) * 0.5,
    //                    bg.cols,
    //                    bg.rows),
    //          cv::Scalar(255, 0, 127),
    //          30);

    // for (unsigned int k = 0; k < xref.cols(); k++) {
    //   cv::drawMarker(bg,
    //                  cv_offset(xref(0, k), xref(1, k), bg.cols, bg.rows),
    //                  cv::Scalar(0, 255, 255),
    //                  cv::MARKER_CROSS,
    //                  20,
    //                  3);
    // }

    // save image in build/bin/pngs
    // struct timeval tp;
    // gettimeofday(&tp, NULL);
    // long int ms = tp.tv_sec * 1000 + tp.tv_usec / 1000;
    // std::string int_count = std::to_string(ms);
    // cv::imwrite("./pngs/"+int_count+".png", bg);

    cv::imshow("mpc", bg);
    cv::waitKey(5);
  }

  // vector<double> x, y;
  // for (size_t i = 0; i < mpc.states_.size(); i++) {
  //   x.push_back(mpc.states_.at(i).x);
  //   y.push_back(mpc.states_.at(i).y);
  // }
  // figure();
  // plot(rx, ry, "-k");
  // title("MPC Controller");
  // plot(x, y, "or");
  // show();

  return 0;
}
