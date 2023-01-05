#include "matplotlibcpp.h"
#include "quintic_polynomial_planner/quintic_polynomial_planner.h"

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

using namespace QuinticPolynomialPlanner;

DEFINE_double(start_x, 10.0, "Start X position.");
DEFINE_double(start_y, 10.0, "Start Y position.");
DEFINE_double(start_yaw, 10.0 * M_PI / 180, "Start Yaw position.");
DEFINE_double(start_v, 1.0, "Start Velocity.");
DEFINE_double(start_a, 0.1, "Start Acceleration.");
DEFINE_double(goal_x, 30.0, "Goal X position.");
DEFINE_double(goal_y, -10.0, "Goal Y position.");
DEFINE_double(goal_yaw, 20.0 * M_PI / 180, "Goal Yaw position.");
DEFINE_double(goal_v, 1.0, "Goal Velocity.");
DEFINE_double(goal_a, 0.1, "Goal Acceleration.");
DEFINE_double(min_t, 5.0, "Minimum sample time.");
DEFINE_double(max_t, 100.0, "Maximum sample time.");
DEFINE_double(dt, 0.1, "Time tick.");
DEFINE_double(max_accel, 1.0, "Maximum Acceleration.");
DEFINE_double(max_jerk, 0.5, "Maximum Jerk.");

cv::Point2i cv_offset(float x, float y, int image_height = 2000) {
  cv::Point2i output;
  output.x = int(x * 100) - 400;
  output.y = image_height - int(y * 100) - image_height / 1 + 1300;
  return output;
}

int main(int argc, char **argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  Input input;
  input.sx = FLAGS_start_x;
  input.sy = FLAGS_start_y;
  input.syaw = FLAGS_start_yaw;
  input.sv = FLAGS_start_v;
  input.sa = FLAGS_start_a;
  input.gx = FLAGS_goal_x;
  input.gy = FLAGS_goal_y;
  input.gyaw = FLAGS_goal_yaw;
  input.gv = FLAGS_goal_v;
  input.ga = FLAGS_goal_a;
  input.min_t = FLAGS_min_t;
  input.max_t = FLAGS_max_t;
  input.dt = FLAGS_dt;
  input.max_accel = FLAGS_max_accel;
  input.max_jerk = FLAGS_max_jerk;

  auto output = plan(input);

  if (!output) {
    LOG(ERROR) << "No path found.";
  } else {
    for (auto i = 0; i < output->rx.size(); i++) {
      cv::Mat bg(3000, 3000, CV_8UC3, cv::Scalar(255, 255, 255));
      cv::circle(bg,
                 cv_offset(input.sx, input.sy, bg.rows),
                 30,
                 cv::Scalar(255, 0, 0),
                 -1);
      cv::circle(bg,
                 cv_offset(input.gx, input.gy, bg.rows),
                 30,
                 cv::Scalar(255, 0, 0),
                 -1);
      cv::arrowedLine(bg,
                      cv_offset(input.sx, input.sy, bg.rows),
                      cv_offset(input.sx + std::cos(input.syaw),
                                input.sy + std::sin(input.syaw),
                                bg.rows),
                      cv::Scalar(255, 0, 255),
                      7);
      cv::arrowedLine(bg,
                      cv_offset(input.gx, input.gy, bg.rows),
                      cv_offset(input.gx + std::cos(input.gyaw),
                                input.gy + std::sin(input.gyaw),
                                bg.rows),
                      cv::Scalar(255, 0, 255),
                      7);
      cv::arrowedLine(bg,
                      cv_offset(output->rx.at(i), output->ry.at(i), bg.rows),
                      cv_offset(output->rx.at(i) + std::cos(output->ryaw.at(i)),
                                output->ry.at(i) + std::sin(output->ryaw.at(i)),
                                bg.rows),
                      cv::Scalar(255, 0, 255),
                      7);
      decltype(bg) outImg;
      cv::resize(bg, outImg, cv::Size(), 0.2, 0.2);
      cv::imshow("quintic", outImg);
      cv::waitKey(5);
    }

    std::vector<double> pts_x, pts_y;
    pts_x.push_back(FLAGS_start_x);
    pts_x.push_back(FLAGS_goal_x);
    pts_y.push_back(FLAGS_start_y);
    pts_y.push_back(FLAGS_goal_y);
    matplotlibcpp::figure();
    matplotlibcpp::plot(output->rx, output->ry, "-k");
    matplotlibcpp::plot(pts_x, pts_y, "ob");
    matplotlibcpp::title("Quintic Polynomial Path");
    matplotlibcpp::figure();
    matplotlibcpp::plot(output->time, output->ryaw, "-r");
    matplotlibcpp::title("Yaw");
    matplotlibcpp::figure();
    matplotlibcpp::plot(output->time, output->rv, "-r");
    matplotlibcpp::title("Speed");
    matplotlibcpp::figure();
    matplotlibcpp::plot(output->time, output->ra, "-r");
    matplotlibcpp::title("Accel");
    matplotlibcpp::figure();
    matplotlibcpp::plot(output->time, output->rj, "-r");
    matplotlibcpp::title("Jerk");
    matplotlibcpp::show();
  }
  return 0;
}
