#include "dwa/dwa.h"
#include <gflags/gflags.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

DEFINE_string(config_file, "", "Set path to configuration file.");

cv::Point2i cv_offset(float x, float y, int image_height = 2000) {
  cv::Point2i output;
  output.x = int(x * 100) + 300;
  output.y = image_height - int(y * 100) - image_height / 3;
  return output;
}

int main(int argc, char **argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  Obstacles obs = {Obstacle{.x = -1.0, .y = -1.0},
                   Obstacle{.x = 0.0, .y = 2.0},
                   Obstacle{.x = 4.0, .y = 2.0},
                   Obstacle{.x = 5.0, .y = 4.0},
                   Obstacle{.x = 5.0, .y = 5.0},
                   Obstacle{.x = 5.0, .y = 6.0},
                   Obstacle{.x = 5.0, .y = 9.0},
                   Obstacle{.x = 8.0, .y = 9.0},
                   Obstacle{.x = 7.0, .y = 9.0},
                   Obstacle{.x = 12.0, .y = 12.0}};
  ::types::Point goal{.x = 10.0, .y = 10.0};
  ::types::State init_state{.x = 0.0, .y = 0.0, .yaw = M_PI / 8.0, .v = 0.0};
  DWA dwa(DWA::parseConfigFile(FLAGS_config_file), obs, goal, init_state);
  bool goal_reached = false;

  std::vector<double> goal_x, goal_y;
  goal_x.push_back(goal.x);
  goal_x.push_back(goal.x);
  goal_y.push_back(goal.y);
  goal_y.push_back(goal.y);
  std::vector<double> obs_x, obs_y;
  for (const auto &ob : obs) {
    obs_x.push_back(ob.x);
    obs_y.push_back(ob.y);
  }

  while (!goal_reached) {
    goal_reached = dwa.dwaControls();

    // visualization
    cv::Mat bg(2000, 2000, CV_8UC3, cv::Scalar(255, 255, 255));
    cv::circle(
        bg, cv_offset(goal.x, goal.y, bg.rows), 30, cv::Scalar(255, 0, 0), -1);
    for (unsigned int j = 0; j < obs.size(); j++) {
      cv::circle(bg,
                 cv_offset(obs.at(j).x, obs.at(j).y, bg.rows),
                 20,
                 cv::Scalar(0, 0, 0),
                 -1);
    }
    for (unsigned int j = 0; j < dwa.viz_traj.size(); j++) {
      cv::circle(bg,
                 cv_offset(dwa.viz_traj.at(j).x, dwa.viz_traj.at(j).y, bg.rows),
                 7,
                 cv::Scalar(0, 255, 0),
                 -1);
    }
    cv::circle(
        bg,
        cv_offset(dwa.viz_traj.front().x, dwa.viz_traj.front().y, bg.rows),
        30,
        cv::Scalar(0, 0, 255),
        -1);

    cv::arrowedLine(
        bg,
        cv_offset(dwa.viz_traj.front().x, dwa.viz_traj.front().y, bg.rows),
        cv_offset(dwa.viz_traj.front().x + std::cos(dwa.viz_traj.front().yaw),
                  dwa.viz_traj.front().y + std::sin(dwa.viz_traj.front().yaw),
                  bg.rows),
        cv::Scalar(255, 0, 255),
        7);
    decltype(bg) outImg;
    cv::resize(bg, outImg, cv::Size(), 0.2, 0.2);
    cv::imshow("frenet", outImg);
    cv::waitKey(5);
  }
  return 0;
}