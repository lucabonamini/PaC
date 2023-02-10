#include "bezier/bezier.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <sys/time.h>

cv::Point2i cv_offset(float x, float y, int image_height = 2000) {
  cv::Point2i output;
  output.x = int(x * 100) + 800;
  output.y = image_height - int(y * 100) - image_height / 2 - 200;
  return output;
}

int main() {
  double start_x = 5.0;
  double start_y = 1.0;
  double end_x = -6.0;
  double end_y = -8.0;
  std::vector<std::array<double, 2>> control_points = {{{start_x, start_y}},
                                                       {{-2.0, 1.0}},
                                                       {{-6.5, -4.5}},
                                                       {{-3.0, -3.0}},
                                                       {{end_x, end_y}}};
  auto path = bezier::calcBezierPath(control_points);

  // Visualization
  int count = 0;
  while (count < 60) {
    cv::Mat bg(1500, 1500, CV_8UC3, cv::Scalar(255, 255, 255));

    if (count > 30) {
      for (size_t i = 1; i < path.size(); i++) {
        cv::line(bg,
                 cv_offset(path.at(i - 1).at(0), path.at(i - 1).at(1), bg.rows),
                 cv_offset(path.at(i).at(0), path.at(i).at(1), bg.rows),
                 cv::Scalar(0, 0, 0),
                 10);
      }
      for (size_t i = 0; i < control_points.size(); i++) {
        cv::circle(bg,
                   cv_offset(control_points.at(i).at(0),
                             control_points.at(i).at(1),
                             bg.rows),
                   30,
                   cv::Scalar(0, 255, 0),
                   -1);
      }
      cv::circle(bg,
                 cv_offset(start_x, start_y, bg.rows),
                 40,
                 cv::Scalar(0, 0, 255),
                 -1);
      cv::circle(
          bg, cv_offset(end_x, end_y, bg.rows), 40, cv::Scalar(255, 0, 0), -1);
    }

    decltype(bg) outImg;
    cv::resize(bg, outImg, cv::Size(), 0.2, 0.2);
    cv::imshow("bezier", outImg);
    cv::waitKey(5);
    struct timeval tp;
    gettimeofday(&tp, NULL);
    long int ms = tp.tv_sec * 1000 + tp.tv_usec / 1000;
    std::string int_count = std::to_string(ms);
    cv::imwrite("/home/nvidia/WIP_Projects/pngs/" + int_count + ".png", bg);
    count++;
  }

  return 0;
}