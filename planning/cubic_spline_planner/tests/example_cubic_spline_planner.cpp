#include "cubic_spline_planner/cubic_spline_planner.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

cv::Point2i
cv_offset(float x, float y, int image_width = 2000, int image_height = 2000) {
  cv::Point2i output;
  output.x = int(x * 100) + image_width / 2;
  output.y = image_height - int(y * 100) - image_height / 3;
  return output;
}

int main() {

  std::vector<double> wx{-2.5, 0.0, 2.5, 5.0, 7.5, 3.0, -1.0};
  std::vector<double> wy{0.7, -6.0, 5.0, 6.5, 0.0, 5.0, -2.0};

  std::vector<double> res_x, res_y;

  planning::Spline2D csp(wx, wy);

  for (size_t count = 0; count < csp.s.back() * 10.0; ++count) {
    auto i = count / 10.0;
    auto p = csp.calc_position(i);
    res_x.push_back(p.at(0));
    res_y.push_back(p.at(1));
  }

  cv::Mat bg(2000, 2000, CV_8UC3, cv::Scalar(255, 255, 255));
  for (size_t i = 1; i < res_x.size(); ++i) {
    cv::line(bg,
             cv_offset(res_x.at(i - 1), res_y.at(i - 1), bg.cols, bg.rows),
             cv_offset(res_x.at(i), res_y.at(i), bg.cols, bg.rows),
             cv::Scalar(0, 0, 0),
             10);
  }
  for (size_t i = 0; i < wx.size(); ++i) {
    cv::circle(bg,
               cv_offset(wx.at(i), wy.at(i), bg.cols, bg.rows),
               40,
               cv::Scalar(255, 0, 0),
               -1);
  }
  decltype(bg) outImg;
  cv::resize(bg, outImg, cv::Size(), 0.2, 0.2);
  cv::imshow("cubic_spline_planner", outImg);
  cv::waitKey(5000);

  return 0;
}
