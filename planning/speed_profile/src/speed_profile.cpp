#include "speed_profile/speed_profile.h"

#include <algorithm>
#include <cmath>

const double max_vel = 2.0;
const double max_acc = 0.5;

namespace planning {
// Reference: https://arxiv.org/pdf/1902.00606v1.pdf
std::vector<double> computeSpeedProfile(const std::vector<double> &s,
    const std::vector<double> &k) {
        std::vector<double> tmp1;
        std::vector<double> tmp2;
        std::vector<double> tmp3;
        std::vector<double> speed_profile;
        tmp1.reserve(s.size());
        tmp2.reserve(s.size());
        tmp3.reserve(s.size());
        speed_profile.reserve(s.size());
        // Calc maximum permissible steady state vehicle speed
        for (const auto& el : k) {
          double velocity = 0.0;
          if (el < 0.0) {
              velocity = max_vel;
          } else {
              velocity = sqrt(max_acc / std::abs(el));
              if (velocity > max_vel) {
                  velocity = max_vel;
              }
          }
          tmp1.push_back(velocity);
        }
        // Forward integration step
        for (size_t i = 0; i < tmp1.size()-1; i++) {
            tmp2.push_back(std::min(sqrt(pow(tmp1.at(i),2)+2*max_acc*s.at(i)),
                tmp1.at(i)));
        }
        // TO DO fix backward step
        // Backward integration step
        for (size_t i = tmp2.size()-1; i > 0; i--) {
            tmp3.push_back(std::min(sqrt(std::abs(pow(tmp2.at(i),2)-2*max_acc*s.at(i))),
                tmp2.at(i)));
        }
        std::reverse(tmp3.begin(),tmp3.end());
        // speed_profile = std::move(tmp3);
        return tmp1;
}
} // namespace planning