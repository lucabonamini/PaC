#include "speed_profile/speed_profile.h"

#include <algorithm>
#include <cmath>

namespace planning {
// Reference: https://arxiv.org/pdf/1902.00606v1.pdf
std::vector<double> computeSpeedProfile(const std::vector<double> &s,
                                        const std::vector<double> &k,
                                        const double &max_longitudinal_velocity,
                                        const double &max_acceleration) {
  std::vector<double> v1;
  std::vector<double> v2;
  std::vector<double> v3;

  // Calc maximum permissible steady state vehicle speed
  for (const auto &el : k) {
    if (std::abs(el) > 0.0001) {
      auto velocity = sqrt(max_acceleration / std::abs(el));
      if (velocity > max_longitudinal_velocity) {
        velocity = max_longitudinal_velocity;
      }
      v1.push_back(velocity);
    }
  }
  // Forward integration step
  for (size_t i = 1; i < v1.size(); i++) {
    v2.push_back(std::min(
        sqrt(pow(v1.at(i - 1), 2) + 2 * max_acceleration * (s.at(i) - s.at(i - 1))),
        v1.at(i)));
  }
  // TO DO fix backward step
  for (size_t i = v2.size() - 1; i > 1; i--) {
    v3.push_back(
        std::min(sqrt(std::abs(pow(v2.at(i - 1), 2) -
                               2 * max_acceleration * (s.at(i) - s.at(i - 1)))),
                 v2.at(i)));
  }
  std::reverse(v3.begin(), v3.end());
  return v2;
}
} // namespace planning