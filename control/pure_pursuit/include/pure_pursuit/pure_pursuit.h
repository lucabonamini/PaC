#pragma once

#include "utilities/math.h"
#include "utilities/types.h"
#include <string>
#include <utility>

namespace control {
class PurePursuit {
public:
  struct Config {
    double lookahead_time = 0.0;
    double target_velocity = 0.0;
    double path_resolution = 0.0;
    types::Path path;
    std::string type;
  };
  explicit PurePursuit(const Config &config) : config_(config) {}
  virtual ~PurePursuit() = default;
  types::Controls computeCommands(const types::State &robot_state);
  void setCurrentVelocity(double linear_velocity) {
    current_velocity_ = linear_velocity;
  }

private:
  double calcLookaheadDistance_() const;
  double calcAdaptiveLookaheadDistance_(const double &current_velocity) const;
  types::Point calcTargetPoint_(const types::State &robot_state);
  static types::Point
  transformToLocalCoordinates_(const types::Point &target_point,
                               const types::State &robot_state);
  Config config_;
  int closest_index_ = 0;
  double current_velocity_ = 0.0;
};

} // namespace control