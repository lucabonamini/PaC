#pragma once

#include "utilities/math.h"
#include "utilities/types.h"

namespace control {
class PurePursuit {
public:
  struct Config {
    double lookahead_time = 0.0;
    double target_velocity = 0.0;
    double path_resolution = 0.0;
    ::types::Traj path;
  };
  struct Output {
    ::types::Controls controls;
    size_t target_point_id;
  };
  explicit PurePursuit(const Config &config) : config_(config) {}
  virtual ~PurePursuit() = default;
  Output computeCommands(const ::types::State &robot_state);

private:
  double calcLookAheadDistance_(const double &current_velocity) const;
  double calcAdaptiveLookaheadDistance_(const double &current_velocity) const;
  size_t calcLookAheadPoint_(const ::types::State &robot_state);
  static ::types::Point
  transformToLocalCoordinates_(const ::types::Point &target_point,
                               const ::types::State &robot_state);
  Config config_;
  size_t closest_index_ = 0;
  double current_velocity_ = 0.0;
};

} // namespace control