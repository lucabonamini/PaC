#pragma once

#include "utilities/math.h"

namespace model {
class RobotModel {
public:
  RobotModel() = default;
  RobotModel(const RobotModel &) = delete;
  RobotModel &operator=(const RobotModel &) = delete;
  RobotModel(RobotModel &&) = delete;
  RobotModel &operator=(RobotModel &&) = delete;
  virtual ~RobotModel() = default;
  virtual void updateState(types::State &state,
                           const types::Controls &controls) = 0;
  virtual double calcTrackError(const types::State &state,
                                const double &ref_x,
                                const double &ref_y) = 0;
};
} // namespace model