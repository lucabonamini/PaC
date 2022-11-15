#pragma once

#include "utilities/types.h"

namespace model {
class RobotModel {
public:
  RobotModel() = default;
  RobotModel(const RobotModel &) = delete;
  RobotModel &operator=(const RobotModel &) = delete;
  RobotModel(RobotModel &&) = delete;
  RobotModel &operator=(RobotModel &&) = delete;
  virtual ~RobotModel() = default;
  virtual void updateState(::types::State &state,
                           const ::types::Controls &controls) = 0;
  virtual double calcTrackError(const ::types::State &state,
                                const ::types::Point &point) = 0;
};
} // namespace model