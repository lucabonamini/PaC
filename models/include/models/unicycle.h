#pragma once

#include "model.h"

namespace model {
class Unicycle : public RobotModel {
public:
  Unicycle() = default;
  explicit Unicycle(const double &frequency);
  void updateState(types::State &state,
                   const types::Controls &controls) override;
  double calcTrackError(const types::State &state,
                        const ::types::Point &point) override;

private:
  double frequency_ = 0.0;
};
} // namespace model