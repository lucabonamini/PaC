#pragma once

#include "model.h"

namespace model {
class DynamicUnicycle : public RobotModel {
public:
  DynamicUnicycle() = default;
  explicit DynamicUnicycle(const double &frequency);
  void updateState(types::State &state,
                   const types::Controls &controls) override;
  double calcTrackError(const types::State &state,
                        const ::types::Point &point) override;

private:
  double frequency_ = 0.0;
};
} // namespace model