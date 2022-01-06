#pragma once

#include "model.h"

namespace types {
struct Controls;
}
namespace types {
struct State;
}

namespace model {
class Unicycle : public RobotModel {
public:
  Unicycle() = default;
  explicit Unicycle(const double &frequency);
  void updateState(types::State &state,
                   const types::Controls &controls) override;
  double calcTrackError(const types::State &state,
                        const double &ref_x,
                        const double &ref_y) override;

private:
  double frequency_ = 0.0;
};
} // namespace model