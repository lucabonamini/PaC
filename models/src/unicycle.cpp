#include "models/unicycle.h"
#include "utilities/math.h"

namespace model {
Unicycle::Unicycle(const double &frequency) : frequency_(frequency) {}
void Unicycle::updateState(types::State &state,
                           const types::Controls &controls) {
  state.v = controls.v;
  state.w = controls.steer;
  state.yaw += controls.steer * 1.0 / frequency_;
  utilities::math::normalizeAngle(state.yaw);
  state.x += state.v * cos(state.yaw) * 1.0 / frequency_;
  state.y += state.v * sin(state.yaw) * 1.0 / frequency_;
}
double Unicycle::calcTrackError(const types::State &state,
                                const ::types::Point &point) {
  double error =
      sqrt(pow((state.x - point.x), 2) + pow((state.y - point.y), 2));
  return error;
}
} //  namespace model