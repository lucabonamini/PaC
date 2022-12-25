#include "pid/pid.h"

namespace control {
double Pid::calculateValue(const double &value) {
  auto d_err = value - p_err_;
  p_err_ = value;
  i_err_ += value;
  return (Kp_ * p_err_ + Kd_ * d_err + Ki_ * i_err_);
}
} // namespace control