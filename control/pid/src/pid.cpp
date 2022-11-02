#include "pid/pid.h"

namespace control {
double Pid::calcCommand(const double &value) {
  auto d_err = value - p_err_;
  p_err_ = value;
  i_err_ += value;
  double cmd = (Kp_ * p_err_ + Kd_ * d_err + Ki_ * i_err_);
  return cmd;
}
} // namespace control