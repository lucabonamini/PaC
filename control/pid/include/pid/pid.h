#pragma once

namespace control {
class Pid {
public:
  Pid(double Kp, double Ki, double Kd) : Kp_(Kp), Ki_(Ki), Kd_(Kd){};
  double calculateValue(const double &value);

private:
  double p_err_ = 0.0;
  double i_err_ = 0.0;
  double Kp_ = 0.0;
  double Ki_ = 0.0;
  double Kd_ = 0.0;
};
} // namespace control