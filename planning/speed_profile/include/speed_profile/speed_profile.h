#pragma once

#include "utilities/types.h"

namespace planning {
::types::Traj computeNominalProfile(const ::types::Path &path,
                                  double start_speed,
                                  double desired_speed);
::types::Traj computeDecelerateProfile(const ::types::Path &path, double start_speed);
double calcDistance(double start_speed, double desired_speed, double a_max);
double calcFinalSpeed(double vi, double a_max, double dist);
} // namespace planning