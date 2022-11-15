#pragma once

#include "utilities/types.h"

namespace planning {
std::vector<double> computeSpeedProfile(const std::vector<double> &s,
                                        const std::vector<double> &k,
                                        const double &max_longitudinal_velocity,
                                        const double &max_acceleration);
} // namespace planning