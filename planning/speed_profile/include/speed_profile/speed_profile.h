#pragma once

#include "utilities/types.h"

namespace planning {
std::vector<double> computeSpeedProfile(const std::vector<double> &s,
    const std::vector<double> &k);
} // namespace planning