#pragma once

#include "utilities/types.h"

#include <cmath>
#include <limits>
#include <memory>

namespace utilities {
namespace planning {
/**
 * @brief Calculate the closest point's index on a reference path.
 * @param index Variable used to store resulting index
 * @param point Coordinates to evaluate
 * @param path Reference path to use for calculation
 */
void findClosestIndex(size_t &index,
                      const ::types::Point &point,
                      const ::types::Path &path) {
  double closest_distance = std::numeric_limits<double>::max();
  for (size_t i = index; i < path.size(); i++) {
    double g_x = path.at(i).point.x;
    double g_y = path.at(i).point.y;
    double current_closest_distance =
        pow((point.x - g_x), 2) + pow((point.y - g_y), 2);
    if (current_closest_distance < closest_distance) {
      closest_distance = current_closest_distance;
      index = i;
    }
  }
}
} // namespace planning 
} // namespace utilities