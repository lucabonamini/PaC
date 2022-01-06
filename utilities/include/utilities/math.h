#pragma once

#include "utilities/types.h"
#include <cmath>
#include <cstddef>
#include <vector>

namespace utilities::math {
// TODO(lucabonamini): functions returning void have to be avoided
/**
 * @brief Calculate the closest point's index on a reference path.
 * @param index Variable used to store resulting index
 * @param point Coordinates to evaluate
 * @param path Reference path to use for calculation
 */
static inline void findClosestIndex(int &index,
                                    const types::Point &point,
                                    const types::Path &path);

template <typename T>
std::vector<T> cumSum(const std::vector<T> &input) {
  std::vector<T> output;
  T temp = 0;
  for (size_t i = 0; i < input.size(); i++) {
    temp += input.at(i);
    output.push_back(temp);
  }
  return output;
}

template <typename T>
std::vector<T> vecDiff(const std::vector<T> &input) {
  std::vector<T> output;
  for (size_t i = 1; i < input.size(); i++) {
    output.push_back(input.at(i) - input.at(i - 1));
  }
  return output;
}

template <typename T>
T normalizeAnglePositive(T angle) {
  return fmod(fmod(angle, 2.0 * M_PI) + 2.0 * M_PI, 2.0 * M_PI);
}

template <typename T>
T normalizeAngle(T angle) {
  T a = normalizeAnglePositive(angle);
  if (a > M_PI) {
    a -= 2.0 * M_PI;
  }
  return a;
}

template <typename T>
T shortestAngularDistance(T from, T to) {
  return normalizeAngle(to - from);
}

template <typename T>
T sumOfPower(std::vector<T> value_list) {
  T sum = 0;
  for (T item : value_list) {
    sum += item * item;
  }
  return sum;
}
} // namespace utilities::math
