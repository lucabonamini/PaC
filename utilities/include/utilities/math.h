#pragma once

#include <cmath>
#include <numeric>
#include <vector>

constexpr double pi_2 = 2 * M_PI;

namespace utilities {
namespace math {
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
  return fmod(fmod(angle, pi_2) + pi_2, pi_2);
}

template <typename T>
T normalizeAngle(T angle) {
  T a = normalizeAnglePositive(angle);
  if (a > M_PI) {
    a -= pi_2;
  }
  return a;
}

template <typename T>
T shortestAngularDistance(T from, T to) {
  return normalizeAngle(to - from);
}

template <typename T>
T sumOfPower(std::vector<T> value_list) {
  auto sum_of_power = std::accumulate(
      value_list.begin(), value_list.end(), 0.0, [](auto product, auto x) {
        return product + std::pow(x, 2);
      });
  return sum_of_power;
}
} // namespace math
} // namespace utilities
