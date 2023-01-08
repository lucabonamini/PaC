#pragma once

#include "types.h"
#include <optional>

constexpr double kj = 0.1;
constexpr double kt = 0.1;
constexpr double kd = 0.1;
constexpr double ks = 0.1;
constexpr double klat = 1.0;
constexpr double klon = 1.0;
constexpr double d_vec[9] = {-4.0, -3.0, -2.0, -1.0, 0.0, 1.0, 2.0, 3.0, 4.0};
constexpr double t_vec[2] = {4.0, 5.0};

namespace Frenet {
std::optional<FrenetPath> planFrenetPath(const Input &input);
std::vector<FrenetPath> calculatePathsInFrenetCoordinates(const Input &input);
void convertPathsToCartesianCoordinates(std::vector<FrenetPath> &paths_list,
                                        const Input &input);
std::optional<FrenetPath> findBestPath(std::vector<FrenetPath> &paths_list);
bool isPathInCollision(const FrenetPath &path, const Input &input);
void calculatePathCost(FrenetPath &path, const Input &input);
} // namespace Frenet