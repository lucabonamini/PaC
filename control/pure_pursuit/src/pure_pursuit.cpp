#include "pure_pursuit/pure_pursuit.h"

namespace control {
double PurePursuit::calcLookaheadDistance_() const {
  double lookahead_distance = config_.lookahead_time * current_velocity_;
  return lookahead_distance;
}
types::Point PurePursuit::calcTargetPoint_(const types::State &robot_state) {
  utilities::math::findClosestIndex(
      closest_index_, {robot_state.x, robot_state.y}, config_.path);
  double lookahead_distance = calcLookaheadDistance_();
  size_t id =
      round(lookahead_distance / config_.path_resolution) + closest_index_;
  if (id > config_.path.size() - 1) {
    id = config_.path.size() - 1;
  }
  return types::Point{config_.path.at(id).point.x, config_.path.at(id).point.y};
}
types::Point
PurePursuit::transformToLocalCoordinates_(const types::Point &target_point,
                                          const types::State &robot_state) {
  auto t_x = (target_point.x - robot_state.x) * cos(-robot_state.yaw) -
             (target_point.y - robot_state.y) * sin(-robot_state.yaw);
  auto t_y = (target_point.x - robot_state.x) * sin(-robot_state.yaw) +
             (target_point.y - robot_state.y) * cos(-robot_state.yaw);
  return types::Point{t_x, t_y};
}

types::Controls PurePursuit::computeCommands(const types::State &robot_state) {
  // TODO(lucabonamini): better manage controller type choice
  if (config_.type == "Adaptive") {
    current_velocity_ = robot_state.v;
  } else {
    current_velocity_ = config_.target_velocity;
  }
  auto target_point = calcTargetPoint_(robot_state);
  auto target_point_local =
      transformToLocalCoordinates_(target_point, robot_state);
  double dist2 =
      pow((target_point_local.x), 2) + pow((target_point_local.y), 2);
  auto curvature = 2.0 * target_point_local.y / dist2;
  return types::Controls{
      .steer = curvature * current_velocity_, .v = current_velocity_, .a = 0.0};
}

} // namespace control