#include "pure_pursuit/pure_pursuit.h"

namespace control {
double
PurePursuit::calcLookAheadDistance_(const double &current_velocity) const {
  double lookahead_distance = config_.lookahead_time * current_velocity;
  return lookahead_distance;
}
size_t PurePursuit::calcLookAheadPoint_(const ::types::State &robot_state) {
  double lookahead_distance = calcLookAheadDistance_(robot_state.v);
  for (size_t i = closest_index_; i < config_.path.size(); i++) {
    double g_x = config_.path.at(i).x;
    double g_y = config_.path.at(i).y;
    double current_closest_distance =
        pow((robot_state.x - g_x), 2) + pow((robot_state.y - g_y), 2);
    if (current_closest_distance > lookahead_distance) {
      closest_index_ = i;
      break;
    }
  }
  if (closest_index_ > config_.path.size() - 1) {
    closest_index_ = config_.path.size() - 1;
  }
  return closest_index_;
}
::types::Point
PurePursuit::transformToLocalCoordinates_(const ::types::Point &target_point,
                                          const ::types::State &robot_state) {
  auto t_x = (target_point.x - robot_state.x) * cos(-robot_state.yaw) -
             (target_point.y - robot_state.y) * sin(-robot_state.yaw);
  auto t_y = (target_point.x - robot_state.x) * sin(-robot_state.yaw) +
             (target_point.y - robot_state.y) * cos(-robot_state.yaw);
  return ::types::Point{t_x, t_y};
}
PurePursuit::Output
PurePursuit::computeCommands(const ::types::State &robot_state) {
  auto id = calcLookAheadPoint_(robot_state);
  double linear_velocity = config_.path.at(id).v;
  auto target_point_local = transformToLocalCoordinates_(
      ::types::Point{config_.path.at(id).x, config_.path.at(id).y},
      robot_state);
  double dist2 =
      pow((target_point_local.x), 2) + pow((target_point_local.y), 2);
  auto curvature = 2.0 * target_point_local.y / dist2;
  Output output;
  output.controls = ::types::Controls{
      .steer = curvature * linear_velocity, .v = linear_velocity, .a = 0.0};
  output.target_point_id = id;
  return output;
}

} // namespace control