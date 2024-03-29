#include "dwa/dwa.h"

#include <rapidjson/document.h>
#include <rapidjson/filereadstream.h>
#include <rapidjson/stringbuffer.h>

#include <iostream>

DWA::DWA(const Config &config,
         Obstacles obstacles,
         const ::types::Point &goal,
         const ::types::State &init_state)
    : config_(config)
    , obstacles_(std::move(obstacles))
    , goal_(goal)
    , state_(init_state) {
  unicycle_ = std::make_unique<::model::Unicycle>(config_.dt);
}

Config DWA::parseConfigFile(const std::string &config_file) {
  // auto *fp = fopen(config_file.c_str(), "re");
  char buffer[65536]; // NOLINT
  rapidjson::FileReadStream is(
      fopen(config_file.c_str(), "re"), buffer, sizeof(buffer)); // NOLINT
  rapidjson::Document d;
  d.ParseStream(is);
  return Config{
      .max_lin_vel =
          static_cast<double>(d["max_lin_vel"].GetDouble()), // NOLINT
      .min_lin_vel = static_cast<double>(d["min_lin_vel"].GetDouble()),
      .max_ang_vel = static_cast<double>(d["max_ang_vel"].GetDouble()),
      .max_acc = static_cast<double>(d["max_acc"].GetDouble()),
      .robot_radius = static_cast<double>(d["robot_radius"].GetDouble()),
      .max_delta_ang_vel =
          static_cast<double>(d["max_delta_ang_vel"].GetDouble()),
      .lin_vel_resolution =
          static_cast<double>(d["lin_vel_resolution"].GetDouble()),
      .ang_vel_resolution =
          static_cast<double>(d["ang_vel_resolution"].GetDouble()),
      .dt = static_cast<double>(d["dt"].GetDouble()),
      .prediction_time = static_cast<double>(d["prediction_time"].GetDouble()),
      .to_goal_cost_gain =
          static_cast<double>(d["to_goal_cost_gain"].GetDouble()),
      .lin_vel_cost_gain =
          static_cast<double>(d["lin_vel_cost_gain"].GetDouble())};
}

DynamicWindow DWA::calcDynamicWindow() const {
  return DynamicWindow{
      .min_lin_vel_limit = std::max(
          (state_.v - config_.max_acc * 1.0 / config_.dt), config_.min_lin_vel),
      .max_lin_vel_limit = std::min(
          (state_.v + config_.max_acc * 1.0 / config_.dt), config_.max_lin_vel),
      .min_ang_vel_limit =
          std::max((state_.w - config_.max_delta_ang_vel * 1.0 / config_.dt),
                   -config_.max_ang_vel),
      .max_ang_vel_limit =
          std::min((state_.w + config_.max_delta_ang_vel * 1.0 / config_.dt),
                   config_.max_ang_vel)};
}

::types::Traj DWA::calcTrajectory(const double &lin_vel,
                                  const double &ang_vel,
                                  const ::types::State &state) const {
  ::types::Traj traj;
  double time = 0.0;
  auto tmp = state;
  traj.push_back(tmp);
  while (time <= config_.prediction_time) {
    unicycle_->updateState(
        tmp, ::types::Controls{.steer = ang_vel, .v = lin_vel, .a = 0.0});
    traj.push_back(::types::State{
        .x = tmp.x, .y = tmp.y, .yaw = tmp.yaw, .v = tmp.v, .w = tmp.w});
    time += 1.0 / config_.dt;
  }
  return traj;
}

double DWA::calcToGoalCost(const ::types::Traj &trajectory) const {
  double goal_magnitude = sqrt(pow(goal_.x, 2) + pow(goal_.y, 2));
  double traj_magnitude =
      sqrt(pow(trajectory.back().x, 2) + pow(trajectory.back().y, 2));
  double dot_product =
      (goal_.x * trajectory.back().x) + (goal_.y * trajectory.back().y);
  double error = dot_product / (goal_magnitude * traj_magnitude);
  double error_angle = std::acos(error);
  double cost = config_.to_goal_cost_gain * error_angle;
  return cost;
}

double DWA::calcSpeedCost(const double &last_lin_vel) const {
  return config_.lin_vel_cost_gain * (config_.max_lin_vel - last_lin_vel);
}

double DWA::calcObstacleCost(const ::types::Traj &trajectory) const {
  double min = std::numeric_limits<double>::max();
  for (size_t i = 0; i < trajectory.size(); i += 2) {
    for (const auto &obstacle : obstacles_) {
      double dx = trajectory.at(i).x - obstacle.x;
      double dy = trajectory.at(i).y - obstacle.y;
      double r = sqrt(pow(dx, 2) + pow(dy, 2));
      if (r <= config_.robot_radius) {
        return std::numeric_limits<double>::max();
      }
      if (min >= r) {
        min = r;
      }
    }
  }
  return 1.0 / min;
}

double DWA::calcTrajectoryCost(const ::types::Traj &trajectory) const {
  double to_goal_cost = calcToGoalCost(trajectory);
  auto speed_cost = calcSpeedCost(trajectory.back().v);
  auto obstacle_cost = calcObstacleCost(trajectory);
  auto total_cost = to_goal_cost + speed_cost + obstacle_cost;
  return total_cost;
}

::types::Controls DWA::calcBestControls(const DynamicWindow &dw) {
  ::types::Traj best_trajectory;
  ::types::Controls best_controls;
  auto tmp_state = state_;
  double min_cost = std::numeric_limits<double>::max();
  for (double v = dw.min_lin_vel_limit; v <= dw.max_lin_vel_limit;   // NOLINT
       v += config_.lin_vel_resolution) {                            // NOLINT
    for (double w = dw.min_ang_vel_limit; w <= dw.max_ang_vel_limit; // NOLINT
         w += config_.ang_vel_resolution) {                          // NOLINT
      auto traj = calcTrajectory(v, w, tmp_state);
      auto total_cost = calcTrajectoryCost(traj);
      if (min_cost >= total_cost) {
        min_cost = total_cost;
        best_controls = ::types::Controls{.steer = w, .v = v, .a = 0.0};
        best_trajectory = traj;
      }
    }
  }
  viz_traj = best_trajectory;
  return best_controls;
}

bool DWA::isGoalReached() const {
  return sqrt(pow((state_.x - goal_.x), 2) + pow((state_.y - goal_.y), 2)) <=
         config_.robot_radius;
}

bool DWA::dwaControls() {
  auto dynamic_window = calcDynamicWindow();
  auto controls = calcBestControls(dynamic_window);
  unicycle_->updateState(state_, controls);
  return isGoalReached();
}