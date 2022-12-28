#pragma once

#include "dwa/types.h"
#include "models/unicycle.h"
#include <memory>
#include <string>

class DWA {
public:
  DWA(const Config &config,
      Obstacles obstacles,
      const ::types::Point &goal,
      const ::types::State &init_state);
  ~DWA(){};
  static Config parseConfigFile(const std::string &config_file);
  bool dwaControls();
  std::unique_ptr<::model::Unicycle> unicycle_;
  ::types::Traj viz_traj;

private:
  DynamicWindow calcDynamicWindow() const;
  ::types::Controls calcBestControls(const DynamicWindow &dw);
  ::types::Traj calcTrajectory(const double &lin_vel,
                               const double &ang_vel,
                               const ::types::State &state) const;
  double calcTrajectoryCost(const ::types::Traj &trajectory) const;
  double calcToGoalCost(const ::types::Traj &trajectory) const;
  double calcSpeedCost(const double &last_lin_vel) const;
  double calcObstacleCost(const ::types::Traj &trajectory) const;
  bool isGoalReached() const;
  Config config_;
  Obstacles obstacles_;
  ::types::Point goal_;
  ::types::State state_;
};