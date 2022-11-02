#include "speed_profile/speed_profile.h"

#include <cmath>

namespace planning {
::types::Traj computeNominalProfile(const ::types::Path &path,
                                  double start_speed,
                                  double desired_speed) {
  ::types::Traj profile;
  auto accel_distance = 0.0;
  auto a_max = 0.1;
  if (desired_speed < start_speed) {
    accel_distance = calcDistance(start_speed, desired_speed, -a_max);
  } else {
    accel_distance = calcDistance(start_speed, desired_speed, a_max);
  }

  auto ramp_end_index = 0;
  auto distance = 0.0;
  while (ramp_end_index < path.size() - 1 && distance < accel_distance) {
    distance += sqrt(pow((path.at(ramp_end_index + 1).point.x -
                          path.at(ramp_end_index).point.x),
                         2) +
                     pow((path.at(ramp_end_index + 1).point.y -
                          path.at(ramp_end_index).point.y),
                         2));
    ramp_end_index++;
  }

  auto vi = start_speed;
  for (int i = 0; i < ramp_end_index; i++) {
    auto dist = sqrt(pow((path.at(i + 1).point.x - path.at(i).point.x), 2) +
                     pow((path.at(i + 1).point.y - path.at(i).point.y), 2));
    auto vf = 0.0;
    if (desired_speed < start_speed) {
      vf = calcFinalSpeed(vi, -a_max, dist);
      if (vf < desired_speed) {
        vf = desired_speed;
      }
    } else {
      vf = calcFinalSpeed(vi, a_max, dist);
      if (vf > desired_speed) {
        vf = desired_speed;
      }
    }
    profile.push_back(::types::State{.x = path.at(i).point.x,
                                     .y = path.at(i).point.y,
                                     .yaw = 0.0,
                                     .v = vi,
                                     .w = 0.0});
    vi = vf;
  }

  for (int i = ramp_end_index + 1; i < path.size(); i++) {
    profile.push_back(::types::State{.x = path.at(i).point.x,
                                     .y = path.at(i).point.y,
                                     .yaw = 0.0,
                                     .v = desired_speed,
                                     .w = 0.0});
  }

  return profile;
}
::types::Traj computeDecelerateProfile(const ::types::Traj &traj, double start_speed) {
  ::types::Traj profile;
  auto slow_speed = 0.5;
  auto stop_line_buffer = 1.0;
  auto a_max = 0.5;

  auto decel_distance = calcDistance(start_speed, slow_speed, -a_max);
  auto brake_distance = calcDistance(slow_speed, 0.0, -a_max);

  auto path_legth = 0.0;
  for (size_t i = 0; i < traj.size()-1; i++) {
    path_legth += sqrt(pow((traj.at(i + 1).x -
                      traj.at(i).x),
                      2) +
                  pow((traj.at(i + 1).y -
                      traj.at(i).y),
                      2));
  }
  auto stop_index = traj.size()-1;
  auto temp_dist = 0.0;
  while (stop_index > 0 && temp_dist < stop_line_buffer) {
    temp_dist += sqrt(pow((traj.at(stop_index).x -
                  traj.at(stop_index-1).x),
                  2) +
              pow((traj.at(stop_index)).y -
                  traj.at(stop_index-1).y,
                  2));
    stop_index--;
  }
  std::vector<double> speeds;
  if ((brake_distance+decel_distance+stop_line_buffer) > path_legth) {
    auto vf = 0.0;
    auto vi = 0.0;
    for (size_t i = traj.size(); i >= stop_index; i--) {
      auto dist = sqrt(pow((traj.at(i+1).x -
              traj.at(i).x),
              2) +
          pow((traj.at(i+1)).y -
              traj.at(i).y,
              2));
      vi = calcFinalSpeed(vf, -a_max, dist);
      if (vi > start_speed) {
        vi = start_speed;
      }
      speeds.push_back(vi);
      vf = vi;
    }
    for (size_t i = speeds.size(); i >= 0 ; i--) {
      profile.at(i).v = speeds.at(i);
    }
  }
  
}
double calcDistance(double start_speed, double desired_speed, double a_max) {
  return (pow(desired_speed, 2) - pow(start_speed, 2)) / (2 * a_max);
}
double calcFinalSpeed(double vi, double a_max, double dist) {
  return sqrt(pow(vi, 2) + 2 * a_max * dist);
}
} // namespace planning