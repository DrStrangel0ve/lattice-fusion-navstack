#pragma once

#include "lfn/a_star.hpp"
#include "lfn/controller.hpp"
#include "lfn/ekf.hpp"
#include "lfn/lattice_planner.hpp"
#include "lfn/lidar.hpp"

#include <vector>

namespace lfn {

struct MissionConfig {
  double dt{0.1};
  int max_steps{900};
  int replan_interval{12};
  int gps_interval{5};
  int yaw_interval{3};
  int lidar_interval{10};
  double goal_tolerance{0.55};
  double robot_radius{0.30};
};

struct MissionReport {
  bool success{false};
  int steps{0};
  int replans{0};
  int lidar_scans{0};
  double final_error{0.0};
  double covariance_trace{0.0};
  double global_path_length{0.0};
  double min_lidar_range{0.0};
  Pose2D final_true_pose{};
  Pose2D final_estimated_pose{};
  std::vector<Vec2> global_path{};
  std::vector<Pose2D> true_trace{};
  std::vector<Pose2D> estimated_trace{};
};

GridMap make_demo_map(double robot_radius);

class MissionSimulator {
public:
  explicit MissionSimulator(MissionConfig config = {});

  MissionReport run();
  const GridMap &map() const { return map_; }
  Vec2 start() const { return start_; }
  Vec2 goal() const { return goal_; }

private:
  MissionConfig config_{};
  GridMap map_;
  Vec2 start_{1.0, 1.0};
  Vec2 goal_{18.0, 10.5};
};

} // namespace lfn
