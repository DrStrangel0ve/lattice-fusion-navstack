#pragma once

#include "lfn/geometry.hpp"
#include "lfn/grid_map.hpp"

#include <cstdint>
#include <vector>

namespace lfn {

struct LatticePlannerConfig {
  double horizon{2.2};
  double dt{0.15};
  double robot_radius{0.45};
  double min_linear_speed{0.0};
  double max_linear_speed{2.2};
  double max_angular_speed{1.4};
  double max_linear_delta{0.8};
  double max_angular_delta{1.2};
  int linear_samples{7};
  int angular_samples{9};
  double progress_weight{5.0};
  double goal_weight{2.0};
  double path_weight{3.5};
  double clearance_weight{0.8};
  double smoothness_weight{0.4};
  double reference_weight{1.2};
  std::uint8_t collision_threshold{220};
};

struct Trajectory {
  std::vector<TrajectoryPoint> points{};
  Twist2D command{};
  double score{0.0};
  double min_clearance{0.0};
  bool collision_free{false};
};

struct LocalPlan {
  bool success{false};
  Twist2D command{};
  Trajectory best{};
  std::vector<Trajectory> candidates{};
};

class LatticePlanner {
public:
  explicit LatticePlanner(LatticePlannerConfig config = {});

  LocalPlan plan(const GridMap &map, Pose2D pose, Twist2D current_command,
                 const std::vector<Vec2> &global_path, Twist2D reference_command = {}) const;

private:
  LatticePlannerConfig config_{};
};

} // namespace lfn
