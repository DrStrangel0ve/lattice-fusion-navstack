#pragma once

#include "lfn/geometry.hpp"
#include "lfn/grid_map.hpp"

#include <vector>

namespace lfn {

struct LidarConfig {
  int beams{181};
  double field_of_view{kPi};
  double min_range{0.05};
  double max_range{12.0};
  double step{0.05};
};

struct LidarScan {
  Pose2D origin{};
  double angle_min{0.0};
  double angle_increment{0.0};
  std::vector<double> ranges{};
  std::vector<Vec2> hits{};
};

class LidarSimulator {
public:
  explicit LidarSimulator(LidarConfig config = {});

  LidarScan scan(const GridMap &map, Pose2D sensor_pose) const;
  double raycast(const GridMap &map, Pose2D sensor_pose, double relative_angle, Vec2 *hit = nullptr) const;

private:
  LidarConfig config_{};
};

} // namespace lfn
