#include "lfn/lidar.hpp"

#include <algorithm>
#include <cmath>

namespace lfn {

LidarSimulator::LidarSimulator(LidarConfig config) : config_(config) {}

double LidarSimulator::raycast(const GridMap &map, Pose2D sensor_pose, double relative_angle, Vec2 *hit) const {
  const double angle = normalize_angle(sensor_pose.yaw + relative_angle);
  const Vec2 direction{std::cos(angle), std::sin(angle)};
  const double step = std::max(config_.step, map.resolution() * 0.25);

  for (double range = config_.min_range; range <= config_.max_range; range += step) {
    const Vec2 point = sensor_pose.position + direction * range;
    const auto cell = map.world_to_cell(point);
    if (!cell) {
      if (hit != nullptr) {
        *hit = point;
      }
      return range;
    }
    if (map.is_occupied(*cell)) {
      if (hit != nullptr) {
        *hit = point;
      }
      return range;
    }
  }

  if (hit != nullptr) {
    *hit = sensor_pose.position + direction * config_.max_range;
  }
  return config_.max_range;
}

LidarScan LidarSimulator::scan(const GridMap &map, Pose2D sensor_pose) const {
  LidarScan scan;
  scan.origin = sensor_pose;
  scan.angle_min = -0.5 * config_.field_of_view;
  scan.angle_increment = config_.beams > 1 ? config_.field_of_view / static_cast<double>(config_.beams - 1) : 0.0;
  scan.ranges.reserve(static_cast<std::size_t>(std::max(config_.beams, 0)));
  scan.hits.reserve(static_cast<std::size_t>(std::max(config_.beams, 0)));

  for (int i = 0; i < config_.beams; ++i) {
    Vec2 hit;
    const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
    scan.ranges.push_back(raycast(map, sensor_pose, angle, &hit));
    scan.hits.push_back(hit);
  }
  return scan;
}

} // namespace lfn
