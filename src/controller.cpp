#include "lfn/controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace lfn {

PurePursuitController::PurePursuitController(PurePursuitConfig config) : config_(config) {}

Twist2D PurePursuitController::command(Pose2D pose, const std::vector<Vec2> &path) const {
  if (path.empty()) {
    return {};
  }

  const Vec2 goal = path.back();
  const double goal_distance = distance(pose.position, goal);
  if (goal_distance <= config_.goal_tolerance) {
    return {};
  }

  std::size_t closest_index = 0;
  double closest_distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < path.size(); ++i) {
    const double d = distance(pose.position, path[i]);
    if (d < closest_distance) {
      closest_distance = d;
      closest_index = i;
    }
  }

  Vec2 target = goal;
  for (std::size_t i = closest_index; i < path.size(); ++i) {
    if (distance(pose.position, path[i]) >= config_.lookahead_distance) {
      target = path[i];
      break;
    }
  }

  const double alpha = normalize_angle(heading_to(pose.position, target) - pose.yaw);
  const double lookahead = std::max(config_.lookahead_distance, distance(pose.position, target));
  const double curvature = 2.0 * std::sin(alpha) / lookahead;
  const double slow_scale = std::clamp(goal_distance / config_.slow_radius, 0.2, 1.0);
  const double heading_scale = std::clamp(1.0 - std::abs(alpha) / kPi, 0.25, 1.0);
  const double linear = config_.max_linear_speed * slow_scale * heading_scale;
  const double angular = clamp(linear * curvature, -config_.max_angular_speed, config_.max_angular_speed);
  return {linear, angular};
}

} // namespace lfn
