#include "lfn/lattice_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace lfn {
namespace {

std::vector<double> sample_range(double low, double high, int samples) {
  std::vector<double> values;
  if (samples <= 1 || std::abs(high - low) < 1.0e-9) {
    values.push_back(0.5 * (low + high));
    return values;
  }
  values.reserve(static_cast<std::size_t>(samples));
  for (int i = 0; i < samples; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(samples - 1);
    values.push_back(low + t * (high - low));
  }
  return values;
}

void add_unique(std::vector<double> &values, double value) {
  const auto exists = std::any_of(values.begin(), values.end(), [value](double candidate) {
    return std::abs(candidate - value) < 1.0e-6;
  });
  if (!exists) {
    values.push_back(value);
  }
}

double distance_to_path(Vec2 point, const std::vector<Vec2> &path) {
  double best = std::numeric_limits<double>::infinity();
  for (const Vec2 waypoint : path) {
    best = std::min(best, distance(point, waypoint));
  }
  return std::isfinite(best) ? best : 0.0;
}

} // namespace

LatticePlanner::LatticePlanner(LatticePlannerConfig config) : config_(config) {}

LocalPlan LatticePlanner::plan(const GridMap &map, Pose2D pose, Twist2D current_command,
                               const std::vector<Vec2> &global_path, Twist2D reference_command) const {
  LocalPlan result;
  const Vec2 goal = global_path.empty() ? pose.position : global_path.back();
  const double start_goal_distance = distance(pose.position, goal);

  const double linear_low =
      std::max(config_.min_linear_speed, current_command.linear - config_.max_linear_delta);
  const double linear_high =
      std::min(config_.max_linear_speed, current_command.linear + config_.max_linear_delta);
  const double angular_low = std::max(-config_.max_angular_speed, current_command.angular - config_.max_angular_delta);
  const double angular_high = std::min(config_.max_angular_speed, current_command.angular + config_.max_angular_delta);

  std::vector<double> linear_samples = sample_range(linear_low, linear_high, config_.linear_samples);
  std::vector<double> angular_samples = sample_range(angular_low, angular_high, config_.angular_samples);
  add_unique(linear_samples, std::clamp(reference_command.linear, config_.min_linear_speed, config_.max_linear_speed));
  add_unique(linear_samples, 0.0);
  add_unique(angular_samples, std::clamp(reference_command.angular, -config_.max_angular_speed, config_.max_angular_speed));
  add_unique(angular_samples, 0.0);

  double best_score = std::numeric_limits<double>::infinity();
  for (const double linear : linear_samples) {
    for (const double angular : angular_samples) {
      Trajectory trajectory;
      trajectory.command = {linear, angular};
      trajectory.score = std::numeric_limits<double>::infinity();
      trajectory.collision_free = true;
      trajectory.min_clearance = std::numeric_limits<double>::infinity();

      Pose2D rollout_pose = pose;
      for (double time = config_.dt; time <= config_.horizon + 1.0e-9; time += config_.dt) {
        rollout_pose = integrate_unicycle(rollout_pose, trajectory.command, config_.dt);
        const double clearance = map.distance_to_nearest_obstacle(rollout_pose.position, 4.0);
        trajectory.min_clearance = std::min(trajectory.min_clearance, clearance);
        trajectory.points.push_back({rollout_pose, trajectory.command, time});
        if (map.collides(rollout_pose, config_.robot_radius, config_.collision_threshold)) {
          trajectory.collision_free = false;
          break;
        }
      }

      if (trajectory.collision_free && !trajectory.points.empty()) {
        const Vec2 end = trajectory.points.back().pose.position;
        const double end_goal_distance = distance(end, goal);
        const double progress = start_goal_distance - end_goal_distance;
        const double path_error = distance_to_path(end, global_path);
        const double command_delta =
            std::abs(linear - current_command.linear) + 0.35 * std::abs(angular - current_command.angular);
        const double reference_delta =
            std::abs(linear - reference_command.linear) + 0.45 * std::abs(angular - reference_command.angular);
        const double clearance_reward = std::min(trajectory.min_clearance, 3.0);

        trajectory.score = config_.goal_weight * end_goal_distance + config_.path_weight * path_error -
                           config_.progress_weight * progress - config_.clearance_weight * clearance_reward +
                           config_.smoothness_weight * command_delta + config_.reference_weight * reference_delta;
      }

      if (trajectory.collision_free && trajectory.score < best_score) {
        best_score = trajectory.score;
        result.success = true;
        result.command = trajectory.command;
        result.best = trajectory;
      }
      result.candidates.push_back(std::move(trajectory));
    }
  }

  if (!result.success) {
    result.command = {};
  }
  return result;
}

} // namespace lfn
