#include "lfn/mission.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace lfn {
namespace {

Vec2 deterministic_position_noise(int step) {
  return {0.05 * std::sin(0.37 * static_cast<double>(step)),
          -0.04 * std::cos(0.23 * static_cast<double>(step))};
}

double deterministic_yaw_noise(int step) { return 0.025 * std::sin(0.19 * static_cast<double>(step)); }

double deterministic_velocity_noise(int step) { return 0.035 * std::cos(0.31 * static_cast<double>(step)); }

} // namespace

GridMap make_demo_map(double robot_radius) {
  GridMap map(40, 24, 0.5, {0.0, 0.0});

  map.add_rect({0.0, 0.0}, {20.0, 0.5});
  map.add_rect({0.0, 11.5}, {20.0, 12.0});
  map.add_rect({0.0, 0.0}, {0.5, 12.0});
  map.add_rect({19.5, 0.0}, {20.0, 12.0});

  map.add_rect({4.0, 0.5}, {4.5, 7.2});
  map.add_rect({8.0, 4.0}, {8.5, 11.5});
  map.add_rect({12.0, 0.5}, {12.5, 7.2});
  map.add_rect({14.5, 8.5}, {16.5, 9.0});
  map.add_rect({15.0, 2.2}, {16.0, 2.7});

  map.inflate_obstacles(robot_radius + 0.20, robot_radius * 0.55);
  return map;
}

MissionSimulator::MissionSimulator(MissionConfig config) : config_(config), map_(make_demo_map(config.robot_radius)) {}

MissionReport MissionSimulator::run() {
  MissionReport report;

  AStarPlanner global_planner({true, 1.05, 2.2, 225, config_.robot_radius});
  PurePursuitController controller({1.3, 1.25, 1.3, config_.goal_tolerance, 2.2});
  LatticePlanner local_planner({2.0,
                                0.1,
                                config_.robot_radius,
                                0.0,
                                1.35,
                                1.35,
                                0.45,
                                0.75,
                                7,
                                9,
                                6.5,
                                1.6,
                                3.5,
                                0.6,
                                0.2,
                                1.7,
                                225});
  LidarSimulator lidar({91, kPi, 0.05, 9.0, 0.05});

  Pose2D true_pose{start_, heading_to(start_, goal_)};
  Ekf2D ekf;
  ekf.set_state({start_ + Vec2{0.08, -0.05}, true_pose.yaw + 0.03}, 0.0);
  ekf.set_covariance_diagonal(0.25, 0.25, 0.08, 0.3);

  Twist2D current_command{};
  std::vector<Vec2> active_path;
  double initial_path_length = 0.0;
  double min_lidar_range = std::numeric_limits<double>::infinity();
  bool collision = false;

  for (int step = 0; step < config_.max_steps; ++step) {
    const bool should_replan = active_path.empty() || step % config_.replan_interval == 0;
    if (should_replan) {
      const PlanResult plan = global_planner.plan(map_, ekf.pose().position, goal_);
      if (plan.success) {
        active_path = plan.path;
        ++report.replans;
        if (initial_path_length == 0.0) {
          initial_path_length = path_length(active_path);
        }
      }
    }

    const Twist2D reference = controller.command(ekf.pose(), active_path);
    const LocalPlan local = local_planner.plan(map_, ekf.pose(), current_command, active_path, reference);
    Twist2D command = local.success ? local.command : reference;
    command.linear = clamp(command.linear, 0.0, 1.35);
    command.angular = clamp(command.angular, -1.35, 1.35);

    const Twist2D realized{command.linear * 0.985, command.angular + 0.006};
    true_pose = integrate_unicycle(true_pose, realized, config_.dt);
    if (map_.collides(true_pose, config_.robot_radius, 225)) {
      collision = true;
      report.steps = step + 1;
      break;
    }

    ekf.predict(command, config_.dt);
    ekf.update_velocity(realized.linear + deterministic_velocity_noise(step), 0.10);
    if (step % config_.gps_interval == 0) {
      ekf.update_position(true_pose.position + deterministic_position_noise(step), 0.14);
    }
    if (step % config_.yaw_interval == 0) {
      ekf.update_yaw(true_pose.yaw + deterministic_yaw_noise(step), 0.06);
    }
    if (step % config_.lidar_interval == 0) {
      const LidarScan scan = lidar.scan(map_, true_pose);
      ++report.lidar_scans;
      for (const double range : scan.ranges) {
        min_lidar_range = std::min(min_lidar_range, range);
      }
    }

    current_command = command;
    report.true_trace.push_back(true_pose);
    report.estimated_trace.push_back(ekf.pose());
    report.steps = step + 1;
    if (distance(true_pose.position, goal_) <= config_.goal_tolerance) {
      report.success = true;
      break;
    }
  }

  report.success = report.success && !collision;
  report.final_true_pose = true_pose;
  report.final_estimated_pose = ekf.pose();
  report.final_error = distance(true_pose.position, goal_);
  report.covariance_trace = ekf.covariance_trace();
  report.global_path_length = initial_path_length;
  report.min_lidar_range = std::isfinite(min_lidar_range) ? min_lidar_range : 0.0;
  report.global_path = active_path;
  return report;
}

} // namespace lfn
