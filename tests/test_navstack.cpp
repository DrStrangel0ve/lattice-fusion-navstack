#include "lfn/a_star.hpp"
#include "lfn/controller.hpp"
#include "lfn/ekf.hpp"
#include "lfn/lattice_planner.hpp"
#include "lfn/lidar.hpp"
#include "lfn/mission.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {

bool near(double lhs, double rhs, double eps) { return std::abs(lhs - rhs) <= eps; }

void require(bool condition, const char *message) {
  if (!condition) {
    std::cerr << "test failure: " << message << '\n';
    std::exit(1);
  }
}

void test_grid_inflation() {
  lfn::GridMap map(20, 20, 0.5, {0.0, 0.0});
  map.set_occupied({10, 10}, true);
  map.inflate_obstacles(1.0, 0.25);

  require(map.is_occupied({10, 10}), "occupied source cell remains lethal");
  require(map.cost({11, 10}) > 0, "neighbor cell receives inflation cost");
  require(map.collides({map.cell_center({10, 10}), 0.0}, 0.25), "footprint collides with obstacle");
  require(!map.collides({map.cell_center({2, 2}), 0.0}, 0.25), "free-space footprint is collision-free");
}

void test_a_star_uses_gap() {
  lfn::GridMap map(20, 12, 1.0, {0.0, 0.0});
  for (int y = 0; y < map.height(); ++y) {
    if (y != 6) {
      map.set_occupied({10, y}, true);
    }
  }
  map.inflate_obstacles(0.0);

  const lfn::PlanResult plan = lfn::AStarPlanner().plan(map, {1.5, 1.5}, {18.5, 10.5});
  require(plan.success, "A* finds a route through the gap");
  require(!plan.path.empty(), "A* route contains waypoints");
  bool used_gap = false;
  for (const lfn::Cell cell : plan.cells) {
    used_gap = used_gap || (cell == lfn::Cell{10, 6});
  }
  require(used_gap, "A* route uses the only wall gap");
}

void test_ekf_converges_with_measurements() {
  lfn::Ekf2D ekf;
  ekf.set_state({{0.0, 0.0}, 0.0}, 0.0);
  ekf.set_covariance_diagonal(1.0, 1.0, 0.5, 1.0);

  ekf.predict({1.0, 0.0}, 1.0);
  ekf.update_position({1.0, 0.0}, 0.1);
  ekf.update_yaw(0.0, 0.05);
  ekf.update_velocity(1.0, 0.1);

  require(near(ekf.pose().position.x, 1.0, 0.15), "EKF x estimate converges");
  require(near(ekf.pose().position.y, 0.0, 0.08), "EKF y estimate converges");
  require(near(ekf.pose().yaw, 0.0, 0.05), "EKF yaw estimate converges");
  require(near(ekf.velocity(), 1.0, 0.08), "EKF velocity estimate converges");
  require(ekf.covariance_trace() < 1.0, "EKF covariance contracts after corrections");
}

void test_lidar_raycast() {
  lfn::GridMap map(20, 20, 0.5, {-5.0, -5.0});
  map.add_rect({2.0, -0.25}, {2.5, 0.25});
  map.inflate_obstacles(0.0);

  const lfn::LidarSimulator lidar({1, 0.0, 0.05, 5.0, 0.02});
  lfn::Vec2 hit;
  const double range = lidar.raycast(map, {{0.0, 0.0}, 0.0}, 0.0, &hit);
  require(range > 1.9 && range < 2.1, "lidar raycast reports expected obstacle range");
  require(hit.x > 1.9, "lidar hit point lies on the forward obstacle");
}

void test_lattice_planner_selects_forward_motion() {
  lfn::GridMap map(30, 20, 0.5, {0.0, 0.0});
  map.inflate_obstacles(0.0);
  const std::vector<lfn::Vec2> path{{1.0, 1.0}, {4.0, 1.0}, {8.0, 1.0}, {12.0, 1.0}};

  lfn::LatticePlanner planner({1.5, 0.1, 0.3, 0.0, 1.0, 1.0, 0.6, 0.8, 5, 7});
  const lfn::LocalPlan plan = planner.plan(map, {{1.0, 1.0}, 0.0}, {}, path, {0.8, 0.0});
  require(plan.success, "lattice planner finds a collision-free rollout");
  require(plan.command.linear > 0.1, "lattice planner selects forward motion");
  require(std::abs(plan.command.angular) < 0.5, "lattice planner stays aligned to a straight path");
}

void test_mission_simulation_reaches_goal() {
  lfn::MissionSimulator simulator;
  const lfn::MissionReport report = simulator.run();
  if (!report.success) {
    std::cerr << "mission diagnostics: final_error=" << report.final_error
              << " collision=" << std::boolalpha << report.collision
              << " timed_out=" << report.timed_out << " steps=" << report.steps
              << " replans=" << report.replans << " lidar_scans=" << report.lidar_scans << '\n';
  }
  require(report.success, "mission simulator reaches the goal");
  require(report.final_error < 0.65, "mission final error stays within tolerance");
  require(report.replans > 0, "mission performs at least one global replan");
  require(report.lidar_scans > 0, "mission produces lidar scans");
  require(report.covariance_trace < 0.5, "mission estimator remains bounded");
}

} // namespace

int main() {
  test_grid_inflation();
  test_a_star_uses_gap();
  test_ekf_converges_with_measurements();
  test_lidar_raycast();
  test_lattice_planner_selects_forward_motion();
  test_mission_simulation_reaches_goal();

  std::cout << "all lattice-fusion-navstack tests passed\n";
  return 0;
}
