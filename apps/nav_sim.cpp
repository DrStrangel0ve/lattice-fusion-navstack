#include "lfn/mission.hpp"

#include <iomanip>
#include <iostream>

int main() {
  lfn::MissionSimulator simulator;
  const lfn::MissionReport report = simulator.run();

  std::cout << "lattice-fusion-navstack mission report\n";
  std::cout << "success: " << std::boolalpha << report.success << '\n';
  std::cout << "steps: " << report.steps << '\n';
  std::cout << "replans: " << report.replans << '\n';
  std::cout << "lidar_scans: " << report.lidar_scans << '\n';
  std::cout << std::fixed << std::setprecision(3);
  std::cout << "global_path_length_m: " << report.global_path_length << '\n';
  std::cout << "final_error_m: " << report.final_error << '\n';
  std::cout << "covariance_trace: " << report.covariance_trace << '\n';
  std::cout << "min_lidar_range_m: " << report.min_lidar_range << '\n';
  std::cout << "final_true_pose: (" << report.final_true_pose.position.x << ", "
            << report.final_true_pose.position.y << ", yaw " << report.final_true_pose.yaw << ")\n";
  std::cout << "final_estimated_pose: (" << report.final_estimated_pose.position.x << ", "
            << report.final_estimated_pose.position.y << ", yaw " << report.final_estimated_pose.yaw << ")\n";
  std::cout << "\nmap preview (# obstacle, + inflated, * global path)\n";
  std::cout << simulator.map().to_ascii(report.global_path, simulator.start(), simulator.goal());

  return report.success ? 0 : 2;
}
