#pragma once

#include "lfn/geometry.hpp"

#include <array>

namespace lfn {

class Ekf2D {
public:
  using Matrix4 = std::array<double, 16>;

  struct Noise {
    double position_process{0.04};
    double yaw_process{0.02};
    double velocity_process{0.08};
    double velocity_response{3.0};
  };

  Ekf2D();

  void set_state(Pose2D pose, double velocity);
  void set_covariance_diagonal(double x, double y, double yaw, double velocity);
  void set_noise(Noise noise) { noise_ = noise; }

  Pose2D pose() const { return pose_; }
  double velocity() const { return velocity_; }
  Matrix4 covariance() const { return covariance_; }
  double covariance_trace() const;

  void predict(Twist2D command, double dt);
  void update_position(Vec2 measured_position, double stddev);
  void update_yaw(double measured_yaw, double stddev);
  void update_velocity(double measured_velocity, double stddev);

private:
  void update_scalar(int state_index, double residual, double variance);
  void symmetrize_covariance();

  Pose2D pose_{};
  double velocity_{0.0};
  Matrix4 covariance_{};
  Noise noise_{};
};

} // namespace lfn
