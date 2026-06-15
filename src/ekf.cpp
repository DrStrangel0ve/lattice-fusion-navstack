#include "lfn/ekf.hpp"

#include <algorithm>
#include <cmath>

namespace lfn {
namespace {

double &at(Ekf2D::Matrix4 &matrix, int row, int col) { return matrix[static_cast<std::size_t>(row * 4 + col)]; }

double at(const Ekf2D::Matrix4 &matrix, int row, int col) {
  return matrix[static_cast<std::size_t>(row * 4 + col)];
}

Ekf2D::Matrix4 multiply(const Ekf2D::Matrix4 &lhs, const Ekf2D::Matrix4 &rhs) {
  Ekf2D::Matrix4 out{};
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      double value = 0.0;
      for (int k = 0; k < 4; ++k) {
        value += at(lhs, r, k) * at(rhs, k, c);
      }
      at(out, r, c) = value;
    }
  }
  return out;
}

Ekf2D::Matrix4 transpose(const Ekf2D::Matrix4 &matrix) {
  Ekf2D::Matrix4 out{};
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      at(out, r, c) = at(matrix, c, r);
    }
  }
  return out;
}

} // namespace

Ekf2D::Ekf2D() { set_covariance_diagonal(1.0, 1.0, 0.2, 0.5); }

void Ekf2D::set_state(Pose2D pose, double velocity) {
  pose_ = pose;
  pose_.yaw = normalize_angle(pose_.yaw);
  velocity_ = velocity;
}

void Ekf2D::set_covariance_diagonal(double x, double y, double yaw, double velocity) {
  covariance_.fill(0.0);
  at(covariance_, 0, 0) = x;
  at(covariance_, 1, 1) = y;
  at(covariance_, 2, 2) = yaw;
  at(covariance_, 3, 3) = velocity;
}

double Ekf2D::covariance_trace() const {
  return at(covariance_, 0, 0) + at(covariance_, 1, 1) + at(covariance_, 2, 2) + at(covariance_, 3, 3);
}

void Ekf2D::predict(Twist2D command, double dt) {
  if (dt <= 0.0) {
    return;
  }

  const double old_yaw = pose_.yaw;
  const double response = std::clamp(noise_.velocity_response * dt, 0.0, 1.0);
  const double predicted_velocity = velocity_ + (command.linear - velocity_) * response;
  const double motion_velocity = 0.5 * (velocity_ + predicted_velocity);

  pose_.position.x += motion_velocity * std::cos(old_yaw) * dt;
  pose_.position.y += motion_velocity * std::sin(old_yaw) * dt;
  pose_.yaw = normalize_angle(pose_.yaw + command.angular * dt);
  velocity_ = predicted_velocity;

  Matrix4 jacobian{};
  at(jacobian, 0, 0) = 1.0;
  at(jacobian, 1, 1) = 1.0;
  at(jacobian, 2, 2) = 1.0;
  at(jacobian, 3, 3) = 1.0 - response;
  at(jacobian, 0, 2) = -motion_velocity * std::sin(old_yaw) * dt;
  at(jacobian, 1, 2) = motion_velocity * std::cos(old_yaw) * dt;
  at(jacobian, 0, 3) = 0.5 * (1.0 + (1.0 - response)) * std::cos(old_yaw) * dt;
  at(jacobian, 1, 3) = 0.5 * (1.0 + (1.0 - response)) * std::sin(old_yaw) * dt;

  covariance_ = multiply(multiply(jacobian, covariance_), transpose(jacobian));
  at(covariance_, 0, 0) += noise_.position_process * dt;
  at(covariance_, 1, 1) += noise_.position_process * dt;
  at(covariance_, 2, 2) += noise_.yaw_process * dt;
  at(covariance_, 3, 3) += noise_.velocity_process * dt;
  symmetrize_covariance();
}

void Ekf2D::update_position(Vec2 measured_position, double stddev) {
  const double variance = stddev * stddev;
  update_scalar(0, measured_position.x - pose_.position.x, variance);
  update_scalar(1, measured_position.y - pose_.position.y, variance);
}

void Ekf2D::update_yaw(double measured_yaw, double stddev) {
  update_scalar(2, normalize_angle(measured_yaw - pose_.yaw), stddev * stddev);
  pose_.yaw = normalize_angle(pose_.yaw);
}

void Ekf2D::update_velocity(double measured_velocity, double stddev) {
  update_scalar(3, measured_velocity - velocity_, stddev * stddev);
}

void Ekf2D::update_scalar(int state_index, double residual, double variance) {
  const double innovation = at(covariance_, state_index, state_index) + variance;
  if (innovation <= 0.0) {
    return;
  }

  double gain[4]{};
  for (int r = 0; r < 4; ++r) {
    gain[r] = at(covariance_, r, state_index) / innovation;
  }

  pose_.position.x += gain[0] * residual;
  pose_.position.y += gain[1] * residual;
  pose_.yaw = normalize_angle(pose_.yaw + gain[2] * residual);
  velocity_ += gain[3] * residual;

  Matrix4 prior = covariance_;
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      at(covariance_, r, c) = at(prior, r, c) - gain[r] * at(prior, state_index, c);
    }
  }
  symmetrize_covariance();
}

void Ekf2D::symmetrize_covariance() {
  for (int r = 0; r < 4; ++r) {
    for (int c = r + 1; c < 4; ++c) {
      const double value = 0.5 * (at(covariance_, r, c) + at(covariance_, c, r));
      at(covariance_, r, c) = value;
      at(covariance_, c, r) = value;
    }
  }
}

} // namespace lfn
