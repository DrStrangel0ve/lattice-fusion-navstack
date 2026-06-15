#include "lfn/geometry.hpp"

#include <algorithm>

namespace lfn {

Vec2 &Vec2::operator+=(const Vec2 &other) {
  x += other.x;
  y += other.y;
  return *this;
}

Vec2 &Vec2::operator-=(const Vec2 &other) {
  x -= other.x;
  y -= other.y;
  return *this;
}

double dot(Vec2 lhs, Vec2 rhs) { return lhs.x * rhs.x + lhs.y * rhs.y; }

double norm(Vec2 value) { return std::sqrt(dot(value, value)); }

double distance(Vec2 lhs, Vec2 rhs) { return norm(lhs - rhs); }

double heading_to(Vec2 from, Vec2 to) { return std::atan2(to.y - from.y, to.x - from.x); }

double normalize_angle(double radians) {
  while (radians > kPi) {
    radians -= 2.0 * kPi;
  }
  while (radians < -kPi) {
    radians += 2.0 * kPi;
  }
  return radians;
}

double clamp(double value, double low, double high) { return std::max(low, std::min(value, high)); }

Vec2 rotate(Vec2 value, double radians) {
  const double c = std::cos(radians);
  const double s = std::sin(radians);
  return {c * value.x - s * value.y, s * value.x + c * value.y};
}

Pose2D integrate_unicycle(Pose2D pose, Twist2D twist, double dt) {
  pose.position.x += twist.linear * std::cos(pose.yaw) * dt;
  pose.position.y += twist.linear * std::sin(pose.yaw) * dt;
  pose.yaw = normalize_angle(pose.yaw + twist.angular * dt);
  return pose;
}

double path_length(const std::vector<Vec2> &path) {
  double length = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    length += distance(path[i - 1], path[i]);
  }
  return length;
}

} // namespace lfn
