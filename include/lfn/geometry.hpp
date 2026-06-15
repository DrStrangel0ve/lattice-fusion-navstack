#pragma once

#include <cmath>
#include <vector>

namespace lfn {

constexpr double kPi = 3.141592653589793238462643383279502884;

struct Vec2 {
  double x{0.0};
  double y{0.0};

  constexpr Vec2() = default;
  constexpr Vec2(double x_value, double y_value) : x(x_value), y(y_value) {}

  constexpr Vec2 operator+(const Vec2 &other) const { return {x + other.x, y + other.y}; }
  constexpr Vec2 operator-(const Vec2 &other) const { return {x - other.x, y - other.y}; }
  constexpr Vec2 operator*(double scale) const { return {x * scale, y * scale}; }
  constexpr Vec2 operator/(double scale) const { return {x / scale, y / scale}; }

  Vec2 &operator+=(const Vec2 &other);
  Vec2 &operator-=(const Vec2 &other);
};

struct Pose2D {
  Vec2 position{};
  double yaw{0.0};
};

struct Twist2D {
  double linear{0.0};
  double angular{0.0};
};

struct TrajectoryPoint {
  Pose2D pose{};
  Twist2D command{};
  double time{0.0};
};

double dot(Vec2 lhs, Vec2 rhs);
double norm(Vec2 value);
double distance(Vec2 lhs, Vec2 rhs);
double heading_to(Vec2 from, Vec2 to);
double normalize_angle(double radians);
double clamp(double value, double low, double high);
Vec2 rotate(Vec2 value, double radians);
Pose2D integrate_unicycle(Pose2D pose, Twist2D twist, double dt);
double path_length(const std::vector<Vec2> &path);

} // namespace lfn
