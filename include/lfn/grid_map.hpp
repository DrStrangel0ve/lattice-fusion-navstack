#pragma once

#include "lfn/geometry.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace lfn {

struct Cell {
  int x{0};
  int y{0};
};

constexpr bool operator==(Cell lhs, Cell rhs) { return lhs.x == rhs.x && lhs.y == rhs.y; }
constexpr bool operator!=(Cell lhs, Cell rhs) { return !(lhs == rhs); }

class GridMap {
public:
  static constexpr std::uint8_t kFree = 0;
  static constexpr std::uint8_t kLethal = 255;

  GridMap(int width, int height, double resolution, Vec2 origin = {});

  int width() const noexcept { return width_; }
  int height() const noexcept { return height_; }
  double resolution() const noexcept { return resolution_; }
  Vec2 origin() const noexcept { return origin_; }
  int cell_count() const noexcept { return width_ * height_; }

  bool in_bounds(Cell cell) const noexcept;
  int index(Cell cell) const;
  Cell cell_from_index(int index) const;

  std::optional<Cell> world_to_cell(Vec2 point) const;
  Vec2 cell_center(Cell cell) const;

  std::uint8_t occupancy(Cell cell) const;
  std::uint8_t cost(Cell cell) const;
  bool is_occupied(Cell cell) const;
  bool is_traversable(Cell cell, std::uint8_t occupied_threshold = 220) const;

  void clear();
  void set_occupied(Cell cell, bool occupied = true);
  void set_occupied_world(Vec2 point, bool occupied = true);
  void add_rect(Vec2 min_corner, Vec2 max_corner);
  void inflate_obstacles(double inflation_radius, double inscribed_radius = 0.0);

  bool collides(Pose2D pose, double radius, std::uint8_t occupied_threshold = 220) const;
  double distance_to_nearest_obstacle(Vec2 point, double max_distance) const;
  std::string to_ascii(const std::vector<Vec2> &path = {}, std::optional<Vec2> start = {},
                       std::optional<Vec2> goal = {}) const;

private:
  int width_{0};
  int height_{0};
  double resolution_{1.0};
  Vec2 origin_{};
  std::vector<std::uint8_t> occupancy_;
  std::vector<std::uint8_t> cost_;
};

} // namespace lfn
