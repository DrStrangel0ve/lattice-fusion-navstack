#include "lfn/grid_map.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace lfn {
namespace {

std::uint8_t inflation_cost(double distance, double inflation_radius, double inscribed_radius) {
  if (distance <= inscribed_radius) {
    return GridMap::kLethal;
  }
  if (inflation_radius <= inscribed_radius || distance >= inflation_radius) {
    return GridMap::kFree;
  }
  const double t = 1.0 - ((distance - inscribed_radius) / (inflation_radius - inscribed_radius));
  return static_cast<std::uint8_t>(std::clamp(32.0 + 210.0 * t, 1.0, 254.0));
}

} // namespace

GridMap::GridMap(int width, int height, double resolution, Vec2 origin) {
  if (width <= 0 || height <= 0) {
    throw std::invalid_argument("GridMap dimensions must be positive");
  }
  if (resolution <= 0.0) {
    throw std::invalid_argument("GridMap resolution must be positive");
  }
  width_ = width;
  height_ = height;
  resolution_ = resolution;
  origin_ = origin;
  occupancy_.assign(static_cast<std::size_t>(width_ * height_), kFree);
  cost_.assign(static_cast<std::size_t>(width_ * height_), kFree);
}

bool GridMap::in_bounds(Cell cell) const noexcept {
  return cell.x >= 0 && cell.y >= 0 && cell.x < width_ && cell.y < height_;
}

int GridMap::index(Cell cell) const {
  if (!in_bounds(cell)) {
    throw std::out_of_range("GridMap cell is out of bounds");
  }
  return cell.y * width_ + cell.x;
}

Cell GridMap::cell_from_index(int flat_index) const {
  if (flat_index < 0 || flat_index >= cell_count()) {
    throw std::out_of_range("GridMap index is out of bounds");
  }
  return {flat_index % width_, flat_index / width_};
}

std::optional<Cell> GridMap::world_to_cell(Vec2 point) const {
  const int x = static_cast<int>(std::floor((point.x - origin_.x) / resolution_));
  const int y = static_cast<int>(std::floor((point.y - origin_.y) / resolution_));
  const Cell cell{x, y};
  if (!in_bounds(cell)) {
    return std::nullopt;
  }
  return cell;
}

Vec2 GridMap::cell_center(Cell cell) const {
  if (!in_bounds(cell)) {
    throw std::out_of_range("GridMap cell is out of bounds");
  }
  return {origin_.x + (static_cast<double>(cell.x) + 0.5) * resolution_,
          origin_.y + (static_cast<double>(cell.y) + 0.5) * resolution_};
}

std::uint8_t GridMap::occupancy(Cell cell) const { return occupancy_[static_cast<std::size_t>(index(cell))]; }

std::uint8_t GridMap::cost(Cell cell) const { return cost_[static_cast<std::size_t>(index(cell))]; }

bool GridMap::is_occupied(Cell cell) const { return occupancy(cell) >= kLethal; }

bool GridMap::is_traversable(Cell cell, std::uint8_t occupied_threshold) const {
  return in_bounds(cell) && cost(cell) < occupied_threshold;
}

void GridMap::clear() {
  std::fill(occupancy_.begin(), occupancy_.end(), kFree);
  std::fill(cost_.begin(), cost_.end(), kFree);
}

void GridMap::set_occupied(Cell cell, bool occupied) {
  const auto flat = static_cast<std::size_t>(index(cell));
  occupancy_[flat] = occupied ? kLethal : kFree;
  cost_[flat] = occupancy_[flat];
}

void GridMap::set_occupied_world(Vec2 point, bool occupied) {
  if (const auto cell = world_to_cell(point)) {
    set_occupied(*cell, occupied);
  }
}

void GridMap::add_rect(Vec2 min_corner, Vec2 max_corner) {
  const double min_x = std::min(min_corner.x, max_corner.x);
  const double max_x = std::max(min_corner.x, max_corner.x);
  const double min_y = std::min(min_corner.y, max_corner.y);
  const double max_y = std::max(min_corner.y, max_corner.y);

  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      const Cell cell{x, y};
      const Vec2 center = cell_center(cell);
      if (center.x >= min_x && center.x <= max_x && center.y >= min_y && center.y <= max_y) {
        set_occupied(cell, true);
      }
    }
  }
}

void GridMap::inflate_obstacles(double inflation_radius, double inscribed_radius) {
  cost_ = occupancy_;
  if (inflation_radius <= 0.0) {
    return;
  }

  std::vector<Cell> obstacles;
  obstacles.reserve(occupancy_.size() / 8U);
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      const Cell cell{x, y};
      if (is_occupied(cell)) {
        obstacles.push_back(cell);
      }
    }
  }

  const int radius_cells = static_cast<int>(std::ceil(inflation_radius / resolution_));
  for (const Cell obstacle : obstacles) {
    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
      for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
        const Cell cell{obstacle.x + dx, obstacle.y + dy};
        if (!in_bounds(cell)) {
          continue;
        }
        const double distance_m = std::hypot(static_cast<double>(dx), static_cast<double>(dy)) * resolution_;
        if (distance_m > inflation_radius) {
          continue;
        }
        const auto inflated = inflation_cost(distance_m, inflation_radius, inscribed_radius);
        auto &stored = cost_[static_cast<std::size_t>(index(cell))];
        stored = std::max(stored, inflated);
      }
    }
  }
}

bool GridMap::collides(Pose2D pose, double radius, std::uint8_t occupied_threshold) const {
  const auto center = world_to_cell(pose.position);
  if (!center) {
    return true;
  }

  const int radius_cells = static_cast<int>(std::ceil((radius + resolution_) / resolution_));
  for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      const Cell cell{center->x + dx, center->y + dy};
      const Vec2 cell_point{origin_.x + (static_cast<double>(cell.x) + 0.5) * resolution_,
                            origin_.y + (static_cast<double>(cell.y) + 0.5) * resolution_};
      if (distance(pose.position, cell_point) > radius + 0.5 * resolution_) {
        continue;
      }
      if (!in_bounds(cell) || !is_traversable(cell, occupied_threshold)) {
        return true;
      }
    }
  }
  return false;
}

double GridMap::distance_to_nearest_obstacle(Vec2 point, double max_distance) const {
  const auto center = world_to_cell(point);
  if (!center) {
    return 0.0;
  }

  double best = max_distance;
  const int radius_cells = static_cast<int>(std::ceil(max_distance / resolution_));
  for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      const Cell cell{center->x + dx, center->y + dy};
      if (!in_bounds(cell) || !is_occupied(cell)) {
        continue;
      }
      best = std::min(best, distance(point, cell_center(cell)));
    }
  }
  return best;
}

std::string GridMap::to_ascii(const std::vector<Vec2> &path, std::optional<Vec2> start,
                              std::optional<Vec2> goal) const {
  std::vector<char> chars(static_cast<std::size_t>(cell_count()), '.');
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      const Cell cell{x, y};
      const auto flat = static_cast<std::size_t>(index(cell));
      if (is_occupied(cell)) {
        chars[flat] = '#';
      } else if (cost(cell) > 0) {
        chars[flat] = '+';
      }
    }
  }

  for (const Vec2 point : path) {
    if (const auto cell = world_to_cell(point)) {
      chars[static_cast<std::size_t>(index(*cell))] = '*';
    }
  }
  if (start) {
    if (const auto cell = world_to_cell(*start)) {
      chars[static_cast<std::size_t>(index(*cell))] = 'S';
    }
  }
  if (goal) {
    if (const auto cell = world_to_cell(*goal)) {
      chars[static_cast<std::size_t>(index(*cell))] = 'G';
    }
  }

  std::string out;
  out.reserve(static_cast<std::size_t>((width_ + 1) * height_));
  for (int y = height_ - 1; y >= 0; --y) {
    for (int x = 0; x < width_; ++x) {
      out.push_back(chars[static_cast<std::size_t>(index({x, y}))]);
    }
    out.push_back('\n');
  }
  return out;
}

} // namespace lfn
