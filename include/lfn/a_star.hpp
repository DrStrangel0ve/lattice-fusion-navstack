#pragma once

#include "lfn/geometry.hpp"
#include "lfn/grid_map.hpp"

#include <cstdint>
#include <vector>

namespace lfn {

struct AStarOptions {
  bool allow_diagonal{true};
  double heuristic_weight{1.05};
  double cost_weight{3.0};
  std::uint8_t occupied_threshold{220};
  double footprint_radius{0.0};
};

struct PlanResult {
  bool success{false};
  std::vector<Cell> cells{};
  std::vector<Vec2> path{};
  double cost{0.0};
  int expanded_nodes{0};
};

class AStarPlanner {
public:
  explicit AStarPlanner(AStarOptions options = {});

  PlanResult plan(const GridMap &map, Vec2 start, Vec2 goal) const;

private:
  AStarOptions options_{};
};

} // namespace lfn
