#include "lfn/a_star.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

namespace lfn {
namespace {

struct QueueNode {
  int index{0};
  double g{0.0};
  double f{0.0};
};

struct QueueNodeGreater {
  bool operator()(const QueueNode &lhs, const QueueNode &rhs) const { return lhs.f > rhs.f; }
};

double heuristic(const GridMap &map, Cell cell, Cell goal) {
  const double dx = static_cast<double>(cell.x - goal.x);
  const double dy = static_cast<double>(cell.y - goal.y);
  return std::hypot(dx, dy) * map.resolution();
}

bool diagonal_step(Cell from, Cell to) { return from.x != to.x && from.y != to.y; }

} // namespace

AStarPlanner::AStarPlanner(AStarOptions options) : options_(options) {}

PlanResult AStarPlanner::plan(const GridMap &map, Vec2 start, Vec2 goal) const {
  PlanResult result;
  const auto start_cell = map.world_to_cell(start);
  const auto goal_cell = map.world_to_cell(goal);
  const auto is_searchable = [this, &map](Cell cell) {
    if (!map.is_traversable(cell, options_.occupied_threshold)) {
      return false;
    }
    if (options_.footprint_radius <= 0.0) {
      return true;
    }
    return !map.collides({map.cell_center(cell), 0.0}, options_.footprint_radius, options_.occupied_threshold);
  };

  if (!start_cell || !goal_cell || !is_searchable(*start_cell) || !is_searchable(*goal_cell)) {
    return result;
  }

  const int start_index = map.index(*start_cell);
  const int goal_index = map.index(*goal_cell);
  const int count = map.cell_count();
  std::vector<double> g_score(static_cast<std::size_t>(count), std::numeric_limits<double>::infinity());
  std::vector<int> parent(static_cast<std::size_t>(count), -1);
  std::vector<bool> closed(static_cast<std::size_t>(count), false);
  std::priority_queue<QueueNode, std::vector<QueueNode>, QueueNodeGreater> open;

  g_score[static_cast<std::size_t>(start_index)] = 0.0;
  open.push({start_index, 0.0, options_.heuristic_weight * heuristic(map, *start_cell, *goal_cell)});

  constexpr int kDirections[8][2] = {{1, 0},  {-1, 0}, {0, 1},  {0, -1},
                                     {1, 1},  {1, -1}, {-1, 1}, {-1, -1}};

  while (!open.empty()) {
    const QueueNode node = open.top();
    open.pop();
    if (closed[static_cast<std::size_t>(node.index)]) {
      continue;
    }
    closed[static_cast<std::size_t>(node.index)] = true;
    ++result.expanded_nodes;

    if (node.index == goal_index) {
      result.success = true;
      result.cost = node.g;
      break;
    }

    const Cell current = map.cell_from_index(node.index);
    for (const auto &direction : kDirections) {
      const Cell neighbor{current.x + direction[0], current.y + direction[1]};
      if (!map.in_bounds(neighbor)) {
        continue;
      }
      if (!options_.allow_diagonal && diagonal_step(current, neighbor)) {
        continue;
      }
      if (!is_searchable(neighbor)) {
        continue;
      }
      if (diagonal_step(current, neighbor)) {
        const Cell side_a{neighbor.x, current.y};
        const Cell side_b{current.x, neighbor.y};
        if (!is_searchable(side_a) || !is_searchable(side_b)) {
          continue;
        }
      }

      const int neighbor_index = map.index(neighbor);
      if (closed[static_cast<std::size_t>(neighbor_index)]) {
        continue;
      }
      const double step_distance = std::hypot(static_cast<double>(direction[0]), static_cast<double>(direction[1])) *
                                   map.resolution();
      const double cost_scale = 1.0 + options_.cost_weight * (static_cast<double>(map.cost(neighbor)) / 255.0);
      const double tentative_g = node.g + step_distance * cost_scale;
      auto &best_g = g_score[static_cast<std::size_t>(neighbor_index)];
      if (tentative_g < best_g) {
        best_g = tentative_g;
        parent[static_cast<std::size_t>(neighbor_index)] = node.index;
        const double f = tentative_g + options_.heuristic_weight * heuristic(map, neighbor, *goal_cell);
        open.push({neighbor_index, tentative_g, f});
      }
    }
  }

  if (!result.success) {
    return result;
  }

  std::vector<Cell> reversed;
  for (int index = goal_index; index >= 0; index = parent[static_cast<std::size_t>(index)]) {
    reversed.push_back(map.cell_from_index(index));
    if (index == start_index) {
      break;
    }
  }
  std::reverse(reversed.begin(), reversed.end());
  result.cells = reversed;
  result.path.reserve(result.cells.size());
  for (const Cell cell : result.cells) {
    result.path.push_back(map.cell_center(cell));
  }
  return result;
}

} // namespace lfn
