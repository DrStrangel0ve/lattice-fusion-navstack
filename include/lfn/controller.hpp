#pragma once

#include "lfn/geometry.hpp"

#include <vector>

namespace lfn {

struct PurePursuitConfig {
  double lookahead_distance{1.2};
  double max_linear_speed{2.0};
  double max_angular_speed{1.4};
  double goal_tolerance{0.35};
  double slow_radius{2.0};
};

class PurePursuitController {
public:
  explicit PurePursuitController(PurePursuitConfig config = {});

  Twist2D command(Pose2D pose, const std::vector<Vec2> &path) const;

private:
  PurePursuitConfig config_{};
};

} // namespace lfn
