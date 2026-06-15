# Lattice Fusion Navstack

A self-contained C++20 autonomy stack that demonstrates the building blocks used in mobile robots and autonomous vehicle prototypes: probabilistic state estimation, occupancy-grid planning, local trajectory rollout, raycast lidar simulation, and closed-loop mission execution.

The project is intentionally dependency-light so reviewers can inspect the algorithms directly. It is not a ROS wrapper; it is the core planning and estimation logic that a ROS 2, Cyber RT, Isaac Sim, or hardware-in-loop integration could call.

## What is inside

- Extended Kalman filter for 2D position, yaw, and forward velocity fusion.
- Occupancy grid and costmap with world/cell transforms, obstacle inflation, and circular footprint collision checks.
- A* global planner with diagonal motion, weighted heuristic search, obstacle cost penalties, footprint checks, and corner-cut prevention.
- Dynamic-window style local lattice planner that rolls out sampled linear/angular commands and scores progress, path adherence, clearance, smoothness, and controller agreement.
- Pure pursuit controller for a path-following reference command.
- Raycast lidar simulator for deterministic scan and hit-point generation.
- Mission simulator that replans, fuses synthetic GPS/yaw/velocity updates, raycasts lidar, and drives through a cluttered map.
- Self-contained tests using `assert`, with no external test framework.
- GitHub Actions CI matrix for Ubuntu GCC and Clang CMake build/test.

## Build

```bash
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure
./build/lfn_nav_sim
```

If Ninja is not installed, omit `-G Ninja` and use the platform default generator:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

## Why this is portfolio-relevant

Autonomy teams care about engineers who understand the data path from uncertain sensors to motion decisions. This repo keeps that path visible:

1. The map models drivable/free/occupied space and inflates obstacles by robot radius.
2. The EKF predicts motion under commanded twist and corrects with noisy absolute measurements.
3. A* produces a global route in grid space.
4. The local planner samples dynamically feasible trajectories and scores progress, clearance, and command smoothness.
5. The mission loop closes the system and reports reachability, final error, covariance trace, and replans.

The code favors explicit math and small interfaces over opaque dependencies. That makes it useful as a reviewable signal for C++ autonomy roles where the relevant questions are: can the engineer reason about state, uncertainty, costmaps, feasibility, and deterministic validation?

## Example simulator output

`lfn_nav_sim` prints mission metrics and an ASCII preview of the inflated costmap:

```text
lattice-fusion-navstack mission report
success: true
steps: ...
replans: ...
lidar_scans: ...
global_path_length_m: ...
final_error_m: ...
covariance_trace: ...
min_lidar_range_m: ...
```

## Repository layout

```text
include/lfn/       Public C++ headers
src/               Algorithm implementations
apps/nav_sim.cpp   End-to-end autonomy simulation
tests/             Lightweight unit tests
docs/              Architecture notes
```

## Main APIs

- `lfn::GridMap`: stores raw occupancy and inflated traversal costs.
- `lfn::AStarPlanner`: plans a global path through the costmap.
- `lfn::Ekf2D`: predicts unicycle motion and corrects position, yaw, and velocity measurements.
- `lfn::LatticePlanner`: evaluates local command rollouts against the map and global path.
- `lfn::LidarSimulator`: raycasts beams against occupied cells.
- `lfn::MissionSimulator`: runs the full deterministic scenario and returns metrics/traces.

## Notes

This is an educational and portfolio implementation, not a certified safety stack. The architecture is intentionally compatible with later production-style extensions such as layered costmaps, sensor timestamping, map frame transforms, kinodynamic global search, MPC, and scenario-file based regression tests.
