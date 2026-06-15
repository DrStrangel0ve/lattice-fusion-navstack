# Architecture

The stack is organized as a deterministic autonomy pipeline:

```text
synthetic controls -> EKF predict -> GPS/yaw corrections -> estimated pose
        |                                                   |
        v                                                   v
     true robot ---- raycast lidar / occupancy grid ---- A* global path
                                                            |
                                                            v
                                           dynamic-window local trajectory
                                                            |
                                                            v
                                                    commanded twist
```

## Design goals

- Keep algorithms readable and auditable.
- Avoid runtime framework dependencies.
- Prefer deterministic simulation so CI failures are reproducible.
- Keep public APIs narrow enough for later ROS 2, Isaac Sim, or hardware-in-loop adapters.

## Components

### Occupancy grid and costmap

`GridMap` stores two layers:

- Raw occupancy: lethal obstacle cells used by lidar and inflation.
- Inflated cost: a traversal cost layer used by A* and local collision checks.

Inflation applies a radial cost gradient around occupied cells. The inscribed radius can remain lethal while the outer band remains traversable but expensive, which lets the global planner prefer centerline clearance without making every narrow gap impossible.

### Global planning

`AStarPlanner` performs 8-connected search with Euclidean heuristic distance. Diagonal moves are allowed, but diagonal corner cutting is rejected when either adjacent cardinal cell is blocked. Step cost is scaled by inflated cost so paths can cross low-risk cells when required while still preferring clearance. Missions can also enable a circular footprint check during global search so the path is compatible with the local collision model.

### State estimation

`Ekf2D` tracks `[x, y, yaw, velocity]`. Prediction uses a unicycle motion model and first-order velocity response to commanded speed. Correction is implemented as scalar Kalman updates for position, yaw, and velocity, keeping the code dependency-free and easy to inspect.

### Local trajectory rollout

`LatticePlanner` samples linear and angular velocity commands around the current command and the controller reference. Each candidate is rolled out with the unicycle model, checked against the inflated footprint cost, and scored using:

- progress toward the global goal,
- distance back to the global path,
- minimum obstacle clearance,
- smoothness relative to the previous command,
- agreement with the pure-pursuit reference.

The resulting command is a lightweight dynamic-window style local decision layer.

### Lidar raycasting

`LidarSimulator` casts evenly spaced beams through raw occupancy cells. It returns both ranges and hit points so tests and future visualization code can inspect sensor geometry directly.

### Mission simulation

`MissionSimulator` builds a fixed slalom map with boundary walls, runs global replanning, chooses local commands, applies deterministic motion bias, fuses noisy synthetic measurements, and periodically raycasts lidar. The final report includes success, final error, covariance trace, path length, scan count, and pose traces.

## Validation strategy

The test binary intentionally uses plain `assert` checks to stay portable:

- cost inflation and collision checks,
- A* routing through a wall gap,
- EKF convergence after prediction and measurement updates,
- lidar range accuracy against a known obstacle,
- local planner forward command selection,
- end-to-end mission reachability.

## Extension ideas

- Replace the deterministic measurement perturbation with a noise model seeded from scenario files.
- Add a costmap layer interface for semantic obstacles and lane boundaries.
- Swap the local rollout score for MPC or timed elastic band optimization.
- Emit `nav_msgs/Path` and `sensor_msgs/LaserScan` in a ROS 2 adapter package.
- Add timestamped sensor queues and delayed EKF correction.
- Add multi-resolution planning or hybrid A* for vehicle-like nonholonomic constraints.
