# trajectory_server

Receding-horizon trajectory optimizer. It consumes the corridor published by
[`corridor_planning`](../corridor_planning) as `polytope_msgs/Polytopes` and solves a
corridor-constrained, dynamically feasible trajectory with **GCOPTER / MINCO**, publishing the
sampled setpoint (position through snap, plus yaw) that the geometric controller tracks.

The optimizer itself ([`include/gcopter/`](include/gcopter)) is
[GCOPTER](https://github.com/ZJU-FAST-Lab/GCOPTER) by Zhepei Wang and Fei Gao, vendored under its
own MIT license ([`LICENSE`](LICENSE)). What this package adds is the ROS node, the receding-horizon
loop, and a thin solver wrapper that makes the MINCO order selectable at runtime.

## How the loop runs

A 100 Hz timer ([`trajectory_server_node.cpp`](src/trajectory_server_node.cpp)) drives everything:

1. **Replan** every `ReplanPeriod` seconds. Boundary conditions are re-seeded from the drone's
   *actual* current state each time — position and velocity from odometry, acceleration (and
   jerk, for the snap-cost case) set to zero rather than carried forward from the previous
   solve, so a solve's own artifacts are never fed back in as the next replan's hard boundary
   condition.
2. **Terminal state** is the corridor's `goal` field — the local target where the reference path
   exits the last polytope — with zero terminal velocity/acceleration. Because only the near-term
   part of each solve is executed before the next replan, the terminal condition mostly shapes
   the unexecuted tail and is deliberately kept neutral.
3. **Sample** the current trajectory at the timer rate and publish it as
   `quadrotor_msgs/TrajectoryCommand`.
4. **Hover** if there is no corridor yet, or if the first solve has not succeeded. A failed
   replan with a previously valid trajectory keeps flying the old one.

Yaw is generated separately from the desired horizontal velocity direction, rate-limited by
`YawDotMax` and frozen below `LowSpeedThreshold` so the vehicle does not spin while nearly
stationary.

### Corridor handling

Each incoming `Polytope` is converted from $Ay \le b$ to GCOPTER's
$n^\top y + d \le 0$ form, with `b` shrunk by `SafetyMargin * ||A_row||` — scaling by the row norm
means the inward shift is exactly `SafetyMargin` metres whether or not the rows are normalized.
This is applied *on top of* `corridor_planning`'s `DeflationFactor`; the two stack, so setting
both means the corridor is shrunk twice.

A `Polytopes` message with no polytopes, or without a valid 3-element `goal`, is rejected and the
node holds position.

## Parameters — [`config/gcopter_params.yaml`](config/gcopter_params.yaml)

### Topics (required)

| Parameter | Meaning |
|---|---|
| `OdometryTopic` | Robot odometry — supplies the replan's initial position and velocity |
| `PolyhedraTopic` | Input corridor (`polytope_msgs/Polytopes`) |
| `TrajectoryTopic` | Output setpoint (`quadrotor_msgs/TrajectoryCommand`) |

### Dynamic feasibility bounds

`v_max` is required; the rest have defaults.

| Parameter | Default | Meaning |
|---|---|---|
| `v_max` | — | Max velocity (m/s) |
| `omg_max` | `3.0` | Max body rate (rad/s) |
| `theta_max` | `0.7854` | Max tilt angle (rad) |
| `thrust_min` / `thrust_max` | `2.0` / `20.0` | Thrust bounds (N) |

These are enforced as *soft* constraints, weighted by `weight_pos`, `weight_vel`, `weight_omg`,
`weight_theta`, `weight_thrust` (all default `1.0e4`). `weight_pos` is the corridor-violation
weight.

### Vehicle model (flatness map)

| Parameter | Default | Meaning |
|---|---|---|
| `vehicle_mass` | `1.0` | Mass (kg) — must match the URDF's inertial mass (`0.716` for the bundled quadrotor) |
| `grav_acc` | `9.81` | Gravity (m/s²) |
| `horiz_drag`, `vert_drag`, `paras_drag` | `0.0` | Drag coefficients |
| `speed_eps` | `1.0e-3` | Speed smoothing, avoids a division by zero at rest |

### MINCO / L-BFGS

| Parameter | Default | Meaning |
|---|---|---|
| `cost_order` | `3` | `3` = minimum jerk, degree-5 pieces; `4` = minimum snap, degree-7 (smoother, more expensive, adds initial/terminal jerk boundary conditions). Any other value is fatal at startup |
| `weight_time` | `100.0` | $\rho$, the time-regularization weight — the aggressiveness knob |
| `length_per_piece` | `2.0` | Desired path length per MINCO piece (m); sets the piece count |
| `integral_resolution` | `16` | Integration steps per piece for the penalty integral |
| `smoothing_eps` | `0.01` | Penalty smoothing factor |
| `rel_cost_tol` | `1.0e-4` | L-BFGS relative cost convergence tolerance |

### Receding horizon, safety, and yaw

| Parameter | Default | Meaning |
|---|---|---|
| `SafetyMargin` | `0.3` | Inward shrink of each corridor half-space before optimization (m) |
| `ReplanPeriod` | `0.5` | Seconds between replans |
| `GoalReachedThreshold` | `0.2` | Goal tolerance (m); inside it the node stops publishing setpoints |
| `YawDotMax` | `π` | Yaw rate limit (rad/s) |
| `LowSpeedThreshold` | `0.1` | Below this horizontal speed, yaw is held |

## ROS interface — `trajectory_server_node`

| Direction | Topic | Type |
|---|---|---|
| sub | `PolyhedraTopic` (`/quadrotor/polytopes`) | `polytope_msgs/Polytopes` |
| sub | `OdometryTopic` (`/quadrotor/ground_truth`) | `nav_msgs/Odometry` |
| pub | `TrajectoryTopic` (`/quadrotor/trajectory`) | `quadrotor_msgs/TrajectoryCommand` (position → snap, yaw, yaw rate) |
| pub | `/colored_trajectory` | `nav_msgs/Path` — the solved trajectory sampled at 20 Hz, for RViz |

## Swapping this out

Nothing above is required by PathCover. The corridor is published on a plain ROS topic, so any
downstream planner that can read $Ay \le b$ can replace this package entirely — see the
[polytope_msgs README](../polytope_msgs/README.md).

## Package layout

```
include/
  trajectory_server.hpp    ROS node: parameters, callbacks, receding-horizon control loop
  gcopter_solver.hpp       templated wrapper selecting MINCO_S3NU (jerk) / MINCO_S4NU (snap)
  trajectory_interface.hpp type-erased trajectory handle, so the node is degree-agnostic
  gcopter/                 vendored GCOPTER (MIT): gcopter, minco, flatness, lbfgs,
                           trajectory, geo_utils, quickhull, root_finder, sdlp
src/
  trajectory_server_node.cpp   node implementation + main()
config/
  gcopter_params.yaml          parameters documented above
```

## Dependencies

`roscpp`, `nav_msgs`, `geometry_msgs`, `visualization_msgs`,
[`polytope_msgs`](../polytope_msgs), [`quadrotor_msgs`](../quadrotor_sim/quadrotor_msgs), Eigen.

## Licence

This package's own code is BSD 3-Clause, as with the rest of the repository. The vendored
`include/gcopter/` tree is MIT and governed by [`LICENSE`](LICENSE).
