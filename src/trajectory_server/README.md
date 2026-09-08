# trajectory_server

Receding-horizon trajectory optimizer. It consumes the corridor published by
[`corridor_planning`](../corridor_planning) as `polytope_msgs/Polytopes` and solves a
corridor-constrained, dynamically feasible trajectory with **GCOPTER / MINCO**. The 3-D build
publishes flat-output setpoints to the quadrotor controller; the 2-D build tracks the planar
trajectory with differential-drive velocity commands.

The optimizer itself ([`include/gcopter/`](include/gcopter)) is
[GCOPTER](https://github.com/ZJU-FAST-Lab/GCOPTER) by Zhepei Wang and Fei Gao, vendored under its
own MIT license ([`LICENSE.GCOPTER`](LICENSE.GCOPTER)). What this package adds is the ROS node, the receding-horizon
loop, and a thin solver wrapper that makes the MINCO order selectable at runtime.

## How the loop runs

The implementation is selected with `PATHCOVER_EXAMPLE`: quadrotor uses 3-D at 100 Hz and Jackal
uses 2-D at 50 Hz. The control loop lives in
[`trajectory_server.cpp`](src/trajectory_server.cpp); `trajectory_server_node.cpp` is the shared
ROS entry point.

1. **Replan** every `ReplanPeriod` seconds. Boundary conditions are re-seeded from the robot's
   *actual* current state each time — position and velocity from odometry, acceleration (and
   jerk, for the snap-cost case) set to zero rather than carried forward from the previous
   solve, so a solve's own artifacts are never fed back in as the next replan's hard boundary
   condition.
2. **Terminal state** is the corridor's `goal` field — the local target where the reference path
   exits the last polytope — with zero terminal velocity/acceleration. Because only the near-term
   part of each solve is executed before the next replan, the terminal condition mostly shapes
   the unexecuted tail and is deliberately kept neutral.
3. **Sample** the current trajectory at the timer rate. Quadrotor publishes
   `quadrotor_msgs/TrajectoryCommand`; Jackal applies a unicycle tracking law and publishes
   `geometry_msgs/Twist`.
4. **Hold** if there is no corridor yet, or if the first solve has not succeeded. A failed
   replan with a previously valid trajectory keeps using the old one.

For the quadrotor, yaw is generated separately from the desired horizontal velocity direction, rate-limited by
`YawDotMax` and frozen below `LowSpeedThreshold` so the vehicle does not spin while nearly
stationary.

### Corridor handling

Each incoming `Polytope` is converted from $Ay \le b$ to GCOPTER's
$n^\top y + d \le 0$ form, with `b` shrunk by `SafetyMargin * ||A_row||` — scaling by the row norm
means the inward shift is exactly `SafetyMargin` metres whether or not the rows are normalized.
`corridor_planning` publishes its corridor unshrunk, so this is the only place robot-radius
clearance is applied and `SafetyMargin` has to cover the full circumscribed footprint.

A `Polytopes` message with no polytopes, or without a goal matching the selected dimension, is
rejected and the node holds position.

## Parameters

Quadrotor uses [`config/gcopter_params_drone.yaml`](config/gcopter_params_drone.yaml); Jackal uses
[`config/gcopter_params_jackal.yaml`](config/gcopter_params_jackal.yaml).

### Topics (required)

| Parameter | Meaning |
|---|---|
| `OdometryTopic` | Robot odometry — supplies the replan's initial position and velocity |
| `PolyhedraTopic` | Input corridor (`polytope_msgs/Polytopes`) |
| `TrajectoryTopic` | Output setpoint (`quadrotor_msgs/TrajectoryCommand`) |

For Jackal, `CmdVelTopic` replaces `TrajectoryTopic`; `OdomTwistInBodyFrame` declares the frame of
the odometry twist.

### Quadrotor dynamic feasibility bounds

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

### Quadrotor vehicle model (flatness map)

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

In the Jackal build the corresponding topics default to `/jackal/polytopes`,
`/jackal/ground_truth`, and `/jackal_velocity_controller/cmd_vel`; the command type is
`geometry_msgs/Twist`. Its additional feasibility parameters are `a_max` and `curvature_eps`, and
its tracking gains are `k_x`, `k_y`, `k_theta`, `k_align`, and `HeadingAlignTolerance`.

The planar build accepts corridors of either dimension. It reads the coefficients-per-row from
`A.size() / b.size()` rather than assuming a planar corridor, because `corridor_planning`'s
constraint matrix is hardcoded to three columns and so publishes 3-D polytopes even for a ground
robot. A 3-D corridor is reduced to the planar one the optimizer needs by taking its horizontal
cross-section at `CorridorSliceHeight` (default `0.0`): substituting that `z` into
$a_x x + a_y y + a_z z \le b$ leaves $a_x x + a_y y \le b - a_z z$. Faces whose normal is purely
vertical carry no planar information and are dropped; if one of them is *violated* at that height
the polytope does not reach it, and the corridor is rejected rather than silently truncated.
Set `CorridorSliceHeight` to the height the corridor was generated around.

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
  trajectory_server.cpp        build-selected 2-D/3-D implementation
  trajectory_server_node.cpp   shared main()
config/
  gcopter_params_drone.yaml    quadrotor parameters
  gcopter_params_jackal.yaml   Jackal parameters
```

## Dependencies

`roscpp`, `nav_msgs`, `geometry_msgs`, `visualization_msgs`,
[`polytope_msgs`](../polytope_msgs), Eigen, and either
[`quadrotor_msgs`](../quadrotor_sim/quadrotor_msgs) (quadrotor) or `tf` (Jackal).

## Licence

This package's own code is BSD 3-Clause, as with the rest of the repository. The vendored
GCOPTER / MINCO files are MIT and governed by [`LICENSE.GCOPTER`](LICENSE.GCOPTER). The embedded
`sdlp.hpp` retains Michael E. Hohmeyer's permissive redistribution notice and Zhepei Wang's
modification notice; `quickhull.hpp` credits Antti Kuukka and is marked public domain. Those
notices remain in the respective source files.
