# trajectory_server

Receding-horizon trajectory optimization for PathCover. The ROS 2 node consumes `polytope_msgs/msg/PolytopeArray`, solves a corridor-constrained trajectory using GCOPTER/MINCO, and publishes differentially flat setpoints for the quadrotor controller.

## Run

The full PathCover launch starts this node automatically. To start only the optimizer from a built and sourced workspace:

```bash
ros2 launch trajectory_server traj_opt.launch.py
```

Parameters live in [`config/gcopter_params.yaml`](config/gcopter_params.yaml). The default interface is:

| Direction | Topic | Type |
|---|---|---|
| Subscribe | `/quadrotor/polytopes` | `polytope_msgs/msg/PolytopeArray` |
| Subscribe | `/quadrotor/ground_truth` | `nav_msgs/msg/Odometry` |
| Publish | `/quadrotor/trajectory` | `quadrotor_msgs/msg/TrajectoryCommand` |

`cost_order` selects minimum jerk (`3`, degree 5) or minimum snap (`4`, degree 7). `SafetyMargin` shrinks the corridor before optimization, while the velocity, attitude, body-rate, thrust, and vehicle parameters define dynamic feasibility.

## Licence

The package integration code follows the repository's [BSD 3-Clause licence](../../LICENSE). Vendored GCOPTER/MINCO code and embedded utilities retain the notices in their source files and the package's [MIT licence](LICENSE).
