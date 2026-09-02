# polytope_msgs

ROS 2 interfaces used to pass a PathCover corridor to downstream planners.

- `Polytope.msg` stores one convex region as flattened `a`, `b`, and `seed` arrays, representing the half-space system `A y <= b`.
- `PolytopeArray.msg` stores the ordered corridor, its header, and the local planning goal.

Build the interface package with:

```bash
colcon build --packages-select polytope_msgs
```

The corridor generator publishes `PolytopeArray`; `trajectory_server` is the supplied consumer. Array dimensions are determined by the application, so consumers must reconstruct each row-major matrix consistently with the planner dimension.

## Licence

BSD 3-Clause; see the [repository licence](../../LICENSE).
