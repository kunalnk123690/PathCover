# polytope_msgs

The interface between corridor generation and everything downstream. A corridor is just a
sequence of convex polytopes in H-representation, so these two messages are all that
[`corridor_planning`](../corridor_planning) and [`trajectory_server`](../trajectory_server) share
— swap either side out and the other keeps working.

## Messages

### `Polytope`

```
float64[] A
float64[] b
float64[] seed
```

One convex polytope as the half-space set $\{y : Ay \le b\}$.

| Field | Layout |
|---|---|
| `A` | The constraint matrix, **flattened row-major**. With `dim` columns it has `A.size() / dim` rows; row `i` is `A[dim*i .. dim*i+dim-1]` |
| `b` | The offset vector, one entry per row of `A` |
| `seed` | The `dim`-vector the polytope was grown around. Guaranteed strictly interior: `A * seed < b` elementwise |

`dim` is 3 in this stack, and is not carried in the message — a consumer knows the dimension of
the world it is planning in. Rows are **not** normalized: `||A.row(i)||` is arbitrary, so any
metric shift of a half-space must scale by the row norm (shrinking constraint `i` inward by
`m` metres means `b[i] -= m * ||A.row(i)||`).

The `seed` is not redundant with the polytope. It is the interior point that generated the set,
which is what redundancy removal in the dual space and H→V conversion both need, and it is the
natural warm-start point for a downstream solver.

### `Polytopes`

```
Header header
Polytope[] polytope
float64[] goal
```

A full corridor.

| Field | Meaning |
|---|---|
| `header` | `frame_id` is `world`; stamped at publication |
| `polytope` | The corridor, **ordered along the path**. Consecutive polytopes overlap, which is what makes the sequence traversable |
| `goal` | The local target $y_{\text{int}}$ — the point where the reference path exits the last polytope. This is the receding-horizon goal, **not** the mission goal, whenever the corridor was truncated by `horizon` |

A publisher may leave `goal` empty (`corridor_planning` fills it only once the corridor has a
local target to report). Consumers should treat a message with an empty `polytope` array, or with
fewer than 3 `goal` entries, as "no valid target yet" and hold position — which is what
`trajectory_server` does.

## Consuming a corridor

```cpp
#include <polytope_msgs/Polytopes.h>

void callback(const polytope_msgs::Polytopes::ConstPtr &msg) {
    for (const auto &poly : msg->polytope) {
        const int rows = static_cast<int>(poly.A.size() / 3);
        Eigen::Matrix<double, -1, 3, Eigen::RowMajor> A(rows, 3);
        std::copy(poly.A.begin(), poly.A.end(), A.data());
        Eigen::Map<const Eigen::VectorXd> b(poly.b.data(), rows);
        Eigen::Map<const Eigen::Vector3d> seed(poly.seed.data());
        // ... A * y <= b, with seed strictly inside
    }
}
```

Producing one is the mirror image — see `convertConstraints()` in
[`SubscribeAndPublish.cpp`](../corridor_planning/src/SubscribeAndPublish.cpp), and
`polytopesCallback()` in
[`trajectory_server_node.cpp`](../trajectory_server/src/trajectory_server_node.cpp) for a
consumer that converts to the $n^\top y + d \le 0$ convention GCOPTER expects.

## Package layout

```
msg/
  Polytope.msg     one convex polytope: A, b, seed
  Polytopes.msg    a corridor: header, ordered polytopes, local target
```

## Dependencies

`std_msgs`, `message_generation` / `message_runtime`.
