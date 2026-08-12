# corridor_planning

**The core package.** Header-only implementations of **RISP** (Randomized Iterative Space
Partitioning) and **PathCover** (path-covering convex decomposition), the JPS + distance-map
front end that feeds them, the ROS node that runs the whole loop at sensor rate, and the RViz
visualization. The system-level launch files and configuration live here too.

Everything in [`include/planner/PathCover.hpp`](include/planner/PathCover.hpp) is ROS-free and
templated on scalar type and dimension (2-D and 3-D), so it can be lifted out of this package
with only Eigen and Qhull as dependencies — see [Library usage](#library-usage).

## The planning pipeline

Per cycle, `corridor_planning_node`:

1. Converts the latest voxel-map cloud to an Eigen point cloud, dropping returns within
   `FilterRadius` (in XY) of the robot so the seed stays strictly interior. The Jackal build also
   keeps only `ObstacleZMin <= z <= ObstacleZMax` and projects that slice into XY.
2. Rasterizes it into a fixed-extent occupancy grid and runs **JPS** followed by a
   **distance-map planner (DMP)** refinement to get a collision-free reference path.
3. Runs **PathCover** from the robot's position along that path, capped at `horizon`
   polytopes (`max_iter` — the receding-horizon knob).
4. Publishes the corridor as `polytope_msgs/Polytopes` (an `A`, `b`, `seed` triple per
   polytope) plus RViz meshes and timing diagnostics.

The planner runs on a dedicated thread with **process-latest, drop-stale** semantics
([`SubscribeAndPublish.cpp:70`](src/SubscribeAndPublish.cpp#L70)): the subscriber callback only
stashes the newest cloud, so a slow cycle never backs up a queue or inflates end-to-end latency.

The occupancy grid used by the search is allocated once from `MapLowerBound` / `MapUpperBound` /
`VoxelResolution` and reset per frame over only the cells touched on the previous frame, so the
reset is `O(occupied)` rather than `O(total_voxels)`.

## RISP — [`PathCover.hpp:85`](include/planner/PathCover.hpp#L85)

Given a seed $y_{\text{seed}}$ in free space and a point cloud $\mathcal{P}$, repeat until
$\mathcal{P}$ is empty: sample $p \in \mathcal{P}$ uniformly, form the half-space

$$a = p - y_{\text{seed}}, \qquad b = a^\top y_{\text{seed}} + (1-\alpha)\lVert a \rVert_2^2$$

and discard every $q$ with $a^\top q > b$. The plane passes through
$\alpha y_{\text{seed}} + (1-\alpha)p$, so it strictly separates $p$ from the open ball of radius
$(1-\alpha)\lVert a \rVert_2$ around the seed — which is what guarantees the seed stays interior
and, since the sampled point itself is always eliminated, that the loop terminates.

Workspace bounds (`A_bound`, `b_bound`) are appended once the loop ends, and redundant
half-spaces are removed in [`noredund`](include/planner/PathCover.hpp#L37) by mapping each
constraint to a dual point $d_m = a_m / (b_m - a_m^\top y_{\text{seed}})$ and keeping only those
on the Qhull convex hull.

$\alpha \in (0,1)$ trades polytope size against margin: smaller $\alpha$ gives a larger polytope
with less clearance around the seed. It defaults to `0.01` and is a function argument, **not** a
ROS parameter.

**Complexity**: expected `O(n)`, worst case `O(n²)`. The expected bound holds whenever a
uniformly sampled point eliminates at least a $\beta$ fraction of the remaining points with
probability at least $r$ — which is what makes the point set of a structured, locally coplanar
LiDAR cloud decay geometrically. The `O(n²)` case requires a contrived geometry in which every
sample eliminates only itself.

## PathCover — [`PathCover.hpp:192`](include/planner/PathCover.hpp#L192)

Seed RISP at the path start. Walk the waypoints: any waypoint already inside the current
polytope is skipped (one polytope may span many segments); the first one outside triggers a new
RISP call seeded at the intersection of the current polytope's boundary with the segment leading
to it ([`computePolyhedraIntersection`](include/planner/PathCover.hpp#L158)).

This is what decouples the polytope count from the path resolution: methods that emit one
polytope per path segment by construction produce many more of them, and every extra polytope
enlarges the constraint set handed to the optimizer.

Termination in finitely many polytopes, with all three path-coverage conditions (strict
separation, sequential intersection, full coverage), is guaranteed under mild clearance
assumptions.

`seed.back()` is the **local target** $y_{\text{int}}$ — where the reference path exits the last
polytope. That is what the receding-horizon planner drives toward, and it is what the node
publishes in the `goal` field of `polytope_msgs/Polytopes`.

## Parameters

The quadrotor uses [`config/global_planning_drone.yaml`](config/global_planning_drone.yaml); Jackal
uses [`config/global_planning_jackal.yaml`](config/global_planning_jackal.yaml).

| Parameter | Meaning |
|---|---|
| `MapTopic` | Input occupied-voxel cloud (default: `nanovoxmap`'s output) |
| `GroundTruthTopic` | Robot odometry; supplies the corridor's start point |
| `VoxelResolution` | Cell size of the JPS/DMP search grid, in metres. **Not** the map resolution |
| `MapLowerBound` / `MapUpperBound` | Search-grid extent *and* the workspace bounds appended to every polytope. Per-world |
| `InitialPose` / `GoalPose` | Start, and the initial goal before an RViz nav goal arrives |
| `horizon` | **`max_iter`** — caps how far the corridor extends per cycle. `6` is the default for this quadrotor demo; `1` gives a single look-ahead polytope per cycle |
| `FilterRadius` | Discard cloud points within this XY radius of the robot. Guards the seed's interiority against self-returns |
| `DeflationFactor` | Shrink every half-space inward by this many metres (robot radius / safety margin). `0` disables |
| `DmPotentialRadius`, `DmPSearchRadius` | Distance-map planner potential and search radii |

`MapLowerBound` / `MapUpperBound` and `VoxelResolution` are **per-world** and must be updated for
a new environment — they set both the search grid and the box every polytope is intersected with.

RISP's $\alpha$ is not exposed here; it defaults to `0.01` in
[`PathCover.hpp:94`](include/planner/PathCover.hpp#L94).

`FilteredCloudTopic` and `VelodynePoseTopic` are legacy entries kept for the commented-out
filtered-cloud publisher; neither affects planning.

Jackal additionally uses `ObstacleZMin` / `ObstacleZMax` for its 3-D map slice, `VizHeight` for
planar RViz markers, `JpsHeuristicWeight`, and the configurable `PolytopeTopic`.

## ROS interface — `corridor_planning_node`

| Direction | Topic | Type |
|---|---|---|
| sub | `MapTopic` (`/nanovoxmap/Voxel_map`) | `sensor_msgs/PointCloud2` |
| sub | `GroundTruthTopic` (`/quadrotor/ground_truth`) | `nav_msgs/Odometry` |
| sub | `/move_base_simple/goal` | `geometry_msgs/PoseStamped` (RViz **2D Nav Goal**; `z` is fixed at `0.75`) |
| pub | `/quadrotor/polytopes` | `polytope_msgs/Polytopes` |
| pub | `/Polyhedra/mesh`, `/Polyhedra/edge` | `visualization_msgs/Marker` — corridor faces and edges |
| pub | `/visualizer/route`, `/seeds`, `/waypoints`, `/wayseeds`, `/start`, `/goal` | `visualization_msgs/Marker` — path, seeds, endpoints |
| pub | `/benchmark/computation_time_ms` | `std_msgs/Float64` — PathCover time **in isolation** (conversion, grid, and search excluded) |
| pub | `/benchmark/remaining_points` | `std_msgs/Int64MultiArray` — per-iteration $N_m$ decay for the first polytope |

The Jackal build uses `/jackal/ground_truth` and `/jackal/polytopes`, accepts the same RViz goal
topic, and serializes 2-D `A`, `seed`, and `goal` arrays. Its visualization is lifted to
`VizHeight`; the wire-level message intentionally carries no dimension field.

## Launch

[`launch/corridor_generation.launch`](launch/corridor_generation.launch) is the system-level
entry point — it brings up Gazebo and the world, spawns the quadrotor, and starts
`nanovoxmap_node`, `corridor_planning_node`, `trajectory_server_node`, and RViz:

```sh
roslaunch corridor_planning corridor_generation.launch
```

The planar Jackal stack is launched with:

```sh
roslaunch corridor_planning corridor_generation_jackal.launch
```

These executables are build-time variants of the same sources. Configure with
`PATHCOVER_EXAMPLE=quadrotor` (3-D) or `PATHCOVER_EXAMPLE=jackal` (2-D); the root launcher scripts
set this automatically.

To fly a different world, point `world_name` in that launch file at it and update
`MapLowerBound` / `MapUpperBound` / `VoxelResolution` in
[`config/global_planning_drone.yaml`](config/global_planning_drone.yaml) to match. See the
[quadrotor_sim README](../quadrotor_sim/README.md) for where world files live.

## Library usage

RISP and PathCover need only Eigen and Qhull — no ROS, no catkin.

```cpp
#include "planner/PathCover.hpp"

// Workspace bounds, as half-spaces A_bound * y <= b_bound.
Eigen::Matrix<double, -1, 3> A_bound(6, 3);
Eigen::Matrix<double, -1, 1> b_bound(6);
A_bound << -Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Identity();
b_bound << 20.0, 10.0, 0.3, 20.0, 10.0, 2.2;   // -lower, then upper

// --- One polytope around a seed ---
Eigen::Matrix<double, -1, 3> A;
Eigen::Matrix<double, -1, 1> b;
std::vector<int> pts_remaining;                // N_m after each cut

PathCover::RISP<double, 3>(cloud, seed, A_bound, b_bound, A, b, pts_remaining,
                           /*offset=*/0.0, /*alpha=*/0.01);

// --- A corridor along a path ---
std::vector<Eigen::Matrix<double, -1, 3>> As;
std::vector<Eigen::Matrix<double, -1, 1>> bs;
std::vector<Eigen::Vector3d> seeds;            // one per polytope, plus the local target
std::vector<std::vector<int>> decay;

PathCover::pathCover<double, 3>(cloud, path, A_bound, b_bound, As, bs, seeds, decay,
                                /*offset=*/0.0, /*max_horizon=*/6, /*alpha=*/0.01);
```

Both take `cloud` and `path` as `std::vector<Eigen::Matrix<T, Dim, 1>>`. `offset` is the same
inward deflation as `DeflationFactor`; `pts_remaining` / `decay` record how many obstacle points
survive each cut, which is what the benchmark topics publish.

To feed your own downstream planner instead, subscribe to `/quadrotor/polytopes` — see the
[polytope_msgs README](../polytope_msgs/README.md) for the memory layout.

## Package layout

```
include/
  planner/
    PathCover.hpp        RISP + PathCover + redundancy removal (header-only, ROS-free)
    VoxelGrid.hpp        fixed-extent occupancy grid for the JPS/DMP search
  geometry/
    polytope_utils.h     H-representation <-> V-representation via Qhull (visualization)
  visual/
    polytope_publisher.hpp   corridor faces/edges as RViz markers
    visualizer.hpp           path, seeds, start/goal markers
  SubscribeAndPublish.hpp    ROS node: config struct, callbacks, planner thread
  eigen_conversions.h        PointCloud2 -> Eigen, fused with the radius filter
src/
  main.cpp                   node entry point
  SubscribeAndPublish.cpp    node implementation (pipeline above)
launch/
  corridor_generation.launch        quadrotor system launch
  corridor_generation_jackal.launch Jackal system launch
config/
  global_planning_drone.yaml  3-D quadrotor parameters
  global_planning_jackal.yaml 2-D Jackal parameters
```

## Dependencies

`roscpp`, `sensor_msgs`, `geometry_msgs`, `std_msgs`, `visualization_msgs`,
[`polytope_msgs`](../polytope_msgs), [`jps_lib`](../third_party/jps_lib),
[`qhull_lib`](../third_party/qhull_lib), Eigen, OpenCV.
