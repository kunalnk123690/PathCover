# third_party

Vendored dependencies, wrapped as catkin packages so `catkin_make` builds them with the rest of
the workspace and no system-wide install is needed. Both are **unmodified upstream code** and
keep their own licences — the repository's BSD 3-Clause grant does not extend to them.

| Package | Upstream | Used for | Licence |
|---|---|---|---|
| [`jps_lib`](jps_lib) | [jps3d](https://github.com/KumarRobotics/jps3d), Sikang Liu (KumarRobotics) | The global path search that PathCover covers | BSD 3-Clause — [`LICENSE`](jps_lib/LICENSE) |
| [`qhull_lib`](qhull_lib) | [Qhull](http://www.qhull.org/), C. B. Barber, D. P. Dobkin, H. Huhdanpaa | Convex hulls in the dual space | [Qhull licence](http://www.qhull.org/COPYING.txt) |

## `jps_lib`

Builds one shared library, `jps_lib`, from two planners:

- **JPS** (`jps_planner/`) — Jump Point Search over the occupancy grid.
  `corridor_planning` calls it first to get a collision-free grid path.
- **DMP** (`distance_map_planner/`) — a distance-map planner that refines the raw JPS path,
  pushing it away from obstacles. Its `setPotentialRadius` / `setSearchRadius` are what
  `DmPotentialRadius` / `DmPSearchRadius` in
  [`global_planning_drone.yaml`](../corridor_planning/config/global_planning_drone.yaml) configure.

The refined path is the input to PathCover — corridor quality depends on it, since PathCover
covers whatever path it is handed. `jps_basis/` supplies the shared `Vec3f` / `vec_Vecf` types
and `jps_collision/map_util.h` the grid wrapper (`VoxelMapUtil`) that the node fills each cycle.

Only the planning headers are used; the upstream package's tests, examples, and yaml map loaders
are not included.

## `qhull_lib`

Qhull's reentrant C library (`libqhull_r/`) plus its C++ wrapper (`libqhullcpp/`), built as
`qhull_lib`. Two call sites:

- **Redundancy removal** in [`PathCover.hpp`](../corridor_planning/include/planner/PathCover.hpp)
  — RISP emits one half-space per sampled point, most of which are redundant. Mapping each
  constraint to a dual point $d_m = a_m / (b_m - a_m^\top y_{\text{seed}})$ turns "is this
  half-space redundant?" into "is this dual point interior to the hull?", so a single convex hull
  call keeps exactly the non-redundant constraints. This is on the critical path of every RISP
  invocation.
- **H → V conversion** in
  [`polytope_utils.h`](../corridor_planning/include/geometry/polytope_utils.h) — the same dual
  construction, run the other way, to get the vertices needed to draw a polytope in RViz.

The `qhull_lib` target links against `cdd` (`libcdd-dev`), so that package must be installed even
though Qhull itself is vendored.

## Updating

These are dropped in verbatim; the only local file in each is the catkin `CMakeLists.txt` and
`package.xml`. To move to a newer upstream release, replace `include/` and `src/` and leave those
two files alone.
