# third_party

Vendored planning dependencies wrapped as ROS 2 `ament_cmake` packages. They retain their upstream licences; the repository's BSD 3-Clause grant does not replace those terms.

| Package | Upstream | Used for | Licence |
|---|---|---|---|
| [`jps_lib`](jps_lib) | [JPS3D](https://github.com/KumarRobotics/jps3d) | Jump Point Search and distance-map refinement for the global reference path | [BSD 3-Clause](jps_lib/LICENSE) |
| [`qhull_lib`](qhull_lib) | [Qhull](http://www.qhull.org/) | Convex hull operations used for half-space redundancy removal and visualization geometry | [Qhull licence](qhull_lib/include/libqhull_r/COPYING.txt) |

Both packages are built with the rest of the workspace by `colcon build`; no separate source build or system-wide installation is needed. Qhull's wrapper links to `libcdd`, so install `libcdd-dev` on non-container hosts.
