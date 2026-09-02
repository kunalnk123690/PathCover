# corridor_planning

The core PathCover package. It contains the ROS-independent, header-only RISP and PathCover implementation, the JPS/distance-map planning front end, the ROS 2 node, visualization helpers, parameters, and the full-system launch file.

## Run

From a built and sourced workspace:

```bash
ros2 launch corridor_planning corridor_generation.launch.py
```

The launch starts Gazebo Harmonic, the quadrotor simulation, `nanovoxmap_ros`, corridor generation, trajectory optimization, and RViz. Set a destination with RViz's **2D Goal Pose** tool on `/goal_pose`.

## Configuration and interface

Planning parameters are in [`config/global_planning.yaml`](config/global_planning.yaml). The important inputs are the occupied-voxel cloud (`MapTopic`) and robot odometry (`GroundTruthTopic`). The node publishes an ordered `polytope_msgs/PolytopeArray` corridor on `/quadrotor/polytopes`, together with RViz markers for the route, corridor, seeds, start, and goal.

`include/planner/PathCover.hpp` is templated on scalar type and dimension and does not depend on ROS. Its direct dependencies are Eigen and Qhull, so it can also be embedded in another project without the ROS 2 node.

## Licence

Original project code is BSD 3-Clause; see the [repository licence](../../LICENSE). Vendored dependencies retain their own licences.
