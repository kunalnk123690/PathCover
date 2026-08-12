# quadrotor_sim

The test vehicle and its environment. Nothing here is required by PathCover — it exists so the
corridor generator and the optimizer can be exercised in closed loop against a simulated LiDAR.
Swap it for a real robot (or a different simulator) and the rest of the stack is unchanged.

Three catkin packages:

| Package | Contents |
|---|---|
| [`quadrotor_description`](quadrotor_description) | URDF/xacro robot and sensor descriptions, meshes, the Gazebo world, the RViz config, and the ground-truth TF/odometry node |
| [`quadrotor_gazebo_plugin`](quadrotor_gazebo_plugin) | Gazebo model plugin: an SE(3) geometric tracking controller that turns a trajectory setpoint into a wrench on the base link |
| [`quadrotor_msgs`](quadrotor_msgs) | `TrajectoryCommand` — the setpoint message between the optimizer and the controller |

## The robot

[`urdf/quadrotor.xacro`](quadrotor_description/urdf/quadrotor.xacro): a 0.716 kg F250-class
airframe (`ixx = iyy = 0.007`, `izz = 0.012`) with three Gazebo attachments —

- **`libgazebo_ros_p3d.so`** — ground-truth pose/twist of `base_link` in the `world` frame at
  500 Hz on `/quadrotor/ground_truth`. No noise. This is what both the planner and the optimizer
  use as odometry, so state estimation is deliberately out of the loop.
- **`libquadrotor_gazebo_plugin.so`** — the geometric controller (below).
- **`lidar.xacro`** — a VLP-16 (`libgazebo_ros_velodyne_laser.so`) on `lidar_link`, 0.25 m above
  the airframe: 440 × 16 beams over a full 360° azimuth and ±15° elevation, 0.1–100 m range,
  1 mm Gaussian noise, **5 Hz**. That 5 Hz is the replanning rate of the whole system —
  corridor generation is driven by the map, and the map is driven by this scan.

`depth_camera.xacro`, `imu.xacro`, and `monocular_camera.xacro` ship alongside and are
**commented out** in `quadrotor.xacro`. Re-enabling the depth camera also enables `nanovoxmap`'s
no-return frustum clearing (it needs an organized cloud plus `CameraInfo`); re-enabling the IMU
requires `hector_gazebo_plugins`.

### Controller gains

Set as SDF elements on the plugin, so they live in the xacro rather than a yaml:

| Element | Value | Meaning |
|---|---|---|
| `KP` | `14.5 14.5 14.5` | Position error gain |
| `KD` | `5.8 5.8 5.8` | Velocity error gain |
| `KR` | `2.2 2.2 1.7` | Attitude error gain |
| `KW` | `0.21 0.21 0.25` | Angular-velocity error gain |
| `wrenchFrame` | `base_link` | Link the computed force/torque is applied to |
| `trajectoryTopic` | `/quadrotor/trajectory` | Setpoint input |

These are the aggressive set; a gentler set (`KP 6.4`, `KD 4.3`, `KR 1.0 1.0 0.77`,
`KW 0.15 0.15 0.19`) is commented out directly above them and is the one to fall back to if the
vehicle oscillates.

## Geometric controller

[`GeometricController.hpp`](quadrotor_gazebo_plugin/include/GeometricController.hpp) implements
the SE(3) tracking controller of Lee, Leok & McClamroch: a PD law on position error produces a
desired total force, the direction of that force plus the desired yaw fixes the desired attitude
$R_d$, and an attitude law produces body torques. Working directly on SE(3) avoids the
singularities of an Euler-angle parameterization, which matters at the tilt angles the optimizer
is allowed to command (`theta_max` defaults to 45°).

The plugin runs its ROS callbacks on a private callback queue in a separate thread, so message
handling never blocks Gazebo's physics update.

Mass and inertia are read from the Gazebo model, not from a parameter file — but
`trajectory_server`'s `vehicle_mass` is separate and must be kept in sync with the URDF
(`0.716` kg) or the optimizer will plan against the wrong flatness map.

## `quadrotor_ground_truth_node`

Consumes `/quadrotor/ground_truth` and republishes it at the frames each consumer needs:

| Direction | Topic | Type |
|---|---|---|
| sub | `/quadrotor/ground_truth` | `nav_msgs/Odometry` (from the p3d plugin) |
| pub | `/quadrotor/velodyne/ground_truth` | `nav_msgs/Odometry` — pose of `lidar_link` in `world` |
| pub | `/quadrotor/realsense/ground_truth` | `nav_msgs/Odometry` — pose of `front_realsense_optical_link` in `world` |
| pub | TF `world → base_link` | `tf` broadcast |

This exists because `nanovoxmap` wants the pose of the **sensor**, not of the robot base: it
transforms each scan to the world frame with the odometry pose directly. The sensor offsets are
hardcoded here (lidar at `+0.2 z`; the RealSense chain at `+0.1 x, +0.095 z` with the standard
optical-frame rotation), so moving a sensor in the xacro means updating
[`ground_truth.cpp`](quadrotor_description/src/ground_truth.cpp) to match.

## `TrajectoryCommand`

```
Header header
geometry_msgs/Point position
geometry_msgs/Point velocity
geometry_msgs/Point acceleration
geometry_msgs/Point jerk
geometry_msgs/Point snap
float64 yaw
float64 yaw_velocity
float64 yaw_acceleration
```

A full differentially-flat setpoint. The optimizer fills position through jerk (snap is left at
zero, and `yaw_acceleration` with it); the controller uses position/velocity/acceleration and yaw
for tracking, with the higher derivatives available as feedforward terms.

## The world

[`worlds/floorplan1/floorplan1_static.world`](quadrotor_description/worlds/floorplan1) — a static
40 × 20 m office-like space of walls and box obstacles forming narrow passages and dead ends.
From [uav_simulator](https://github.com/Zhefan-Xu/uav_simulator) by Zhefan Xu, MIT-licensed; see
[`LICENSE.uav_simulator`](quadrotor_description/worlds/floorplan1/LICENSE.uav_simulator).

To fly a different one:

1. Drop the `.world` file in [`worlds/`](quadrotor_description/worlds).
2. Point `world_name` in
   [`corridor_generation.launch`](../corridor_planning/launch/corridor_generation.launch) at it.
3. Update `MapLowerBound` / `MapUpperBound` / `VoxelResolution` in
   [`global_planning_drone.yaml`](../corridor_planning/config/global_planning_drone.yaml) — the search grid
   and the workspace bounds appended to every polytope are per-world.

`nanovoxmap` needs no world-specific change: its map is an unbounded sparse block hash.

## Running the simulation alone

[`run_simulation.launch`](quadrotor_description/launch/run_simulation.launch) brings up Gazebo,
the robot, the ground-truth node, and RViz — no mapping, no planning:

```sh
roslaunch quadrotor_description run_simulation.launch
```

Useful for checking the vehicle, the sensor, or a new world in isolation. The full stack is
launched from [`corridor_planning`](../corridor_planning) instead.

## Package layout

```
quadrotor_description/
  urdf/          quadrotor.xacro (+ lidar, and the commented-out depth/IMU/mono sensors)
  meshes/        f250.dae airframe, VLP16_*.dae lidar, d435.dae depth camera
  worlds/        floorplan1_static.world
  config/        quadrotor_visual.rviz — the RViz layout the launch files open
  include/src/   quadrotor_ground_truth_node
  launch/        run_simulation.launch
quadrotor_gazebo_plugin/
  include/       GeometricController.hpp (SE(3) control law), plugin header
  src/           plugin implementation
quadrotor_msgs/
  msg/           TrajectoryCommand.msg
```

## Dependencies

`roscpp`, `gazebo_ros`, `gazebo_msgs`, `nav_msgs`, `geometry_msgs`, `sensor_msgs`, `tf`, Eigen,
and `velodyne_simulator` for the VLP-16 plugin.

## Acknowledgements

The world, VLP-16 meshes, and D435 mesh are third-party; everything else here — the URDF/xacro
descriptions, the controller plugin, the ground-truth node, and the RViz configuration — is
original work under this repository's licence. Provenance and licences are listed in the
[top-level README](../../README.md#acknowledgements).
