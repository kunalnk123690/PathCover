# quadrotor

ROS 2 and Gazebo Harmonic test vehicle for the PathCover demo. These packages are not required by the core corridor algorithm and can be replaced by another robot or real sensor/odometry sources.

| Package | Purpose |
|---|---|
| `quadrotor_description` | URDF, meshes, warehouse world, RViz configuration, sensor setup, and ground-truth odometry helper |
| `quadrotor_gazebo_plugin` | Gazebo Harmonic system plugin with the geometric SE(3) tracking controller |
| `quadrotor_msgs` | `TrajectoryCommand`, the optimizer-to-controller setpoint interface |
| `quadrotor_teleop` | Keyboard teleoperation utility |

Run the complete demonstration from the repository root with:

```bash
./launch_quadrotor.sh
```

The description launch file can be used for simulation-focused development:

```bash
ros2 launch quadrotor_description quadrotor_gazebo_teleop.launch.py
```

The default world is `quadrotor_description/worlds/warehouse.world`. If it is changed, update the search bounds in `../corridor_planning/config/global_planning.yaml` as well.

## Licence

Original project code is BSD 3-Clause; see the [repository licence](../../LICENSE). Third-party meshes and other assets retain their original terms.
