# jackal_description

Gazebo support for PathCover's differential-drive example. This package contains the Jackal
URDF/xacro model, meshes, wheel controllers, Velodyne and depth-camera descriptions,
ground-truth TF/odometry publisher, RViz configuration, and the imported example worlds.

The full PathCover stack is launched from the repository root:

```sh
./launch_sim_jackal.sh
```

That script configures the workspace with `PATHCOVER_EXAMPLE=jackal` and launches
`corridor_planning/corridor_generation_jackal.launch`. The default world is
`worlds/parking_lot.world`. Select another bundled world with:

```sh
source devel/setup.bash
roslaunch corridor_planning corridor_generation_jackal.launch \
  world_name:=$(rospack find jackal_description)/worlds/office.world
```

Send goals with RViz's **2D Nav Goal** tool. The planar trajectory server publishes directly to
`/jackal_velocity_controller/cmd_vel`; the imported `twist_mux.yaml` and interactive-marker
configuration are available for manual-control integrations but are not started by the full
PathCover launch.

## Package layout

```text
config/   wheel-controller, twist-mux, and RViz configuration
launch/   standalone Jackal mapping launch
meshes/   Jackal and sensor meshes
urdf/     robot and sensor xacros
worlds/   imported Gazebo worlds (parking_lot.world is the PathCover default)
include/  ground-truth TF/odometry publisher declaration
src/      ground-truth TF/odometry publisher implementation and entry point
```

## Runtime dependencies

The model uses `gazebo_ros`, `gazebo_ros_control`, `robot_state_publisher`, `xacro`,
`controller_manager`, `joint_state_controller`, `diff_drive_controller`, and the Gazebo Velodyne
plugin. The standalone `mapping.launch` additionally uses `twist_mux`,
`interactive_marker_twist_server`, `nanovoxmap`, and RViz.

## License and attribution

This imported package retains the BSD 3-Clause license and copyright notice from Clearpath
Robotics Inc.; see [`LICENSE`](LICENSE). The VLP-16 meshes retain Dataspeed Inc.'s BSD terms, and
the D435 mesh retains the Apache-2.0 terms of Intel's `realsense2_description`; both full notices
are stored at the repository root. PathCover's planner, mapper, and trajectory integration live
in their respective packages and are governed by the repository-root license and its third-party
notices.

The supplied archive does not contain more granular provenance records for individual world
files. Several worlds refer to Gazebo `model://` resources that are not bundled here; those
runtime-resolved models remain governed by their respective model-database licenses.
