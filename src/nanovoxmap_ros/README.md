# nanovoxmap_ros

ROS 2 wrapper for the vendored [NanoVoxMap](nanovoxmap) library. It synchronizes `sensor_msgs/msg/PointCloud2` with `nav_msgs/msg/Odometry`, incrementally builds a sparse occupancy map and signed ESDF, and publishes both as point clouds.

This package is derived from the standalone [NanoVoxMap ROS 2 wrapper](https://github.com/kunalnk123690/nanovoxmapROS). Use that repository when you want NanoVoxMap independently of PathCover.

## Run

Edit [`config/mapping.yaml`](config/mapping.yaml) for the sensor topics and frames, then run:

```bash
ros2 run nanovoxmap_ros nanovoxmap_ros_node --ros-args \
  --params-file "$(ros2 pkg prefix nanovoxmap_ros)/share/nanovoxmap_ros/config/mapping.yaml"
```

The PathCover launch starts this node automatically. In the supplied configuration it consumes `/quadrotor/velodyne/points` and `/quadrotor/velodyne/ground_truth`, publishes occupied voxels on `/nanovoxmap/Voxel_map`, and publishes the ESDF on `/nanovoxmap/esdf`.

CUDA acceleration is optional. The package builds and runs with a CPU fallback when CUDA or a compatible GPU is unavailable.

## Licence

BSD 3-Clause; see this package's [`LICENSE`](LICENSE) and the vendored library's [`LICENSE`](nanovoxmap/LICENSE).
