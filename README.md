# PathCover (ROS 2)

[![ROS 2](https://img.shields.io/badge/ROS-2%20Jazzy-22314E.svg)](https://github.com/kunalnk123690/PathCover/tree/ros2) [![ROS Noetic](https://img.shields.io/badge/ROS-Noetic-22314E.svg)](https://github.com/kunalnk123690/PathCover/tree/master) [![License: BSD 3-Clause](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](LICENSE)

<p align="center">
  <img src="media/simulation.webp" width="100%">
</p>

**PathCover** enables fast convex decomposition along a path, at sensor frequency, directly on point clouds. At its core is **RISP** (Randomized Iterative Space Partitioning), a fast geometric operation that replaces the optimization loop used by state-of-the-art conventional convex decomposition methods. Around it, this repository ships everything needed to run the algorithm in closed loop: a sparse voxel mapper, a global path search, a corridor-constrained trajectory optimizer, and a Gazebo Harmonic simulation for a quadrotor.

This is the **ROS 2 / Gazebo Harmonic** port of [PathCover (ROS Noetic)](https://github.com/kunalnk123690/PathCover), targeting **Ubuntu 24.04 with ROS 2 Jazzy**.

**Authors**: [Kunal Sanjay Narkhede](https://github.com/kunalnk123690), Abhijeet Mangesh Kulkarni, Guoquan Huang and Ioannis Poulakakis from the Department of Mechanical Engineering, University of Delaware.

Please give us a star and cite our paper if you use this project in your research:

```
@article{pathcover,
  title  = {\texttt{PathCover}: A Fast Convex Decomposition along a Path via Randomized Iterative Space Partitioning (\texttt{RISP}) on Point Clouds},
  author = {Narkhede, Kunal Sanjay and Kulkarni Mangesh, Abhijeet and  Guoquan, Huang and Poulakakis, Ioannis},
  journal = {arXiv:2608.05586},
  year   = {2026}
}
```

## Table of Contents

- [Quick Start](#1-quick-start)
- [Build without Docker](#2-build-without-docker)
- [Run](#3-run)
- [Packages](#4-packages)
- [Acknowledgements](#acknowledgements)
- [Licence](#licence)

## 1. Quick Start

Tested on Ubuntu 24.04 (ROS 2 Jazzy) with C++17. The fastest path is the bundled Docker image, which carries ROS 2 Jazzy desktop, Gazebo Harmonic, and the CUDA toolkit — three commands from a clone to a robot flying through a warehouse.

### Step 1 — build the Docker image

```bash
git clone https://github.com/kunalnk123690/PathCover.git --branch ros2
cd PathCover
./run_docker.sh
```

[`run_docker.sh`](run_docker.sh) builds the image and drops you into a shell at `/home/PathCover/PathCover_ws`, with this repository mounted there and X11 forwarded so Gazebo opens on your desktop. It probes for `nvidia-smi` **and** the NVIDIA container runtime, and requests a GPU only when both are present, so the same script works unchanged on a CPU-only machine. Set `INSTALL_CUDA=true|false` to force the toolkit in or out of the image regardless of the build host.

If you use VS Code, open the folder and *Reopen in Container* instead — [`.devcontainer/`](.devcontainer/) sets up the same mount and X11 forwarding, and declares `hostRequirements.gpu: optional` so it attaches a GPU when one exists and starts normally when it does not.

### Step 2 — Build and launch

Run from the workspace root, inside the container:

```bash
./launch_quadrotor.sh
```

This script runs `colcon build`, sources the workspace, and launches the full stack. The first build takes a few minutes; later runs only rebuild what changed.

### Step 3 — Send it a goal

Gazebo comes up with the warehouse world, the quadrotor, and the mapper already running. Pick a destination with the RViz **2D Goal Pose** tool (published on `/goal_pose`) and the robot plans and flies to it: the corridor is drawn in blue, the reference path in red, and the optimized trajectory in green, all regenerated from scratch on every scan.

## 2. Build without Docker

### Prerequisites

Ubuntu 24.04 with ROS 2 Jazzy. We use [**Eigen**](https://eigen.tuxfamily.org/) for linear algebra throughout, [**Qhull**](http://www.qhull.org/) (vendored) for convex hulls in the dual space, and **Gazebo Harmonic** with the `ros_gz` bridge for simulated sensors:

```bash
sudo apt install ros-jazzy-desktop ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge \
                 ros-jazzy-message-filters ros-jazzy-xacro \
                 libeigen3-dev libcdd-dev build-essential
```

### Build

```bash
git clone https://github.com/kunalnk123690/PathCover_ros2.git
cd PathCover_ros2
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### GPU acceleration (optional)

CUDA is **optional and affects `nanovoxmap` only** — corridor generation is CPU-only by design. If `nvcc` is on `PATH`, the GPU raycast + ESDF kernels are compiled in; otherwise the build falls back to CPU with no source changes. At runtime the node falls back again if no device is visible, so a GPU-enabled build still runs on a CPU-only host.

For installation of CUDA, please go to [CUDA Toolkit](https://developer.nvidia.com/cuda-toolkit).

## 3. Run

Build the workspace, source it, and launch the whole system:

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch corridor_planning corridor_generation.launch.py
```

The three commands above are exactly what [`launch_quadrotor.sh`](launch_quadrotor.sh) runs.

This brings up Gazebo Harmonic, the quadrotor, the mapper, corridor generation, the trajectory optimizer, and RViz. Trigger the planner using the **2D Goal Pose** tool. When a point is clicked in RViz, the corridor (blue), the reference path (red), and the optimized trajectory (green) are regenerated from scratch on every scan.

Related algorithms are detailed in [this paper](https://arxiv.org/abs/2608.05586).

For setup problems, like compilation errors caused by different versions of ROS/Eigen, please first refer to existing **issues**, **pull requests**, and **Google** before raising a new issue.

## 4. Packages

All algorithms, parameters, ROS interfaces, and instructions for using PathCover in your own project are documented in the per-package READMEs:

| Package | Contents |
|---------|----------|
| [**corridor_planning**](src/corridor_planning) | **The core contribution.** Header-only [RISP + PathCover](src/corridor_planning/include/planner/PathCover.hpp), the JPS/DMP front end, the ROS 2 node, and RViz visualization. The system-level launch file and the planning parameters live here. |
| [**nanovoxmap_ros**](src/nanovoxmap_ros) | Online mapping: an unbounded sparse log-odds voxel map with an incremental signed ESDF and an optional CUDA backend. This package is based on [NanoVoxMap ROS](https://github.com/kunalnk123690/nanovoxmapROS) and wraps the vendored [NanoVoxMap](src/nanovoxmap_ros/nanovoxmap) C++ library. |
| [**trajectory_server**](src/trajectory_server) | Receding-horizon trajectory optimizer. GCOPTER/MINCO over the corridor, with a generalized min-jerk (degree 5) / min-snap (degree 7) solver. |
| [**polytope_msgs**](src/polytope_msgs) | `Polytope` (`a`, `b`, `seed`) and `PolytopeArray` (corridor + goal) messages — the interface between corridor generation and any downstream planner. |
| [**quadrotor**](src/quadrotor) | The test vehicle: a URDF with a VLP-16 lidar, a geometric SE(3) controller Gazebo Harmonic plugin, ground-truth odometry, keyboard teleop, and the warehouse world it flies in. Contains four sub-packages: `quadrotor_description`, `quadrotor_gazebo_plugin`, `quadrotor_msgs`, and `quadrotor_teleop`. |
| [**third_party**](src/third_party) | [JPS3D + distance-map planner](src/third_party/jps_lib) for the global path search, and [Qhull](src/third_party/qhull_lib) for dual-space redundancy removal. |

To use PathCover in your own project, start with `corridor_planning` — RISP and PathCover are header-only, ROS-free, and templated on scalar type and dimension, needing only Eigen and Qhull. To keep the corridor generator but replace everything downstream, subscribe to `/quadrotor/polytopes` and see `polytope_msgs`.

## Acknowledgements

This project stands on a number of open-source works. Their licenses are retained in full at the paths listed below and govern those files; the repository's own BSD 3-Clause license does not replace their terms.

### Bundled code

| Component | Author | Used for | License |
|-----------|--------|----------|---------|
| [GCOPTER / MINCO](https://github.com/ZJU-FAST-Lab/GCOPTER) — `trajectory_server/include/gcopter/` | Zhepei Wang, Fei Gao (ZJU FAST Lab) | The corridor-constrained trajectory optimizer, and the L-BFGS, flatness, and geometry utilities around it | MIT — [`src/trajectory_server/LICENSE`](src/trajectory_server/LICENSE) |
| [JPS3D + distance-map planner](https://github.com/KumarRobotics/jps3d) — `third_party/jps_lib` | Sikang Liu (KumarRobotics) | The global path search feeding PathCover | BSD 3-Clause |
| [Qhull](http://www.qhull.org/) — `third_party/qhull_lib` | C. B. Barber, D. P. Dobkin, H. Huhdanpaa | Convex hulls in the dual space, for RISP's redundant-constraint removal | [Qhull License](http://www.qhull.org/COPYING.txt) |
| [NanoVoxMap](src/nanovoxmap_ros/nanovoxmap) | Kunal Sanjay Narkhede | Sparse voxel occupancy mapping with incremental ESDF | BSD 3-Clause |

### Runtime dependencies

Not vendored here; installed from apt and used through their public interfaces.

- **[ROS 2 Jazzy](https://docs.ros.org/en/jazzy/)** and **[Gazebo Harmonic](https://gazebosim.org/docs/harmonic/)** — the middleware and simulator the whole stack runs on.
- **[ros_gz](https://github.com/gazebosim/ros_gz)** — the ROS 2 ↔ Gazebo Harmonic bridge for sensor data and clock sync.
- **[Eigen](https://eigen.tuxfamily.org/)** — linear algebra throughout.
- **[CUDA Toolkit](https://developer.nvidia.com/cuda-toolkit)** — optional; `nanovoxmap`'s GPU raycast and ESDF kernels only.

## Licence

The source code is released under the [BSD 3-Clause License](LICENSE).

Copyright (c) 2026 Kunal Sanjay Narkhede, Department of Mechanical Engineering, University of Delaware.

Bundled third-party code and assets keep their own terms — see [Acknowledgements](#acknowledgements) above.
