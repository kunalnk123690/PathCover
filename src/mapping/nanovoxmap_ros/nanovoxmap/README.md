# NanoVoxMap

A small, ROS-independent, sparse block-hashed 3D occupancy-map library
(`NanoVoxMap::OccupancyMap`). Space is split into 8x8x8 voxel blocks that are
allocated only where observations or manual voxel edits occur, so the map has
no preconfigured spatial bounds and its memory grows with touched space rather
than the volume of a fixed bounding box. It accumulates quantized probabilistic
occupancy evidence (OctoMap-style
log-odds) from rays and batched point clouds, offers sparse retrieval of the
occupied set, and derives a Euclidean distance field (ESDF) with trilinear
distance + gradient queries for gradient-based planning. The CPU path is
header-only; an optional CUDA backend accelerates batched point-cloud inserts
and ESDF construction. Distance and gradient queries run on the CPU.

The authoritative map always remains in host memory, so a CUDA build can fall
back immediately without downloading or reconstructing state. Blocks live in a
stable paged pool; the hash table stores compact handles, and every block keeps
a 512-bit occupied mask. CUDA performs DDA event generation and reduction, then
applies block-grouped updates in parallel on the host.

## Layout

```
include/nanovoxmap/nanovoxmap.hpp  core header-only library
src/nanovoxmap_cuda.cu             optional CUDA backend (built when CUDA support is enabled)
test/nanovoxmap_example.cpp        PCL-based example (load cloud, build map, visualize)
test/core_test.cpp                  ROS-independent clearing regression tests
data/cloud.ply                     sample point cloud
cmake/                             package-config + uninstall templates
Doxyfile                           Doxygen configuration for the API reference
```

## Dependencies

- **Eigen3** (required) — core math.
- **CUDA Toolkit** (optional) — CUDA support defaults on when CMake finds a CUDA
  compiler and accelerates point-cloud insertion and ESDF construction. Toggle
  it with `-DNANOVOXMAP_WITH_CUDA=ON/OFF`.
- **PCL** (>= 1.8, example only) — point-cloud loading and visualization.
- **Doxygen** (optional, docs only) — generates the API reference; Graphviz
  (`dot`) is an optional extra for diagrams.

## Build

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

The core, its lightweight tests, and the PCL example build by default. For a
core-only build without PCL, configure with `-DBUILD_EXAMPLES=OFF`.

## Install (globally)

```bash
sudo cmake --install build            # default prefix /usr/local
# or a custom prefix:
cmake --install build --prefix /your/prefix
```

This installs the header, the library (when CUDA is enabled), and a CMake
package config so downstream projects can:

```cmake
find_package(nanovoxmap REQUIRED)
target_link_libraries(my_app PRIVATE nanovoxmap::nanovoxmap)
```

Uninstall with `cmake --build build --target uninstall`.

## Example

A ~60-line demo ([test/nanovoxmap_example.cpp](test/nanovoxmap_example.cpp)):
load a cloud with PCL, voxelize it into a sparse occupancy map at `0.05 m`
resolution, and visualize the occupied voxels (coloured by height). The map is
not clipped to the demo's `[-6, -6, 0] → [6, 6, 4]` reference frame; that box
is used only for the height-colour range and the viewer wireframe.

```bash
./build/nanovoxmap_example [cloud.ply]
```

For live sensor data you would instead call `map.insertPointCloud(origin, pts)`,
which ray-casts each return to carve free space; the demo uses `setOccupied()`
because a dense reconstructed cloud has no single sensor origin.

Backend selection is explicit when needed and automatic by default:

```cpp
using NanoVoxMap::Backend;
NanoVoxMap::OccupancyMapd automatic(0.05);  // CUDA when usable, CPU fallback
NanoVoxMap::OccupancyMapd cpu(0.05, 0.7, 0.4, 0.1192, 0.971,
                              Backend::kCpu);
NanoVoxMap::OccupancyMapd strict_cuda(0.05, 0.7, 0.4, 0.1192, 0.971,
                                      Backend::kCuda, false);
```


## Euclidean distance field (ESDF)

`OccupancyMap` offers two distance-field backends over the same occupied set.

### Signed, truncated block ESDF (recommended)

Lives in the same 8x8x8 block layout as occupancy. Free-space blocks are
materialized only within the truncation band around a surface, while occupied
interior blocks remain materialized to preserve negative distances. Storage
therefore scales with the surface band plus the volume of solid interiors,
rather than with the full explored volume. The implementation computes exact
Euclidean distances between voxel centres using the separable distance
transform of Felzenszwalb & Huttenlocher (three 1-D parabola lower-envelope
passes), then applies a half-voxel correction to approximate distance to the
occupancy-defined surface. Samples are positive in free voxels and negative in
occupied voxels; trilinear interpolation places the zero crossing at their
shared face.

Updates are incremental: only the blocks whose occupancy changed, plus the
band of cells around them whose nearest obstacle could have moved, are
recomputed, so a per-frame update costs O(touched region) rather than O(map).
`setEsdfMaxDistance()` bounds the free-space band and update radius, but not the
storage needed for occupied interiors. The requested default is 1 m, rounded up
to a whole number of voxels.

```cpp
map.insertPointCloud(origin, pts);
map.setEsdfMaxDistance(1.5);             // truncation band, in metres (optional)
map.updateEsdf();                        // optional: patch now instead of on first query

double d = map.getSignedDistance(p);     // metres, negative in occupied regions

Eigen::Vector3d grad;
bool in_band = map.getSignedDistanceWithGradient(p, d, grad);

std::vector<double> ds;                  // batched: field brought up to date once
std::vector<Eigen::Vector3d> gs;
map.querySignedEsdf(points, ds, gs);

double clearance = map.fieldDistanceAt(p);  // containing-voxel sample, clamped to >= 0
```

When CUDA is enabled and still available, each dirty region's dense solve is
attempted on the GPU first (`esdfUsesGpu()`). A region that exceeds the GPU
memory budget uses the CPU solver without disabling later GPU attempts; a
device or driver failure switches subsequent solves to CPU as well.
`setEsdfForceCpu(true)` pins the CPU solver for A/B testing.

### Unsigned k-d tree ESDF (exact nearest-voxel)

The original backend: a sparse balanced k-d tree over occupied voxel centres,
rebuilt on demand and queried for the exact nearest occupied voxel centre. It
has no configured spatial bounds or distance truncation, but also no sign or
incremental update.

```cpp
double d = map.getDistance(p);           // metres to nearest occupied voxel centre

Eigen::Vector3d grad;
map.getDistanceWithGradient(p, d, grad);

std::vector<double> ds;
std::vector<Eigen::Vector3d> gs;
map.queryEsdf(points, ds, gs);           // computeEsdf() rebuilds the tree lazily first
```

## Documentation

The public and internal API are documented inline in Doxygen format. Generate
the HTML reference into `docs/html` with either:

```bash
doxygen Doxyfile                                   # from the repo root
# or via CMake:
cmake -S . -B build -DBUILD_DOCS=ON
cmake --build build --target docs
```

Then open `docs/html/index.html`. The Doxyfile defines `NANOVOXMAP_WITH_CUDA`
so the CUDA-only members are documented too; set `HAVE_DOT = NO` in the Doxyfile
if Graphviz is not installed.

## License

NanoVoxMap is distributed under the [BSD 3-Clause License](LICENSE). Copyright
(c) 2026 Kunal Sanjay Narkhede.
