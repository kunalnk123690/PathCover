# nanovoxmap

A lightweight, real-time 3D voxel occupancy mapping ROS node. It fuses a point cloud
stream with odometry to build an unbounded, sparse log-odds occupancy map
(OctoMap-style probabilistic update), with an optional CUDA-accelerated insertion path
for high point-rate sensors.

## Features

- **Unbounded sparse block-hash** — space is split into 8×8×8 voxel blocks stored in a
  hash map; blocks are allocated only where rays have actually been observed, so memory
  grows with explored space rather than with a preconfigured map volume. No world
  bounds to set — only the voxel resolution.
- **Log-odds occupancy model** — each voxel stores a clamped log-odds value instead of
  a tri-state flag, so noisy/spurious returns are averaged out instead of flipping a
  cell outright. Tunable via `ProbHit` / `ProbMiss` / `ProbMin` / `ProbMax`.
- **Batched point-cloud raycasting** — whole scans are inserted in one
  `insertPointCloud()` call. Each ray is traversed with an integer 3D Bresenham walk,
  and updates are written into the block hash through a memoized block cursor, so a ray
  staying inside a block costs roughly one hash lookup per 8-voxel block crossing rather
  than one per voxel.
- **Incremental signed Euclidean distance field (ESDF)** — alongside occupancy the map
  maintains a *signed* distance field: positive in free space (metres to the nearest
  obstacle surface), negative inside obstacles (depth below that surface), saturating at
  a configurable truncation band. It lives in the same 8×8×8 block hash, so it is stored
  only where the band around a surface reaches. Updates are **incremental** — only the
  blocks whose occupancy actually changed, plus the band around them, are recomputed —
  and it is queried as a **continuous** field by trilinear interpolation with the
  analytic gradient of that same interpolant. See
  [Signed Euclidean distance field (ESDF)](#signed-euclidean-distance-field-esdf) below.
- **Optional CUDA backend** — if a CUDA compiler is found at build time, both point-cloud
  *insertion* and the *ESDF solve* run on the GPU. Insertion traces rays (one thread per
  ray) and sorts/reduces the resulting voxel updates on-device; the ESDF runs the same
  exact separable distance transform the CPU does, one thread per line. Both produce
  results identical to the CPU path (the ESDF field matches bit-for-bit — see
  [Tests](#tests)), and both fall back automatically to CPU if no device is available at
  runtime.
- **Free-space clearing from empty beams** — a beam that returns nothing still carries
  information: it left the sensor and hit nothing in range. Instead of dropping those
  beams, the node casts *miss-only* clearing rays out to `MaxRayLength` (free space
  carved along the ray, no occupied hit stamped at the clamped endpoint), so voxels a
  removed obstacle used to occupy get swept out instead of lingering as ghosts. Two
  sources feed it: over-range returns, and — for organized depth clouds with
  `DepthCameraInfoTopic` set — the NaN pixels of the depth image, whose rays are
  reconstructed from the camera intrinsics. See
  [No-return frustum clearing](#no-return-frustum-clearing) below.
- **Decoupled publishing** — the occupied-voxel map is published on a timer
  (`PublishRate`), independent of incoming sensor rate, so a slow publish/serialize
  step never throttles the sensor callback.
- **Approximate-time sync** — point cloud and odometry messages are paired with
  `message_filters::sync_policies::ApproximateTime`.
- **Sparse output** — only occupied voxel centroids are published, as a
  `sensor_msgs/PointCloud2` (`O(num_occupied)` instead of `O(total_voxels)`).

## Status

Where each piece actually stands, including the parts that are still limited:

| Component | Status |
|---|---|
| Sparse voxel storage | Unbounded 8×8×8 block hash; memory tracks explored space |
| CPU ray insertion | Single fused pass with a memoized block cursor. Good for moderate scans; still serial — parallelizing it needs per-thread sharded storage, which is not implemented |
| CUDA ray insertion | One thread per ray, updates sorted/reduced on-device. Wins on large, overlapping scans; identical result to CPU |
| Signed ESDF | Implemented. Sparse block-hashed, truncated, exact within the band |
| Incremental ESDF | Implemented. Dirty-block tracking; ~100× cheaper than a rebuild per scan on a large map |
| GPU ESDF | Implemented in [`src/nanovoxmap_cuda.cu`](src/nanovoxmap_cuda.cu). Exact separable transform, bit-identical to CPU, automatic fallback |
| Continuous surface SDF | Implemented. Trilinear interpolation with the analytic gradient; zero level set on the obstacle face, unit slope across it |
| Sparse nearest-obstacle queries (k-d tree) | Exact and untruncated, but rebuilt in full after any insert — suited to offline or infrequent-update use, **not** a per-frame loop |
| Sub-voxel surface reconstruction | Not implemented. The SDF is derived from occupancy, so surface accuracy is bounded by voxel resolution; this is not a TSDF (see [What this is not](#what-this-is-not)) |

## Nodes

### `nanovoxmap_node`

Subscribes to a synchronized point cloud + odometry pair, transforms each scan from
sensor frame to world frame using the odometry pose directly, raycasts it into the
occupancy grid, and periodically publishes the set of occupied voxel centroids. Beams
that returned nothing are re-cast as free-space clearing rays (see
[No-return frustum clearing](#no-return-frustum-clearing)).

#### Subscribed Topics

| Topic (param) | Type | Description |
|---|---|---|
| `CloudTopic` | `sensor_msgs/PointCloud2` | Sensor-frame point cloud (e.g. lidar/depth camera) |
| `OdometryTopic` | `nav_msgs/Odometry` | World-frame pose of the point cloud sensor itself (not the robot base) |
| `DepthCameraInfoTopic` | `sensor_msgs/CameraInfo` | Optional. Depth intrinsics used to reconstruct no-return clearing rays; not part of the cloud/odom sync (the first valid message is latched, the rest ignored) |

#### Published Topics

| Topic (param) | Type | Description |
|---|---|---|
| `OutputTopic` | `sensor_msgs/PointCloud2` | Occupied voxel centroids, frame `world` |
| `EsdfTopic` | `sensor_msgs/PointCloud2` | Optional. Signed distance field, frame `world`; `x`/`y`/`z` are voxel centres and the `intensity` field carries the **signed** distance in metres (negative inside obstacles) |

#### Parameters

Required:

| Parameter | Type | Description |
|---|---|---|
| `CloudTopic` | string | Input point cloud topic |
| `OdometryTopic` | string | Input odometry topic (sensor's pose in world frame) |
| `OutputTopic` | string | Output occupied-voxel topic |
| `VoxelResolution` | double | Voxel edge length, in metres |

Optional (defaults shown):

| Parameter | Default | Description |
|---|---|---|
| `PublishRate` | `5.0` | Map publish rate (Hz); `<= 0` disables timer-based publishing |
| `DownsampleCloud` | `true` | Voxel-downsample the input cloud at map resolution before raycasting |
| `SyncQueueSize` | `20` | Approximate-time synchronizer queue size |
| `SubQueueSize` | `5` | Per-topic subscriber queue size |
| `MaxRayLength` | `30.0` | Clip returns farther than this from the sensor, in metres; `<= 0` disables. Also the length of every clearing ray |
| `DepthCameraInfoTopic` | `""` (disabled) | Depth `CameraInfo` topic; enables no-return frustum clearing. Requires an *organized* depth cloud on `CloudTopic` whose dimensions match the `CameraInfo`. Leave empty for lidar |
| `ClearingRayStride` | `4` | Subsample factor for no-return clearing rays (`1` = every empty pixel). Raise if clearing costs too much, lower if clearing is too sparse |
| `ProbHit` | `0.7` | P(occupied \| hit) — log-odds nudge applied to a ray's endpoint |
| `ProbMiss` | `0.4` | P(occupied \| miss) — log-odds nudge applied to traversed (pass-through) voxels |
| `ProbMin` | `0.1192` | Log-odds clamp floor (most-free a voxel can become) |
| `ProbMax` | `0.971` | Log-odds clamp ceiling (most-occupied a voxel can become) |
| `EsdfTopic` | `""` (disabled) | Output topic for the signed distance field. Empty publishes nothing; the field is still available to C++ queries |
| `EsdfMaxDistance` | `1.0` | ESDF truncation distance, in metres. The single cost knob — see [Cost model](#cost-model) |
| `EsdfPublishRate` | `2.0` | ESDF publish rate (Hz), and the rate at which the incremental update runs; `<= 0` disables the timer |
| `EsdfPublishSlice` | `true` | Publish one horizontal slice instead of the whole 3D band (far fewer points, and the usual way to inspect it in RViz) |
| `EsdfSliceHeight` | `0.0` | World `z` of that slice, in metres |

See [`config/mapping_drone.yaml`](config/mapping_drone.yaml) for a working quadrotor example.

## Building

```sh
catkin_make
```

CUDA support is detected automatically (`check_language(CUDA)` in
[`CMakeLists.txt`](CMakeLists.txt)). If a CUDA compiler is available, the GPU paths
(`src/nanovoxmap_cuda.cu` — raycasting *and* the ESDF solve) are compiled in (guarded by
the `NANOVOXMAP_WITH_CUDA` define) and used at runtime; otherwise the build silently
falls back to CPU-only with no source changes required.

Note that the CUDA flags deliberately omit `--use_fast_math`: the raycast kernels are
pure integer arithmetic and gain nothing from it, while the ESDF solver's exactness
depends on IEEE division and `sqrt` — approximate intrinsics would let the GPU field
drift from the CPU one, which is the equivalence the runtime fallback relies on.

## Tests

`catkin_make` also builds `nanovoxmap_esdf_test`
([`test/esdf_test.cpp`](test/esdf_test.cpp)), a standalone binary (no ROS) that pins down
the three ESDF paths that have to agree — the CPU transform, the CUDA transform, and the
incremental scheduler. Each check is against something independent:

```sh
./devel/lib/nanovoxmap/nanovoxmap_esdf_test    # exit 0 = all checks passed
```

| Check | Against |
|---|---|
| Signed distances | Brute force over every occupied/free voxel centre |
| Incremental update | A full rebuild of the same scene |
| CUDA solver | The CPU solver (skipped in CPU-only builds) |
| Interpolated gradient | Central differences of the same interpolant |
| Zero level set | The known surface plane of an axis-aligned slab |

It also prints the incremental-vs-rebuild timings quoted above. On a Quadro RTX 5000 the
GPU and CPU fields come out bit-identical (max difference `0.000e+00`).

## Running

The node is started as part of the full stack, with
[`config/mapping_drone.yaml`](config/mapping_drone.yaml) loaded onto it:

```sh
roslaunch corridor_planning corridor_generation.launch
```

To run the mapper on its own against an existing cloud + odometry source:

```sh
rosrun nanovoxmap nanovoxmap_node _CloudTopic:=/quadrotor/velodyne/points \
                                 _OdometryTopic:=/quadrotor/velodyne/ground_truth \
                                 _OutputTopic:=/nanovoxmap/Voxel_map \
                                 _VoxelResolution:=0.05
```

## No-return frustum clearing

Surface returns only ever land on obstacles, so a scan's rays never sweep the open air
above the floor line. When an object is removed from the scene, the voxels it used to
occupy get no rays at all and linger as ghosts — nothing ever marks them free again.

The beams that *would* clear them are exactly the ones that come back empty. The node
turns them into **miss-only clearing rays**: free space is carved along the ray, but no
occupied hit is applied at the endpoint, so the clamped range never stamps a false
surface. Two sources feed the same mechanism:

1. **Over-range returns** — a return farther than `MaxRayLength` means no surface within
   range along that beam, so it is re-cast as a clearing ray to a clamped endpoint
   instead of being dropped. Always on whenever `MaxRayLength > 0`; works for lidar too.
2. **Empty depth pixels** — pixels that return nothing appear as NaN in an organized
   depth cloud and carry no direction of their own. Setting `DepthCameraInfoTopic` lets
   the node cache the depth intrinsics once (the first valid `CameraInfo`) and build a
   per-pixel unit-ray table, so each empty pixel's beam can be reconstructed and cast to
   `MaxRayLength`.

Neighbouring depth rays overlap heavily, so the empty-pixel fan is subsampled by
`ClearingRayStride` (default `4`) — a coarse fan clears the same volume for a fraction
of the cost.

```yaml
MaxRayLength:           5.0
DepthCameraInfoTopic:   "/jackal/realsense/depth/camera_info"   # omit/empty for lidar
ClearingRayStride:      4
```

Notes:

- Requires `MaxRayLength > 0` — a clearing ray has no length to clip to otherwise.
- The empty-pixel path needs an **organized** cloud (`height > 1`) whose `width`/`height`
  match the `CameraInfo`; it silently no-ops otherwise, and the over-range path keeps
  working. It reads the original organized cloud, so it is unaffected by
  `DownsampleCloud`.
- Frames: the ray table is built in the depth **optical** frame (x right, y down,
  z forward), the same frame the organized cloud's points live in.
- Clearing rays are inserted as a second `insertPointCloud(..., apply_hits=false)` call
  and skipped entirely when the batch is empty, so scans with nothing to clear pay
  nothing extra.
- Startup log line `frustum clear : <topic> (stride N)` reports the configured state;
  an info message confirms once intrinsics arrive.

## Library usage

The occupancy grid itself (`NanoVoxMap::OccupancyMap<Scalar>` in
[`include/nanovoxmap/nanovoxmap.hpp`](include/nanovoxmap/nanovoxmap.hpp)) is a standalone,
ROS-independent class — it can be used outside of the ROS node for offline mapping,
testing, or embedding in another pipeline. The CPU path is header-only; the optional
CUDA methods are defined out-of-line in [`src/nanovoxmap_cuda.cu`](src/nanovoxmap_cuda.cu)
and only linked in when built with `NANOVOXMAP_WITH_CUDA`. It supports `double`
(`OccupancyMapd`) and `float` (`OccupancyMapf`) precision via the `Scalar` template
parameter.

```cpp
#include "nanovoxmap/nanovoxmap.hpp"

NanoVoxMap::OccupancyMapd map(0.05);       // voxel resolution (m) — no bounds needed

map.insertPointCloud(origin, endpoints);   // batched raycast insert (origin + world points)

// Miss-only variant: carve free space along each ray, stamp no hit at the endpoint.
// This is what the node uses for no-return / over-range clearing beams.
map.insertPointCloud(origin, clearing_endpoints, /*apply_hits=*/false);

auto voxels = map.getOccupiedVoxels();     // sparse occupied centroids

// 2D column projection / full 3D grid as OccupancyGrid-style values (100/0/-1)
auto grid_2d = map.getMap2D();
auto grid_3d = map.getMap3D();
```

`insertRay()` / `setOccupied()` / `setFree()` are also available for single-ray and
manual edits. Thread-safety is the caller's responsibility (see the node's
`map_mutex_`).

## Signed Euclidean distance field (ESDF)

Alongside the occupancy grid, `OccupancyMap` maintains a **signed** Euclidean distance
field — the obstacle-distance field a gradient-based local planner (CHOMP/TEB-style
optimizers) consumes:

| | Value |
|---|---|
| Free space | **positive** — metres to the nearest obstacle surface |
| Inside an obstacle | **negative** — depth below the nearest surface |
| The surface | the **zero level set**, on the face between an occupied voxel and its free neighbour |
| Beyond the truncation band | saturates at `±EsdfMaxDistance` |

Being signed is what makes it usable when a trajectory (or a robot's collision volume)
has actually penetrated an obstacle: an unsigned field is flat at 0 everywhere inside,
so it tells an optimizer nothing about which way to escape, whereas the signed field
keeps a well-defined gradient pointing out.

### Representation

The field lives in the same 8×8×8 block hash as occupancy, so a block is materialized
only where the band around a surface reaches — storage scales with obstacle **surface
area × band**, not with explored volume. A block absent from the hash reads as `+band`.

Values are distances to the obstacle *surface*, not to a voxel centre. A surface voxel
reads `-0.5` voxels and its free neighbour `+0.5`, so the field crosses zero exactly at
the boundary and has unit slope across it — which is what an optimizer treating
`|∇d| = 1` relies on. (The k-d tree backend below answers the different question of
distance to the nearest occupied voxel *centre*, and so reads half a voxel larger.)

### Incremental updates

`updateEsdf()` recomputes only the blocks whose **occupancy** actually changed since the
last update, plus the band of cells around them whose nearest obstacle could have moved.
Truncation is what makes that dependency set finite — a voxel can only influence
distances within one band of itself — and so what turns an `O(map)` rebuild into
`O(touched region)`. A voxel whose log-odds moved without crossing the occupied
threshold changes no distance and marks nothing dirty.

Dirty blocks are grouped into boxes for the dense solve, and a scattered dirty set is
split by recursive median bisection so one edit at each end of the map does not produce
a single enormous box. Blocks that fall entirely outside the band are released, so
clearing an obstacle shrinks the field back rather than leaving a ghost well.

Measured on the bundled benchmark (12 k occupied voxels, 0.05 m, 0.5 m band): one scan
costs **1.1 ms** incrementally against **113 ms** for a full rebuild, and the gap widens
as the map grows.

### Continuous queries

`getSignedDistanceWithGradient()` trilinearly interpolates the voxel-centred field and
returns the **analytic gradient of that same interpolant**. Distance and gradient are
therefore mutually consistent — the gradient really is the derivative of the value being
returned, which finite differences over a stepwise field would not give. The result is
C⁰-continuous with no voxel-sized staircase.

```cpp
#include "nanovoxmap/nanovoxmap.hpp"

NanoVoxMap::OccupancyMapd map(0.05);
map.setEsdfMaxDistance(1.0);                 // truncation band, metres
map.insertPointCloud(origin, endpoints);

// Continuous signed distance + gradient. `d` is metres (negative inside an
// obstacle); `g` points towards increasing distance, i.e. away from the surface.
// Returns false if the point is non-finite or lies entirely outside the band,
// where the field has no gradient information to give.
double d;
Eigen::Vector3d g;
bool inside_band = map.getSignedDistanceWithGradient(query, d, g);

double dist = map.getSignedDistance(query);  // distance only
map.querySignedEsdf(points, dists, grads);   // batched: updates the field once

map.updateEsdf();                            // force the update now, not on first query
map.rebuildEsdf();                           // discard and recompute everything
```

Also available: `signedDistanceAt()` (voxel-centre value, no interpolation),
`fieldDistanceAt()` (unsigned clearance — the signed value clamped to 0 inside
obstacles), `forEachEsdfVoxel()` (visit the materialized band), and `numEsdfBlocks()` /
`esdfAllocatedBytes()`.

### Cost model

`EsdfMaxDistance` is the single knob, and it controls both halves:

- **Memory** — only the band around a surface is stored. Doubling the band roughly
  doubles the materialized shell.
- **Update cost** — a dirty box is grown by the band to get the write region, and by the
  band again to gather every obstacle that could become its nearest. So the solved volume
  grows as `(extent + 4 × band)³`. A band far larger than a planner actually reads is
  the most common way to make this slow.

Set it to the clearance your planner genuinely queries (1 m is a sane default), not to
the sensor range.

### Backends

The dense solve is an exact separable Felzenszwalb–Huttenlocher distance transform, run
twice per box — once seeded on occupied cells and once on free cells — which is where
the sign comes from. In CUDA builds it runs on the GPU (each of the three separable
passes is a set of independent 1-D transforms, so one thread per line needs no
synchronization); otherwise on the CPU. Jump flooding would be the more usual GPU choice
but is only approximate — keeping the exact algorithm is why the two backends agree
bit-for-bit and why falling back changes nothing but the timing.

### k-d tree backend (unbounded, unsigned)

Separately, `getDistance()` / `getDistanceWithGradient()` / `queryEsdf()` answer the
**unsigned, untruncated** distance to the nearest occupied voxel *centre*, from a
balanced k-d tree over the occupied set (`O(#occupied)` memory, typically logarithmic
queries). It has no truncation, so it is the right tool when you need a meaningful
distance arbitrarily far from any obstacle.

It is **not** incremental: the tree is rebuilt from scratch on the first query after any
insert. That is fine for offline use or infrequent updates, and unsuitable for a
per-frame real-time loop — use the block ESDF above for that.

### Using it from the ROS node

Set `EsdfTopic` to publish the field (see [Parameters](#parameters)); the incremental
update then runs on the ESDF timer, deliberately off the sensor callback so field
maintenance never adds integration latency. Colour by the `intensity` channel in RViz.

To query it directly instead, the node keeps its map in `map_` under `map_mutex_`:

```cpp
double d; Eigen::Vector3d g;
{
    std::lock_guard<std::mutex> lock(map_mutex_);
    map_->getSignedDistanceWithGradient(query_world, d, g);
}
```

### What this is not

The field is derived from the **occupancy grid**, so the surface it describes is the
boundary of the occupied voxel set. The zero level set is continuous and sits on the
voxel face rather than snapping to centres, but its accuracy is still bounded by the
voxel resolution — this is not a TSDF fusing per-ray depths into a sub-voxel surface
estimate. If you need sub-voxel surface reconstruction (meshing, photometric alignment),
that is a different pipeline.

## Package layout

```
include/
  nanovoxmap/
    nanovoxmap.hpp       OccupancyMap<Scalar>: ROS-independent log-odds voxel grid + signed ESDF
  NanoVoxMapNode.hpp     ROS node: parameters, subscriptions, publish timers
  eigen_conversions.h    sensor_msgs <-> Eigen point cloud conversions
src/
  main.cpp               node entry point
  NanoVoxMapNode.cpp     node implementation
  nanovoxmap_cuda.cu     optional CUDA backend: raycasting + signed ESDF (built if CUDA found)
test/
  esdf_test.cpp          standalone ESDF correctness + benchmark checks (no ROS)
config/
  mapping_drone.yaml     quadrotor parameter set
  mapping_jackal.yaml    Jackal parameter set
```
