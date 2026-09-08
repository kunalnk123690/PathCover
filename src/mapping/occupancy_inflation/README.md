# occupancy_inflation

Obstacle inflation for online planning.

The node subscribes to a `sensor_msgs/PointCloud2`, snaps the points onto a
voxel lattice, dilates the occupied voxels by a configurable radius and shape,
and republishes the result as another `PointCloud2`. The output is the map a
planner should search: a point in its free space is a pose the vehicle fits in,
not just a point that misses an obstacle.

It can accumulate the map across messages or rebuild it from each one, run the
dilation on the GPU when there is one, and publish only the boundary shell of
the inflated map to keep messages small.

## Build and run

```bash
catkin_make
source devel/setup.bash
roslaunch occupancy_inflation occupancy_inflation.launch \
  config:=$(rospack find occupancy_inflation)/config/inflation_params_jackal.yaml
```

Pass `config:` explicitly. The launch file's own default names a
`config/inflation.yaml` that the package does not ship, so the bare
`roslaunch occupancy_inflation occupancy_inflation.launch` fails to resolve it.

Individual keys can still be overridden after the `rosparam` load, e.g.
`<param name="radius_xy" value="0.35" />`.

Nothing in the simulation depends on this launch file — it is for tuning the
inflation against a live cloud on its own. In the simulation the node is brought
up by the per-robot launch files in `corridor_planning/launch/`, each loading
its own config.

## Topics and services

| Direction | Default topic                         | Type                               |
|-----------|---------------------------------------|------------------------------------|
| in        | `/nanovoxmap/Voxel_map`               | `sensor_msgs/PointCloud2`          |
| out       | `/occupancy_inflation/inflated_cloud` | `sensor_msgs/PointCloud2` (`XYZI`) |
| out       | `/occupancy_inflation/seed_cloud`     | `sensor_msgs/PointCloud2` (`XYZI`) |
| service   | `~reset`                              | `std_srvs/Empty`                   |

Both output clouds carry one point per occupied voxel, at the voxel centre.

In `inflated_cloud`, **intensity marks where the point came from**:

- `0` — the voxel held at least one input point
- `1` — the voxel exists only because of the inflation

Colour by `intensity` in rviz to see exactly what the inflation added. The
optional `seed_cloud` is the voxelised input with no inflation at all, useful as
an A/B overlay. Set `publish_intensity: false` to drop the field once you have
stopped looking, which takes a point from 16 bytes to 12.

`~reset` clears an accumulated map. The map also clears itself if the incoming
cloud changes `frame_id`, because voxels accumulated in one frame mean nothing
in another.

## Configuration

The package ships two configs, both loaded into the node's private namespace so
every key is read as `~<key>` (e.g. `~radius_xy`, `~input_filter/min`):

- [`config/inflation_params_drone.yaml`](config/inflation_params_drone.yaml) —
  volumetric inflation, for the quadrotor
- [`config/inflation_params_jackal.yaml`](config/inflation_params_jackal.yaml) —
  planar inflation, for the ground robot

They carry the same keys and differ in exactly three values: `radius_xy`,
`radius_z` and the `input_filter` floor. Everything below applies to both.

| Parameter | Meaning |
|---|---|
| `input_topic` / `output_topic` / `seed_topic` | topic names |
| `publish_seed_cloud` | also publish the un-inflated voxelised cloud |
| `publish_intensity` | write the intensity field; off makes points 12 bytes |
| `publish_on_change_only` | skip publishing a map identical to the last one |
| `publish_rate_hz` | publish on a timer at this rate instead of once per input cloud; `0` keeps one output per input |
| `frame_id` | frame for the output; empty reuses the input cloud's frame |
| `queue_size`, `latch` | standard ROS pub/sub plumbing |
| `max_rate_hz` | cap on how often incoming clouds are processed; `0` disables throttling |
| `verbose` | log voxel counts and timing per cloud |
| `resolution` | voxel edge length in metres |
| `radius_xy` | inflation radius in x and y, metres |
| `radius_z` | vertical inflation half-extent, metres; `0.0` inflates only in xy |
| `shape` | `box` or `ellipsoid` |
| `origin` | origin of the voxel lattice |
| `publish_surface_only` | publish the boundary shell of the inflated map, not its solid interior |
| `surface_thickness` | thickness of that shell in voxels |
| `incremental` | accumulate across messages, or rebuild from each one |
| `num_threads` | threads for voxelising, dilating and serialising; `0` means all |
| `use_cuda` | dilate on the GPU when one is available; falls back to the CPU silently |
| `cuda_min_grid_voxels` | dilations smaller than this stay on the CPU; `0` sends everything to the device |
| `max_grid_mib` | ceiling on one occupancy grid |
| `input_filter/{enable,min,max}` | drop input points outside this box before voxelising |
| `output_bounds/{enable,min,max}` | drop inflated voxels whose centre falls outside this box |

### Sizing the radii

`radius_xy` and `radius_z` are metres and should come from the vehicle, not from
a guess. Two things make that harder than it looks.

**The clearance you get is not the radius you ask for.** The kernel keeps only
offsets whose voxel *centre* falls inside the shape, so what it actually
guarantees is the distance from the seed to the nearest voxel the kernel left
out — always short of the radius requested.

**That shortfall steps rather than slides.** The kernel is a set of integer
offsets, so a whole range of radii produce an identical kernel and therefore an
identical clearance. Measured at `resolution: 0.1`:

| `radius_xy` | planar kernel cells | realised clearance |
|---|---|---|
| 0.30 – 0.31 | 25 – 29 | 0.250 – 0.255 |
| 0.32 – 0.36 | 37 | 0.292 |
| 0.37 – 0.39 | 45 | 0.350 |
| 0.40 – 0.42 | 49 – 57 | 0.354 |
| 0.43 – 0.44 | 61 | 0.381 |
| 0.45 – 0.46 | 69 | 0.430 |

Asking for 0.36 and asking for 0.32 buy the same map, and neither clears a
0.31 m vehicle. Check a candidate against this table rather than assuming it
rounds the way you expect, and prefer a value mid-plateau: a later tweak to the
radius then cannot silently drop the kernel a step.

The two shipped values, both derived in the `base_link` frame the planner
tracks:

- **quadrotor — `0.45` / `0.25`**, realising 0.430 m in the plane and 0.250 m
  vertically. The horizontal driver is the prop-tip radius,
  `sqrt(0.268² + 0.268²) = 0.379 m`; the vertical one is the lidar mount
  reaching `z = +0.203`, both measured off the collision mesh.
- **Jackal — `0.38` / `0.0`**, realising 0.350 m in the plane. The driver is the
  wheels, not the chassis box: `wheelbase/2 + wheel_radius = 0.229 m` in x and
  `track/2 + wheel_width/2 = 0.208 m` in y circumscribe to 0.309 m, against the
  0.261 m the 0.420 × 0.310 m chassis alone would suggest. 0.38 is the smallest
  value on the table that clears it — the intuitive "footprint plus a bit" of
  0.35 lands on the 0.292 plateau and leaves the robot's own corners outside the
  inflated set.

Re-derive both if the vehicle geometry or the resolution changes.

### Planar and volumetric use

There is no 2-D build of this package and no dimension switch. The grid, the
input cloud and the output cloud are always three-dimensional, and one binary
serves a ground robot and a flying one — unlike `corridor_planning` and
`trajectory_server`, which are compiled per-robot. What changes is `radius_z`.

`radius_z: 0.0` degenerates the structuring element to a disc one voxel thick:
the kernel test treats a zero radius as an axis that must not be crossed at all,
so each z layer is dilated within its own plane and none bleeds into another.
This is a supported configuration, not a happy accident — a single isolated seed
at `radius_xy: 0.38, resolution: 0.1` produces exactly 45 voxels, the planar
disc's cell count, against 121 for the same radius at `radius_z: 0.2`. Both
backends agree, and `radius_z` is nowhere divided by.

**A planar kernel does not flatten the map.** Every z layer of the input
survives, each carrying its own dilated outline; what goes away is the vertical
smearing between layers. A 2.8 m wall still publishes about 28 of them. So if
the consumer only reads a height band — `corridor_planning` slices the cloud and
projects that band down to the plane — cut the rest away with `input_filter`
rather than paying to dilate and serialise layers that are discarded on arrival.

The one shape the package cannot express is an anisotropic footprint. The kernel
is built from `{radius_xy, radius_xy, radius_z}`, so the plane is always
isotropic, and `box` is square in plan rather than rectangular. A robot that can
rotate in place is described honestly by its circumscribed radius, so this costs
the Jackal nothing; a car-like platform that cannot spin wants an oriented
footprint and has to get that from a check downstream.

### `incremental` is the one to get right

`true` suits a mapper whose map only grows, such as an accumulating global
cloud: each message pays only for what it adds, and a message that repeats what
the map already holds is not republished at all.

`false` suits a sliding local window, where voxels also disappear. Every message
is inflated from scratch, so the map can shrink as well as grow. In this mode
the node still skips republishing when a message produces the same map.

An accumulated map only grows, so bound it with `output_bounds` (or call
`~reset`) if the source can wander. `max_grid_mib` is the backstop: it turns a
stray point far from the map into an error instead of an allocation.

### The two filters do different jobs

`input_filter` runs **before** voxelisation and decides which points become
obstacles. It is the cheapest place to drop anything the consumer will not read,
because a point rejected here is never dilated and never serialised. Both
shipped configs use it to reject the ground: `min.z = 0.0` for the quadrotor,
and `min.z = 0.05` for the Jackal, whose lidar sits 0.366 m up with a 0.01 m
noise sigma — five sigma clear of the ground returns. Without it every ground
return inflates into a disc and the drivable surface itself reads as occupied.

`output_bounds` runs **after** dilation and decides which inflated voxels
survive. It is what stops inflation from leaking past the edge of a bounded map,
and it is off by default. With a volumetric kernel that leak shows up
immediately: seeds just above the ground inflate downwards, so you will see
inflated voxels slightly below `z = 0` until you enable it. With `radius_z: 0.0`
there is no vertical component to leak, so the planar configs do not need it.

### `box` vs `ellipsoid`

`box` dilates by a cuboid. `ellipsoid` keeps only the offsets inside the
ellipsoid spanned by the radii, which is a closer approximation of a real
clearance envelope and produces far fewer voxels to publish — a vehicle that is
round in plan gets no benefit from the corners a box pushes out to
`sqrt(2)·radius_xy`.

The gap is just as wide in the planar case, where the choice is a disc against a
square. At `radius_xy: 0.38, resolution: 0.1` the disc costs 45 cells and
guarantees 0.350 m in every direction; the square costs 81 and pushes obstacles
out to 0.636 m along the diagonals — clearance the vehicle never asked for, paid
for by closing gaps it could have driven through.

### `publish_surface_only` keeps the message small

The published cloud otherwise grows with the *volume* of the dilated set, and
most of those points are buried inside the obstacle where nothing can reach
them. With `publish_surface_only: true` the dilation is unchanged and only its
boundary shell is published — every voxel with a free voxel within
`surface_thickness` on each axis — so the map grows with area instead of volume.
On a typical indoor map that is roughly a third of the points and a third of the
serialisation time.

That ratio tracks how much volume the kernel adds. A planar kernel inflates far
less per seed than a volumetric one, so it produces a thinner solid for the peel
to work on and takes proportionally less off it — still a win, by less.

Nothing that could block a path is removed: an interior voxel has no free
neighbour, so no 26-connected step from free space can land on one, and the free
space this cloud describes is the free space the solid cloud described.

The caveat is the hollow interior. A consumer that asks "is this arbitrary point
inside an obstacle" by voxel lookup will read it as free. A consumer that
searches over connected free voxels, or grows regions outward until they meet a
point, cannot get behind a watertight shell. Raise `surface_thickness` for a
consumer whose free-space test is a line rather than a voxel walk, which could
otherwise pass between two neighbouring surface points.

### `num_threads` and `use_cuda`

`num_threads` covers voxelising, dilating and serialising. The default of 4
leaves cores for the planners; `0` takes every core.

`use_cuda` moves the dilation to the GPU and is safe to leave on everywhere: the
fallback to the CPU is silent and covers a build with no CUDA compiler, a
machine where no device answers, and a device that errors mid-run. Both backends
produce a bit-identical map, so nothing downstream can tell which ran. Turn it
off to leave the GPU to the mapper, or to compare backends.

## Test and benchmark

`voxel_bit_grid` and `point_cloud_inflater` pull in neither ROS nor PCL, so the
test and the benchmark build and run on their own:

```bash
g++ -O3 -std=c++14 -fopenmp -I include -I /usr/include/eigen3 \
    src/voxel_bit_grid.cpp src/point_cloud_inflater.cpp \
    test/inflation_test.cpp -o /tmp/inflation_test && /tmp/inflation_test
```

That build is CPU-only, and the test says so and skips the GPU sections. To
include the CUDA backend, compile the `.cu` alongside it and define
`OCCUPANCY_INFLATION_WITH_CUDA` — which is what the CMakeLists does for you
whenever `check_language(CUDA)` finds a compiler:

```bash
nvcc -O3 -std=c++17 -DOCCUPANCY_INFLATION_WITH_CUDA \
     -I include -I /usr/include/eigen3 -c src/inflation_cuda.cu -o /tmp/inflation_cuda.o
g++ -O3 -std=c++17 -fopenmp -DOCCUPANCY_INFLATION_WITH_CUDA \
    -I include -I /usr/include/eigen3 \
    src/voxel_bit_grid.cpp src/point_cloud_inflater.cpp \
    test/inflation_test.cpp /tmp/inflation_cuda.o \
    -lcudart -o /tmp/inflation_test && /tmp/inflation_test
```

Under catkin:

```bash
catkin_make --pkg occupancy_inflation                 # then, in build/occupancy_inflation:
make inflation_test && ctest -R inflation_test -V
```

The timing harness needs no ROS either. It takes the point count, the span in
metres, the resolution, the two radii, the shape and the thread count, and runs
both backends when a device is present:

```bash
rosrun occupancy_inflation inflation_benchmark 500000 25 0.1 0.45 0.25 ellipsoid 4  # volumetric
rosrun occupancy_inflation inflation_benchmark 500000 25 0.1 0.38 0.0  ellipsoid 4  # planar
```

A tiny point count makes it a kernel-inspection tool rather than a timing one:
with one seed the voxel count it reports *is* the kernel's cell count, which is
the quickest way to check a radius before trusting it.
