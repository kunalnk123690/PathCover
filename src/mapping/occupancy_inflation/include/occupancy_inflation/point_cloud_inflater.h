/**
 * @file point_cloud_inflater.h
 * @brief Standalone obstacle inflation sized for an online, growing map.
 *
 * The inflater voxelises an input cloud onto a global lattice and dilates the
 * occupied voxels. Two things make it cheap enough to run on every message from
 * a mapping node:
 *
 *  - Occupancy is a dense bitset (see voxel_bit_grid.h), so dilation is a
 *    handful of shifted ORs over 64 voxels at a time instead of one hash-set
 *    insert per (voxel, kernel cell) pair.
 *  - In incremental mode the seed and inflated sets persist across messages, so
 *    a growing map only pays for the voxels it has just added. A message that
 *    adds nothing new costs one scan of the points and nothing else.
 *
 * Points come in through a caller-supplied reader (size() plus point(i,x,y,z))
 * and results go out through serialize(), so the class never allocates a copy
 * of either cloud and stays free of ROS and PCL.
 */

#ifndef OCCUPANCY_INFLATION_POINT_CLOUD_INFLATER_H_
#define OCCUPANCY_INFLATION_POINT_CLOUD_INFLATER_H_

#include <Eigen/Core>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "occupancy_inflation/inflation_cuda.h"
#include "occupancy_inflation/voxel_bit_grid.h"

#ifdef _OPENMP
#include <omp.h>
#endif

namespace occupancy_inflation {

/** Shape of the structuring element used for the dilation. */
enum class InflationShape {
  kBox,        //!< every voxel in the (2*rx+1)^2 x (2*rz+1) block
  kEllipsoid,  //!< voxels inside the ellipsoid spanned by the radii
};

struct InflationParams {
  double resolution = 0.1;

  /* Inflation radii in metres. radius_z is the vertical half-extent; set it to
   * 0.0 to inflate purely in the xy plane. */
  double radius_xy = 0.2;
  double radius_z = 0.2;
  InflationShape shape = InflationShape::kBox;

  /* Origin of the voxel lattice. Only shifts which points share a voxel; it
   * does not bound the map. */
  Eigen::Vector3d origin = Eigen::Vector3d::Zero();

  /* Drop input points outside this box before voxelising. */
  bool input_filter_enable = false;
  Eigen::Vector3d input_min = Eigen::Vector3d::Zero();
  Eigen::Vector3d input_max = Eigen::Vector3d::Zero();

  /* Drop inflated voxels whose centre falls outside this box. This is what
   * stops inflation from leaking past the edge of a bounded map. */
  bool output_bounds_enable = false;
  Eigen::Vector3d output_min = Eigen::Vector3d::Zero();
  Eigen::Vector3d output_max = Eigen::Vector3d::Zero();

  /**
   * Publish only the boundary shell of the inflated map instead of its solid
   * interior.
   *
   * The inflation is unchanged: the map is dilated exactly as before, and then
   * every voxel all of whose 26 neighbours are also occupied is dropped from
   * the output. Nothing that could block a path is removed, because a
   * 26-connected path from free space into the discarded interior would have to
   * step from a free voxel onto an interior voxel, and an interior voxel has no
   * free neighbour by definition. What it removes is the fill behind the
   * surface, which no planner working from outside the obstacle can ever reach.
   *
   * The point of it is message size: the solid map grows with the volume of the
   * dilated set, the shell only with its area, so the published cloud comes out
   * at roughly the size of the input map rather than a multiple of it. On a
   * planner whose cost scales with the number of points that is the difference
   * the inflation makes to it.
   *
   * The caveat is that a consumer testing "is this arbitrary point inside an
   * obstacle" by voxel lookup will read the hollow interior as free. Consumers
   * that search over connected free voxels, or grow convex regions until they
   * meet a point, are unaffected.
   */
  bool publish_surface_only = false;

  /**
   * Thickness of that shell in voxels. 1 is the thinnest watertight answer;
   * raise it to leave a thicker skin, e.g. for a consumer whose free-space test
   * is a line rather than a voxel walk and could otherwise pass between two
   * neighbouring surface points. Ignored while publish_surface_only is off.
   */
  int surface_thickness = 1;

  /**
   * Accumulate across messages. Right for a mapping node that publishes a map
   * that only ever grows, because then each message only has to pay for its new
   * voxels. Turn it off for a sliding local window, where voxels also
   * disappear and every message has to be inflated from scratch.
   */
  bool incremental = true;

  /* Threads for the voxelisation, dilation and serialisation. 1 disables the
   * parallel paths; 0 means "use every core". Ignored without OpenMP. */
  int num_threads = 4;

  /**
   * Run the dilation on the GPU when one is there. The CUDA backend performs
   * the same shifted ORs in the same order, so it produces a bit-identical
   * grid; a build without a CUDA compiler, a machine without a device, and a
   * device that errors mid-run all fall back to the CPU passes silently.
   * Set false to pin the dilation to the CPU, e.g. to leave the GPU to the
   * mapper or to compare the two backends.
   */
  bool use_cuda = true;

  /**
   * Grids smaller than this stay on the CPU. A device round trip costs tens of
   * microseconds of launch and transfer whatever the size, which a grid of a
   * few thousand voxels does not come close to earning back. Applies to the
   * padded window a message dilates, not to the map. 0 sends everything to the
   * GPU, which is what the equivalence tests do.
   */
  std::size_t cuda_min_grid_voxels = 1u << 20;  // 1 Mi voxels, i.e. 128 KiB of grid

  /**
   * Ceiling on the voxels one grid may cover, as a guard against a stray point
   * a kilometre away turning into an enormous allocation. At 8 bits per byte
   * the default is 64 MB per grid.
   */
  std::size_t max_grid_voxels = 512u * 1024u * 1024u;
};

/** Why a message could not be folded into the map. */
enum class InflationError {
  kNone,
  kNotConfigured,  //!< configure() was never called, or it failed
  kOutOfRange,     //!< a point sits outside the representable lattice
  kGridTooLarge,   //!< the map would need more than max_grid_voxels
};

/** Per-message counters, useful when measuring what inflation actually costs. */
struct InflationStats {
  std::size_t input_points = 0;      //!< points in the incoming cloud
  std::size_t filtered_points = 0;   //!< points that survived the input filter
  std::size_t new_seed_voxels = 0;   //!< occupied voxels this message added
  std::size_t seed_voxels = 0;       //!< occupied voxels held after this message
  std::size_t inflated_voxels = 0;   //!< voxels held after dilation
  std::size_t output_voxels = 0;     //!< voxels serialize() will write
  bool changed = false;              //!< false when the message added nothing
  bool gpu_dilation = false;         //!< true when the dilation ran on the GPU
  bool ok = true;                    //!< false when the message was rejected
  InflationError error = InflationError::kNone;
  double duration_ms = 0.0;          //!< wall time spent in inflate()
};

class PointCloudInflater {
 public:
  /**
   * @brief Validate the parameters and build the structuring element.
   * @param error set to a human readable reason when configure() fails
   * @return false if the parameters are unusable; the inflater is left unusable
   */
  bool configure(const InflationParams &params, std::string *error);

  /** @brief Drop the accumulated map, e.g. when the input frame changes. */
  void reset();

  /**
   * @brief Fold one cloud into the map.
   *
   * @tparam Reader anything exposing size() and point(i, x, y, z); see
   *         EigenPointCloudConversions::PointCloud2XyzReader.
   *
   * Two passes over the points: one to find their extent so the grid can be
   * sized exactly, one to set the bits. Both read the message buffer directly.
   *
   * When stats->changed comes back false the map is byte for byte what it was
   * before, so the caller can skip republishing it.
   */
  template <typename Reader>
  void inflate(const Reader &reader, InflationStats *stats);

  /**
   * @brief Voxels serialize() will write.
   *
   * The whole inflated map, or just its boundary shell with
   * publish_surface_only set.
   */
  std::size_t occupiedCount() const {
    return params_.publish_surface_only ? surface_count_ : occupied_count_;
  }

  /** @brief Voxels in the inflated map, shell or not. */
  std::size_t inflatedCount() const { return occupied_count_; }
  /** @brief Voxels in the seed map, i.e. points serializeSeeds() will write. */
  std::size_t seedCount() const { return seed_count_; }

  /**
   * @brief Write the inflated map as packed XYZ(I) records.
   *
   * @param dst      buffer of at least occupiedCount() * stride bytes
   * @param stride   bytes per record (>= 12, or >= 16 with intensity)
   * @param with_intensity 0 for voxels that held an input point, 1 for voxels
   *                 that exist only because of the inflation
   * @return records written
   */
  std::size_t serialize(uint8_t *dst, std::size_t stride,
                        bool with_intensity) const;

  /** @brief Write the seed map, i.e. the voxelised input with no inflation. */
  std::size_t serializeSeeds(uint8_t *dst, std::size_t stride,
                             bool with_intensity) const;

  /** @brief Number of voxels in the structuring element. */
  std::size_t kernelSize() const { return kernel_size_; }

  /**
   * @brief Whether the next dilation will be offered to the GPU.
   *
   * False in a CPU-only build, with use_cuda off, or once a device has failed
   * and the inflater has stopped asking. A grid under cuda_min_grid_voxels
   * still runs on the CPU while this reads true.
   */
  bool cudaEnabled() const {
    return params_.use_cuda && !cuda_unavailable_ && cuda::available();
  }

  /** @brief Bytes currently held by the occupancy grids. */
  std::size_t memoryBytes() const;

  const InflationParams &params() const { return params_; }

  static bool parseShape(const std::string &name, InflationShape *shape);

 private:
  /* Split out of the inflate() template so only the point loops are inlined
   * into the caller; everything heavy stays in the translation unit. */
  std::size_t maxWords() const { return params_.max_grid_voxels / kWordBits; }
  void finishIncremental(InflationStats *stats);
  void finishOneShot(InflationStats *stats);
  void rebuildSurface(InflationStats *stats);
  bool dilateInto(const VoxelBitGrid &src, const VoxelWindow &tight,
                  InflationStats *stats);
  std::size_t serializeGrid(const VoxelBitGrid &grid, uint8_t *dst,
                            std::size_t stride, bool with_intensity,
                            bool all_seed) const;

  void buildKernel();

  /**
   * @brief Lattice index of a point.
   *
   * Biasing the coordinate positive before truncating gives the same answer as
   * std::floor without the call: a portable build has no way to know the CPU
   * has roundsd, so every std::floor becomes a call into libm, and at three per
   * point over two passes that alone dominated the voxelisation. A double at
   * 2^20 still resolves 2^-32, far finer than any voxel, so the bias costs no
   * accuracy inside the range checked by indexInRange().
   */
  static constexpr double kIndexBias = 1048576.0;  // 2^20
  static constexpr int kIndexBiasInt = 1048576;

  /* Below this many points a parallel region costs more than it saves. */
  static constexpr long kMinPointsForThreads = 20000;
  /* See the note on the scatter loop for where this number comes from. */
  static constexpr int kMinThreadsForScatter = 2;

  Eigen::Vector3i posToIndex(float x, float y, float z) const {
    return Eigen::Vector3i(
        static_cast<int>((static_cast<double>(x) - params_.origin.x()) *
                             inv_resolution_ + kIndexBias) - kIndexBiasInt,
        static_cast<int>((static_cast<double>(y) - params_.origin.y()) *
                             inv_resolution_ + kIndexBias) - kIndexBiasInt,
        static_cast<int>((static_cast<double>(z) - params_.origin.z()) *
                             inv_resolution_ + kIndexBias) - kIndexBiasInt);
  }

  /** @brief Whether a coordinate lands on the lattice posToIndex can address. */
  bool indexInRange(float v, double origin) const {
    const double t = (static_cast<double>(v) - origin) * inv_resolution_;
    return t > -(kIndexBias - 1.0) && t < (kIndexBias - 1.0);
  }

  /** @brief Offer one padded window to the CUDA backend. */
  cuda::Status dilateOnGpu(uint64_t *bits, const VoxelWindow &win);
  bool mergeDilated(const uint64_t *result, const VoxelWindow &win,
                    InflationStats *stats);

  InflationParams params_;
  double inv_resolution_ = 1.0;
  bool configured_ = false;
  int threads_ = 1;
  /* Sticky: a device that has errored once is not asked again, so a broken GPU
   * costs one failed dilation rather than one per message. Cleared by
   * configure(). */
  bool cuda_unavailable_ = false;

  /* Structuring element, as an x half-extent per (dy, dz) column, sorted by
   * that half-extent so the x dilation can be grown once and reused. */
  std::vector<KernelRun> kernel_runs_;
  Eigen::Vector3i reach_ = Eigen::Vector3i::Zero();
  std::size_t kernel_size_ = 0;

  /* Voxel index range output_bounds admits, precomputed from the metric box. */
  VoxelWindow bounds_window_;

  /* The map. cur_ holds this message's own seed set in one-shot mode, and the
   * voxels this message added in incremental mode. */
  VoxelBitGrid seed_;
  VoxelBitGrid inflated_;
  VoxelBitGrid cur_;
  /* Boundary shell of inflated_, rebuilt whenever inflated_ changes. Shares
   * inflated_'s window, so a seed lookup during serialisation is still the
   * same bit of a word a fixed distance away. Empty unless
   * publish_surface_only is set. */
  VoxelBitGrid surface_;
  std::vector<uint64_t> work_a_;
  std::vector<uint64_t> work_b_;

  std::size_t occupied_count_ = 0;
  std::size_t seed_count_ = 0;
  std::size_t surface_count_ = 0;
};

// ---------------------------------------------------------------------------
// inflate() is a template so the point loops specialise on the reader and the
// coordinates are pulled straight from the message buffer.
// ---------------------------------------------------------------------------

template <typename Reader>
void PointCloudInflater::inflate(const Reader &reader, InflationStats *stats) {
  const auto t_start = std::chrono::steady_clock::now();

  InflationStats local;
  local.input_points = reader.size();
  local.seed_voxels = seed_count_;
  local.inflated_voxels = occupied_count_;

  const auto done = [&](InflationStats *out) {
    local.output_voxels = occupiedCount();
    local.duration_ms = std::chrono::duration<double, std::milli>(
                            std::chrono::steady_clock::now() - t_start)
                            .count();
    if (out != nullptr) *out = local;
  };

  if (!configured_) {
    local.ok = false;
    local.error = InflationError::kNotConfigured;
    done(stats);
    return;
  }

  const long n = static_cast<long>(reader.size());
  const bool filter = params_.input_filter_enable;
  const float fx_lo = static_cast<float>(params_.input_min.x());
  const float fy_lo = static_cast<float>(params_.input_min.y());
  const float fz_lo = static_cast<float>(params_.input_min.z());
  const float fx_hi = static_cast<float>(params_.input_max.x());
  const float fy_hi = static_cast<float>(params_.input_max.y());
  const float fz_hi = static_cast<float>(params_.input_max.z());

  /* Pass 1: extent of the points that will become seeds, so the grid is sized
   * once at exactly the right size instead of grown as points arrive. */
  float lo_x = std::numeric_limits<float>::infinity();
  float lo_y = lo_x, lo_z = lo_x;
  float hi_x = -lo_x, hi_y = -lo_x, hi_z = -lo_x;
  long kept = 0;

#ifdef _OPENMP
  const bool par = threads_ > 1 && n > kMinPointsForThreads;
  /* The scatter is a read-modify-write into a shared grid, so its threads
   * contend on cache lines rather than on work, which is why it is gated
   * separately from pass 1 at all. The gate used to sit at six threads. On a
   * real map that was far too conservative: the bits a surface cloud sets are
   * spread over the whole grid, so the atomics almost never collide, and the
   * parallel scatter wins from two threads up. Measured on a 24-core Xeon
   * against a 386k-point floorplan map, whole callback, threads : ms serial
   * scatter -> ms parallel scatter, 2 : 11.2 -> 8.1, 3 : 10.5 -> 6.6,
   * 4 : 10.5 -> 6.1, 5 : 10.5 -> 5.4. A cloud dense enough to make several
   * threads fight over one cache line would change that, but at that point
   * pass 1 dominates anyway. */
  const bool par_scatter = par && threads_ >= kMinThreadsForScatter;
#pragma omp parallel for schedule(static) num_threads(threads_) if (par) \
    reduction(min : lo_x, lo_y, lo_z) reduction(max : hi_x, hi_y, hi_z)   \
    reduction(+ : kept)
#endif
  for (long i = 0; i < n; ++i) {
    float x, y, z;
    reader.point(static_cast<std::size_t>(i), x, y, z);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
    if (filter && (x < fx_lo || x > fx_hi || y < fy_lo || y > fy_hi ||
                   z < fz_lo || z > fz_hi)) {
      continue;
    }
    if (x < lo_x) lo_x = x;
    if (y < lo_y) lo_y = y;
    if (z < lo_z) lo_z = z;
    if (x > hi_x) hi_x = x;
    if (y > hi_y) hi_y = y;
    if (z > hi_z) hi_z = z;
    ++kept;
  }

  local.filtered_points = static_cast<std::size_t>(kept);

  if (kept == 0) {
    /* Nothing to add, though in one-shot mode that means the map is now empty. */
    cur_.release();
    params_.incremental ? finishIncremental(&local) : finishOneShot(&local);
    done(stats);
    return;
  }

  /* posToIndex() is only exact inside the biased range, and one stray point a
   * thousand kilometres out would otherwise wrap into the middle of the map. */
  if (!indexInRange(lo_x, params_.origin.x()) ||
      !indexInRange(lo_y, params_.origin.y()) ||
      !indexInRange(lo_z, params_.origin.z()) ||
      !indexInRange(hi_x, params_.origin.x()) ||
      !indexInRange(hi_y, params_.origin.y()) ||
      !indexInRange(hi_z, params_.origin.z())) {
    local.ok = false;
    local.error = InflationError::kOutOfRange;
    done(stats);
    return;
  }

  const VoxelWindow cloud_win =
      snapWindow(posToIndex(lo_x, lo_y, lo_z), posToIndex(hi_x, hi_y, hi_z));

  if (!cur_.reset(cloud_win, maxWords())) {
    local.ok = false;
    local.error = InflationError::kGridTooLarge;
    done(stats);
    return;
  }

  /* Pass 2: one bit per point, into this message's own grid. Whether those
   * voxels are new, and which of them have to be dilated, is decided in one
   * sweep of the finished grid rather than per point. */
  const VoxelWindow win = cur_.window();
  uint64_t *bits = cur_.data();
  const int base_x = win.base.x(), base_y = win.base.y(), base_z = win.base.z();
  const std::size_t nw = win.wordsPerRow();
  const std::size_t ny = static_cast<std::size_t>(win.dims.y());

#ifdef _OPENMP
#pragma omp parallel for schedule(static) num_threads(threads_) if (par_scatter)
#endif
  for (long i = 0; i < n; ++i) {
    float x, y, z;
    reader.point(static_cast<std::size_t>(i), x, y, z);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
    if (filter && (x < fx_lo || x > fx_hi || y < fy_lo || y > fy_hi ||
                   z < fz_lo || z > fz_hi)) {
      continue;
    }
    const Eigen::Vector3i idx = posToIndex(x, y, z);
    const int lx = idx.x() - base_x;
    const std::size_t w = (static_cast<std::size_t>(idx.z() - base_z) * ny +
                           static_cast<std::size_t>(idx.y() - base_y)) *
                              nw +
                          static_cast<std::size_t>(lx) / kWordBits;
    const uint64_t mask = uint64_t(1) << (static_cast<unsigned>(lx) % kWordBits);
#ifdef _OPENMP
    if (par_scatter) {
      /* Setting a bit twice is harmless, so the threads need nothing stronger
       * than a relaxed OR. */
      __atomic_fetch_or(bits + w, mask, __ATOMIC_RELAXED);
      continue;
    }
#endif
    bits[w] |= mask;
  }

  params_.incremental ? finishIncremental(&local) : finishOneShot(&local);
  done(stats);
}

}  // namespace occupancy_inflation

#endif  // OCCUPANCY_INFLATION_POINT_CLOUD_INFLATER_H_
