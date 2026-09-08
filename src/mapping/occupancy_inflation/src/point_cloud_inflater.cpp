/**
 * @file point_cloud_inflater.cpp
 * @brief Kernel construction, the dilation pipeline and serialisation.
 */

#include "occupancy_inflation/point_cloud_inflater.h"

#include <algorithm>
#include <cstring>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace occupancy_inflation {

constexpr double PointCloudInflater::kIndexBias;
constexpr int PointCloudInflater::kIndexBiasInt;
constexpr long PointCloudInflater::kMinPointsForThreads;
constexpr int PointCloudInflater::kMinThreadsForScatter;

namespace {

/* A kernel wider than this is almost certainly a units mistake in the config,
 * and it would make every dilated block enormous. */
constexpr int kMaxKernelRadius = 1024;

inline void writeRecord(uint8_t *dst, float x, float y, float z,
                        bool with_intensity, float intensity) {
  float rec[4] = {x, y, z, intensity};
  std::memcpy(dst, rec, with_intensity ? 4 * sizeof(float) : 3 * sizeof(float));
}

}  // namespace

bool PointCloudInflater::parseShape(const std::string &name,
                                    InflationShape *shape) {
  if (name == "box") {
    *shape = InflationShape::kBox;
    return true;
  }
  if (name == "ellipsoid" || name == "sphere") {
    *shape = InflationShape::kEllipsoid;
    return true;
  }
  return false;
}

bool PointCloudInflater::configure(const InflationParams &params,
                                   std::string *error) {
  configured_ = false;

  if (!(params.resolution > 0.0)) {
    *error = "resolution must be > 0";
    return false;
  }
  if (params.radius_xy < 0.0 || params.radius_z < 0.0) {
    *error = "inflation radii must be >= 0";
    return false;
  }
  if (params.input_filter_enable &&
      (params.input_min.array() > params.input_max.array()).any()) {
    *error = "input_filter/min must be <= input_filter/max on every axis";
    return false;
  }
  if (params.output_bounds_enable &&
      (params.output_min.array() > params.output_max.array()).any()) {
    *error = "output_bounds/min must be <= output_bounds/max on every axis";
    return false;
  }
  if (params.max_grid_voxels < 64) {
    *error = "max_grid_voxels must be at least 64";
    return false;
  }
  if (params.publish_surface_only && params.surface_thickness < 1) {
    *error = "surface_thickness must be >= 1";
    return false;
  }
  if (params.surface_thickness > kMaxKernelRadius) {
    *error = "surface_thickness is more than 1024 voxels; check the units "
             "(it is a voxel count, not a distance)";
    return false;
  }

  const double inv_res = 1.0 / params.resolution;
  if (std::ceil(std::max(params.radius_xy, params.radius_z) * inv_res) >
      kMaxKernelRadius) {
    *error = "inflation radius is more than 1024 voxels; check the units";
    return false;
  }

  params_ = params;
  inv_resolution_ = inv_res;
  cuda_unavailable_ = false;

  threads_ = params_.num_threads;
#ifdef _OPENMP
  if (threads_ <= 0) threads_ = omp_get_max_threads();
#else
  threads_ = 1;
#endif
  if (threads_ < 1) threads_ = 1;

  buildKernel();

  /* Turn the metric output box into the voxel indices whose centres fall
   * inside it, so the clip is a pure integer test from here on. */
  if (params_.output_bounds_enable) {
    Eigen::Vector3i lo, hi;
    for (int i = 0; i < 3; ++i) {
      lo(i) = static_cast<int>(std::ceil(
          (params_.output_min(i) - params_.origin(i)) * inv_resolution_ - 0.5));
      hi(i) = static_cast<int>(std::floor(
          (params_.output_max(i) - params_.origin(i)) * inv_resolution_ - 0.5));
    }
    bounds_window_.base = lo;
    bounds_window_.dims = (hi - lo + Eigen::Vector3i::Ones()).cwiseMax(0);
  } else {
    bounds_window_ = VoxelWindow();
  }

  /* Build the device context here rather than inside the first message. */
  if (params_.use_cuda) cuda::warmUp();

  reset();
  configured_ = true;
  return true;
}

void PointCloudInflater::buildKernel() {
  kernel_runs_.clear();
  kernel_size_ = 0;
  reach_.setZero();

  const int rx = static_cast<int>(std::ceil(params_.radius_xy * inv_resolution_));
  const int rz = static_cast<int>(std::ceil(params_.radius_z * inv_resolution_));

  if (params_.shape == InflationShape::kBox) {
    kernel_runs_.reserve(static_cast<std::size_t>(2 * rx + 1) * (2 * rz + 1));
    for (int dy = -rx; dy <= rx; ++dy) {
      for (int dz = -rz; dz <= rz; ++dz) {
        kernel_runs_.push_back(KernelRun{dy, dz, rx});
      }
    }
    kernel_size_ = static_cast<std::size_t>(2 * rx + 1) * (2 * rx + 1) *
                   (2 * rz + 1);
    reach_ = Eigen::Vector3i(rx, rx, rz);
    return;
  }

  /* Ellipsoid. The test matches the original element voxel for voxel: measure
   * from the voxel centre and treat a zero radius as an axis that must not be
   * crossed at all. At a fixed (dy, dz) the admissible dx form one symmetric
   * interval, so the whole element is a list of runs. */
  const double radii[3] = {params_.radius_xy, params_.radius_xy, params_.radius_z};
  auto inside = [&](int dx, int dy, int dz) {
    const double ex[3] = {dx * params_.resolution, dy * params_.resolution,
                          dz * params_.resolution};
    double norm = 0.0;
    for (int i = 0; i < 3; ++i) {
      if (radii[i] <= 0.0) {
        if (ex[i] != 0.0) return false;
        continue;
      }
      const double t = ex[i] / radii[i];
      norm += t * t;
    }
    return norm <= 1.0;
  };

  for (int dy = -rx; dy <= rx; ++dy) {
    for (int dz = -rz; dz <= rz; ++dz) {
      if (!inside(0, dy, dz)) continue;
      int span = 0;
      while (span + 1 <= rx && inside(span + 1, dy, dz)) ++span;
      kernel_runs_.push_back(KernelRun{dy, dz, span});
      kernel_size_ += static_cast<std::size_t>(2 * span + 1);
      reach_.x() = std::max(reach_.x(), span);
      reach_.y() = std::max(reach_.y(), std::abs(dy));
      reach_.z() = std::max(reach_.z(), std::abs(dz));
    }
  }

  /* Sorted by x half-extent so the x dilation is grown once, in place, and
   * every run at that extent reuses it. */
  std::sort(kernel_runs_.begin(), kernel_runs_.end(),
            [](const KernelRun &a, const KernelRun &b) { return a.rx < b.rx; });
}

void PointCloudInflater::reset() {
  seed_.release();
  inflated_.release();
  cur_.release();
  surface_.release();
  std::vector<uint64_t>().swap(work_a_);
  std::vector<uint64_t>().swap(work_b_);
  occupied_count_ = 0;
  seed_count_ = 0;
  surface_count_ = 0;
}

std::size_t PointCloudInflater::memoryBytes() const {
  return (seed_.wordCount() + inflated_.wordCount() + cur_.wordCount() +
          surface_.wordCount() + work_a_.capacity() + work_b_.capacity()) *
         sizeof(uint64_t);
}

void PointCloudInflater::finishIncremental(InflationStats *stats) {
  /* Reduce this message's grid to the voxels the map does not already hold.
   * Dilation distributes over union, so those are exactly the voxels that have
   * to be grown, and everything the previous messages contributed stands. */
  cur_.subtract(seed_, threads_);

  VoxelWindow tight;
  const std::size_t added = cur_.occupiedExtent(&tight);

  stats->new_seed_voxels = added;
  stats->seed_voxels = seed_count_;
  stats->inflated_voxels = occupied_count_;
  if (added == 0) {
    stats->changed = false;
    return;
  }

  if (!seed_.growToInclude(tight, maxWords())) {
    stats->ok = false;
    stats->error = InflationError::kGridTooLarge;
    return;
  }
  seed_count_ += seed_.orFromRaw(cur_.data(), cur_.window(), tight, threads_);

  stats->changed = true;
  stats->seed_voxels = seed_count_;
  if (!dilateInto(cur_, tight, stats)) return;
  stats->inflated_voxels = occupied_count_;
  rebuildSurface(stats);
}

void PointCloudInflater::finishOneShot(InflationStats *stats) {
  VoxelWindow tight;
  const std::size_t count = cur_.occupiedExtent(&tight);

  if (count == seed_count_ && cur_.isSubsetOf(seed_)) {
    stats->new_seed_voxels = 0;
    stats->seed_voxels = seed_count_;
    stats->inflated_voxels = occupied_count_;
    stats->changed = false;
    return;
  }

  seed_.swap(cur_);  // cur_ keeps the old buffer for the next message
  seed_count_ = count;
  stats->new_seed_voxels = count;
  stats->changed = true;
  stats->seed_voxels = count;

  /* Voxels can have disappeared as well as appeared, so this one is rebuilt. */
  inflated_.release();
  occupied_count_ = 0;

  if (count != 0 && !dilateInto(seed_, tight, stats)) return;
  stats->inflated_voxels = occupied_count_;
  rebuildSurface(stats);
}

/**
 * Reduce the inflated map to its boundary shell.
 *
 * The shell is the map minus its own erosion by a (2t+1)^3 box: a voxel
 * survives when the box centred on it does not fit inside the map, i.e. when
 * at least one voxel within t of it is free. That leaves exactly the skin, and
 * the skin is watertight for anything that moves voxel to voxel -- a
 * 26-connected step from a free voxel onto a discarded interior voxel is
 * impossible, because an interior voxel has no free neighbour to be stepped
 * from. So the free space this output describes, outside the obstacles, is the
 * free space the solid map described.
 *
 * The whole map is re-eroded rather than only the window a message touched.
 * Adding one voxel can turn any of its neighbours from surface into interior,
 * so the affected window is not the dilated one anyway, and a 3x3x3 erosion is
 * separable: three axis passes of two shifted ANDs each, against the 225 runs
 * the ellipsoid dilation ORs into place. On a room-sized map it is a fraction
 * of a millisecond. It runs on the CPU on either backend; there is no CUDA
 * path for it and at this cost there is no case for one.
 *
 * Voxels outside the grid's window read as free, which is what makes the outer
 * face of a map clipped by output_bounds part of the shell rather than an edge
 * the erosion silently keeps.
 */
void PointCloudInflater::rebuildSurface(InflationStats *stats) {
  if (!params_.publish_surface_only) return;

  surface_count_ = 0;
  if (inflated_.empty() || occupied_count_ == 0) {
    surface_.release();
    stats->output_voxels = 0;
    return;
  }

  const VoxelWindow win = inflated_.window();
  const std::size_t n = inflated_.wordCount();
  const int t = params_.surface_thickness;

  work_a_.assign(n, 0);
  std::memcpy(work_a_.data(), inflated_.data(), n * sizeof(uint64_t));
  work_b_.assign(n, 0);

  erodeXInPlace(work_a_.data(), win, 0, t, threads_);
  uint64_t *a = work_a_.data();
  uint64_t *b = work_b_.data();
  uint64_t *cur = erodeAxisPingPong(a, b, win, 1, t, threads_);
  uint64_t *other = (cur == a) ? b : a;
  const uint64_t *eroded = erodeAxisPingPong(cur, other, win, 2, t, threads_);

  if (!surface_.reset(win, maxWords())) {
    /* inflated_ already fits, so this can only trip if maxWords() changed
     * under us; treat it the same way as any other grid that will not fit. */
    stats->ok = false;
    stats->error = InflationError::kGridTooLarge;
    return;
  }
  surface_count_ =
      andNotInto(surface_.data(), inflated_.data(), eroded, n, threads_);
  stats->output_voxels = surface_count_;
}

/**
 * Dilation distributes over union, so growing only the voxels a message added
 * and OR-ing the result into the map gives exactly the map that redoing all of
 * it would give.
 */
bool PointCloudInflater::dilateInto(const VoxelBitGrid &src,
                                    const VoxelWindow &tight,
                                    InflationStats *stats) {
  if (tight.empty()) return true;

  const VoxelWindow win =
      snapWindow(tight.base - reach_, tight.maxIndex() + reach_);
  if (win.wordCount() > maxWords()) {
    stats->ok = false;
    stats->error = InflationError::kGridTooLarge;
    return false;
  }

  work_a_.assign(win.wordCount(), 0);
  orWindowed(work_a_.data(), win, src.data(), src.window(), tight, threads_);

  /* The GPU consumes and returns the seeded window in place, so the CPU
   * scratch is only laid out when the device does not take the work. */
  const cuda::Status gpu = dilateOnGpu(work_a_.data(), win);
  if (gpu == cuda::Status::kOk) {
    stats->gpu_dilation = true;
    return mergeDilated(work_a_.data(), win, stats);
  }
  if (gpu == cuda::Status::kFailed) {
    /* A device that gave up during the download may have left the buffer half
     * written, so the seeds go back rather than the CPU dilating whatever
     * survived. Costs a re-seed once, on the message that loses the GPU. */
    work_a_.assign(win.wordCount(), 0);
    orWindowed(work_a_.data(), win, src.data(), src.window(), tight, threads_);
  }

  work_b_.assign(win.wordCount(), 0);

  uint64_t *result = work_a_.data();
  if (params_.shape == InflationShape::kBox) {
    /* A box is separable, so three cheap axis passes replace the full
     * (2rx+1)^2 (2rz+1) element. */
    dilateXInPlace(work_a_.data(), win, 0, reach_.x(), threads_);
    uint64_t *a = work_a_.data();
    uint64_t *b = work_b_.data();
    uint64_t *cur = dilateAxisPingPong(a, b, win, 1, reach_.y(), threads_);
    uint64_t *other = (cur == a) ? b : a;
    result = dilateAxisPingPong(cur, other, win, 2, reach_.z(), threads_);
  } else {
    /* An ellipsoid is not separable, but it is a stack of x runs: grow the rows
     * along x once per distinct half-extent and OR each run into place. */
    uint64_t *rows = work_a_.data();
    uint64_t *acc = work_b_.data();
    int grown = 0;
    for (const KernelRun &run : kernel_runs_) {
      dilateXInPlace(rows, win, grown, run.rx, threads_);
      grown = run.rx;
      orShiftedYZ(acc, rows, win, run.dy, run.dz, threads_);
    }
    result = acc;
  }

  return mergeDilated(result, win, stats);
}

/**
 * Offer one padded window to the CUDA backend.
 *
 * Both backends run the same shifted ORs in the same order over the same
 * words, so the grid that comes back is bit-identical to the one the CPU
 * passes would have produced -- there is no tolerance to reason about and
 * nothing downstream can tell which ran. A device that errors is not asked
 * again; a device that merely declines this window (too small to be worth the
 * transfer, or not enough free memory) is asked again next message.
 */
cuda::Status PointCloudInflater::dilateOnGpu(uint64_t *bits,
                                             const VoxelWindow &win) {
  if (!params_.use_cuda || cuda_unavailable_ || !cuda::available() ||
      win.wordCount() * kWordBits < params_.cuda_min_grid_voxels) {
    return cuda::Status::kDeclined;
  }

  const cuda::Status status =
      params_.shape == InflationShape::kBox
          ? cuda::dilateBox(bits, win, reach_.x(), reach_.y(), reach_.z())
          : cuda::dilateRuns(bits, win, kernel_runs_.data(), kernel_runs_.size());

  if (status == cuda::Status::kFailed) cuda_unavailable_ = true;
  return status;
}

/** Clip the dilated window to output_bounds and OR it into the map. */
bool PointCloudInflater::mergeDilated(const uint64_t *result,
                                      const VoxelWindow &win,
                                      InflationStats *stats) {
  const VoxelWindow clip = params_.output_bounds_enable
                               ? intersectWindow(win, bounds_window_)
                               : win;
  if (!clip.empty()) {
    if (!inflated_.growToInclude(clip, maxWords())) {
      stats->ok = false;
      stats->error = InflationError::kGridTooLarge;
      return false;
    }
    occupied_count_ += inflated_.orFromRaw(result, win, clip, threads_);
  }
  return true;
}

std::size_t PointCloudInflater::serialize(uint8_t *dst, std::size_t stride,
                                          bool with_intensity) const {
  return serializeGrid(params_.publish_surface_only ? surface_ : inflated_, dst,
                       stride, with_intensity, false);
}

std::size_t PointCloudInflater::serializeSeeds(uint8_t *dst, std::size_t stride,
                                               bool with_intensity) const {
  return serializeGrid(seed_, dst, stride, with_intensity, true);
}

std::size_t PointCloudInflater::serializeGrid(const VoxelBitGrid &grid,
                                              uint8_t *dst, std::size_t stride,
                                              bool with_intensity,
                                              bool all_seed) const {
  if (grid.empty()) return 0;

  const VoxelWindow win = grid.window();
  const std::size_t nw = win.wordsPerRow();
  const int nz = win.dims.z();
  const int ny = win.dims.y();
  const uint64_t *bits = grid.data();

  /* Per-plane counts turned into write offsets, so the planes can be filled in
   * parallel straight into the caller's buffer. */
  std::vector<std::size_t> offset(static_cast<std::size_t>(nz) + 1, 0);
  const std::size_t plane_words = win.wordsPerPlane();
#ifdef _OPENMP
#pragma omp parallel for schedule(static) num_threads(threads_) \
    if (threads_ > 1 && win.wordCount() > (1u << 14))
#endif
  for (int z = 0; z < nz; ++z) {
    offset[static_cast<std::size_t>(z) + 1] =
        popCount(bits + static_cast<std::size_t>(z) * plane_words, plane_words);
  }
  for (int z = 0; z < nz; ++z) {
    offset[static_cast<std::size_t>(z) + 1] += offset[static_cast<std::size_t>(z)];
  }
  const std::size_t total = offset[static_cast<std::size_t>(nz)];
  if (total == 0) return 0;

  const double res = params_.resolution;
  const double x0 = (win.base.x() + 0.5) * res + params_.origin.x();
  const double y0 = (win.base.y() + 0.5) * res + params_.origin.y();
  const double z0 = (win.base.z() + 0.5) * res + params_.origin.z();

  /* Both windows keep their x base on a word boundary, so a seed lookup is the
   * same bit of a word a fixed distance away: no per-voxel index maths. */
  const VoxelWindow seed_win = seed_.window();
  const uint64_t *seed_bits = seed_.data();
  const bool need_seed = with_intensity && !all_seed && !seed_.empty();
  const long seed_word_shift =
      need_seed ? (win.base.x() - seed_win.base.x()) / kWordBits : 0;
  const std::size_t seed_nw = need_seed ? seed_win.wordsPerRow() : 0;

#ifdef _OPENMP
#pragma omp parallel for schedule(static) num_threads(threads_) \
    if (threads_ > 1 && total > 20000)
#endif
  for (int z = 0; z < nz; ++z) {
    std::size_t out = offset[static_cast<std::size_t>(z)];
    const double pz = z0 + z * res;
    const int gz = win.base.z() + z;
    for (int y = 0; y < ny; ++y) {
      const uint64_t *row = bits + win.rowOffset(y, z);
      const double py = y0 + y * res;
      const int gy = win.base.y() + y;

      const uint64_t *seed_row = nullptr;
      if (need_seed && gz >= seed_win.base.z() && gz <= seed_win.maxIndex().z() &&
          gy >= seed_win.base.y() && gy <= seed_win.maxIndex().y()) {
        seed_row = seed_bits + seed_win.rowOffset(gy - seed_win.base.y(),
                                                  gz - seed_win.base.z());
      }

      for (std::size_t w = 0; w < nw; ++w) {
        uint64_t m = row[w];
        if (m == 0) continue;

        uint64_t seed_word = 0;
        if (seed_row != nullptr) {
          const long sw = static_cast<long>(w) + seed_word_shift;
          if (sw >= 0 && static_cast<std::size_t>(sw) < seed_nw) {
            seed_word = seed_row[sw];
          }
        }

        const double px_word = x0 + static_cast<double>(w * kWordBits) * res;
        while (m != 0) {
          const unsigned b = static_cast<unsigned>(__builtin_ctzll(m));
          m &= m - 1;
          const float intensity =
              all_seed ? 0.0f : (((seed_word >> b) & 1u) ? 0.0f : 1.0f);
          writeRecord(dst + out * stride,
                      static_cast<float>(px_word + b * res),
                      static_cast<float>(py), static_cast<float>(pz),
                      with_intensity, intensity);
          ++out;
        }
      }
    }
  }

  return total;
}

}  // namespace occupancy_inflation
