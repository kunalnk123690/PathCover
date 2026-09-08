/**
 * @file inflation_cuda.cu
 * @brief The word-parallel morphology, run on the GPU.
 *
 * The CPU dilation is already the right algorithm -- shifted ORs over 64
 * voxels at a time, with the reach doubling every pass -- and it is a pure
 * streaming job over a dense buffer, which is what a GPU is for. So the
 * kernels here are the CPU passes transposed: one thread per 64-voxel word,
 * gathering from the words its own value has to absorb.
 *
 * Gather rather than scatter is the one structural difference. The CPU grows a
 * row in place against a scratch copy; here each pass reads one buffer and
 * writes another, so no thread ever reads a word another thread is writing and
 * the passes need no synchronisation beyond the launch boundary. Two buffers
 * ping-pong for the separable case, a third accumulates for the run case.
 *
 * The device buffers persist across calls and only ever grow, so a steady
 * stream of messages reallocates nothing. They are deliberately leaked at exit
 * rather than freed from a static destructor, which is the usual way to avoid
 * a teardown-order crash against the CUDA runtime.
 */

#include "occupancy_inflation/inflation_cuda.h"

#ifdef OCCUPANCY_INFLATION_WITH_CUDA

#include <cuda_runtime.h>

#include <algorithm>
#include <iostream>
#include <mutex>

namespace occupancy_inflation {
namespace cuda {
namespace {

/* One warp-friendly block per 256 words, i.e. per 16384 voxels. */
constexpr int kThreads = 256;

/* Leave this much of the free device memory alone. A robot's GPU is shared
 * with whatever else is mapping or planning on it, and refusing the work is
 * cheap: the CPU path produces the same grid. */
constexpr double kFreeMemoryShare = 0.75;

bool cudaCheck(cudaError_t error, const char *operation) {
  if (error == cudaSuccess) return true;
  std::cerr << "[occupancy_inflation][cuda] " << operation << " failed: "
            << cudaGetErrorString(error) << std::endl;
  return false;
}

/** A device allocation that grows to fit and is never handed back. */
struct DeviceWords {
  uint64_t *ptr = nullptr;
  std::size_t words = 0;

  /** Bytes a request for @p n words would newly take from the device. */
  std::size_t extraBytes(std::size_t n) const {
    return n > words ? (n - words) * sizeof(uint64_t) : 0;
  }

  bool ensure(std::size_t n) {
    if (words >= n) return true;
    uint64_t *fresh = nullptr;
    if (!cudaCheck(cudaMalloc(&fresh, n * sizeof(uint64_t)), "allocate device grid")) {
      return false;
    }
    if (ptr != nullptr) cudaFree(ptr);
    ptr = fresh;
    words = n;
    return true;
  }
};

struct DeviceRuns {
  KernelRun *ptr = nullptr;
  std::size_t count = 0;

  bool upload(const KernelRun *runs, std::size_t n) {
    if (count < n) {
      KernelRun *fresh = nullptr;
      if (!cudaCheck(cudaMalloc(&fresh, n * sizeof(KernelRun)),
                     "allocate device kernel runs")) {
        return false;
      }
      if (ptr != nullptr) cudaFree(ptr);
      ptr = fresh;
      count = n;
    }
    return cudaCheck(cudaMemcpy(ptr, runs, n * sizeof(KernelRun),
                                cudaMemcpyHostToDevice),
                     "upload kernel runs");
  }
};

struct Scratch {
  DeviceWords a;    //!< the grid, and one half of every ping-pong
  DeviceWords b;    //!< the other half
  DeviceWords acc;  //!< accumulator for the non-separable element
  DeviceRuns runs;
};

Scratch &scratch() {
  static Scratch *s = new Scratch();  // leaked on purpose, see the file comment
  return *s;
}

std::mutex &gpuMutex() {
  static std::mutex m;
  return m;
}

/** @brief Whether @p bytes can be taken without crowding the device. */
bool roomFor(std::size_t bytes) {
  if (bytes == 0) return true;
  std::size_t free_bytes = 0, total_bytes = 0;
  if (!cudaCheck(cudaMemGetInfo(&free_bytes, &total_bytes), "query device memory")) {
    return false;
  }
  return static_cast<double>(bytes) <= static_cast<double>(free_bytes) * kFreeMemoryShare;
}

int blocksFor(std::size_t total) {
  return static_cast<int>((total + kThreads - 1) / kThreads);
}

// ---------------------------------------------------------------------------
// Kernels. Each thread owns one output word; `total` is the whole grid, so the
// launches are one-dimensional and the row/plane coordinates are recovered by
// division. That division costs far less than the global loads it guards.
// ---------------------------------------------------------------------------

/**
 * out = in | (in shifted +c along x) | (in shifted -c along x).
 *
 * @p whole and @p bit are c split into words and bits. Bits leaving a row drop,
 * which is what the CPU's orShiftedX() does at the same edges; the caller has
 * padded the window by the reach so nothing that matters falls off.
 */
__global__ void orShiftXKernel(uint64_t *__restrict__ out,
                               const uint64_t *__restrict__ in,
                               std::size_t total, std::size_t words_per_row,
                               std::size_t whole, unsigned bit) {
  const std::size_t i = blockIdx.x * static_cast<std::size_t>(blockDim.x) + threadIdx.x;
  if (i >= total) return;

  const std::size_t row = i / words_per_row;
  const std::size_t w = i - row * words_per_row;
  const uint64_t *r = in + row * words_per_row;

  uint64_t v = r[w];
  if (w >= whole) {  // bit x moves to x+c: gather from the lower words
    uint64_t t = r[w - whole] << bit;
    if (bit != 0 && w > whole) t |= r[w - whole - 1] >> (kWordBits - bit);
    v |= t;
  }
  if (w + whole < words_per_row) {  // and from the higher words for x-c
    uint64_t t = r[w + whole] >> bit;
    if (bit != 0 && w + whole + 1 < words_per_row) {
      t |= r[w + whole + 1] << (kWordBits - bit);
    }
    v |= t;
  }
  out[i] = v;
}

/** out = in | in(y-c) | in(y+c). A y step is one row of words. */
__global__ void orShiftYKernel(uint64_t *__restrict__ out,
                               const uint64_t *__restrict__ in,
                               std::size_t total, std::size_t words_per_row,
                               int ny, int c) {
  const std::size_t i = blockIdx.x * static_cast<std::size_t>(blockDim.x) + threadIdx.x;
  if (i >= total) return;

  const int y = static_cast<int>((i / words_per_row) % static_cast<std::size_t>(ny));
  const std::size_t step = static_cast<std::size_t>(c) * words_per_row;

  uint64_t v = in[i];
  if (y - c >= 0) v |= in[i - step];
  if (y + c < ny) v |= in[i + step];
  out[i] = v;
}

/** out = in | in(z-c) | in(z+c). A z step is one plane of words. */
__global__ void orShiftZKernel(uint64_t *__restrict__ out,
                               const uint64_t *__restrict__ in,
                               std::size_t total, std::size_t words_per_plane,
                               int nz, int c) {
  const std::size_t i = blockIdx.x * static_cast<std::size_t>(blockDim.x) + threadIdx.x;
  if (i >= total) return;

  const int z = static_cast<int>(i / words_per_plane);
  const std::size_t step = static_cast<std::size_t>(c) * words_per_plane;

  uint64_t v = in[i];
  if (z - c >= 0) v |= in[i - step];
  if (z + c < nz) v |= in[i + step];
  out[i] = v;
}

/**
 * acc |= rows translated by every (dy, dz) in runs[begin, end).
 *
 * All the runs in that range share an rx, so @p rows is already grown along x
 * to exactly what they need and one pass serves all of them. Every thread
 * reads the same run entries in the same order, so they broadcast out of cache
 * rather than costing a load each.
 */
__global__ void orRunsKernel(uint64_t *__restrict__ acc,
                             const uint64_t *__restrict__ rows,
                             std::size_t total, std::size_t words_per_row,
                             int ny, int nz, const KernelRun *__restrict__ runs,
                             int begin, int end) {
  const std::size_t i = blockIdx.x * static_cast<std::size_t>(blockDim.x) + threadIdx.x;
  if (i >= total) return;

  const std::size_t row = i / words_per_row;
  const std::size_t w = i - row * words_per_row;
  const int y = static_cast<int>(row % static_cast<std::size_t>(ny));
  const int z = static_cast<int>(row / static_cast<std::size_t>(ny));

  uint64_t v = acc[i];
  for (int k = begin; k < end; ++k) {
    const int sy = y - runs[k].dy;
    const int sz = z - runs[k].dz;
    if (sy < 0 || sy >= ny || sz < 0 || sz >= nz) continue;
    v |= rows[(static_cast<std::size_t>(sz) * ny + sy) * words_per_row + w];
  }
  acc[i] = v;
}

// ---------------------------------------------------------------------------

/**
 * Grow every row from radius @p from_r to @p to_r along x, doubling per pass.
 *
 * The schedule is the CPU's: a set already grown by `a` grows to `a + c` for
 * any c <= 2a+1. @p cur is left pointing at the buffer holding the result.
 */
bool growX(uint64_t *&cur, uint64_t *&other, std::size_t total,
           std::size_t words_per_row, int from_r, int to_r) {
  const int blocks = blocksFor(total);
  int a = from_r;
  while (a < to_r) {
    const int c = std::min(2 * a + 1, to_r - a);
    orShiftXKernel<<<blocks, kThreads>>>(other, cur, total, words_per_row,
                                         static_cast<std::size_t>(c) / kWordBits,
                                         static_cast<unsigned>(c) % kWordBits);
    if (!cudaCheck(cudaGetLastError(), "dilate x")) return false;
    std::swap(cur, other);
    a += c;
  }
  return true;
}

/** As above, for y (axis 1) or z (axis 2). */
bool growYZ(uint64_t *&cur, uint64_t *&other, std::size_t total,
            std::size_t words_per_row, int ny, int nz, int axis, int radius) {
  const int blocks = blocksFor(total);
  const std::size_t words_per_plane = words_per_row * static_cast<std::size_t>(ny);
  int achieved = 0;
  while (achieved < radius) {
    const int c = std::min(2 * achieved + 1, radius - achieved);
    if (axis == 1) {
      orShiftYKernel<<<blocks, kThreads>>>(other, cur, total, words_per_row, ny, c);
    } else {
      orShiftZKernel<<<blocks, kThreads>>>(other, cur, total, words_per_plane, nz, c);
    }
    if (!cudaCheck(cudaGetLastError(), axis == 1 ? "dilate y" : "dilate z")) return false;
    std::swap(cur, other);
    achieved += c;
  }
  return true;
}

}  // namespace

bool available() {
  static const bool ok = [] {
    int devices = 0;
    const cudaError_t error = cudaGetDeviceCount(&devices);
    if (error != cudaSuccess) {
      std::cerr << "[occupancy_inflation][cuda] no usable device: "
                << cudaGetErrorString(error) << "; inflating on the CPU"
                << std::endl;
      return false;
    }
    return devices > 0;
  }();
  return ok;
}

void warmUp() {
  if (!available()) return;
  /* cudaFree(nullptr) is the cheapest call that forces the runtime to build
   * the context; it does nothing else. */
  cudaCheck(cudaFree(nullptr), "initialise device context");
}

Status dilateBox(uint64_t *bits, const VoxelWindow &win, int rx, int ry, int rz) {
  const std::size_t total = win.wordCount();
  /* Nothing to grow, or nothing to grow it in: the CPU path is already a no-op
   * for these and the round trip would be pure loss. */
  if (total == 0 || (rx <= 0 && ry <= 0 && rz <= 0)) return Status::kDeclined;
  if (!available()) return Status::kFailed;

  const std::size_t words_per_row = win.wordsPerRow();
  const int ny = win.dims.y();
  const int nz = win.dims.z();

  std::lock_guard<std::mutex> lock(gpuMutex());
  Scratch &s = scratch();
  if (!roomFor(s.a.extraBytes(total) + s.b.extraBytes(total))) return Status::kDeclined;
  if (!s.a.ensure(total) || !s.b.ensure(total)) return Status::kFailed;

  if (!cudaCheck(cudaMemcpy(s.a.ptr, bits, total * sizeof(uint64_t),
                            cudaMemcpyHostToDevice),
                 "upload grid")) {
    return Status::kFailed;
  }

  uint64_t *cur = s.a.ptr;
  uint64_t *other = s.b.ptr;
  if (!growX(cur, other, total, words_per_row, 0, rx) ||
      !growYZ(cur, other, total, words_per_row, ny, nz, 1, ry) ||
      !growYZ(cur, other, total, words_per_row, ny, nz, 2, rz)) {
    return Status::kFailed;
  }

  if (!cudaCheck(cudaMemcpy(bits, cur, total * sizeof(uint64_t),
                            cudaMemcpyDeviceToHost),
                 "download grid")) {
    return Status::kFailed;
  }
  return Status::kOk;
}

Status dilateRuns(uint64_t *bits, const VoxelWindow &win, const KernelRun *runs,
                  std::size_t count) {
  const std::size_t total = win.wordCount();
  if (total == 0 || count == 0) return Status::kDeclined;
  if (!available()) return Status::kFailed;

  const std::size_t words_per_row = win.wordsPerRow();
  const int ny = win.dims.y();
  const int nz = win.dims.z();

  std::lock_guard<std::mutex> lock(gpuMutex());
  Scratch &s = scratch();
  if (!roomFor(s.a.extraBytes(total) + s.b.extraBytes(total) +
               s.acc.extraBytes(total))) {
    return Status::kDeclined;
  }
  if (!s.a.ensure(total) || !s.b.ensure(total) || !s.acc.ensure(total) ||
      !s.runs.upload(runs, count)) {
    return Status::kFailed;
  }

  if (!cudaCheck(cudaMemcpy(s.a.ptr, bits, total * sizeof(uint64_t),
                            cudaMemcpyHostToDevice),
                 "upload grid") ||
      !cudaCheck(cudaMemset(s.acc.ptr, 0, total * sizeof(uint64_t)),
                 "clear accumulator")) {
    return Status::kFailed;
  }

  /* The runs arrive sorted by x half-extent, so the rows are grown along x
   * once per distinct extent and every run at that extent is stamped into the
   * accumulator from the same buffer -- the CPU's loop, one launch per group
   * instead of one per run. */
  const int blocks = blocksFor(total);
  uint64_t *cur = s.a.ptr;
  uint64_t *other = s.b.ptr;
  int grown = 0;
  std::size_t k = 0;
  while (k < count) {
    std::size_t j = k;
    while (j < count && runs[j].rx == runs[k].rx) ++j;

    if (!growX(cur, other, total, words_per_row, grown, runs[k].rx)) {
      return Status::kFailed;
    }
    grown = runs[k].rx;

    orRunsKernel<<<blocks, kThreads>>>(s.acc.ptr, cur, total, words_per_row, ny,
                                       nz, s.runs.ptr, static_cast<int>(k),
                                       static_cast<int>(j));
    if (!cudaCheck(cudaGetLastError(), "stamp kernel runs")) return Status::kFailed;
    k = j;
  }

  if (!cudaCheck(cudaMemcpy(bits, s.acc.ptr, total * sizeof(uint64_t),
                            cudaMemcpyDeviceToHost),
                 "download grid")) {
    return Status::kFailed;
  }
  return Status::kOk;
}

}  // namespace cuda
}  // namespace occupancy_inflation

#endif  // OCCUPANCY_INFLATION_WITH_CUDA
