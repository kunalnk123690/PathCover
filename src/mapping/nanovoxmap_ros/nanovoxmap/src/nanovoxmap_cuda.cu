/**
 * @file nanovoxmap_cuda.cu
 * @brief Bounded-memory CUDA ray traversal and signed-ESDF solve for the sparse
 *        voxel map.
 *
 * CUDA never owns a second copy of the map.  Ray insertion traverses rays into
 * sparse integer voxel coordinates, sorts/reduces repeated coordinates
 * on-device, and returns compact updates to the block hash in nanovoxmap.hpp.
 * The ESDF solver takes one dense occupancy box from the incremental scheduler
 * and returns its signed distances.  Both run the same algorithm as the CPU
 * path, so GPU and CPU builds produce the same map and the same field, and
 * falling back on a device-less robot changes nothing but the timing.
 *
 * Every device allocation here is sized from the work actually queued and
 * capped against the device's *free* memory rather than a fixed constant,
 * because on a Tegra part (Jetson) that memory is the same pool the rest of the
 * robot's stack is running out of.
 */

#include "nanovoxmap/nanovoxmap.hpp"

#ifdef NANOVOXMAP_WITH_CUDA

#include <cuda_runtime.h>

#include <thrust/copy.h>
#include <thrust/device_vector.h>
#include <thrust/execution_policy.h>
#include <thrust/functional.h>
#include <thrust/iterator/constant_iterator.h>
#include <thrust/reduce.h>
#include <thrust/sort.h>
#include <thrust/system_error.h>

#include <algorithm>
#include <cstdint>
#include <iostream>
#include <limits>
#include <map>
#include <mutex>
#include <new>
#include <thread>
#include <vector>

namespace {

/// Largest CUDA grid this file will launch.  Every kernel below is a grid-stride
/// loop, so this is purely a cap on concurrency and never a limit on work size.
constexpr int64_t kMaxGridBlocks = 1 << 20;

int gridFor(int64_t items, int threads) {
    const int64_t blocks = (items + threads - 1) / threads;
    return static_cast<int>(std::max<int64_t>(1, std::min<int64_t>(blocks, kMaxGridBlocks)));
}

/**
 * Caching device allocator.  A mapping session runs thousands of
 * sort/reduce_by_key calls whose temporary device buffers are nearly the same
 * size every frame; handing those back to cudaMalloc/cudaFree each time stalls
 * on the driver.  This recycles freed blocks (best-fit by size) so steady-state
 * frames reuse memory instead of reallocating it, and it backs both thrust's
 * internal temporaries (via the execution policy) and our explicit scratch.
 *
 * The recycled pool is capped: an unbounded cache would hold every peak-sized
 * temporary the session ever needed, which on an integrated GPU is memory taken
 * directly from the rest of the system.
 */
class CachedAllocator {
public:
    using value_type = char;

    /// Bytes of *freed* blocks kept for reuse.  Past this, blocks go back to the
    /// driver instead of being retained.
    static constexpr std::ptrdiff_t kMaxCachedBytes = 128 * 1024 * 1024;

    CachedAllocator() = default;
    ~CachedAllocator() { releaseAll(); }

    CachedAllocator(const CachedAllocator&) = delete;
    CachedAllocator& operator=(const CachedAllocator&) = delete;

    char* allocate(std::ptrdiff_t num_bytes) {
        char* result = nullptr;
        const auto free_block = free_blocks_.lower_bound(num_bytes);  // smallest block >= request
        if (free_block != free_blocks_.end()) {
            result = free_block->second;
            cached_bytes_ -= free_block->first;
            allocated_blocks_.emplace(result, free_block->first);     // remember the real size
            free_blocks_.erase(free_block);
        } else {
            if (cudaMalloc(&result, static_cast<size_t>(num_bytes)) != cudaSuccess) {
                // Give the cache back and retry once: the pool may be holding
                // exactly the memory this larger request needs.
                cudaGetLastError();                                   // clear the sticky-free error state
                releaseFree();
                if (cudaMalloc(&result, static_cast<size_t>(num_bytes)) != cudaSuccess) {
                    cudaGetLastError();
                    throw std::bad_alloc();
                }
            }
            allocated_blocks_.emplace(result, num_bytes);
        }
        return result;
    }

    void deallocate(char* ptr, size_t) {
        const auto block = allocated_blocks_.find(ptr);
        // Thrust only ever hands back pointers this allocator produced, but a
        // miss here would otherwise dereference end() and corrupt the heap.
        if (block == allocated_blocks_.end()) return;
        const std::ptrdiff_t num_bytes = block->second;
        allocated_blocks_.erase(block);
        if (cached_bytes_ + num_bytes > kMaxCachedBytes) {
            cudaFree(ptr);
            return;
        }
        free_blocks_.emplace(num_bytes, ptr);                         // recycle, keep its real size
        cached_bytes_ += num_bytes;
    }

private:
    std::multimap<std::ptrdiff_t, char*> free_blocks_;
    std::map<char*, std::ptrdiff_t> allocated_blocks_;
    std::ptrdiff_t cached_bytes_ = 0;

    void releaseFree() {
        for (auto& b : free_blocks_) cudaFree(b.second);
        free_blocks_.clear();
        cached_bytes_ = 0;
    }

    void releaseAll() {
        releaseFree();
        for (auto& b : allocated_blocks_) cudaFree(b.first);
        allocated_blocks_.clear();
    }
};

// Packing (dx,dy,dz) into one 63-bit integer key lets the on-device reduction
// sort with thrust's radix sort instead of a comparison sort over a 3-int
// struct -- radix is several times faster and the sort dominates the GPU cost.
// Each axis gets 21 bits, so a coordinate must stay within +/-(2^20 - 1) voxels
// of the traversal start.  Every ray admitted to a GPU chunk is checked against
// that range so every traced voxel is representable; longer rays go to the CPU.
constexpr int kPackBits = 21;
constexpr int64_t kPackBias = int64_t(1) << (kPackBits - 1);   // 2^20
constexpr uint64_t kPackMask = (uint64_t(1) << kPackBits) - 1;

// Longest single ray the GPU will trace: bounded by the packable range so every
// key is encodable.  Longer rays are rare on robots (range is clipped) and are
// traced on the CPU instead.
constexpr int64_t kMaxRayVoxels = kPackBias - 1;

// Total events batched into one device sort/reduce.  Packed keys are only 8
// bytes so temporary device memory stays modest, while batching several rays'
// worth of events per sort amortizes launch/allocation overhead.  Measured to
// be near-optimal on the target hardware; larger batches regress because the
// single large radix sort and host merge lose more than the fewer launches gain.
constexpr int64_t kMaxMissEventsPerChunk = 2 * 1024 * 1024;

struct DeviceVoxel {
    int x;
    int y;
    int z;
};

/**
 * Persistent device/host scratch reused across insertPointCloudCuda() calls.
 * Every buffer only ever grows (resize never shrinks capacity), so after the
 * first few frames no device reallocation happens at all.  Guarded by a mutex
 * so it is safe even if two map instances share the single GPU.
 */
struct GpuScratch {
    CachedAllocator alloc;
    thrust::device_vector<DeviceVoxel> d_ends;
    thrust::device_vector<int64_t>     d_offsets;
    thrust::device_vector<uint64_t>    d_events;   // packed miss keys, or hit keys
    thrust::device_vector<uint64_t>    d_unique;
    thrust::device_vector<int>         d_counts;
    std::vector<uint64_t>              h_keys;
    std::vector<int>                   h_counts;
};

GpuScratch& scratch() {
    static GpuScratch* s = new GpuScratch();   // leaked on purpose: avoids CUDA teardown-order crashes at exit
    return *s;
}
std::mutex& gpuMutex() {
    static std::mutex m;
    return m;
}

bool cudaCheck(cudaError_t error, const char* operation) {
    if (error == cudaSuccess) return true;
    std::cerr << "[nanovoxmap][cuda] " << operation << " failed: "
              << cudaGetErrorString(error) << std::endl;
    return false;
}

/**
 * Is there a device this process can actually use?
 *
 * Probed once and cached.  cudaGetDeviceCount() is not free and, on a machine
 * with no driver at all, it is the call that reports so -- which is exactly the
 * case a CPU-fallback build needs to survive without paying for the answer on
 * every scan.
 */
bool haveCudaDevice() {
    static const bool available = [] {
        int device_count = 0;
        const cudaError_t status = cudaGetDeviceCount(&device_count);
        if (status != cudaSuccess || device_count == 0) {
            cudaGetLastError();                     // do not leave the error latched
            std::cerr << "[nanovoxmap][cuda] no usable CUDA device ("
                      << cudaGetErrorString(status) << "); using the CPU path" << std::endl;
            return false;
        }
        cudaDeviceProp prop{};
        if (cudaGetDeviceProperties(&prop, 0) == cudaSuccess) {
            std::cerr << "[nanovoxmap][cuda] using device 0: " << prop.name
                      << " (sm_" << prop.major << prop.minor
                      << (prop.integrated ? ", integrated" : ", discrete") << ")" << std::endl;
        }
        return true;
    }();
    return available;
}

/**
 * Cell budget for one GPU ESDF solve.
 *
 * The solve needs occupancy (1 B) + three float fields (12 B) + the parabola
 * vertex and breakpoint rows (8 B) per cell, so roughly 21 B/cell.  Deriving
 * the cap from the device's free memory rather than a constant is what keeps
 * this honest on an integrated GPU, where "device memory" is the same DRAM the
 * ROS graph, the sensor drivers and the page cache are living in; a quarter of
 * what is free is a deliberate margin. Boxes above the cap are not a failure --
 * the CPU solver takes them and the GPU stays enabled for the next region.
 */
int64_t maxEsdfCells() {
    static const int64_t cells = [] {
        constexpr int64_t kBytesPerCell = 21;
        constexpr int64_t kCeiling = 16 * 1024 * 1024;   // never more than this
        constexpr int64_t kFloor = 1 * 1024 * 1024;      // below this the GPU is not worth it
        size_t free_bytes = 0, total_bytes = 0;
        if (cudaMemGetInfo(&free_bytes, &total_bytes) != cudaSuccess) {
            cudaGetLastError();
            return kFloor;
        }
        const int64_t budget = static_cast<int64_t>(free_bytes / 4) / kBytesPerCell;
        return std::max(kFloor, std::min(kCeiling, budget));
    }();
    return cells;
}

__host__ __device__ inline bool packable(const DeviceVoxel& start, const DeviceVoxel& v) {
    const int64_t dx = int64_t(v.x) - start.x;
    const int64_t dy = int64_t(v.y) - start.y;
    const int64_t dz = int64_t(v.z) - start.z;
    return dx > -kPackBias && dx < kPackBias &&
           dy > -kPackBias && dy < kPackBias &&
           dz > -kPackBias && dz < kPackBias;
}

__host__ __device__ inline uint64_t packKey(const DeviceVoxel& start, const DeviceVoxel& v) {
    const uint64_t px = uint64_t(int64_t(v.x) - start.x + kPackBias) & kPackMask;
    const uint64_t py = uint64_t(int64_t(v.y) - start.y + kPackBias) & kPackMask;
    const uint64_t pz = uint64_t(int64_t(v.z) - start.z + kPackBias) & kPackMask;
    return px | (py << kPackBits) | (pz << (2 * kPackBits));
}

inline DeviceVoxel unpackKey(const DeviceVoxel& start, uint64_t key) {
    const int64_t px = int64_t(key & kPackMask) - kPackBias;
    const int64_t py = int64_t((key >> kPackBits) & kPackMask) - kPackBias;
    const int64_t pz = int64_t((key >> (2 * kPackBits)) & kPackMask) - kPackBias;
    return DeviceVoxel{static_cast<int>(start.x + px),
                       static_cast<int>(start.y + py),
                       static_cast<int>(start.z + pz)};
}

struct HostUpdate {
    DeviceVoxel voxel;
    int count;
};

/// Voxels the DDA visits before the endpoint. One-axis boundary stepping makes
/// this exactly the Manhattan distance, matching traceMisses() in the header.
__host__ __device__ int64_t rayLength(const DeviceVoxel& start, const DeviceVoxel& end) {
    const int64_t dx = end.x >= start.x ? static_cast<int64_t>(end.x) - start.x
                                        : static_cast<int64_t>(start.x) - end.x;
    const int64_t dy = end.y >= start.y ? static_cast<int64_t>(end.y) - start.y
                                        : static_cast<int64_t>(start.y) - end.y;
    const int64_t dz = end.z >= start.z ? static_cast<int64_t>(end.z) - start.z
                                        : static_cast<int64_t>(start.z) - end.z;
    return dx + dy + dz;
}

/**
 * One thread per ray; output ranges are precomputed and non-overlapping.
 * Emits packed 63-bit voxel keys (relative to @p start) so the reduction can
 * radix-sort them.  Only rays whose length fits the packable range are ever
 * batched into a chunk (see the admission test in insertPointCloudCuda), so
 * packing never overflows here.
 */
__global__ void traceMissKernel(DeviceVoxel start,
                                const DeviceVoxel* ends,
                                const int64_t* offsets,
                                int64_t ray_count,
                                uint64_t* output) {
    for (int64_t ray = blockIdx.x * static_cast<int64_t>(blockDim.x) + threadIdx.x;
         ray < ray_count;
         ray += static_cast<int64_t>(gridDim.x) * blockDim.x) {
        const DeviceVoxel end = ends[ray];
        int x = start.x, y = start.y, z = start.z;
        const int64_t dx_signed = static_cast<int64_t>(end.x) - x;
        const int64_t dy_signed = static_cast<int64_t>(end.y) - y;
        const int64_t dz_signed = static_cast<int64_t>(end.z) - z;
        const int sx = (dx_signed > 0) - (dx_signed < 0);
        const int sy = (dy_signed > 0) - (dy_signed < 0);
        const int sz = (dz_signed > 0) - (dz_signed < 0);
        const int64_t dx = dx_signed < 0 ? -dx_signed : dx_signed;
        const int64_t dy = dy_signed < 0 ? -dy_signed : dy_signed;
        const int64_t dz = dz_signed < 0 ? -dz_signed : dz_signed;
        int64_t write = offsets[ray];

        const double kInf = HUGE_VAL;
        double tMaxX = dx > 0 ? 0.5 / static_cast<double>(dx) : kInf;
        double tMaxY = dy > 0 ? 0.5 / static_cast<double>(dy) : kInf;
        double tMaxZ = dz > 0 ? 0.5 / static_cast<double>(dz) : kInf;
        const double tDeltaX = dx > 0 ? 1.0 / static_cast<double>(dx) : kInf;
        const double tDeltaY = dy > 0 ? 1.0 / static_cast<double>(dy) : kInf;
        const double tDeltaZ = dz > 0 ? 1.0 / static_cast<double>(dz) : kInf;

        const int64_t steps = dx + dy + dz;
        for (int64_t i = 0; i < steps; ++i) {
            output[write++] = packKey(start, DeviceVoxel{x, y, z});
            if (tMaxX <= tMaxY && tMaxX <= tMaxZ) { x += sx; tMaxX += tDeltaX; }
            else if (tMaxY <= tMaxZ)              { y += sy; tMaxY += tDeltaY; }
            else                                   { z += sz; tMaxZ += tDeltaZ; }
        }
    }
}

/**
 * Radix-sort the first @p n packed keys in the shared events buffer, run-length
 * encode them, and append the (voxel, count) updates to the host list.  @p start
 * is the frame used when the keys were packed, needed to unpack back to absolute
 * voxel coordinates.  All device temporaries come from the persistent scratch and
 * the cached allocator, so a steady-state frame performs no cudaMalloc/cudaFree.
 */
bool reduceUpdates(GpuScratch& s, size_t n,
                   DeviceVoxel start,
                   std::vector<HostUpdate>& output) {
    if (n == 0) return true;
    try {
        auto policy = thrust::cuda::par(s.alloc);   // thrust's internal temporaries reuse cached blocks
        const auto keys_begin = s.d_events.begin();
        const auto keys_end = keys_begin + static_cast<std::ptrdiff_t>(n);
        thrust::sort(policy, keys_begin, keys_end);   // integral keys -> radix sort
        if (s.d_unique.size() < n) s.d_unique.resize(n);
        if (s.d_counts.size() < n) s.d_counts.resize(n);
        const auto end = thrust::reduce_by_key(
            policy, keys_begin, keys_end,
            thrust::make_constant_iterator(1),
            s.d_unique.begin(), s.d_counts.begin());
        const size_t count = static_cast<size_t>(end.first - s.d_unique.begin());

        s.h_keys.resize(count);
        s.h_counts.resize(count);
        thrust::copy_n(s.d_unique.begin(), count, s.h_keys.begin());
        thrust::copy_n(s.d_counts.begin(), count, s.h_counts.begin());
        output.reserve(output.size() + count);
        for (size_t i = 0; i < count; ++i) {
            output.push_back(HostUpdate{unpackKey(start, s.h_keys[i]), s.h_counts[i]});
        }
        return cudaCheck(cudaGetLastError(), "reduce sparse voxel updates");
    } catch (const thrust::system_error& error) {
        std::cerr << "[nanovoxmap][cuda] sparse update reduction failed: "
                  << error.what() << std::endl;
    } catch (const std::bad_alloc&) {
        std::cerr << "[nanovoxmap][cuda] sparse update reduction ran out of memory" << std::endl;
    }
    cudaGetLastError();
    return false;
}

bool traceChunk(GpuScratch& s,
                DeviceVoxel start,
                const std::vector<DeviceVoxel>& ends,
                size_t first,
                size_t last,
                std::vector<HostUpdate>& updates) {
    const size_t ray_count = last - first;
    std::vector<int64_t> offsets(ray_count + 1, 0);
    for (size_t i = 0; i < ray_count; ++i) {
        offsets[i + 1] = offsets[i] + rayLength(start, ends[first + i]);
    }
    const int64_t event_count = offsets.back();
    if (event_count == 0) return true;

    try {
        // Reused device buffers: resize only grows, so no realloc after warm-up.
        if (s.d_ends.size() < ray_count) s.d_ends.resize(ray_count);
        if (s.d_offsets.size() < ray_count + 1) s.d_offsets.resize(ray_count + 1);
        if (s.d_events.size() < static_cast<size_t>(event_count))
            s.d_events.resize(static_cast<size_t>(event_count));
        thrust::copy(ends.begin() + first, ends.begin() + last, s.d_ends.begin());
        thrust::copy(offsets.begin(), offsets.end(), s.d_offsets.begin());
        const int threads = 256;
        traceMissKernel<<<gridFor(static_cast<int64_t>(ray_count), threads), threads>>>(
            start,
            thrust::raw_pointer_cast(s.d_ends.data()),
            thrust::raw_pointer_cast(s.d_offsets.data()),
            static_cast<int64_t>(ray_count),
            thrust::raw_pointer_cast(s.d_events.data()));
        if (!cudaCheck(cudaGetLastError(), "trace sparse ray chunk") ||
            !cudaCheck(cudaDeviceSynchronize(), "synchronize sparse ray chunk")) {
            return false;
        }
        return reduceUpdates(s, static_cast<size_t>(event_count), start, updates);
    } catch (const thrust::system_error& error) {
        std::cerr << "[nanovoxmap][cuda] sparse ray chunk failed: "
                  << error.what() << std::endl;
    } catch (const std::bad_alloc&) {
        std::cerr << "[nanovoxmap][cuda] sparse ray chunk ran out of memory" << std::endl;
    }
    cudaGetLastError();
    return false;
}

// ===========================================================================
// Signed ESDF solve
// ===========================================================================
//
// The same exact separable Felzenszwalb-Huttenlocher transform the CPU runs,
// mapped onto the GPU the way the algorithm invites: each of the three passes
// is a set of completely independent 1-D transforms over lines of the box, so
// one thread per line needs no synchronization and no atomics.  Jump flooding
// would have been the more common GPU choice but is only approximate; keeping
// the exact algorithm means the GPU and CPU fields agree, which is what makes
// the runtime fallback invisible.

/// Must equal OccupancyMap::kFieldInf so the two backends saturate identically.
constexpr float kEsdfInfDevice = 1e18f;

struct EsdfScratch {
    thrust::device_vector<uint8_t> occupancy;
    thrust::device_vector<float>   a, b, c;   // ping-pong + the retained pass result
    thrust::device_vector<int>     v;         // parabola vertices, one row per line
    thrust::device_vector<float>   z;         // envelope breakpoints, one row per line
};

EsdfScratch& esdfScratch() {
    static EsdfScratch* s = new EsdfScratch();  // leaked deliberately, as above
    return *s;
}

__global__ void seedEsdfKernel(const uint8_t* occupancy, float* outside, float* inside,
                               int64_t total) {
    for (int64_t i = blockIdx.x * static_cast<int64_t>(blockDim.x) + threadIdx.x;
         i < total;
         i += static_cast<int64_t>(gridDim.x) * blockDim.x) {
        const bool occupied = occupancy[i] != 0;
        outside[i] = occupied ? 0.0f : kEsdfInfDevice;
        inside[i] = occupied ? kEsdfInfDevice : 0.0f;
    }
}

/**
 * One thread transforms one line of @p src into @p dst.
 *
 * A line is addressed by (base, stride); the line id decomposes into the two
 * axes orthogonal to the pass via @p mod / @p s0 / @p s1, which is what lets a
 * single kernel serve all three passes.  The per-thread scratch rows are
 * interleaved (`row * num_lines + line`) rather than contiguous per thread, so
 * neighbouring threads touch neighbouring addresses on every step and the
 * accesses coalesce.
 *
 * Writing to a separate destination is what keeps the envelope sampling
 * correct: it reads src[v[k]], which an in-place pass could already have
 * overwritten.
 */
__global__ void dtLineKernel(const float* src, float* dst,
                             int64_t n, int64_t stride, int64_t num_lines,
                             int64_t mod, int64_t s0, int64_t s1,
                             int* v, float* z) {
    for (int64_t line = blockIdx.x * static_cast<int64_t>(blockDim.x) + threadIdx.x;
         line < num_lines;
         line += static_cast<int64_t>(gridDim.x) * blockDim.x) {
        const int64_t base = (line % mod) * s0 + (line / mod) * s1;
        const float* in = src + base;
        float* out = dst + base;

        int64_t first = 0;
        while (first < n && in[first * stride] >= kEsdfInfDevice) ++first;
        if (first >= n) {                       // no seed on this line: stays empty
            for (int64_t q = 0; q < n; ++q) out[q * stride] = kEsdfInfDevice;
            continue;
        }

        int64_t k = 0;
        v[line] = static_cast<int>(first);
        z[line] = -kEsdfInfDevice;
        z[num_lines + line] = kEsdfInfDevice;
        for (int64_t q = first + 1; q < n; ++q) {
            const float fq = in[q * stride];
            if (fq >= kEsdfInfDevice) continue;   // an empty cell contributes no parabola
            float s;
            while (true) {
                const int vk = v[k * num_lines + line];
                const float vkf = static_cast<float>(vk);
                s = ((fq + static_cast<float>(q) * static_cast<float>(q)) -
                     (in[static_cast<int64_t>(vk) * stride] + vkf * vkf)) /
                    (2.0f * (static_cast<float>(q) - vkf));
                if (s > z[k * num_lines + line]) break;
                --k;
            }
            ++k;
            v[k * num_lines + line] = static_cast<int>(q);
            z[k * num_lines + line] = s;
            z[(k + 1) * num_lines + line] = kEsdfInfDevice;
        }

        k = 0;
        for (int64_t q = 0; q < n; ++q) {
            while (z[(k + 1) * num_lines + line] < static_cast<float>(q)) ++k;
            const int vk = v[k * num_lines + line];
            const float d = static_cast<float>(q) - static_cast<float>(vk);
            out[q * stride] = d * d + in[static_cast<int64_t>(vk) * stride];
        }
    }
}

/**
 * Combine the two squared-distance fields into one saturated signed field.
 * The half-voxel shift matches solveSignedEsdfCpu exactly -- see the comment
 * there for why the zero crossing has to land on the face rather than the
 * voxel centre.
 */
__global__ void combineEsdfKernel(const uint8_t* occupancy, const float* outside,
                                  const float* inside, float band, float* out,
                                  int64_t total) {
    for (int64_t i = blockIdx.x * static_cast<int64_t>(blockDim.x) + threadIdx.x;
         i < total;
         i += static_cast<int64_t>(gridDim.x) * blockDim.x) {
        const float d = occupancy[i] != 0 ? -(sqrtf(inside[i]) - 0.5f)
                                          : (sqrtf(outside[i]) - 0.5f);
        out[i] = fminf(band, fmaxf(-band, d));
    }
}

/**
 * Run the three separable passes over @p src / @p dst, returning the buffer
 * holding the result (the pointers ping-pong, so which one that is depends on
 * the pass count).
 */
bool runDistanceTransform(float*& src, float*& dst, int64_t nx, int64_t ny, int64_t nz,
                          int* v, float* z) {
    const int64_t sxy = nx * ny;
    const int threads = 128;
    struct Pass {
        int64_t n, stride, num_lines, mod, s0, s1;
    };
    const Pass passes[3] = {
        {nx, 1,   ny * nz, ny, nx,  sxy},   // lines along x, indexed by (y, z)
        {ny, nx,  nx * nz, nx, 1,   sxy},   // lines along y, indexed by (x, z)
        {nz, sxy, nx * ny, nx, 1,   nx},    // lines along z, indexed by (x, y)
    };
    for (const Pass& pass : passes) {
        dtLineKernel<<<gridFor(pass.num_lines, threads), threads>>>(
            src, dst, pass.n, pass.stride, pass.num_lines,
            pass.mod, pass.s0, pass.s1, v, z);
        if (!cudaCheck(cudaGetLastError(), "esdf separable pass")) return false;
        std::swap(src, dst);
    }
    return cudaCheck(cudaDeviceSynchronize(), "synchronize esdf transform");
}

}  // namespace

namespace NanoVoxMap {

template <typename Scalar>
GpuSolveStatus OccupancyMap<Scalar>::solveSignedEsdfCuda(
    const std::vector<uint8_t>& occupancy,
    Idx nx, Idx ny, Idx nz, float band,
    std::vector<float>& out) const {
    const int64_t total = static_cast<int64_t>(occupancy.size());
    if (total == 0 || nx <= 0 || ny <= 0 || nz <= 0) return GpuSolveStatus::kNotEligible;
    if (!haveCudaDevice()) return GpuSolveStatus::kDeviceFailed;
    // Too big for the memory budget is a routing decision, not a device fault:
    // the CPU solves this box and the GPU still gets the next one.
    if (total > maxEsdfCells()) return GpuSolveStatus::kNotEligible;

    // Each pass writes one vertex and one breakpoint row per line, and for every
    // pass n * num_lines is exactly the cell count -- so the scratch is `total`
    // entries, not max_n * max_lines. Sizing it from the per-axis maxima instead
    // over-allocates badly on any box that is not a cube (a flat region asks for
    // several times the field itself, and a very flat one for orders of
    // magnitude more), which on an integrated GPU is memory taken from the rest
    // of the system for nothing.
    const int64_t max_lines = std::max(ny * nz, std::max(nx * nz, nx * ny));

    std::lock_guard<std::mutex> gpu_lock(gpuMutex());
    EsdfScratch& s = esdfScratch();
    try {
        const size_t cells = static_cast<size_t>(total);
        if (s.occupancy.size() < cells) s.occupancy.resize(cells);
        if (s.a.size() < cells) s.a.resize(cells);
        if (s.b.size() < cells) s.b.resize(cells);
        if (s.c.size() < cells) s.c.resize(cells);
        const size_t v_cells = cells;
        const size_t z_cells = cells + static_cast<size_t>(max_lines);
        if (s.v.size() < v_cells) s.v.resize(v_cells);
        if (s.z.size() < z_cells) s.z.resize(z_cells);

        thrust::copy(occupancy.begin(), occupancy.end(), s.occupancy.begin());
        uint8_t* d_occ = thrust::raw_pointer_cast(s.occupancy.data());
        float* d_a = thrust::raw_pointer_cast(s.a.data());
        float* d_b = thrust::raw_pointer_cast(s.b.data());
        float* d_c = thrust::raw_pointer_cast(s.c.data());
        int* d_v = thrust::raw_pointer_cast(s.v.data());
        float* d_z = thrust::raw_pointer_cast(s.z.data());

        const int threads = 256;
        const int seed_blocks = gridFor(total, threads);
        // Seed outside into a (distance out of free space to a surface) and
        // inside into c (depth of a solid cell below its surface).
        seedEsdfKernel<<<seed_blocks, threads>>>(d_occ, d_a, d_c, total);
        if (!cudaCheck(cudaGetLastError(), "seed esdf")) return GpuSolveStatus::kDeviceFailed;

        // Outside pass ping-pongs a/b and leaves its result in one of them;
        // the inside pass then uses c and whichever of a/b came free.
        float* src = d_a;
        float* dst = d_b;
        if (!runDistanceTransform(src, dst, nx, ny, nz, d_v, d_z)) {
            return GpuSolveStatus::kDeviceFailed;
        }
        float* outside = src;
        float* spare = dst;

        float* in_src = d_c;
        float* in_dst = spare;
        if (!runDistanceTransform(in_src, in_dst, nx, ny, nz, d_v, d_z)) {
            return GpuSolveStatus::kDeviceFailed;
        }
        float* inside = in_src;
        float* result = in_dst;

        combineEsdfKernel<<<seed_blocks, threads>>>(
            d_occ, outside, inside, band, result, total);
        if (!cudaCheck(cudaGetLastError(), "combine signed esdf") ||
            !cudaCheck(cudaDeviceSynchronize(), "synchronize signed esdf")) {
            return GpuSolveStatus::kDeviceFailed;
        }

        out.resize(static_cast<size_t>(total));
        thrust::copy_n(thrust::device_pointer_cast(result), cells, out.begin());
        return GpuSolveStatus::kSolved;
    } catch (const thrust::system_error& error) {
        std::cerr << "[nanovoxmap][cuda] signed esdf solve failed: " << error.what() << std::endl;
    } catch (const std::bad_alloc&) {
        std::cerr << "[nanovoxmap][cuda] signed esdf solve ran out of memory" << std::endl;
    }
    cudaGetLastError();
    return GpuSolveStatus::kDeviceFailed;
}

template GpuSolveStatus OccupancyMap<double>::solveSignedEsdfCuda(
    const std::vector<uint8_t>&, std::int64_t, std::int64_t, std::int64_t, float,
    std::vector<float>&) const;
template GpuSolveStatus OccupancyMap<float>::solveSignedEsdfCuda(
    const std::vector<uint8_t>&, std::int64_t, std::int64_t, std::int64_t, float,
    std::vector<float>&) const;


template <typename Scalar>
bool OccupancyMap<Scalar>::insertPointCloudCuda(
    const Vec3& origin, const std::vector<Vec3>& endpoints, bool apply_hits) {
    if (cuda_unavailable_) return false;
    if (!haveCudaDevice()) {
        cuda_unavailable_ = true;
        return false;
    }

    GridIndex start_index;
    if (!worldToGrid(origin, start_index)) return true;
    const DeviceVoxel start{start_index.x, start_index.y, start_index.z};

    std::vector<DeviceVoxel> valid_ends;
    valid_ends.reserve(endpoints.size());
    for (const Vec3& endpoint : endpoints) {
        GridIndex end;
        if (worldToGrid(endpoint, end)) valid_ends.push_back(DeviceVoxel{end.x, end.y, end.z});
    }
    if (valid_ends.empty()) return true;

    // Serialize GPU use and bind the persistent scratch for this frame.  The
    // node already holds its map mutex here; this additionally guards the shared
    // device buffers if two map instances ever share the one GPU.
    std::lock_guard<std::mutex> gpu_lock(gpuMutex());
    GpuScratch& s = scratch();

    // Put reduced updates in block-major order, resolve/create blocks before
    // launching workers so the lookup table is read-only during the parallel
    // phase, then give each block to exactly one worker. Occupancy masks are
    // block-local and the global count is atomic; ESDF dirty keys are committed
    // after the join.
    const auto merge_reduced = [this](std::vector<HostUpdate>& updates, bool hits) {
        if (updates.empty()) return;
        // Packed-key order is voxel-lexicographic, not block-major: successive
        // z slices of one 8^3 block can be separated by other x/y blocks. Put
        // the compact (already voxel-unique) list into block-major order before
        // assigning groups, otherwise two workers could mutate one block.
        std::sort(updates.begin(), updates.end(), [this](const HostUpdate& a,
                                                         const HostUpdate& b) {
            const BlockIndex ab = blockIndex(GridIndex{a.voxel.x, a.voxel.y, a.voxel.z});
            const BlockIndex bb = blockIndex(GridIndex{b.voxel.x, b.voxel.y, b.voxel.z});
            if (ab.z != bb.z) return ab.z < bb.z;
            if (ab.y != bb.y) return ab.y < bb.y;
            if (ab.x != bb.x) return ab.x < bb.x;
            if (a.voxel.z != b.voxel.z) return a.voxel.z < b.voxel.z;
            if (a.voxel.y != b.voxel.y) return a.voxel.y < b.voxel.y;
            return a.voxel.x < b.voxel.x;
        });
        struct Group {
            size_t begin;
            size_t end;
            BlockIndex key;
            VoxelBlock* block;
        };
        std::vector<Group> groups;
        groups.reserve(updates.size() / 32 + 1);
        size_t begin = 0;
        while (begin < updates.size()) {
            const GridIndex first_voxel{updates[begin].voxel.x,
                                        updates[begin].voxel.y,
                                        updates[begin].voxel.z};
            const BlockIndex key = blockIndex(first_voxel);
            size_t end = begin + 1;
            while (end < updates.size()) {
                const GridIndex voxel{updates[end].voxel.x,
                                      updates[end].voxel.y,
                                      updates[end].voxel.z};
                if (!(blockIndex(voxel) == key)) break;
                ++end;
            }
            groups.push_back(Group{begin, end, key, &findOrCreateBlock(key)});
            begin = end;
        }

        const unsigned available = std::max(1u, std::thread::hardware_concurrency());
        const unsigned workers = updates.size() >= 4096
            ? std::min<unsigned>(available, static_cast<unsigned>(groups.size())) : 1u;
        std::vector<uint8_t> dirty(groups.size(), 0);
        const auto run = [&](size_t group_begin, size_t group_end) {
            const int step = hits ? static_cast<int>(l_hit_) : static_cast<int>(l_miss_);
            for (size_t g = group_begin; g < group_end; ++g) {
                Group& group = groups[g];
                VoxelBlock& block = *group.block;
                bool occupancy_changed = false;
                for (size_t i = group.begin; i < group.end; ++i) {
                    const DeviceVoxel& dv = updates[i].voxel;
                    const GridIndex voxel{dv.x, dv.y, dv.z};
                    const size_t offset = localOffset(localCoordinate(voxel.x),
                                                      localCoordinate(voxel.y),
                                                      localCoordinate(voxel.z));
                    int8_t& value = block.voxels[offset];
                    const int8_t before = value;
                    int raw = static_cast<int>(before) + step * updates[i].count;
                    if (raw < l_min_) raw = l_min_;
                    else if (raw > l_max_) raw = l_max_;
                    value = static_cast<int8_t>(raw);
                    if ((before > 0) == (value > 0)) continue;
                    uint64_t& word = block.occupied_mask[offset >> 6];
                    const uint64_t bit = uint64_t{1} << (offset & 63);
                    if (value > 0) {
                        word |= bit;
                        ++block.occupied_count;
                        occupied_count_.fetch_add(1, std::memory_order_relaxed);
                    } else {
                        word &= ~bit;
                        --block.occupied_count;
                        occupied_count_.fetch_sub(1, std::memory_order_relaxed);
                    }
                    occupancy_changed = true;
                }
                dirty[g] = occupancy_changed ? 1 : 0;
            }
        };

        if (workers == 1) {
            run(0, groups.size());
        } else {
            std::vector<std::thread> threads;
            threads.reserve(workers);
            for (unsigned worker = 0; worker < workers; ++worker) {
                const size_t first_group = groups.size() * worker / workers;
                const size_t last_group = groups.size() * (worker + 1) / workers;
                threads.emplace_back(run, first_group, last_group);
            }
            for (std::thread& thread : threads) thread.join();
        }
        for (size_t g = 0; g < groups.size(); ++g)
            if (dirty[g]) esdf_dirty_.insert(groups[g].key);
    };

    // Each completed chunk is immediately merged into the host block hash.
    // If a later device operation fails, only the unprocessed misses and the
    // still-pending hit pass are completed on the CPU.
    const auto apply_hits_on_cpu = [this, &valid_ends]() {
        BlockCursor cursor;
        for (const DeviceVoxel& end : valid_ends) {
            applyHitCached(cursor, GridIndex{end.x, end.y, end.z});
        }
    };

    const auto finish_misses_on_cpu = [this, &start_index, &valid_ends](size_t first_ray) {
        BlockCursor cursor;
        for (size_t i = first_ray; i < valid_ends.size(); ++i) {
            const DeviceVoxel& end = valid_ends[i];
            traceMisses(start_index, GridIndex{end.x, end.y, end.z},
                        [&](const GridIndex& voxel) { applyMissCached(cursor, voxel); });
        }
    };

    size_t first = 0;
    while (first < valid_ends.size()) {
        // An exceptionally long ray would defeat the chunk budget and, more
        // importantly, cannot be encoded as a packed key.  It is uncommon on
        // robots (sensor range is normally clipped), so handle it locally and
        // keep the device allocation strictly bounded.
        if (rayLength(start, valid_ends[first]) > kMaxRayVoxels) {
            const DeviceVoxel& end = valid_ends[first];
            traceMisses(start_index, GridIndex{end.x, end.y, end.z},
                        [this](const GridIndex& voxel) { applyMissIdx(voxel); });
            ++first;
            continue;
        }
        size_t last = first;
        int64_t events = 0;
        while (last < valid_ends.size()) {
            const int64_t length = rayLength(start, valid_ends[last]);
            // Admission test, applied to every ray and not only to the chunk's
            // first: an unpackable ray let into a batch would wrap its key and
            // silently write misses into the wrong voxels.  Ending the chunk
            // here hands it to the CPU branch above on the next iteration.
            if (length > kMaxRayVoxels) break;
            if (last > first && events + length > kMaxMissEventsPerChunk) break;
            events += length;
            ++last;
            if (events >= kMaxMissEventsPerChunk) break;
        }
        if (last == first) continue;   // cannot happen: the long-ray branch consumed it
        std::vector<HostUpdate> chunk_updates;
        if (!traceChunk(s, start, valid_ends, first, last, chunk_updates)) {
            // Earlier chunks are already committed.  Complete only the
            // remaining misses and all hits on the CPU, then latch fallback.
            finish_misses_on_cpu(first);
            if (apply_hits) apply_hits_on_cpu();
            cuda_unavailable_ = true;
            return true;
        }
        // chunk_updates arrive sorted by (x,y,z) from reduce_by_key, so
        // consecutive updates land in the same block: a memoized cursor turns
        // the host merge into ~one hash lookup per block instead of per voxel.
        merge_reduced(chunk_updates, false);
        first = last;
    }

    // Clearing rays mark only the traversed free space; no occupied hit is
    // stamped at the (clamped, over-range) endpoint.  All misses are committed
    // above, so we are done.
    if (!apply_hits) return true;

    // Endpoints from long rays whose misses were traced on CPU can lie outside
    // the packable range; if any does, apply all hits on CPU for this frame
    // (still cheap: one per endpoint) without latching GPU fallback.
    std::vector<uint64_t> hit_keys;
    hit_keys.reserve(valid_ends.size());
    for (const DeviceVoxel& end : valid_ends) {
        if (!packable(start, end)) { apply_hits_on_cpu(); return true; }
        hit_keys.push_back(packKey(start, end));
    }

    std::vector<HostUpdate> hit_updates;
    try {
        // Reuse the shared events buffer for the hit keys, then reduce in place.
        if (s.d_events.size() < hit_keys.size()) s.d_events.resize(hit_keys.size());
        thrust::copy(hit_keys.begin(), hit_keys.end(), s.d_events.begin());
        if (!reduceUpdates(s, hit_keys.size(), start, hit_updates)) {
            apply_hits_on_cpu();
            cuda_unavailable_ = true;
            return true;
        }
    } catch (const thrust::system_error& error) {
        std::cerr << "[nanovoxmap][cuda] endpoint reduction failed: "
                  << error.what() << std::endl;
        cudaGetLastError();
        apply_hits_on_cpu();
        cuda_unavailable_ = true;
        return true;
    } catch (const std::bad_alloc&) {
        std::cerr << "[nanovoxmap][cuda] endpoint reduction ran out of memory" << std::endl;
        cudaGetLastError();
        apply_hits_on_cpu();
        cuda_unavailable_ = true;
        return true;
    }
    merge_reduced(hit_updates, true);
    return true;
}

template bool OccupancyMap<double>::insertPointCloudCuda(
    const OccupancyMap<double>::Vec3&,
    const std::vector<OccupancyMap<double>::Vec3>&, bool);
template bool OccupancyMap<float>::insertPointCloudCuda(
    const OccupancyMap<float>::Vec3&,
    const std::vector<OccupancyMap<float>::Vec3>&, bool);

}  // namespace NanoVoxMap

#endif  // NANOVOXMAP_WITH_CUDA
