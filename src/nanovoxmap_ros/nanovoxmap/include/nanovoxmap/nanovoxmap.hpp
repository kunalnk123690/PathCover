/**
 * @file nanovoxmap.hpp
 * @brief ROS-independent, sparse block-hashed 3D occupancy map.
 *
 * The map allocates small dense voxel blocks only where rays have been
 * observed.  World coordinates are therefore not clipped to a preconfigured
 * box and memory grows with explored space rather than map volume.
 */

#ifndef NANOVOXMAP_HPP
#define NANOVOXMAP_HPP

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace NanoVoxMap {

/**
 * Outcome of an attempted GPU solve.
 *
 * "The device could not do this" and "the device is broken" are different
 * events and must not share a return value: the first is a per-region routing
 * decision (an oversized box goes to the CPU) while the second is what latches
 * the CPU fallback for the rest of the process.  Collapsing them into a bool
 * means one unusually large region permanently disables the GPU.
 */
enum class GpuSolveStatus {
    kSolved,        ///< The GPU produced the field.
    kNotEligible,   ///< Not routed to the GPU; the CPU takes it and the GPU stays enabled.
    kDeviceFailed,  ///< The device/driver failed; latch the CPU fallback.
};

enum class Backend { kAuto, kCpu, kCuda };

/**
 * @brief Unbounded sparse log-odds voxel map.
 *
 * Space is split into 8x8x8 blocks.  A hash lookup finds a block and access
 * inside a block is a cache-friendly array lookup.  Unknown blocks consume no
 * storage.  The class is not internally synchronized; callers sharing an
 * instance across threads must provide a mutex.
 */
template <typename Scalar = double>
class OccupancyMap {
    static_assert(std::is_floating_point<Scalar>::value,
                  "OccupancyMap: Scalar must be a floating-point type");

public:
    using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
    using Idx = std::int64_t;

    /** Construct an unbounded map whose voxel lattice is anchored at world zero. */
    explicit OccupancyMap(Scalar resolution,
                          Scalar p_hit = Scalar(0.7),
                          Scalar p_miss = Scalar(0.4),
                          Scalar p_min = Scalar(0.1192),
                          Scalar p_max = Scalar(0.971),
                          Backend backend = Backend::kAuto,
                          bool allow_cuda_fallback = true)
        : resolution_(checkedResolution(resolution)),
          inv_resolution_(Scalar(1) / resolution_),
          l_hit_(toLogOddsUnitsChecked(p_hit)),
          l_miss_(toLogOddsUnitsChecked(p_miss)),
          l_min_(toLogOddsUnitsChecked(p_min)),
          l_max_(toLogOddsUnitsChecked(p_max)),
          requested_backend_(backend),
          allow_cuda_fallback_(allow_cuda_fallback) {
        if (!(p_min < p_max)) {
            throw std::invalid_argument("OccupancyMap: p_min must be less than p_max");
        }
        if (!(p_hit > Scalar(0.5)) || !(p_miss < Scalar(0.5)) ||
            !(p_min < Scalar(0.5)) || !(p_max > Scalar(0.5))) {
            throw std::invalid_argument(
                "OccupancyMap: require p_hit/p_max > 0.5 and p_miss/p_min < 0.5");
        }
        blocks_.reserve(1024);
        // Default ESDF truncation: 1 m of clearance is more than a local
        // planner reads, and keeping the band short is what keeps both the
        // materialized band and the incremental update region small.
        setEsdfMaxDistance(Scalar(1));
#ifdef NANOVOXMAP_WITH_CUDA
        cuda_unavailable_ = backend == Backend::kCpu;
        esdf_cuda_unavailable_ = backend == Backend::kCpu;
#else
        if (backend == Backend::kCuda && !allow_cuda_fallback_) {
            throw std::runtime_error("OccupancyMap: CUDA backend requested in a CPU-only build");
        }
#endif
    }

    /**
     * Snapshot occupied voxels using a temporary dense indexing frame.
     *
     * Storage remains sparse.  Flattening happens only to retain compatibility
     * with the existing ROS publisher, which expects a lower corner plus dense
     * indices.  The frame tightly encloses the occupied voxels at snapshot time.
     */
    void snapshotOccupied(std::vector<Idx>& out,
                          Vec3& lower_bound,
                          Scalar& resolution,
                          Idx& size_x,
                          Idx& size_xy) const {
        out.clear();
        resolution = resolution_;
        if (numOccupied() == 0) {
            lower_bound.setZero();
            size_x = 1;
            size_xy = 1;
            return;
        }

        GridIndex lo, hi;
        occupiedBounds(lo, hi);
        const Idx sx = extent(lo.x, hi.x);
        const Idx sy = extent(lo.y, hi.y);
        const Idx sz = extent(lo.z, hi.z);
        checkedDenseSize(sx, sy, sz);
        size_x = sx;
        size_xy = checkedMultiply(sx, sy);
        lower_bound = voxelCornerToWorld(lo);

        out.reserve(numOccupied());
        for (const auto& item : blocks_) {
            const BlockIndex& block_index = item.first;
            const VoxelBlock& block = blockAt(item.second);
            forEachOccupiedOffset(block, [&](int offset) {
                const GridIndex v = blockLocalToGrid(
                    block_index, offset & kVoxelMask,
                    (offset >> kVoxelShift) & kVoxelMask,
                    offset >> (2 * kVoxelShift));
                const Idx x = static_cast<Idx>(v.x) - lo.x;
                const Idx y = static_cast<Idx>(v.y) - lo.y;
                const Idx z = static_cast<Idx>(v.z) - lo.z;
                out.push_back(z * size_xy + y * size_x + x);
            });
        }
    }

    void setOccupied(const Vec3& point) {
        GridIndex idx;
        if (worldToGrid(point, idx)) setOccupiedIdx(idx);
    }

    void setFree(const Vec3& point) {
        GridIndex idx;
        if (worldToGrid(point, idx)) setFreeIdx(idx);
    }

    bool isOccupied(const Vec3& point) const {
        GridIndex idx;
        return worldToGrid(point, idx) && voxelValue(idx) > 0;
    }

    /** Insert one unbounded ray using a 3-D Bresenham traversal. */
    void insertRay(const Vec3& origin, const Vec3& endpoint) {
        GridIndex start, end;
        if (!worldToGrid(origin, start) || !worldToGrid(endpoint, end)) return;
        BlockCursor cursor;
        traceMisses(start, end,
                    [&](const GridIndex& v) { applyMissCached(cursor, v); });
        applyHitCached(cursor, end);
        invalidateEsdf();
    }

    /**
     * Insert a point cloud.  CUDA builds try sparse GPU ray traversal first;
     * CPU-only machines execute the same update ordering locally.
     *
     * With @p apply_hits false the endpoints are treated as clearing rays: the
     * traversed space is marked free (misses) but no occupied hit is applied at
     * the endpoint.  This is how over-range / no-return beams sweep out voxels a
     * removed obstacle used to occupy, without stamping a false surface at the
     * clamped range.
     */
    void insertPointCloud(const Vec3& origin, const std::vector<Vec3>& endpoints,
                          bool apply_hits = true) {
        if (endpoints.empty()) return;
#ifdef NANOVOXMAP_WITH_CUDA
        if (insertPointCloudCuda(origin, endpoints, apply_hits)) {
            if (!apply_hits) applyClearingEndpoints(endpoints);
            invalidateEsdf();
            return;
        }
#endif
        if (requested_backend_ == Backend::kCuda && !allow_cuda_fallback_) {
            throw std::runtime_error("OccupancyMap: CUDA backend unavailable and fallback disabled");
        }
        insertPointCloudCpu(origin, endpoints, apply_hits);
        if (!apply_hits) applyClearingEndpoints(endpoints);
        invalidateEsdf();
    }

    std::vector<Vec3> getOccupiedVoxels() const {
        std::vector<Vec3> result;
        result.reserve(numOccupied());
        for (const auto& item : blocks_) {
            forEachOccupiedOffset(blockAt(item.second), [&](int offset) {
                result.push_back(gridToWorld(blockLocalToGrid(
                    item.first, offset & kVoxelMask,
                    (offset >> kVoxelShift) & kVoxelMask,
                    offset >> (2 * kVoxelShift))));
            });
        }
        return result;
    }

    size_t numOccupied() const { return occupied_count_.load(std::memory_order_relaxed); }

    Backend requestedBackend() const { return requested_backend_; }
    Backend activeBackend() const {
#ifdef NANOVOXMAP_WITH_CUDA
        return cuda_unavailable_ ? Backend::kCpu : Backend::kCuda;
#else
        return Backend::kCpu;
#endif
    }
    bool allowsCudaFallback() const { return allow_cuda_fallback_; }

    /**
     * Order-independent fingerprint of the full map state, for tests/debugging.
     * `.voxels` folds every known voxel's (x, y, z, log-odds); `.occupied`
     * folds the occupied-index set.  Two maps compare equal iff both fields
     * match, which lets the serial and parallel raycast paths be checked for
     * exact equivalence despite unordered iteration order.
     */
    struct Fingerprint {
        uint64_t voxels = 0;
        uint64_t occupied = 0;
        bool operator==(const Fingerprint& o) const {
            return voxels == o.voxels && occupied == o.occupied;
        }
    };
    Fingerprint fingerprint() const {
        Fingerprint fp;
        for (const auto& item : blocks_) {
            const BlockIndex& b = item.first;
            const VoxelBlock& block = blockAt(item.second);
            for (int off = 0; off < kVoxelsPerBlock; ++off) {
                const int8_t value = block.voxels[off];
                if (value == kLogOddsUnknown) continue;
                const GridIndex v = blockLocalToGrid(b, off & kVoxelMask,
                                                     (off >> kVoxelShift) & kVoxelMask,
                                                     off >> (2 * kVoxelShift));
                const uint64_t keyed = IndexHash{}(v) ^
                    (static_cast<uint64_t>(static_cast<uint8_t>(value)) + 0x100u);
                fp.voxels ^= IndexHash::mix(keyed);
            }
        }
        for (const auto& item : blocks_) {
            forEachOccupiedOffset(blockAt(item.second), [&](int offset) {
                const GridIndex v = blockLocalToGrid(
                    item.first, offset & kVoxelMask,
                    (offset >> kVoxelShift) & kVoxelMask,
                    offset >> (2 * kVoxelShift));
                fp.occupied ^= IndexHash::mix(IndexHash{}(v) + 0x2000u);
            });
        }
        return fp;
    }

    size_t numAllocatedBlocks() const { return blocks_.size(); }

    size_t allocatedBytes() const { return block_count_ * sizeof(VoxelBlock); }

    void clear() {
        blocks_.clear();
        block_pages_.clear();
        block_count_ = 0;
        occupied_count_.store(0, std::memory_order_relaxed);
        esdf_nodes_.clear();
        esdf_blocks_.clear();
        esdf_dirty_.clear();
        esdf_field_valid_ = false;
        invalidateEsdf();
    }

    /** Size of the currently allocated block extent, not a fixed map bound. */
    Eigen::Vector3i getGridSize() const {
        if (blocks_.empty()) return Eigen::Vector3i::Zero();
        GridIndex lo, hi;
        allocatedVoxelBounds(lo, hi);
        const Idx sx = extent(lo.x, hi.x);
        const Idx sy = extent(lo.y, hi.y);
        const Idx sz = extent(lo.z, hi.z);
        if (sx > std::numeric_limits<int>::max() ||
            sy > std::numeric_limits<int>::max() ||
            sz > std::numeric_limits<int>::max()) {
            throw std::length_error("OccupancyMap: allocated extent exceeds Eigen::Vector3i");
        }
        return Eigen::Vector3i(static_cast<int>(sx), static_cast<int>(sy), static_cast<int>(sz));
    }

    /** Materialize the allocated sparse extent as a conventional 2-D grid. */
    std::vector<signed char> getMap2D() const {
        if (blocks_.empty()) return {};
        GridIndex lo, hi;
        allocatedVoxelBounds(lo, hi);
        const Idx sx = extent(lo.x, hi.x);
        const Idx sy = extent(lo.y, hi.y);
        const size_t total = checkedVectorSize(sx, sy, 1);
        std::vector<signed char> map(total, static_cast<signed char>(-1));

        for (const auto& item : blocks_) {
            const BlockIndex& b = item.first;
            const VoxelBlock& block = blockAt(item.second);
            for (int lz = 0; lz < kVoxelsPerSide; ++lz) {
                for (int ly = 0; ly < kVoxelsPerSide; ++ly) {
                    for (int lx = 0; lx < kVoxelsPerSide; ++lx) {
                        const int8_t value = block.voxels[localOffset(lx, ly, lz)];
                        if (value == kLogOddsUnknown) continue;
                        const GridIndex v = blockLocalToGrid(b, lx, ly, lz);
                        const size_t out = static_cast<size_t>(static_cast<Idx>(v.y) - lo.y) *
                                           static_cast<size_t>(sx) +
                                           static_cast<size_t>(static_cast<Idx>(v.x) - lo.x);
                        if (value > 0) map[out] = 100;
                        else if (map[out] != 100) map[out] = 0;
                    }
                }
            }
        }
        return map;
    }

    /** Materialize the allocated sparse extent as a conventional 3-D grid. */
    std::vector<signed char> getMap3D() const {
        if (blocks_.empty()) return {};
        GridIndex lo, hi;
        allocatedVoxelBounds(lo, hi);
        const Idx sx = extent(lo.x, hi.x);
        const Idx sy = extent(lo.y, hi.y);
        const Idx sz = extent(lo.z, hi.z);
        const size_t total = checkedVectorSize(sx, sy, sz);
        const size_t sxy = static_cast<size_t>(checkedMultiply(sx, sy));
        std::vector<signed char> map(total, static_cast<signed char>(-1));

        for (const auto& item : blocks_) {
            const BlockIndex& b = item.first;
            const VoxelBlock& block = blockAt(item.second);
            for (int lz = 0; lz < kVoxelsPerSide; ++lz) {
                for (int ly = 0; ly < kVoxelsPerSide; ++ly) {
                    for (int lx = 0; lx < kVoxelsPerSide; ++lx) {
                        const int8_t value = block.voxels[localOffset(lx, ly, lz)];
                        if (value == kLogOddsUnknown) continue;
                        const GridIndex v = blockLocalToGrid(b, lx, ly, lz);
                        const size_t x = static_cast<size_t>(static_cast<Idx>(v.x) - lo.x);
                        const size_t y = static_cast<size_t>(static_cast<Idx>(v.y) - lo.y);
                        const size_t z = static_cast<size_t>(static_cast<Idx>(v.z) - lo.z);
                        map[z * sxy + y * static_cast<size_t>(sx) + x] =
                            static_cast<signed char>(value > 0 ? 100 : 0);
                    }
                }
            }
        }
        return map;
    }

    /** Rebuild the balanced sparse index of occupied ESDF sites. */
    void computeEsdf() const {
        std::vector<GridIndex> sites;
        sites.reserve(numOccupied());
        for (const auto& item : blocks_) {
            forEachOccupiedOffset(blockAt(item.second), [&](int offset) {
                sites.push_back(blockLocalToGrid(
                    item.first, offset & kVoxelMask,
                    (offset >> kVoxelShift) & kVoxelMask,
                    offset >> (2 * kVoxelShift)));
            });
        }
        esdf_nodes_.clear();
        esdf_nodes_.reserve(sites.size());
        buildEsdfTree(sites, 0, sites.size(), 0);
        esdf_built_ = true;
    }

    Scalar getDistance(const Vec3& point) const {
        Scalar distance;
        Vec3 gradient;
        getDistanceWithGradient(point, distance, gradient);
        return distance;
    }

    /**
     * Exact distance to the nearest occupied voxel center.
     *
     * Unlike the old whole-volume ESDF this representation consumes O(number
     * of occupied voxels) memory and has no bounds.  A balanced k-d tree gives
     * typically logarithmic nearest-site queries while keeping storage and
     * rebuild work independent of the unobserved world volume.
     */
    bool getDistanceWithGradient(const Vec3& point, Scalar& distance, Vec3& gradient) const {
        if (!point.allFinite()) {
            distance = noObstacleDistance();
            gradient.setZero();
            return false;
        }
        if (!esdf_built_) computeEsdf();
        if (esdf_nodes_.empty()) {
            distance = noObstacleDistance();
            gradient.setZero();
            return true;
        }

        Scalar best_squared = std::numeric_limits<Scalar>::max();
        Vec3 best_delta = Vec3::Zero();
        nearestEsdfSite(0, point, best_squared, best_delta);
        distance = std::sqrt(best_squared);
        if (distance > Scalar(0)) gradient = best_delta / distance;
        else gradient.setZero();
        return true;
    }

    void queryEsdf(const std::vector<Vec3>& points,
                   std::vector<Scalar>& distances,
                   std::vector<Vec3>& gradients) const {
        distances.resize(points.size());
        gradients.resize(points.size());
        if (!esdf_built_) computeEsdf();
        for (size_t i = 0; i < points.size(); ++i) {
            getDistanceWithGradient(points[i], distances[i], gradients[i]);
        }
    }

    // ================== Signed Euclidean distance field ====================
    //
    // The ESDF lives in the same 8x8x8 block layout as occupancy, so a block is
    // materialized only where the truncation band around a surface reaches:
    // storage scales with obstacle surface area, not with explored volume.
    // Stored values are signed distances in *voxel units*, saturated at the
    // band, measured to the obstacle SURFACE rather than to a voxel centre:
    //     > 0   free space   -- distance out to the nearest obstacle surface
    //     < 0   inside solid -- depth below the nearest surface
    //      0    the surface, which lies on the face between the two
    // A surface voxel therefore reads -0.5 and its free neighbour +0.5, so the
    // field crosses zero with unit slope at the obstacle boundary.  (The k-d
    // tree backend below answers a different question -- exact distance to the
    // nearest occupied voxel *centre* -- and so reads half a voxel larger.)
    //
    // A block absent from the hash reads as +band ("nothing near"), which is
    // why an entirely interior block (all -band) is still stored explicitly.

    /**
     * Set the ESDF truncation distance, in metres.
     *
     * This single number bounds both halves of the cost model: storage, since
     * only the band around a surface is materialized, and incremental update
     * cost, since a change to one voxel can only alter cells within the band
     * of it.  Changing it discards the field and forces one full rebuild.
     */
    void setEsdfMaxDistance(Scalar meters) {
        if (!(meters > Scalar(0)) || !std::isfinite(static_cast<double>(meters))) {
            throw std::invalid_argument("OccupancyMap: ESDF max distance must be finite and > 0");
        }
        const Idx band = std::max<Idx>(
            1, static_cast<Idx>(std::ceil(static_cast<double>(meters) /
                                          static_cast<double>(resolution_))));
        if (band == esdf_band_) return;
        esdf_band_ = band;
        esdf_blocks_.clear();
        esdf_dirty_.clear();
        esdf_field_valid_ = false;
    }

    /** Truncation distance actually in force, in metres (rounded up to a voxel). */
    Scalar esdfMaxDistance() const { return static_cast<Scalar>(esdf_band_) * resolution_; }

    /**
     * Bring the signed ESDF up to date.
     *
     * Only blocks whose *occupancy* changed since the last update are
     * recomputed, along with the band of cells around them whose nearest
     * obstacle could have moved.  Truncation is what makes that dependency set
     * finite, and so what turns an O(map) rebuild into O(touched region).  The
     * first call after construction, clear(), or setEsdfMaxDistance() has no
     * valid field to patch and falls back to a full rebuild.
     */
    void updateEsdf() const {
        if (!esdf_field_valid_) { rebuildEsdf(); return; }
        if (esdf_dirty_.empty()) return;
        std::vector<BlockIndex> dirty(esdf_dirty_.begin(), esdf_dirty_.end());
        esdf_dirty_.clear();
        refreshEsdfRegions(dirty);
    }

    /** Discard the field and recompute all of it (benchmarks, tests, diagnostics). */
    void rebuildEsdf() const {
        esdf_blocks_.clear();
        esdf_dirty_.clear();
        std::unordered_set<BlockIndex, IndexHash> seeds;
        seeds.reserve(numOccupied() / 8 + 1);
        for (const auto& item : blocks_)
            if (blockAt(item.second).occupied_count != 0) seeds.insert(item.first);
        std::vector<BlockIndex> blocks(seeds.begin(), seeds.end());
        refreshEsdfRegions(blocks);
        esdf_field_valid_ = true;
    }

    /**
     * Continuous signed distance and gradient at an arbitrary world point.
     *
     * Trilinear interpolation of the voxel-centred field.  The zero level set
     * of the interpolant is a continuous surface rather than one snapped to
     * voxel centres, and the gradient is the analytic derivative of that same
     * interpolant -- so distance and gradient stay mutually consistent, which
     * is exactly what an optimizer stepping along -grad(d) relies on.
     *
     * @return false, with @p distance set to +esdfMaxDistance() and a zero
     *         gradient, when the point is non-finite or lies entirely outside
     *         the materialized band (i.e. farther than the truncation distance
     *         from every known obstacle, where the field carries no gradient
     *         information to give).
     */
    bool getSignedDistanceWithGradient(const Vec3& point,
                                       Scalar& distance,
                                       Vec3& gradient) const {
        distance = esdfMaxDistance();
        gradient.setZero();
        if (!point.allFinite()) return false;
        updateEsdf();

        // Voxel centre i sits at (i + 0.5) * resolution, so subtracting the
        // half-voxel puts the interpolation lattice on the sample points.
        const Vec3 continuous = point * inv_resolution_ - Vec3::Constant(Scalar(0.5));
        constexpr double low = static_cast<double>(std::numeric_limits<int>::min()) + 2.0;
        constexpr double high = static_cast<double>(std::numeric_limits<int>::max()) - 2.0;
        Idx base[3];
        Scalar frac[3];
        for (int axis = 0; axis < 3; ++axis) {
            const double value = std::floor(static_cast<double>(continuous[axis]));
            if (value < low || value > high) return false;
            base[axis] = static_cast<Idx>(value);
            frac[axis] = continuous[axis] - static_cast<Scalar>(value);
        }

        // Corner samples of the enclosing cell, c[x][y][z].
        Scalar c[2][2][2];
        bool inside_band = false;
        const Scalar band = esdfMaxDistance();
        for (int dx = 0; dx < 2; ++dx)
            for (int dy = 0; dy < 2; ++dy)
                for (int dz = 0; dz < 2; ++dz) {
                    const GridIndex v{static_cast<int>(base[0] + dx),
                                      static_cast<int>(base[1] + dy),
                                      static_cast<int>(base[2] + dz)};
                    bool materialized = false;
                    c[dx][dy][dz] = esdfValueAtVoxel(v, materialized) * resolution_;
                    inside_band = inside_band || materialized;
                }
        if (!inside_band) {
            distance = band;
            return false;
        }

        const Scalar tx = frac[0], ty = frac[1], tz = frac[2];
        const Scalar mx = Scalar(1) - tx, my = Scalar(1) - ty, mz = Scalar(1) - tz;

        // Interpolate along z, then y, then x, keeping the partial results:
        // the same intermediates give the analytic partial derivatives, so the
        // gradient costs a handful of subtractions rather than finite
        // differences (which would need six more field lookups and would not
        // agree with the interpolated value).
        const Scalar c00 = c[0][0][0] * mz + c[0][0][1] * tz;
        const Scalar c01 = c[0][1][0] * mz + c[0][1][1] * tz;
        const Scalar c10 = c[1][0][0] * mz + c[1][0][1] * tz;
        const Scalar c11 = c[1][1][0] * mz + c[1][1][1] * tz;
        const Scalar c0 = c00 * my + c01 * ty;
        const Scalar c1 = c10 * my + c11 * ty;
        distance = c0 * mx + c1 * tx;

        const Scalar d00 = c[0][0][1] - c[0][0][0];
        const Scalar d01 = c[0][1][1] - c[0][1][0];
        const Scalar d10 = c[1][0][1] - c[1][0][0];
        const Scalar d11 = c[1][1][1] - c[1][1][0];
        gradient.x() = (c1 - c0) * inv_resolution_;
        gradient.y() = ((c01 - c00) * mx + (c11 - c10) * tx) * inv_resolution_;
        gradient.z() = ((d00 * my + d01 * ty) * mx +
                        (d10 * my + d11 * ty) * tx) * inv_resolution_;
        return true;
    }

    /** Continuous signed distance in metres; gradient discarded. */
    Scalar getSignedDistance(const Vec3& point) const {
        Scalar distance;
        Vec3 gradient;
        getSignedDistanceWithGradient(point, distance, gradient);
        return distance;
    }

    /** Batched continuous query; the field is brought up to date once. */
    void querySignedEsdf(const std::vector<Vec3>& points,
                         std::vector<Scalar>& distances,
                         std::vector<Vec3>& gradients) const {
        distances.resize(points.size());
        gradients.resize(points.size());
        updateEsdf();
        for (size_t i = 0; i < points.size(); ++i) {
            getSignedDistanceWithGradient(points[i], distances[i], gradients[i]);
        }
    }

    /** Signed distance in metres at the containing voxel's centre (no interpolation). */
    Scalar signedDistanceAt(const Vec3& point) const {
        GridIndex idx;
        if (!worldToGrid(point, idx)) return esdfMaxDistance();
        updateEsdf();
        bool materialized = false;
        return esdfValueAtVoxel(idx, materialized) * resolution_;
    }

    /**
     * Visit every materialized ESDF voxel as (world centre, signed distance in
     * metres).  Used by the ROS node to publish the field; also the cheapest
     * way to dump the band for offline inspection.
     */
    template <typename Visitor>
    void forEachEsdfVoxel(Visitor&& visit) const {
        updateEsdf();
        for (const auto& item : esdf_blocks_) {
            const BlockIndex& b = item.first;
            const EsdfBlock& block = item.second;
            for (int lz = 0; lz < kVoxelsPerSide; ++lz)
                for (int ly = 0; ly < kVoxelsPerSide; ++ly)
                    for (int lx = 0; lx < kVoxelsPerSide; ++lx) {
                        const Scalar d = static_cast<Scalar>(
                            block.dist[localOffset(lx, ly, lz)]) * resolution_;
                        visit(gridToWorld(blockLocalToGrid(b, lx, ly, lz)), d);
                    }
        }
    }

    size_t numEsdfBlocks() const { return esdf_blocks_.size(); }
    size_t esdfAllocatedBytes() const { return esdf_blocks_.size() * sizeof(EsdfBlock); }

    /** Whether the next ESDF solve will be attempted on the GPU. */
    bool esdfUsesGpu() const {
#ifdef NANOVOXMAP_WITH_CUDA
        return !esdf_cuda_unavailable_;
#else
        return false;
#endif
    }

    /**
     * Pin the ESDF to the CPU backend.  Exists so tests and benchmarks can
     * compare the two solvers on identical input; a no-op in CPU-only builds.
     */
    void setEsdfForceCpu(bool force) const {
#ifdef NANOVOXMAP_WITH_CUDA
        esdf_cuda_unavailable_ = force;
#else
        (void)force;
#endif
    }

    /** Whether the next point-cloud insert will be attempted on the GPU. */
    bool raycastUsesGpu() const {
#ifdef NANOVOXMAP_WITH_CUDA
        return !cuda_unavailable_;
#else
        return false;
#endif
    }

    /**
     * Pin point-cloud insertion to the CPU raycaster.  The counterpart of
     * setEsdfForceCpu(), and what lets a test assert that the GPU and CPU
     * traversals build the same map; a no-op in CPU-only builds.
     */
    void setRaycastForceCpu(bool force) {
#ifdef NANOVOXMAP_WITH_CUDA
        cuda_unavailable_ = force;
#else
        (void)force;
#endif
    }

    /** Invalidate the k-d tree backend; the block ESDF tracks dirt per block. */
    void invalidateEsdf() const { esdf_built_ = false; }

    /**
     * Unsigned clearance (m) from the block ESDF: the signed distance in free
     * space, clamped to 0 inside an obstacle.
     */
    Scalar fieldDistanceAt(const Vec3& point) const {
        return std::max(Scalar(0), signedDistanceAt(point));
    }

    /** Force the distance field up to date now instead of on first query. */
    void buildDistanceField() const { updateEsdf(); }

private:
    static constexpr int kVoxelsPerSide = 8;
    static constexpr int kVoxelsPerBlock = kVoxelsPerSide * kVoxelsPerSide * kVoxelsPerSide;
    // kVoxelsPerSide is a power of two, so block index / local coordinate reduce
    // to a shift / mask instead of a signed division in the per-voxel hot path.
    static constexpr int kVoxelShift = 3;
    static constexpr int kVoxelMask = kVoxelsPerSide - 1;
    static_assert(kVoxelsPerSide == (1 << kVoxelShift), "kVoxelShift must match kVoxelsPerSide");
    static_assert((-1 >> 1) == -1, "arithmetic right shift required for floor block index");
    static constexpr int kLogOddsScale = 20;
    static constexpr int8_t kLogOddsUnknown = 0;

    struct GridIndex {
        int x = 0;
        int y = 0;
        int z = 0;

        bool operator==(const GridIndex& other) const {
            return x == other.x && y == other.y && z == other.z;
        }
    };

    using BlockIndex = GridIndex;

    struct IndexHash {
        size_t operator()(const GridIndex& key) const noexcept {
            uint64_t h = mix(static_cast<uint32_t>(key.x));
            h ^= mix(static_cast<uint32_t>(key.y) + 0x9e3779b9u + (h << 6));
            h ^= mix(static_cast<uint32_t>(key.z) + 0x85ebca6bu + (h << 6));
            return static_cast<size_t>(h);
        }

        static uint64_t mix(uint64_t x) noexcept {
            x ^= x >> 30;
            x *= 0xbf58476d1ce4e5b9ULL;
            x ^= x >> 27;
            x *= 0x94d049bb133111ebULL;
            return x ^ (x >> 31);
        }
    };

    struct VoxelBlock {
        std::array<int8_t, kVoxelsPerBlock> voxels{};
        std::array<uint64_t, kVoxelsPerBlock / 64> occupied_mask{};
        uint16_t occupied_count = 0;
    };

    /**
     * One block of the signed ESDF, in the same lattice as VoxelBlock so a
     * voxel's occupancy and its distance share an index computation.  Stored
     * as float in *voxel units*: the band is only a few hundred voxels wide, so
     * float carries far more precision than the field's own discretization
     * error, and halving the footprint against double matters when the band
     * around a large surface is materialized.
     */
    struct EsdfBlock {
        std::array<float, kVoxelsPerBlock> dist{};
    };

    /**
     * Memoizes the last-touched block so a traversal that stays inside one
     * 8x8x8 block (the common case for consecutive voxels along a ray) pays a
     * single hash lookup instead of one per voxel.  Caching the VoxelBlock*
     * across later inserts is safe: std::unordered_map keeps references to
     * existing elements valid across insertion/rehash.
     */
    struct BlockCursor {
        BlockIndex block{};
        VoxelBlock* data = nullptr;
    };


    // Store each site's world center precomputed: the nearest-neighbour query
    // visits many nodes per lookup, so folding gridToWorld() into the build
    // keeps the hot recursion down to a subtract + squared-norm per node.
    struct EsdfNode {
        Vec3 center;
        int left = -1;
        int right = -1;
        uint8_t axis = 0;
    };

    Scalar resolution_;
    Scalar inv_resolution_;
    int8_t l_hit_;
    int8_t l_miss_;
    int8_t l_min_;
    int8_t l_max_;
    Backend requested_backend_ = Backend::kAuto;
    bool allow_cuda_fallback_ = true;

    static constexpr size_t kBlocksPerPage = 1024;
    using BlockHandle = uint32_t;
    std::unordered_map<BlockIndex, BlockHandle, IndexHash> blocks_;
    std::vector<std::unique_ptr<VoxelBlock[]>> block_pages_;
    size_t block_count_ = 0;
    std::atomic<size_t> occupied_count_{0};

    mutable std::vector<EsdfNode> esdf_nodes_;
    mutable bool esdf_built_ = false;

    // --- Signed block ESDF (incremental, truncated) ------------------------
    // esdf_blocks_ holds the materialized band; esdf_dirty_ accumulates the
    // blocks whose occupancy changed since the last update, which is the input
    // to the incremental recompute.  esdf_field_valid_ distinguishes "patchable"
    // from "must rebuild from scratch".
    mutable std::unordered_map<BlockIndex, EsdfBlock, IndexHash> esdf_blocks_;
    mutable std::unordered_set<BlockIndex, IndexHash> esdf_dirty_;
    mutable bool esdf_field_valid_ = false;
    Idx esdf_band_ = 0;  ///< Truncation distance, in voxels; set in the constructor.

    /// Sentinel for "no seed on this line" in the distance transform, and for
    /// "no obstacle anywhere" in the untruncated k-d tree backend.  Must match
    /// kEsdfInfDevice in nanovoxmap_cuda.cu so both backends agree.
    static constexpr float kFieldInf = 1e18f;
    /**
     * Cell budget for a single dense solve.  A dirty set that would need a
     * larger box is split (see partitionDirtyBlocks) so peak scratch memory
     * stays bounded no matter how the dirty blocks are scattered: 4M cells is
     * ~16 MB per float buffer, and the solve needs two.
     */
    static constexpr Idx kEsdfMaxSolveCells = 4 * 1000 * 1000;

#ifdef NANOVOXMAP_WITH_CUDA
    bool cuda_unavailable_ = false;
    mutable bool esdf_cuda_unavailable_ = false;
    bool insertPointCloudCuda(const Vec3& origin, const std::vector<Vec3>& endpoints,
                              bool apply_hits);
    /**
     * GPU signed distance solve for one dense box.  Consumes an occupancy mask
     * (1 = occupied) and writes signed distances in voxel units, saturated at
     * @p band -- byte-for-byte the same contract as solveSignedEsdfCpu, so
     * either backend can serve any box.  Anything but kSolved leaves @p out
     * untouched and means the caller must run the CPU solver; only
     * kDeviceFailed additionally latches the fallback (see GpuSolveStatus).
     */
    GpuSolveStatus solveSignedEsdfCuda(const std::vector<uint8_t>& occupancy,
                                       Idx nx, Idx ny, Idx nz, float band,
                                       std::vector<float>& out) const;
#endif

    static Scalar checkedResolution(Scalar resolution) {
        if (!(resolution > Scalar(0)) || !std::isfinite(static_cast<double>(resolution))) {
            throw std::invalid_argument("OccupancyMap: resolution must be finite and > 0");
        }
        return resolution;
    }

    static int8_t toLogOddsUnits(Scalar p) {
        const double value = std::log(static_cast<double>(p) /
                                      (1.0 - static_cast<double>(p)));
        const long scaled = std::lround(value * kLogOddsScale);
        return static_cast<int8_t>(std::max<long>(-127, std::min<long>(127, scaled)));
    }

    static int8_t toLogOddsUnitsChecked(Scalar p) {
        if (!(p > Scalar(0) && p < Scalar(1)) ||
            !std::isfinite(static_cast<double>(p))) {
            throw std::invalid_argument("OccupancyMap: probabilities must be finite and lie in (0,1)");
        }
        return toLogOddsUnits(p);
    }

    static int8_t clampToRange(int value, int8_t low, int8_t high) {
        return static_cast<int8_t>(std::max<int>(low, std::min<int>(high, value)));
    }

    static BlockIndex blockIndex(const GridIndex& voxel) {
        return BlockIndex{voxel.x >> kVoxelShift,
                          voxel.y >> kVoxelShift,
                          voxel.z >> kVoxelShift};
    }

    /** Voxel coordinate inside its block; kVoxelsPerSide is a power of two. */
    static int localCoordinate(int voxel) {
        return voxel & kVoxelMask;
    }

    static size_t localOffset(int x, int y, int z) {
        return static_cast<size_t>((z * kVoxelsPerSide + y) * kVoxelsPerSide + x);
    }

    template <typename Visitor>
    static void forEachOccupiedOffset(const VoxelBlock& block, Visitor&& visit) {
        for (int word_index = 0; word_index < static_cast<int>(block.occupied_mask.size());
             ++word_index) {
            uint64_t bits = block.occupied_mask[word_index];
            while (bits != 0) {
                const int bit = __builtin_ctzll(bits);
                visit(word_index * 64 + bit);
                bits &= bits - 1;
            }
        }
    }

    static GridIndex blockLocalToGrid(const BlockIndex& block, int x, int y, int z) {
        return GridIndex{
            static_cast<int>(static_cast<int64_t>(block.x) * kVoxelsPerSide + x),
            static_cast<int>(static_cast<int64_t>(block.y) * kVoxelsPerSide + y),
            static_cast<int>(static_cast<int64_t>(block.z) * kVoxelsPerSide + z)};
    }

    VoxelBlock& blockAt(BlockHandle handle) {
        return block_pages_[handle / kBlocksPerPage][handle % kBlocksPerPage];
    }
    const VoxelBlock& blockAt(BlockHandle handle) const {
        return block_pages_[handle / kBlocksPerPage][handle % kBlocksPerPage];
    }
    VoxelBlock& findOrCreateBlock(const BlockIndex& key) {
        const auto found = blocks_.find(key);
        if (found != blocks_.end()) return blockAt(found->second);
        if (block_count_ > std::numeric_limits<BlockHandle>::max()) {
            throw std::length_error("OccupancyMap: block pool exhausted");
        }
        if (block_count_ % kBlocksPerPage == 0) {
            block_pages_.emplace_back(std::make_unique<VoxelBlock[]>(kBlocksPerPage));
        }
        const BlockHandle handle = static_cast<BlockHandle>(block_count_++);
        blocks_.emplace(key, handle);
        return blockAt(handle);
    }

    int8_t& voxelRef(const GridIndex& voxel) {
        const BlockIndex block = blockIndex(voxel);
        VoxelBlock& data = findOrCreateBlock(block);
        return data.voxels[localOffset(localCoordinate(voxel.x),
                                       localCoordinate(voxel.y),
                                       localCoordinate(voxel.z))];
    }

    /** Resolve a voxel through a memoized block cursor (see BlockCursor). */
    int8_t& voxelRefCached(BlockCursor& cursor, const GridIndex& voxel) {
        const BlockIndex block = blockIndex(voxel);
        if (cursor.data == nullptr || !(cursor.block == block)) {
            cursor.data = &findOrCreateBlock(block);
            cursor.block = block;
        }
        return cursor.data->voxels[localOffset(localCoordinate(voxel.x),
                                               localCoordinate(voxel.y),
                                               localCoordinate(voxel.z))];
    }

    int8_t voxelValue(const GridIndex& voxel) const {
        const BlockIndex block = blockIndex(voxel);
        const auto found = blocks_.find(block);
        if (found == blocks_.end()) return kLogOddsUnknown;
        return blockAt(found->second).voxels[localOffset(localCoordinate(voxel.x),
                                                localCoordinate(voxel.y),
                                                localCoordinate(voxel.z))];
    }

    // A single log-odds step never exceeds int32 range (|l_*| <= 127 and the
    // reduced counts are bounded by the ray count), so accumulate in int and
    // let the [l_min_, l_max_] clamp below dominate -- no int64 needed.
    void applyStep(int8_t& value, const GridIndex& voxel, int delta) {
        const int8_t before = value;
        int raw = static_cast<int>(before) + delta;
        if (raw < l_min_) raw = l_min_;
        else if (raw > l_max_) raw = l_max_;
        value = static_cast<int8_t>(raw);
        updateOccupiedSet(voxel, before, value);
    }

    void applyMissCached(BlockCursor& cursor, const GridIndex& voxel, int count = 1) {
        applyStep(voxelRefCached(cursor, voxel), voxel, static_cast<int>(l_miss_) * count);
    }

    void applyHitCached(BlockCursor& cursor, const GridIndex& voxel, int count = 1) {
        applyStep(voxelRefCached(cursor, voxel), voxel, static_cast<int>(l_hit_) * count);
    }

    void applyMissIdx(const GridIndex& voxel, int count = 1) {
        applyStep(voxelRef(voxel), voxel, static_cast<int>(l_miss_) * count);
    }

    void applyHitIdx(const GridIndex& voxel, int count = 1) {
        applyStep(voxelRef(voxel), voxel, static_cast<int>(l_hit_) * count);
    }

    // Normal rays exclude the endpoint from misses because it receives a hit.
    // A no-return/over-range ray has no hit, so its endpoint is observed free.
    void applyClearingEndpoints(const std::vector<Vec3>& endpoints) {
        BlockCursor cursor;
        for (const Vec3& endpoint : endpoints) {
            GridIndex end;
            if (worldToGrid(endpoint, end)) applyMissCached(cursor, end);
        }
    }

    /**
     * Occupancy transitions are the only events the ESDF cares about: a voxel
     * whose log-odds moved without crossing the threshold leaves every distance
     * unchanged.  Marking the containing block here (rather than at the call
     * sites) means every update path -- CPU, CUDA, single-ray, manual edit --
     * feeds the incremental ESDF automatically.
     */
    void updateOccupiedSet(const GridIndex& voxel, int8_t before, int8_t after) {
        if ((before > 0) == (after > 0)) return;
        VoxelBlock& block = blockAt(blocks_.find(blockIndex(voxel))->second);
        const size_t offset = localOffset(localCoordinate(voxel.x),
                                          localCoordinate(voxel.y),
                                          localCoordinate(voxel.z));
        uint64_t& word = block.occupied_mask[offset >> 6];
        const uint64_t bit = uint64_t{1} << (offset & 63);
        if (before <= 0 && after > 0) {
            word |= bit;
            ++block.occupied_count;
            occupied_count_.fetch_add(1, std::memory_order_relaxed);
            esdf_dirty_.insert(blockIndex(voxel));
        } else if (before > 0 && after <= 0) {
            word &= ~bit;
            --block.occupied_count;
            occupied_count_.fetch_sub(1, std::memory_order_relaxed);
            esdf_dirty_.insert(blockIndex(voxel));
        }
    }

    void setFreeIdx(const GridIndex& voxel) {
        int8_t& value = voxelRef(voxel);
        const int8_t before = value;
        value = l_min_;
        updateOccupiedSet(voxel, before, value);
        invalidateEsdf();
    }

    void setOccupiedIdx(const GridIndex& voxel) {
        int8_t& value = voxelRef(voxel);
        const int8_t before = value;
        value = l_max_;
        updateOccupiedSet(voxel, before, value);
        invalidateEsdf();
    }

    bool worldToGrid(const Vec3& point, GridIndex& result) const {
        if (!point.allFinite()) return false;
        const Vec3 scaled = point * inv_resolution_;
        constexpr double low = static_cast<double>(std::numeric_limits<int>::min()) + 1.0;
        constexpr double high = static_cast<double>(std::numeric_limits<int>::max()) - 1.0;
        for (int axis = 0; axis < 3; ++axis) {
            const double value = static_cast<double>(scaled[axis]);
            if (value < low || value > high) return false;
        }
        result = GridIndex{static_cast<int>(std::floor(scaled.x())),
                           static_cast<int>(std::floor(scaled.y())),
                           static_cast<int>(std::floor(scaled.z()))};
        return true;
    }

    Vec3 gridToWorld(const GridIndex& voxel) const {
        return (Vec3(static_cast<Scalar>(voxel.x),
                     static_cast<Scalar>(voxel.y),
                     static_cast<Scalar>(voxel.z)) + Vec3::Constant(Scalar(0.5))) * resolution_;
    }

    Vec3 voxelCornerToWorld(const GridIndex& voxel) const {
        return Vec3(static_cast<Scalar>(voxel.x),
                    static_cast<Scalar>(voxel.y),
                    static_cast<Scalar>(voxel.z)) * resolution_;
    }

    template <typename Visitor>
    static void traceMisses(const GridIndex& start, const GridIndex& end, Visitor&& visit) {
        int x = start.x, y = start.y, z = start.z;
        const int64_t dx64 = static_cast<int64_t>(end.x) - x;
        const int64_t dy64 = static_cast<int64_t>(end.y) - y;
        const int64_t dz64 = static_cast<int64_t>(end.z) - z;
        const int sx = (dx64 > 0) - (dx64 < 0);
        const int sy = (dy64 > 0) - (dy64 < 0);
        const int sz = (dz64 > 0) - (dz64 < 0);
        const int64_t dx = std::llabs(dx64);
        const int64_t dy = std::llabs(dy64);
        const int64_t dz = std::llabs(dz64);

        // Amanatides-Woo boundary stepping on the voxel-centre segment. This
        // is deliberately identical to traceMissKernel() in the CUDA backend.
        constexpr double kInf = std::numeric_limits<double>::infinity();
        double tMaxX = dx > 0 ? 0.5 / static_cast<double>(dx) : kInf;
        double tMaxY = dy > 0 ? 0.5 / static_cast<double>(dy) : kInf;
        double tMaxZ = dz > 0 ? 0.5 / static_cast<double>(dz) : kInf;
        const double tDeltaX = dx > 0 ? 1.0 / static_cast<double>(dx) : kInf;
        const double tDeltaY = dy > 0 ? 1.0 / static_cast<double>(dy) : kInf;
        const double tDeltaZ = dz > 0 ? 1.0 / static_cast<double>(dz) : kInf;

        const int64_t steps = dx + dy + dz;
        for (int64_t i = 0; i < steps; ++i) {
            visit(GridIndex{x, y, z});
            if (tMaxX <= tMaxY && tMaxX <= tMaxZ) { x += sx; tMaxX += tDeltaX; }
            else if (tMaxY <= tMaxZ)              { y += sy; tMaxY += tDeltaY; }
            else                                   { z += sz; tMaxZ += tDeltaZ; }
        }
    }

    void insertPointCloudCpu(const Vec3& origin, const std::vector<Vec3>& endpoints,
                             bool apply_hits = true) {
        GridIndex start;
        if (!worldToGrid(origin, start)) return;
        std::vector<GridIndex> valid_ends;
        valid_ends.reserve(endpoints.size());
        // One cursor for the whole scan: rays share the origin block near the
        // sensor and stay block-local for long stretches, so this collapses
        // the per-voxel hash lookups to roughly one per 8-voxel block crossing.
        //
        // This integration is memory-latency bound on the block hash, so it is
        // deliberately a single fused pass: profiling showed the traversal is
        // only ~14% of the cost and the block update ~86%, and multi-pass
        // parallel schemes (dedup/routing) add enough memory traffic to run
        // slower than this serial path.  The one design that does help --
        // per-thread sharded storage -- is a larger change (see the map header
        // docs) and intentionally left out here.
        BlockCursor cursor;
        for (const Vec3& endpoint : endpoints) {
            GridIndex end;
            if (!worldToGrid(endpoint, end)) continue;
            valid_ends.push_back(end);
            traceMisses(start, end,
                        [&](const GridIndex& v) { applyMissCached(cursor, v); });
        }
        if (apply_hits)
            for (const GridIndex& end : valid_ends) applyHitCached(cursor, end);
    }

    // ---------------- Signed ESDF: lookup, solve, incremental --------------

    /**
     * Signed distance at a voxel, in voxel units.  A block that was never
     * materialized is farther than the band from every obstacle, so it reads
     * +band; @p materialized reports which case the caller got, so an
     * interpolated query can tell "provably far" from "actually measured".
     */
    float esdfValueAtVoxel(const GridIndex& voxel, bool& materialized) const {
        const auto found = esdf_blocks_.find(blockIndex(voxel));
        if (found == esdf_blocks_.end()) {
            materialized = false;
            return static_cast<float>(esdf_band_);
        }
        materialized = true;
        return found->second.dist[localOffset(localCoordinate(voxel.x),
                                              localCoordinate(voxel.y),
                                              localCoordinate(voxel.z))];
    }

    /** Inclusive axis-aligned box of block indices. */
    struct BlockBox {
        Idx lo[3];
        Idx hi[3];
    };

    /**
     * Cells in the dense solve a dirty box implies.  The write region is the
     * box grown by the band (every cell whose nearest obstacle could have
     * moved), and the *seed* region is that grown by the band again (every
     * obstacle that could be the new nearest one) -- hence two dilations.
     */
    Idx solveCellsFor(const BlockBox& box) const {
        const Idx pad = 2 * esdf_band_ + 2 * roundUpBlocks(esdf_band_) * kVoxelsPerSide;
        Idx total = 1;
        for (int axis = 0; axis < 3; ++axis) {
            const Idx side = (box.hi[axis] - box.lo[axis] + 1) * kVoxelsPerSide + pad;
            if (side > 0 && total > kEsdfMaxSolveCells / side) return kEsdfMaxSolveCells + 1;
            total *= side;
        }
        return total;
    }

    static Idx roundUpBlocks(Idx voxels) { return (voxels + kVoxelsPerSide - 1) / kVoxelsPerSide; }

    static BlockBox boundsOf(const std::vector<BlockIndex>& blocks, size_t begin, size_t end) {
        BlockBox box{{0, 0, 0}, {0, 0, 0}};
        box.lo[0] = box.hi[0] = blocks[begin].x;
        box.lo[1] = box.hi[1] = blocks[begin].y;
        box.lo[2] = box.hi[2] = blocks[begin].z;
        for (size_t i = begin + 1; i < end; ++i) {
            const Idx c[3] = {blocks[i].x, blocks[i].y, blocks[i].z};
            for (int axis = 0; axis < 3; ++axis) {
                box.lo[axis] = std::min(box.lo[axis], c[axis]);
                box.hi[axis] = std::max(box.hi[axis], c[axis]);
            }
        }
        return box;
    }

    /**
     * Split the dirty blocks into boxes each small enough to solve densely.
     *
     * A frame's dirty blocks are normally one contiguous sensor footprint, so
     * the common case emits a single box.  Scattered dirt (two edits at
     * opposite ends of the map) would otherwise make one enormous bounding
     * box, so recursive median bisection along the longest axis keeps every
     * solve under the cell budget and the total work proportional to what
     * actually changed.
     */
    void partitionDirtyBlocks(std::vector<BlockIndex>& blocks, size_t begin, size_t end,
                              std::vector<BlockBox>& out) const {
        if (begin >= end) return;
        const BlockBox box = boundsOf(blocks, begin, end);
        if (end - begin == 1 || solveCellsFor(box) <= kEsdfMaxSolveCells) {
            out.push_back(box);
            return;
        }
        int axis = 0;
        for (int a = 1; a < 3; ++a) {
            if (box.hi[a] - box.lo[a] > box.hi[axis] - box.lo[axis]) axis = a;
        }
        const size_t middle = begin + (end - begin) / 2;
        const auto coordinate = [axis](const BlockIndex& b) {
            return axis == 0 ? b.x : (axis == 1 ? b.y : b.z);
        };
        std::nth_element(blocks.begin() + begin, blocks.begin() + middle, blocks.begin() + end,
                         [&](const BlockIndex& a, const BlockIndex& b) {
                             return coordinate(a) < coordinate(b);
                         });
        partitionDirtyBlocks(blocks, begin, middle, out);
        partitionDirtyBlocks(blocks, middle, end, out);
    }

    void refreshEsdfRegions(std::vector<BlockIndex>& dirty) const {
        if (dirty.empty()) return;
        std::vector<BlockBox> regions;
        partitionDirtyBlocks(dirty, 0, dirty.size(), regions);
        for (const BlockBox& region : regions) solveEsdfRegion(region);
    }

    /**
     * Recompute the ESDF for one dirty box.
     *
     * Write region R = the dirty box grown by the band, rounded out to whole
     * blocks so results can be stored block-at-a-time.  Seed region S = R grown
     * by the band again.  Every cell in R then has its true nearest site inside
     * S unless that site is farther than the band away, in which case the value
     * saturates anyway -- so the patch is exact up to the truncation, with no
     * dependence on anything outside S.
     */
    void solveEsdfRegion(const BlockBox& region) const {
        const Idx band_blocks = roundUpBlocks(esdf_band_);
        Idx rlo[3], rhi[3], slo[3], n[3];
        for (int axis = 0; axis < 3; ++axis) {
            rlo[axis] = (region.lo[axis] - band_blocks) * kVoxelsPerSide;
            rhi[axis] = (region.hi[axis] + band_blocks) * kVoxelsPerSide + (kVoxelsPerSide - 1);
            slo[axis] = rlo[axis] - esdf_band_;
            n[axis] = (rhi[axis] + esdf_band_) - slo[axis] + 1;
            // A region reaching past the int voxel lattice cannot be indexed;
            // skipping it leaves those blocks unmaterialized (read as +band),
            // which is the same answer the lattice edge could ever give.
            if (slo[axis] < std::numeric_limits<int>::min() ||
                rhi[axis] + esdf_band_ > std::numeric_limits<int>::max()) {
                return;
            }
        }
        const size_t total = checkedVectorSize(n[0], n[1], n[2]);
        const size_t sxy = static_cast<size_t>(n[0]) * static_cast<size_t>(n[1]);

        // Gather occupancy for the seed region by walking the blocks it spans:
        // touching only allocated blocks keeps this proportional to observed
        // space rather than to the (mostly empty) dense box.
        std::vector<uint8_t> occupancy(total, 0);
        const Idx blo[3] = {floorDivBlock(slo[0]), floorDivBlock(slo[1]), floorDivBlock(slo[2])};
        const Idx bhi[3] = {floorDivBlock(slo[0] + n[0] - 1),
                            floorDivBlock(slo[1] + n[1] - 1),
                            floorDivBlock(slo[2] + n[2] - 1)};
        for (Idx bz = blo[2]; bz <= bhi[2]; ++bz)
            for (Idx by = blo[1]; by <= bhi[1]; ++by)
                for (Idx bx = blo[0]; bx <= bhi[0]; ++bx) {
                    const auto found = blocks_.find(BlockIndex{static_cast<int>(bx),
                                                               static_cast<int>(by),
                                                               static_cast<int>(bz)});
                    if (found == blocks_.end()) continue;
                    const VoxelBlock& block = blockAt(found->second);
                    forEachOccupiedOffset(block, [&](int offset) {
                        const int lx = offset & kVoxelMask;
                        const int ly = (offset >> kVoxelShift) & kVoxelMask;
                        const int lz = offset >> (2 * kVoxelShift);
                        const Idx x = bx * kVoxelsPerSide + lx - slo[0];
                        const Idx y = by * kVoxelsPerSide + ly - slo[1];
                        const Idx z = bz * kVoxelsPerSide + lz - slo[2];
                        if (x < 0 || x >= n[0] || y < 0 || y >= n[1] ||
                            z < 0 || z >= n[2]) return;
                        occupancy[static_cast<size_t>(z) * sxy +
                                  static_cast<size_t>(y) * static_cast<size_t>(n[0]) +
                                  static_cast<size_t>(x)] = 1;
                    });
                }

        const float band = static_cast<float>(esdf_band_);
        std::vector<float> field;
        bool solved = false;
#ifdef NANOVOXMAP_WITH_CUDA
        if (!esdf_cuda_unavailable_) {
            const GpuSolveStatus status =
                solveSignedEsdfCuda(occupancy, n[0], n[1], n[2], band, field);
            solved = status == GpuSolveStatus::kSolved;
            if (status == GpuSolveStatus::kDeviceFailed) esdf_cuda_unavailable_ = true;
        }
#endif
        if (!solved) solveSignedEsdfCpu(occupancy, n[0], n[1], n[2], band, field);

        storeEsdfRegion(region, band_blocks, slo, n, field, band);
    }

    /** Write the solved band back into the block hash, block at a time. */
    void storeEsdfRegion(const BlockBox& region, Idx band_blocks, const Idx (&slo)[3],
                         const Idx (&n)[3], const std::vector<float>& field, float band) const {
        const size_t sxy = static_cast<size_t>(n[0]) * static_cast<size_t>(n[1]);
        for (Idx bz = region.lo[2] - band_blocks; bz <= region.hi[2] + band_blocks; ++bz)
            for (Idx by = region.lo[1] - band_blocks; by <= region.hi[1] + band_blocks; ++by)
                for (Idx bx = region.lo[0] - band_blocks; bx <= region.hi[0] + band_blocks; ++bx) {
                    EsdfBlock block;
                    // A block sitting entirely at +band carries no information
                    // a missing block would not, so it is dropped -- that is
                    // what lets the band shrink again when an obstacle is
                    // cleared.  Interior blocks (all -band) are NOT droppable,
                    // hence the one-sided test.
                    bool keep = false;
                    for (int lz = 0; lz < kVoxelsPerSide; ++lz)
                        for (int ly = 0; ly < kVoxelsPerSide; ++ly)
                            for (int lx = 0; lx < kVoxelsPerSide; ++lx) {
                                const size_t x = static_cast<size_t>(bx * kVoxelsPerSide + lx - slo[0]);
                                const size_t y = static_cast<size_t>(by * kVoxelsPerSide + ly - slo[1]);
                                const size_t z = static_cast<size_t>(bz * kVoxelsPerSide + lz - slo[2]);
                                const float d = field[z * sxy + y * static_cast<size_t>(n[0]) + x];
                                block.dist[localOffset(lx, ly, lz)] = d;
                                if (d < band) keep = true;
                            }
                    const BlockIndex key{static_cast<int>(bx), static_cast<int>(by),
                                         static_cast<int>(bz)};
                    if (keep) esdf_blocks_[key] = block;
                    else esdf_blocks_.erase(key);
                }
    }

    static Idx floorDivBlock(Idx voxel) {
        return voxel >= 0 ? voxel / kVoxelsPerSide
                          : -((-voxel + kVoxelsPerSide - 1) / kVoxelsPerSide);
    }

    /**
     * Exact signed distance over a dense box, on the CPU.
     *
     * Two squared-distance transforms: one seeded on the occupied cells (how
     * far a free cell is from the nearest solid one) and one seeded on
     * everything else (how deep a solid cell sits below the nearest free one).
     * Taking the first in free space and the negated second inside obstacles is
     * what makes the field *signed*.
     *
     * The half-voxel subtraction turns a centre-to-centre distance into a
     * distance to the obstacle *surface*, and it is not cosmetic: without it
     * the last solid voxel reads -1 and its free neighbour +1, so the
     * interpolated field crosses zero with slope 2 and reports a surface half a
     * voxel off.  Shifting both sides by half a voxel puts the zero crossing on
     * the face between them -- where the surface actually is -- and gives the
     * field unit slope across it, which is what a gradient-following planner
     * assumes when it treats |grad(d)| as 1.
     */
    static void solveSignedEsdfCpu(const std::vector<uint8_t>& occupancy,
                                   Idx nx, Idx ny, Idx nz, float band,
                                   std::vector<float>& out) {
        const size_t total = occupancy.size();
        std::vector<float> outside(total), inside(total);
        for (size_t i = 0; i < total; ++i) {
            outside[i] = occupancy[i] ? 0.0f : kFieldInf;
            inside[i] = occupancy[i] ? kFieldInf : 0.0f;
        }
        distanceTransform3D(outside, nx, ny, nz);
        distanceTransform3D(inside, nx, ny, nz);

        out.resize(total);
        for (size_t i = 0; i < total; ++i) {
            const float signed_distance =
                occupancy[i] ? -(std::sqrt(inside[i]) - 0.5f)
                             : (std::sqrt(outside[i]) - 0.5f);
            out[i] = std::max(-band, std::min(band, signed_distance));
        }
    }

    /** Three separable passes; each is a set of independent 1-D transforms. */
    static void distanceTransform3D(std::vector<float>& field, Idx nx, Idx ny, Idx nz) {
        const size_t sxy = static_cast<size_t>(nx) * static_cast<size_t>(ny);
        std::vector<int> v;
        std::vector<float> z, scratch;
        for (Idx z_i = 0; z_i < nz; ++z_i)
            for (Idx y = 0; y < ny; ++y)
                dt1d(field.data() + static_cast<size_t>(z_i) * sxy +
                         static_cast<size_t>(y) * static_cast<size_t>(nx),
                     nx, 1, v, z, scratch);
        for (Idx z_i = 0; z_i < nz; ++z_i)
            for (Idx x = 0; x < nx; ++x)
                dt1d(field.data() + static_cast<size_t>(z_i) * sxy + static_cast<size_t>(x),
                     ny, nx, v, z, scratch);
        for (Idx y = 0; y < ny; ++y)
            for (Idx x = 0; x < nx; ++x)
                dt1d(field.data() + static_cast<size_t>(y) * static_cast<size_t>(nx) +
                         static_cast<size_t>(x),
                     nz, static_cast<Idx>(sxy), v, z, scratch);
    }

    /**
     * Exact 1-D squared-distance transform of a strided line (Felzenszwalb &
     * Huttenlocher, 2004): the lower envelope of the parabolas f[q] + (x-q)^2.
     * @p v, @p z and @p scratch are caller-owned buffers reused across every
     * line of a pass -- the line count runs into the hundreds of thousands, so
     * allocating per line would dominate the transform.  A line with no finite
     * seed is left untouched (all-infinite stays all-infinite).
     */
    static void dt1d(float* line, Idx n, Idx stride,
                     std::vector<int>& v, std::vector<float>& z,
                     std::vector<float>& scratch) {
        bool has_seed = false;
        for (Idx i = 0; i < n; ++i) {
            if (line[i * stride] < kFieldInf) { has_seed = true; break; }
        }
        if (!has_seed) return;

        v.resize(static_cast<size_t>(n));
        z.resize(static_cast<size_t>(n) + 1);
        scratch.resize(static_cast<size_t>(n));

        // Seed the envelope with the first finite parabola: starting at q = 0
        // unconditionally would put an infinite-cost vertex in the hull and
        // divide by its infinite height below.
        Idx first = 0;
        while (first < n && line[first * stride] >= kFieldInf) ++first;
        int k = 0;
        v[0] = static_cast<int>(first);
        z[0] = -kFieldInf;
        z[1] = kFieldInf;
        for (Idx q = first + 1; q < n; ++q) {
            const float fq = line[q * stride];
            if (fq >= kFieldInf) continue;  // an empty cell contributes no parabola
            float s;
            while (true) {
                const float vk = static_cast<float>(v[k]);
                s = ((fq + static_cast<float>(q) * static_cast<float>(q)) -
                     (line[v[k] * stride] + vk * vk)) /
                    (2.0f * (static_cast<float>(q) - vk));
                if (s > z[k]) break;
                --k;
            }
            ++k;
            v[k] = static_cast<int>(q);
            z[k] = s;
            z[k + 1] = kFieldInf;
        }
        // Sample the lower envelope at each integer position.
        k = 0;
        for (Idx q = 0; q < n; ++q) {
            while (z[k + 1] < static_cast<float>(q)) ++k;
            const float d = static_cast<float>(q) - static_cast<float>(v[k]);
            scratch[static_cast<size_t>(q)] = d * d + line[v[k] * stride];
        }
        for (Idx q = 0; q < n; ++q) line[q * stride] = scratch[static_cast<size_t>(q)];
    }

    int buildEsdfTree(std::vector<GridIndex>& sites,
                      size_t begin,
                      size_t end,
                      int depth) const {
        if (begin >= end) return -1;
        const uint8_t axis = static_cast<uint8_t>(depth % 3);
        const size_t middle = begin + (end - begin) / 2;
        const auto coordinate = [axis](const GridIndex& value) {
            return axis == 0 ? value.x : (axis == 1 ? value.y : value.z);
        };
        std::nth_element(sites.begin() + begin, sites.begin() + middle, sites.begin() + end,
                         [&](const GridIndex& a, const GridIndex& b) {
                             return coordinate(a) < coordinate(b);
                         });
        const int node = static_cast<int>(esdf_nodes_.size());
        esdf_nodes_.push_back(EsdfNode{gridToWorld(sites[middle]), -1, -1, axis});
        const int left = buildEsdfTree(sites, begin, middle, depth + 1);
        const int right = buildEsdfTree(sites, middle + 1, end, depth + 1);
        esdf_nodes_[node].left = left;
        esdf_nodes_[node].right = right;
        return node;
    }

    void nearestEsdfSite(int node_index,
                         const Vec3& point,
                         Scalar& best_squared,
                         Vec3& best_delta) const {
        if (node_index < 0) return;
        const EsdfNode& node = esdf_nodes_[static_cast<size_t>(node_index)];
        const Vec3 delta = point - node.center;
        const Scalar squared = delta.squaredNorm();
        if (squared < best_squared) {
            best_squared = squared;
            best_delta = delta;
        }

        const Scalar axis_delta = delta[static_cast<int>(node.axis)];
        const int near_child = axis_delta < Scalar(0) ? node.left : node.right;
        const int far_child = axis_delta < Scalar(0) ? node.right : node.left;
        nearestEsdfSite(near_child, point, best_squared, best_delta);
        if (axis_delta * axis_delta < best_squared) {
            nearestEsdfSite(far_child, point, best_squared, best_delta);
        }
    }

    void occupiedBounds(GridIndex& low, GridIndex& high) const {
        bool initialized = false;
        for (const auto& item : blocks_) {
            forEachOccupiedOffset(blockAt(item.second), [&](int offset) {
                const GridIndex voxel = blockLocalToGrid(
                    item.first, offset & kVoxelMask,
                    (offset >> kVoxelShift) & kVoxelMask,
                    offset >> (2 * kVoxelShift));
                if (!initialized) {
                    low = high = voxel;
                    initialized = true;
                    return;
                }
                low.x = std::min(low.x, voxel.x); high.x = std::max(high.x, voxel.x);
                low.y = std::min(low.y, voxel.y); high.y = std::max(high.y, voxel.y);
                low.z = std::min(low.z, voxel.z); high.z = std::max(high.z, voxel.z);
            });
        }
    }

    void allocatedVoxelBounds(GridIndex& low, GridIndex& high) const {
        const auto first = blocks_.begin();
        low = blockLocalToGrid(first->first, 0, 0, 0);
        high = blockLocalToGrid(first->first, kVoxelsPerSide - 1,
                               kVoxelsPerSide - 1, kVoxelsPerSide - 1);
        for (const auto& item : blocks_) {
            const GridIndex block_low = blockLocalToGrid(item.first, 0, 0, 0);
            const GridIndex block_high = blockLocalToGrid(item.first, kVoxelsPerSide - 1,
                                                          kVoxelsPerSide - 1,
                                                          kVoxelsPerSide - 1);
            low.x = std::min(low.x, block_low.x); high.x = std::max(high.x, block_high.x);
            low.y = std::min(low.y, block_low.y); high.y = std::max(high.y, block_high.y);
            low.z = std::min(low.z, block_low.z); high.z = std::max(high.z, block_high.z);
        }
    }

    static Idx extent(int low, int high) {
        return static_cast<Idx>(high) - static_cast<Idx>(low) + 1;
    }

    static Idx checkedMultiply(Idx a, Idx b) {
        if (a < 0 || b < 0 || (a != 0 && b > std::numeric_limits<Idx>::max() / a)) {
            throw std::length_error("OccupancyMap: dense export extent overflows int64");
        }
        return a * b;
    }

    static void checkedDenseSize(Idx x, Idx y, Idx z) {
        checkedMultiply(checkedMultiply(x, y), z);
    }

    static size_t checkedVectorSize(Idx x, Idx y, Idx z) {
        const Idx total = checkedMultiply(checkedMultiply(x, y), z);
        if (static_cast<uint64_t>(total) > std::numeric_limits<size_t>::max()) {
            throw std::length_error("OccupancyMap: dense export exceeds addressable memory");
        }
        return static_cast<size_t>(total);
    }

    Scalar noObstacleDistance() const {
        return static_cast<Scalar>(std::sqrt(kFieldInf)) * resolution_;
    }
};

using OccupancyMapd = OccupancyMap<double>;
using OccupancyMapf = OccupancyMap<float>;

}  // namespace NanoVoxMap

#endif  // NANOVOXMAP_HPP
