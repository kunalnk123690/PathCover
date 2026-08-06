/**
 * @file esdf_test.cpp
 * @brief Correctness and timing checks for the signed ESDF backends.
 *
 * ROS-independent by design: the map is a standalone header, and the ESDF is
 * the part most worth pinning down, because three code paths have to agree
 * exactly -- the CPU separable transform, the CUDA one, and the incremental
 * scheduler that recomputes only part of the field. Each check below fixes one
 * of those agreements against something independent:
 *
 *   1. signed distances vs. brute force over every occupied voxel
 *   2. incremental update vs. a full rebuild of the same scene
 *   3. GPU solver vs. CPU solver (skipped in CPU-only builds)
 *   4. interpolated gradient vs. central differences of the same interpolant
 *   5. the zero level set actually sitting on the obstacle surface
 *
 * Build: compiled as `nanovoxmap_esdf_test` by the package CMakeLists.
 * Run:   ./nanovoxmap_esdf_test        (exit code 0 = all checks passed)
 */

#include "nanovoxmap/nanovoxmap.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

using NanoVoxMap::OccupancyMapd;
using Vec3 = Eigen::Vector3d;

namespace {

int g_failures = 0;

void check(bool condition, const char* what) {
    if (condition) {
        std::printf("  [ ok ] %s\n", what);
    } else {
        std::printf("  [FAIL] %s\n", what);
        ++g_failures;
    }
}

double elapsedMs(std::chrono::steady_clock::time_point start) {
    return std::chrono::duration<double, std::milli>(
               std::chrono::steady_clock::now() - start).count();
}

/// A handful of solid boxes: enough surface for the sign to matter, and enough
/// interior for the negative side of the field to be exercised.
std::vector<Vec3> boxScene(double resolution, int seed) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> place(-1.5, 1.5);
    std::vector<Vec3> points;
    for (int box = 0; box < 4; ++box) {
        const Vec3 corner(place(rng), place(rng), place(rng));
        const int side = 6;   // 6 voxels per edge -> a genuinely solid interior
        for (int i = 0; i < side; ++i)
            for (int j = 0; j < side; ++j)
                for (int k = 0; k < side; ++k) {
                    points.push_back(corner + Vec3(i, j, k) * resolution);
                }
    }
    return points;
}

/**
 * Signed distance computed the slow, obviously-correct way: scan every site of
 * the opposite class. The half-voxel shift mirrors the map's surface
 * convention (see solveSignedEsdfCpu) -- distances are to the obstacle face,
 * not to a voxel centre.
 */
double bruteForceSigned(const std::vector<Vec3>& occupied_centres,
                        const std::vector<Vec3>& free_centres,
                        const Vec3& centre, bool is_occupied, double resolution) {
    const std::vector<Vec3>& sites = is_occupied ? free_centres : occupied_centres;
    double best = std::numeric_limits<double>::max();
    for (const Vec3& site : sites) best = std::min(best, (site - centre).squaredNorm());
    const double distance = std::sqrt(best) - 0.5 * resolution;
    return is_occupied ? -distance : distance;
}

// -------------------------------------------------------------------------

void testSignedAgainstBruteForce() {
    std::printf("signed distance vs brute force\n");
    const double res = 0.1;
    OccupancyMapd map(res);
    map.setEsdfMaxDistance(0.8);

    for (const Vec3& p : boxScene(res, 7)) map.setOccupied(p);
    map.updateEsdf();

    // Enumerate the voxel centres of a box that clears the scene by more than
    // the band on every side, so the brute-force reference never misses a
    // nearer site than the map could have found.
    const std::vector<Vec3> occupied = map.getOccupiedVoxels();
    std::vector<Vec3> free_centres;
    const int span = 35;
    for (int i = -span; i <= span; ++i)
        for (int j = -span; j <= span; ++j)
            for (int k = -span; k <= span; ++k) {
                const Vec3 centre((i + 0.5) * res, (j + 0.5) * res, (k + 0.5) * res);
                if (!map.isOccupied(centre)) free_centres.push_back(centre);
            }

    const double band = map.esdfMaxDistance();
    double worst = 0.0;
    int compared = 0;
    std::mt19937 rng(11);
    std::uniform_int_distribution<size_t> pick_occupied(0, occupied.size() - 1);
    std::uniform_int_distribution<int> offset(-12, 12);
    for (int trial = 0; trial < 3000; ++trial) {
        // Sample around obstacles rather than uniformly over the box: the
        // interesting cells are the ones inside the band, and a uniform sample
        // of a 7 m box is almost entirely saturated.
        const Vec3 centre = occupied[pick_occupied(rng)] +
                            Vec3(offset(rng), offset(rng), offset(rng)) * res;
        const bool is_occupied = map.isOccupied(centre);
        const double expected =
            bruteForceSigned(occupied, free_centres, centre, is_occupied, res);
        const double actual = map.signedDistanceAt(centre);
        // Only cells inside the band carry an exact value; outside it both the
        // reference and the field are meant to saturate.
        if (std::fabs(expected) >= band) {
            if (std::fabs(actual) < band - 1e-9) {
                std::printf("  [FAIL] saturated cell reported %.4f (band %.4f)\n", actual, band);
                ++g_failures;
                return;
            }
            continue;
        }
        worst = std::max(worst, std::fabs(expected - actual));
        ++compared;
    }
    std::printf("  compared %d in-band voxels, max error %.3e m\n", compared, worst);
    check(compared > 500, "enough in-band samples to be meaningful");
    check(worst < 1e-5, "signed distance matches brute force inside the band");

    // The sign itself: every occupied voxel negative, every free one positive.
    int wrong_sign = 0;
    for (const Vec3& centre : occupied) {
        if (map.signedDistanceAt(centre) >= 0.0) ++wrong_sign;
    }
    std::uniform_int_distribution<size_t> pick_free(0, free_centres.size() - 1);
    for (int i = 0; i < 2000; ++i) {
        if (map.signedDistanceAt(free_centres[pick_free(rng)]) <= 0.0) ++wrong_sign;
    }
    check(wrong_sign == 0, "sign is negative inside obstacles and positive outside");
}

void testIncrementalMatchesFullRebuild() {
    std::printf("incremental update vs full rebuild\n");
    const double res = 0.05;
    OccupancyMapd map(res);
    map.setEsdfMaxDistance(0.5);

    std::mt19937 rng(3);
    std::uniform_real_distribution<double> jitter(-0.02, 0.02);

    // Integrate several scans, updating the ESDF incrementally after each --
    // this is the pattern the ROS node produces, and the one a full rebuild
    // must be indistinguishable from.
    double incremental_ms = 0.0;
    for (int frame = 0; frame < 6; ++frame) {
        std::vector<Vec3> endpoints;
        for (const Vec3& p : boxScene(res, 20 + frame)) {
            endpoints.push_back(p + Vec3(jitter(rng), jitter(rng), jitter(rng)));
        }
        map.insertPointCloud(Vec3(0.0, 0.0, 3.0), endpoints);
        const auto start = std::chrono::steady_clock::now();
        map.updateEsdf();
        incremental_ms += elapsedMs(start);
    }

    // Snapshot the incrementally-maintained field.
    std::vector<std::pair<Vec3, double>> incremental;
    map.forEachEsdfVoxel([&](const Vec3& centre, double d) {
        incremental.emplace_back(centre, d);
    });

    const auto start = std::chrono::steady_clock::now();
    map.rebuildEsdf();
    const double rebuild_ms = elapsedMs(start);

    double worst = 0.0;
    for (const auto& item : incremental) {
        worst = std::max(worst, std::fabs(item.second - map.signedDistanceAt(item.first)));
    }
    std::printf("  %zu esdf blocks, %zu voxels; incremental %.1f ms total, full rebuild %.1f ms\n",
                map.numEsdfBlocks(), incremental.size(), incremental_ms, rebuild_ms);
    check(!incremental.empty(), "the incremental field is non-empty");
    check(worst < 1e-6, "incremental field is identical to a full rebuild");

    // Clearing an obstacle must shrink the band back, not leave a ghost well.
    const size_t before = map.numEsdfBlocks();
    for (const Vec3& centre : map.getOccupiedVoxels()) map.setFree(centre);
    map.updateEsdf();
    check(map.numOccupied() == 0, "scene cleared");
    check(map.numEsdfBlocks() < before, "esdf blocks are released when obstacles disappear");
    check(map.getSignedDistance(Vec3(0.0, 0.0, 0.0)) >= map.esdfMaxDistance() - 1e-9,
          "an emptied map reports saturated distance everywhere");
}

void testGpuMatchesCpu() {
    std::printf("gpu solver vs cpu solver\n");
    OccupancyMapd probe(0.1);
    if (!probe.esdfUsesGpu()) {
        std::printf("  [skip] built without CUDA\n");
        return;
    }

    const double res = 0.05;
    const std::vector<Vec3> scene = boxScene(res, 41);

    OccupancyMapd cpu_map(res), gpu_map(res);
    cpu_map.setEsdfMaxDistance(0.6);
    gpu_map.setEsdfMaxDistance(0.6);
    cpu_map.setEsdfForceCpu(true);
    for (const Vec3& p : scene) { cpu_map.setOccupied(p); gpu_map.setOccupied(p); }

    auto start = std::chrono::steady_clock::now();
    cpu_map.rebuildEsdf();
    const double cpu_ms = elapsedMs(start);

    start = std::chrono::steady_clock::now();
    gpu_map.rebuildEsdf();
    const double gpu_ms = elapsedMs(start);

    if (!gpu_map.esdfUsesGpu()) {
        std::printf("  [skip] no usable CUDA device at runtime\n");
        return;
    }

    double worst = 0.0;
    size_t visited = 0;
    cpu_map.forEachEsdfVoxel([&](const Vec3& centre, double d) {
        worst = std::max(worst, std::fabs(d - gpu_map.signedDistanceAt(centre)));
        ++visited;
    });
    std::printf("  %zu voxels; cpu %.1f ms, gpu %.1f ms, max difference %.3e m\n",
                visited, cpu_ms, gpu_ms, worst);
    check(visited > 0, "cpu field is non-empty");
    check(cpu_map.numEsdfBlocks() == gpu_map.numEsdfBlocks(),
          "both backends materialize the same blocks");
    check(worst < 1e-6, "gpu field matches the cpu field");
}

void testGradientAndSurface() {
    std::printf("continuous field: gradient and zero level set\n");
    const double res = 0.05;
    OccupancyMapd map(res);
    map.setEsdfMaxDistance(0.6);

    // One axis-aligned slab, addressed by voxel index so the surface position
    // is exact rather than at the mercy of accumulating 0.05 in a double:
    // voxels 0..7 along x are solid, so the boundary plane is x = 8 * res = 0.4.
    const int slab_voxels = 8;
    const double surface_x = slab_voxels * res;
    for (int i = 0; i < slab_voxels; ++i)
        for (int j = -12; j < 12; ++j)
            for (int k = -12; k < 12; ++k) {
                map.setOccupied(Vec3((i + 0.5) * res, (j + 0.5) * res, (k + 0.5) * res));
            }
    map.updateEsdf();

    // Analytic gradient vs central differences of the interpolated value.
    std::mt19937 rng(5);
    std::uniform_real_distribution<double> jitter(-0.15, 0.15);
    const double h = 1e-4;
    double worst = 0.0;
    int compared = 0;
    for (int trial = 0; trial < 500; ++trial) {
        const Vec3 p(0.42 + jitter(rng), jitter(rng), jitter(rng));
        double d;
        Vec3 analytic;
        if (!map.getSignedDistanceWithGradient(p, d, analytic)) continue;
        Vec3 numeric;
        bool usable = true;
        for (int axis = 0; axis < 3; ++axis) {
            Vec3 lo = p, hi = p;
            lo[axis] -= h;
            hi[axis] += h;
            double d_lo, d_hi;
            Vec3 unused;
            usable = usable && map.getSignedDistanceWithGradient(lo, d_lo, unused) &&
                     map.getSignedDistanceWithGradient(hi, d_hi, unused);
            numeric[axis] = (d_hi - d_lo) / (2.0 * h);
        }
        if (!usable) continue;
        worst = std::max(worst, (analytic - numeric).norm());
        ++compared;
    }
    std::printf("  compared %d gradients, max deviation %.3e\n", compared, worst);
    check(compared > 100, "enough gradient samples");
    check(worst < 1e-3, "analytic gradient matches central differences");

    // The zero crossing must land on the face between the last solid voxel and
    // the first free one -- the real surface plane -- not on either centre.
    double crossing = 0.0;
    double lo = surface_x - 0.10, hi = surface_x + 0.10;
    for (int i = 0; i < 60; ++i) {
        crossing = 0.5 * (lo + hi);
        if (map.getSignedDistance(Vec3(crossing, 0.013, -0.021)) < 0.0) lo = crossing;
        else hi = crossing;
    }
    std::printf("  zero crossing at x = %.5f (surface at %.5f)\n", crossing, surface_x);
    check(std::fabs(crossing - surface_x) < 0.02 * res,
          "the interpolated surface sits on the obstacle boundary, not a voxel centre");

    // Continuity and unit slope: walking in 0.5 mm steps, no step may exceed
    // 0.5 mm of change. A staircase would jump a whole voxel; the old
    // centre-to-centre convention would double the slope across the surface.
    const double step = 0.0005;
    double max_jump = 0.0;
    double previous = map.getSignedDistance(Vec3(0.0, 0.013, -0.021));
    for (int i = 1; i <= 2000; ++i) {
        const double d = map.getSignedDistance(Vec3(i * step, 0.013, -0.021));
        max_jump = std::max(max_jump, std::fabs(d - previous));
        previous = d;
    }
    std::printf("  largest step along a %.1f mm walk: %.3e m\n", step * 1e3, max_jump);
    check(max_jump <= step * 1.001,
          "the field is continuous with unit slope (no staircase, no kink at the surface)");
}

void benchmarkScaling() {
    std::printf("timing: incremental vs full rebuild as the map grows\n");
    const double res = 0.05;
    OccupancyMapd map(res);
    map.setEsdfMaxDistance(0.5);

    std::mt19937 rng(9);
    std::uniform_real_distribution<double> spread(-6.0, 6.0);
    for (int frame = 0; frame < 40; ++frame) {
        std::vector<Vec3> endpoints;
        const Vec3 corner(spread(rng), spread(rng), spread(rng));
        for (int i = 0; i < 8; ++i)
            for (int j = 0; j < 8; ++j)
                for (int k = 0; k < 8; ++k) {
                    endpoints.push_back(corner + Vec3(i, j, k) * res);
                }
        map.insertPointCloud(Vec3(0.0, 0.0, 8.0), endpoints);
        map.updateEsdf();
    }

    // One more small scan into the now-large map: this is the number that
    // matters, because it is what the sensor callback pays per frame.
    std::vector<Vec3> endpoints;
    const Vec3 corner(1.0, 1.0, 1.0);
    for (int i = 0; i < 8; ++i)
        for (int j = 0; j < 8; ++j)
            for (int k = 0; k < 8; ++k) endpoints.push_back(corner + Vec3(i, j, k) * res);
    map.insertPointCloud(Vec3(0.0, 0.0, 8.0), endpoints);

    auto start = std::chrono::steady_clock::now();
    map.updateEsdf();
    const double incremental_ms = elapsedMs(start);

    start = std::chrono::steady_clock::now();
    map.rebuildEsdf();
    const double rebuild_ms = elapsedMs(start);

    std::printf("  %zu occupied voxels, %zu esdf blocks (%.1f MB)\n",
                map.numOccupied(), map.numEsdfBlocks(),
                map.esdfAllocatedBytes() / (1024.0 * 1024.0));
    std::printf("  one scan: incremental %.2f ms, full rebuild %.2f ms (%.1fx)\n",
                incremental_ms, rebuild_ms,
                incremental_ms > 0.0 ? rebuild_ms / incremental_ms : 0.0);
    check(incremental_ms < rebuild_ms,
          "an incremental update costs less than rebuilding the whole field");
}

}  // namespace

int main() {
    std::printf("nanovoxmap signed ESDF tests\n\n");
    testSignedAgainstBruteForce();
    std::printf("\n");
    testIncrementalMatchesFullRebuild();
    std::printf("\n");
    testGpuMatchesCpu();
    std::printf("\n");
    testGradientAndSurface();
    std::printf("\n");
    benchmarkScaling();
    std::printf("\n%s (%d failure%s)\n", g_failures == 0 ? "PASSED" : "FAILED",
                g_failures, g_failures == 1 ? "" : "s");
    return g_failures == 0 ? 0 : 1;
}
