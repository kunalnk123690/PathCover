/**
 * @file inflation_benchmark.cpp
 * @brief Times the inflater on a synthetic map, with no ROS in the way.
 *
 * Usage: inflation_benchmark [points] [span_m] [resolution] [radius_xy]
 *                            [radius_z] [shape] [threads]
 *
 * Reports the two cases that matter for online planning: the first message,
 * which pays for the whole map, and a follow-up message that grows the map a
 * little, which is all a steady stream of updates should cost.
 *
 * Where there is a CUDA device the whole run is repeated on each backend, so
 * what the GPU dilation is worth on this machine and this map is a difference
 * between two columns rather than a claim.
 */

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <string>
#include <vector>

#include "occupancy_inflation/point_cloud_inflater.h"

using occupancy_inflation::InflationParams;
using occupancy_inflation::InflationShape;
using occupancy_inflation::InflationStats;
using occupancy_inflation::PointCloudInflater;

namespace {

struct Point {
  float x, y, z;
};

struct VectorReader {
  const std::vector<Point> *pts;
  std::size_t size() const { return pts->size(); }
  void point(std::size_t i, float &x, float &y, float &z) const {
    x = (*pts)[i].x;
    y = (*pts)[i].y;
    z = (*pts)[i].z;
  }
};

/** Points on the surfaces of random pillars, so the map is sparse like a real one. */
std::vector<Point> pillarCloud(std::mt19937 &rng, std::size_t n, float span,
                               float height) {
  std::uniform_real_distribution<float> u(-span, span);
  std::uniform_real_distribution<float> h(0.0f, height);
  std::uniform_real_distribution<float> a(0.0f, 6.2831853f);

  const std::size_t pillars = std::max<std::size_t>(1, n / 400);
  std::vector<Point> centres(pillars);
  for (Point &c : centres) c = Point{u(rng), u(rng), 0.0f};

  std::vector<Point> pts(n);
  for (std::size_t i = 0; i < n; ++i) {
    const Point &c = centres[i % pillars];
    const float t = a(rng);
    pts[i] = Point{c.x + 0.35f * std::cos(t), c.y + 0.35f * std::sin(t), h(rng)};
  }
  return pts;
}

double timeInflate(PointCloudInflater &inf, const std::vector<Point> &pts,
                   InflationStats *stats) {
  VectorReader reader{&pts};
  const auto t0 = std::chrono::steady_clock::now();
  inf.inflate(reader, stats);
  return std::chrono::duration<double, std::milli>(
             std::chrono::steady_clock::now() - t0)
      .count();
}

/** One full pass over the cases, on whichever backend @p p selects. */
bool runCases(const InflationParams &p, const std::vector<Point> &first,
              const std::vector<Point> &grown, const char *backend) {
  PointCloudInflater inf;
  std::string error;
  if (!inf.configure(p, &error)) {
    std::fprintf(stderr, "configure failed: %s\n", error.c_str());
    return false;
  }

  std::printf("--- %s ---\n", backend);

  InflationStats stats;
  double ms = timeInflate(inf, first, &stats);
  const double cold_ms = ms;
  std::printf("cold  : %8.2f ms  %8zu seeds -> %8zu voxels  grids %zu KiB%s\n",
              ms, stats.seed_voxels, stats.inflated_voxels,
              inf.memoryBytes() >> 10, stats.gpu_dilation ? "  [gpu]" : "");

  /* A map that has stopped changing: the whole cost is scanning the points, so
   * this is the floor neither backend can go below while the voxelisation
   * stays on the CPU. */
  ms = timeInflate(inf, first, &stats);
  std::printf("repeat: %8.2f ms  no new voxels (changed=%d)\n", ms,
              static_cast<int>(stats.changed));

  /* The steady state online: the mapper has explored a new strip and resends
   * everything it has. Only the strip is new, so only the strip is dilated. */
  ms = timeInflate(inf, grown, &stats);
  std::printf("grow  : %8.2f ms  +%zu seeds -> %zu voxels  grids %zu KiB%s\n",
              ms, stats.new_seed_voxels, stats.inflated_voxels,
              inf.memoryBytes() >> 10, stats.gpu_dilation ? "  [gpu]" : "");

  /* What the same work costs without the incremental shortcut. This is the
   * case the GPU has the most to take off, because it dilates the whole map
   * rather than a strip of it. */
  InflationParams q = p;
  q.incremental = false;
  PointCloudInflater one_shot;
  if (!one_shot.configure(q, &error)) return false;
  ms = timeInflate(one_shot, grown, &stats);
  std::printf("one-shot rebuild: %8.2f ms  %zu seeds -> %zu voxels%s\n", ms,
              stats.seed_voxels, stats.inflated_voxels,
              stats.gpu_dilation ? "  [gpu]" : "");

  /* Publishing is its own cost, and at this map size it is the dominant one.
   * It is the same CPU work on either backend. */
  std::vector<uint8_t> buf(inf.occupiedCount() * 16);
  const auto t0 = std::chrono::steady_clock::now();
  inf.serialize(buf.data(), 16, true);
  const double ser = std::chrono::duration<double, std::milli>(
                         std::chrono::steady_clock::now() - t0)
                         .count();
  std::printf("serialise: %8.2f ms for %zu points, %.1f MiB per message\n",
              ser, inf.occupiedCount(), buf.size() / 1048576.0);

  /* What publish_surface_only takes off the wire. The map behind it is the
   * same one; only the interior stops being serialised, so the comparison to
   * read is published points against the seed count, which is the size the
   * mapper's own cloud was before any of this ran. */
  InflationParams shell = p;
  shell.publish_surface_only = true;
  PointCloudInflater peeled;
  if (!peeled.configure(shell, &error)) return false;
  InflationStats shell_stats;
  const double shell_ms = timeInflate(peeled, first, &shell_stats);
  std::vector<uint8_t> shell_buf(peeled.occupiedCount() * 16);
  const auto t1 = std::chrono::steady_clock::now();
  peeled.serialize(shell_buf.data(), 16, true);
  const double shell_ser = std::chrono::duration<double, std::milli>(
                               std::chrono::steady_clock::now() - t1)
                               .count();
  std::printf(
      "surface-only: %8.2f ms cold (%+.2f ms), serialise %6.2f ms for %zu "
      "points, %.1f MiB\n"
      "              %zu of %zu inflated (%.0f%%), %.2fx the %zu seed voxels\n\n",
      shell_ms, shell_ms - cold_ms, shell_ser, peeled.occupiedCount(),
      shell_buf.size() / 1048576.0, peeled.occupiedCount(),
      peeled.inflatedCount(),
      100.0 * peeled.occupiedCount() / std::max<std::size_t>(1, peeled.inflatedCount()),
      static_cast<double>(peeled.occupiedCount()) /
          std::max<std::size_t>(1, shell_stats.seed_voxels),
      shell_stats.seed_voxels);
  return true;
}

}  // namespace

int main(int argc, char **argv) {
  const std::size_t points = argc > 1 ? std::strtoul(argv[1], nullptr, 10) : 500000;
  const float span = argc > 2 ? std::strtof(argv[2], nullptr) : 25.0f;
  InflationParams p;
  p.resolution = argc > 3 ? std::strtod(argv[3], nullptr) : 0.1;
  p.radius_xy = argc > 4 ? std::strtod(argv[4], nullptr) : 0.2;
  p.radius_z = argc > 5 ? std::strtod(argv[5], nullptr) : 0.2;
  const std::string shape = argc > 6 ? argv[6] : "box";
  p.num_threads = argc > 7 ? std::atoi(argv[7]) : 4;
  p.incremental = true;

  if (!PointCloudInflater::parseShape(shape, &p.shape)) {
    std::fprintf(stderr, "unknown shape '%s'\n", shape.c_str());
    return 1;
  }

  std::mt19937 rng(7);
  const std::vector<Point> first = pillarCloud(rng, points, span, 3.0f);

  /* The mapper has explored a new strip and resends everything it has. */
  std::vector<Point> frontier = pillarCloud(rng, points / 20, span * 0.2f, 3.0f);
  for (Point &q : frontier) q.x += span * 1.1f;
  std::vector<Point> grown = first;
  grown.insert(grown.end(), frontier.begin(), frontier.end());

  {
    PointCloudInflater probe;
    std::string error;
    if (!probe.configure(p, &error)) {
      std::fprintf(stderr, "configure failed: %s\n", error.c_str());
      return 1;
    }
    std::printf(
        "%zu points, span %.1f m, res %.2f m, r_xy %.2f, r_z %.2f, %s, %d thread(s)\n",
        points, span, p.resolution, p.radius_xy, p.radius_z, shape.c_str(),
        p.num_threads);
    std::printf("kernel: %zu voxels\n", probe.kernelSize());
    std::printf("cuda:   %s\n\n",
                occupancy_inflation::cuda::built()
                    ? (occupancy_inflation::cuda::available()
                           ? "device present"
                           : "built, but no device; CPU only")
                    : "not built; CPU only");
  }

  /* The CPU column first, so it is the baseline the GPU column is read
   * against. Only the dilation moves between them: the point scan and the
   * serialisation are the same work either way, which is why "repeat" -- a
   * message that dilates nothing -- should barely shift. */
  InflationParams cpu = p;
  cpu.use_cuda = false;
  if (!runCases(cpu, first, grown, "cpu dilation")) return 1;

  if (occupancy_inflation::cuda::available()) {
    /* Grids this size clear the threshold comfortably, but drop it anyway so
     * the incremental "grow" case is measured on the device too. */
    InflationParams gpu = p;
    gpu.use_cuda = true;
    gpu.cuda_min_grid_voxels = 0;
    if (!runCases(gpu, first, grown, "gpu dilation")) return 1;
  }

  std::printf("(publish_on_change_only keeps the serialised map off the wire "
              "when it is static)\n");
  return 0;
}
