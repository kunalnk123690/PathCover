/**
 * @file inflation_test.cpp
 * @brief Checks the word-parallel inflation against a brute-force reference.
 *
 * The fast path replaces a per-voxel hash set with bit shifts over a dense
 * grid, separable passes for boxes and run decomposition for ellipsoids. None
 * of that is obvious by inspection, so this compares it voxel for voxel with
 * the straightforward "for every seed, for every kernel cell" implementation
 * across shapes, radii, filters, thread counts and incremental replays.
 *
 * When the binary was built with CUDA and a device answers, the whole matrix
 * runs a second time with the dilation forced onto the GPU, and a final
 * section checks the two backends against each other rather than only against
 * the reference: they perform the same shifted ORs on the same words, so their
 * output has to match bit for bit, not merely closely.
 *
 * No ROS and no gtest: build and run it directly.
 *   g++ -O2 -std=c++14 -I include -I /usr/include/eigen3 \
 *       src/voxel_bit_grid.cpp src/point_cloud_inflater.cpp \
 *       test/inflation_test.cpp -o /tmp/inflation_test && /tmp/inflation_test
 * With the GPU backend, compile src/inflation_cuda.cu alongside it and define
 * OCCUPANCY_INFLATION_WITH_CUDA.
 */

#include <algorithm>
#include <climits>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <set>
#include <tuple>
#include <vector>

#include "occupancy_inflation/point_cloud_inflater.h"

using occupancy_inflation::InflationParams;
using occupancy_inflation::InflationShape;
using occupancy_inflation::InflationStats;
using occupancy_inflation::PointCloudInflater;

namespace {

int g_failures = 0;

#define CHECK(cond, ...)                                     \
  do {                                                       \
    if (!(cond)) {                                           \
      std::printf("FAIL %s:%d: ", __FILE__, __LINE__);       \
      std::printf(__VA_ARGS__);                              \
      std::printf("\n");                                     \
      ++g_failures;                                          \
    }                                                        \
  } while (0)

struct Point {
  float x, y, z;
};

/** Minimal reader in the shape PointCloudInflater::inflate() expects. */
struct VectorReader {
  const std::vector<Point> *pts;
  std::size_t size() const { return pts->size(); }
  void point(std::size_t i, float &x, float &y, float &z) const {
    x = (*pts)[i].x;
    y = (*pts)[i].y;
    z = (*pts)[i].z;
  }
};

using Voxel = std::tuple<int, int, int>;

Eigen::Vector3i toIndex(const InflationParams &p, double x, double y, double z) {
  return Eigen::Vector3i(
      static_cast<int>(std::floor((x - p.origin.x()) / p.resolution)),
      static_cast<int>(std::floor((y - p.origin.y()) / p.resolution)),
      static_cast<int>(std::floor((z - p.origin.z()) / p.resolution)));
}

/** The original structuring element, built exactly as the old code built it. */
std::vector<Eigen::Vector3i> referenceKernel(const InflationParams &p) {
  const int rx = static_cast<int>(std::ceil(p.radius_xy / p.resolution));
  const int rz = static_cast<int>(std::ceil(p.radius_z / p.resolution));
  const double radii[3] = {p.radius_xy, p.radius_xy, p.radius_z};

  std::vector<Eigen::Vector3i> kernel;
  for (int dx = -rx; dx <= rx; ++dx) {
    for (int dy = -rx; dy <= rx; ++dy) {
      for (int dz = -rz; dz <= rz; ++dz) {
        if (p.shape == InflationShape::kEllipsoid) {
          const double ex[3] = {dx * p.resolution, dy * p.resolution,
                                dz * p.resolution};
          double norm = 0.0;
          bool ok = true;
          for (int i = 0; i < 3; ++i) {
            if (radii[i] <= 0.0) {
              if (ex[i] != 0.0) {
                ok = false;
                break;
              }
              continue;
            }
            const double t = ex[i] / radii[i];
            norm += t * t;
          }
          if (!ok || norm > 1.0) continue;
        }
        kernel.emplace_back(dx, dy, dz);
      }
    }
  }
  return kernel;
}

/** Brute-force seeds -> inflated voxels, as sets of integer indices. */
void referenceInflate(const InflationParams &p, const std::set<Voxel> &seeds,
                      std::set<Voxel> *inflated) {
  const std::vector<Eigen::Vector3i> kernel = referenceKernel(p);
  inflated->clear();
  for (const Voxel &s : seeds) {
    for (const Eigen::Vector3i &k : kernel) {
      const int ix = std::get<0>(s) + k.x();
      const int iy = std::get<1>(s) + k.y();
      const int iz = std::get<2>(s) + k.z();
      if (p.output_bounds_enable) {
        const double cx = (ix + 0.5) * p.resolution + p.origin.x();
        const double cy = (iy + 0.5) * p.resolution + p.origin.y();
        const double cz = (iz + 0.5) * p.resolution + p.origin.z();
        if (cx < p.output_min.x() || cx > p.output_max.x() ||
            cy < p.output_min.y() || cy > p.output_max.y() ||
            cz < p.output_min.z() || cz > p.output_max.z()) {
          continue;
        }
      }
      inflated->emplace(ix, iy, iz);
    }
  }
}

/** Add the points that pass the input filter to @p seeds. */
void referenceSeeds(const InflationParams &p, const std::vector<Point> &pts,
                    std::set<Voxel> *seeds) {
  for (const Point &pt : pts) {
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
      continue;
    }
    if (p.input_filter_enable &&
        (pt.x < p.input_min.x() || pt.x > p.input_max.x() ||
         pt.y < p.input_min.y() || pt.y > p.input_max.y() ||
         pt.z < p.input_min.z() || pt.z > p.input_max.z())) {
      continue;
    }
    const Eigen::Vector3i idx = toIndex(p, pt.x, pt.y, pt.z);
    seeds->emplace(idx.x(), idx.y(), idx.z());
  }
}

/** Read serialize() back into voxel indices plus the intensity written. */
std::set<std::tuple<int, int, int, int>> readBack(const PointCloudInflater &inf,
                                                  const InflationParams &p,
                                                  bool seeds_only) {
  const std::size_t n = seeds_only ? inf.seedCount() : inf.occupiedCount();
  std::vector<uint8_t> buf(n * 16 + 16);
  const std::size_t written =
      seeds_only ? inf.serializeSeeds(buf.data(), 16, true)
                 : inf.serialize(buf.data(), 16, true);
  CHECK(written == n, "serialize wrote %zu, expected %zu", written, n);

  std::set<std::tuple<int, int, int, int>> out;
  for (std::size_t i = 0; i < written; ++i) {
    const float *rec = reinterpret_cast<const float *>(buf.data() + i * 16);
    const Eigen::Vector3i idx = toIndex(p, rec[0], rec[1], rec[2]);
    out.emplace(idx.x(), idx.y(), idx.z(), static_cast<int>(rec[3]));
  }
  CHECK(out.size() == written, "serialize emitted %zu duplicate voxels",
        written - out.size());
  return out;
}

void compare(const char *label, const InflationParams &p,
             const std::set<Voxel> &seeds, const PointCloudInflater &inf) {
  std::set<Voxel> expected;
  referenceInflate(p, seeds, &expected);

  const auto got = readBack(inf, p, false);
  CHECK(got.size() == expected.size(), "%s: %zu inflated voxels, expected %zu",
        label, got.size(), expected.size());

  for (const auto &g : got) {
    const Voxel v(std::get<0>(g), std::get<1>(g), std::get<2>(g));
    CHECK(expected.count(v) != 0, "%s: unexpected voxel (%d,%d,%d)", label,
          std::get<0>(g), std::get<1>(g), std::get<2>(g));
    const int want_intensity = seeds.count(v) != 0 ? 0 : 1;
    CHECK(std::get<3>(g) == want_intensity,
          "%s: voxel (%d,%d,%d) intensity %d, expected %d", label,
          std::get<0>(g), std::get<1>(g), std::get<2>(g), std::get<3>(g),
          want_intensity);
  }
  for (const Voxel &v : expected) {
    CHECK(got.count(std::make_tuple(std::get<0>(v), std::get<1>(v),
                                    std::get<2>(v), 0)) != 0 ||
              got.count(std::make_tuple(std::get<0>(v), std::get<1>(v),
                                        std::get<2>(v), 1)) != 0,
          "%s: missing voxel (%d,%d,%d)", label, std::get<0>(v),
          std::get<1>(v), std::get<2>(v));
  }

  /* The seed cloud is the voxelised input with no inflation at all. */
  const auto seed_out = readBack(inf, p, true);
  CHECK(seed_out.size() == seeds.size(), "%s: %zu seed voxels, expected %zu",
        label, seed_out.size(), seeds.size());
  for (const auto &s : seed_out) {
    CHECK(seeds.count(Voxel(std::get<0>(s), std::get<1>(s), std::get<2>(s))) != 0,
          "%s: unexpected seed voxel", label);
  }
}

/**
 * The boundary shell of @p solid: every voxel with at least one voxel outside
 * @p solid within @p t on all three axes. Built straight from the definition,
 * against which the separable shifted-AND erosion is checked.
 */
std::set<Voxel> referenceShell(const std::set<Voxel> &solid, int t) {
  std::set<Voxel> shell;
  for (const Voxel &v : solid) {
    bool boundary = false;
    for (int dx = -t; dx <= t && !boundary; ++dx) {
      for (int dy = -t; dy <= t && !boundary; ++dy) {
        for (int dz = -t; dz <= t && !boundary; ++dz) {
          if (solid.count(Voxel(std::get<0>(v) + dx, std::get<1>(v) + dy,
                                std::get<2>(v) + dz)) == 0) {
            boundary = true;
          }
        }
      }
    }
    if (boundary) shell.insert(v);
  }
  return shell;
}

/**
 * The safety property the shell rests on: nothing reaches the interior.
 *
 * Floods the complement of @p published with 26-connected steps, starting from
 * a corner well outside the map, and returns false if the flood ever lands on
 * a voxel of @p solid. Publishing only the shell is only sound because that
 * cannot happen: a step from a free voxel onto a discarded interior voxel would
 * make it a neighbour of free space, which is what "interior" rules out. This
 * is the check, not the argument.
 */
bool shellIsWatertight(const std::set<Voxel> &solid,
                       const std::set<Voxel> &published) {
  if (solid.empty()) return true;

  int lo[3] = {INT_MAX, INT_MAX, INT_MAX};
  int hi[3] = {INT_MIN, INT_MIN, INT_MIN};
  for (const Voxel &v : solid) {
    const int c[3] = {std::get<0>(v), std::get<1>(v), std::get<2>(v)};
    for (int i = 0; i < 3; ++i) {
      lo[i] = std::min(lo[i], c[i]);
      hi[i] = std::max(hi[i], c[i]);
    }
  }
  /* One voxel of margin is all the flood needs to get round the outside. */
  for (int i = 0; i < 3; ++i) {
    lo[i] -= 1;
    hi[i] += 1;
  }

  std::set<Voxel> seen;
  std::vector<Voxel> stack;
  const Voxel start(lo[0], lo[1], lo[2]);
  stack.push_back(start);
  seen.insert(start);

  while (!stack.empty()) {
    const Voxel v = stack.back();
    stack.pop_back();
    if (solid.count(v) != 0) return false;  // reached the hollow interior
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dz = -1; dz <= 1; ++dz) {
          if (dx == 0 && dy == 0 && dz == 0) continue;
          const int c[3] = {std::get<0>(v) + dx, std::get<1>(v) + dy,
                            std::get<2>(v) + dz};
          if (c[0] < lo[0] || c[0] > hi[0] || c[1] < lo[1] || c[1] > hi[1] ||
              c[2] < lo[2] || c[2] > hi[2]) {
            continue;
          }
          const Voxel n(c[0], c[1], c[2]);
          if (published.count(n) != 0) continue;  // blocked by the shell
          if (seen.insert(n).second) stack.push_back(n);
        }
      }
    }
  }
  return true;
}

/** The inflated map as the bytes serialize() would put on the wire. */
std::vector<uint8_t> rawOutput(const PointCloudInflater &inf) {
  std::vector<uint8_t> buf(inf.occupiedCount() * 16);
  inf.serialize(buf.data(), 16, true);
  return buf;
}

std::vector<Point> randomPoints(std::mt19937 &rng, int n, float span) {
  std::uniform_real_distribution<float> u(-span, span);
  std::vector<Point> pts(n);
  for (Point &p : pts) p = Point{u(rng), u(rng), u(rng)};
  return pts;
}

}  // namespace

int main() {
  std::mt19937 rng(12345);

  /* The GPU backend is only a second code path where there is a device to run
   * it on, so every check below that mentions it is skipped otherwise -- and
   * says so, rather than passing quietly on a machine that never ran it. */
  const bool have_gpu = occupancy_inflation::cuda::available();
  const int backends = have_gpu ? 2 : 1;
  std::printf("cuda backend: %s\n",
              occupancy_inflation::cuda::built()
                  ? (have_gpu ? "built, device present" : "built, no device")
                  : "not built");

  /* --- one-shot inflation across shapes, radii, filters and thread counts --- */
  const double radii_xy[] = {0.0, 0.1, 0.2, 0.35, 0.7};
  const double radii_z[] = {0.0, 0.1, 0.25};
  const InflationShape shapes[] = {InflationShape::kBox,
                                   InflationShape::kEllipsoid};
  const int thread_counts[] = {1, 4};

  int cases = 0;
  for (int backend = 0; backend < backends; ++backend) {
  for (InflationShape shape : shapes) {
    for (double rxy : radii_xy) {
      for (double rz : radii_z) {
        for (int threads : thread_counts) {
          for (int variant = 0; variant < 2; ++variant) {
            InflationParams p;
            p.resolution = 0.1;
            p.radius_xy = rxy;
            p.radius_z = rz;
            p.shape = shape;
            p.incremental = false;
            p.num_threads = threads;
            p.use_cuda = backend == 1;
            /* These grids are far too small to be worth a device round trip,
             * so the threshold has to come off for the GPU pass to run at
             * all. */
            p.cuda_min_grid_voxels = 0;
            /* An origin off the lattice and negative coordinates are where
             * index rounding and the word alignment are easiest to get wrong. */
            p.origin = variant ? Eigen::Vector3d(0.03, -0.11, 0.07)
                               : Eigen::Vector3d::Zero();
            if (variant) {
              p.input_filter_enable = true;
              p.input_min = Eigen::Vector3d(-1.5, -1.5, -0.4);
              p.input_max = Eigen::Vector3d(1.5, 1.5, 1.2);
              p.output_bounds_enable = true;
              p.output_min = Eigen::Vector3d(-1.2, -1.3, -0.2);
              p.output_max = Eigen::Vector3d(1.1, 1.4, 1.0);
            }

            PointCloudInflater inf;
            std::string error;
            CHECK(inf.configure(p, &error), "configure failed: %s", error.c_str());

            const std::vector<Point> pts = randomPoints(rng, 400, 2.0f);
            VectorReader reader{&pts};
            InflationStats stats;
            inf.inflate(reader, &stats);
            CHECK(stats.ok, "inflate reported not ok");

            std::set<Voxel> seeds;
            referenceSeeds(p, pts, &seeds);
            CHECK(stats.seed_voxels == seeds.size(),
                  "seed count %zu, expected %zu", stats.seed_voxels,
                  seeds.size());

            char label[128];
            std::snprintf(label, sizeof(label),
                          "shape=%d rxy=%.2f rz=%.2f threads=%d variant=%d %s",
                          static_cast<int>(shape), rxy, rz, threads, variant,
                          backend == 1 ? "gpu" : "cpu");
            compare(label, p, seeds, inf);
            ++cases;
          }
        }
      }
    }
  }
  }
  std::printf("one-shot cases: %d across %d backend(s)\n", cases, backends);

  /* --- publish_surface_only: the shell of the same dilated map --------------
   * The dilation itself must not change, so each case checks two things: the
   * map behind the output is still voxel for voxel what the solid mode built,
   * and what comes out of serialize() is exactly its boundary shell. The
   * watertightness flood is the third: it is the property that makes dropping
   * the interior safe, so it is checked rather than argued. */
  int shell_cases = 0;
  for (int backend = 0; backend < backends; ++backend) {
    for (InflationShape shape : shapes) {
      for (double rxy : {0.2, 0.45}) {
        for (int thickness : {1, 2}) {
          for (int threads : thread_counts) {
            for (int inc = 0; inc < 2; ++inc) {
              InflationParams p;
              p.resolution = 0.1;
              p.radius_xy = rxy;
              p.radius_z = 0.2;
              p.shape = shape;
              p.incremental = inc != 0;
              p.num_threads = threads;
              p.use_cuda = backend == 1;
              p.cuda_min_grid_voxels = 0;
              p.publish_surface_only = true;
              p.surface_thickness = thickness;
              p.origin = Eigen::Vector3d(0.03, -0.11, 0.07);

              PointCloudInflater inf;
              std::string error;
              CHECK(inf.configure(p, &error), "configure failed: %s",
                    error.c_str());

              /* Two messages, so the incremental path has to keep the shell
               * right after voxels that were on the surface became interior. */
              std::set<Voxel> seeds;
              InflationStats stats;
              for (int msg = 0; msg < 2; ++msg) {
                /* A tight span so the dilated set is chunky enough to have an
                 * interior worth dropping. */
                const std::vector<Point> pts =
                    randomPoints(rng, 250, msg == 0 ? 1.1f : 1.6f);
                VectorReader reader{&pts};
                inf.inflate(reader, &stats);
                CHECK(stats.ok, "surface inflate reported not ok");
                if (!p.incremental) seeds.clear();
                referenceSeeds(p, pts, &seeds);
              }

              std::set<Voxel> solid;
              referenceInflate(p, seeds, &solid);
              const std::set<Voxel> shell = referenceShell(solid, thickness);

              char label[160];
              std::snprintf(label, sizeof(label),
                            "shell shape=%d rxy=%.2f t=%d threads=%d inc=%d %s",
                            static_cast<int>(shape), rxy, thickness, threads, inc,
                            backend == 1 ? "gpu" : "cpu");

              /* The map behind the output is untouched by the peel. */
              CHECK(inf.inflatedCount() == solid.size(),
                    "%s: %zu inflated voxels, expected %zu", label,
                    inf.inflatedCount(), solid.size());
              CHECK(stats.inflated_voxels == solid.size(),
                    "%s: stats.inflated_voxels %zu, expected %zu", label,
                    stats.inflated_voxels, solid.size());

              CHECK(inf.occupiedCount() == shell.size(),
                    "%s: %zu published voxels, expected %zu", label,
                    inf.occupiedCount(), shell.size());
              CHECK(stats.output_voxels == shell.size(),
                    "%s: stats.output_voxels %zu, expected %zu", label,
                    stats.output_voxels, shell.size());

              const auto got = readBack(inf, p, false);
              std::set<Voxel> got_voxels;
              for (const auto &g : got) {
                const Voxel v(std::get<0>(g), std::get<1>(g), std::get<2>(g));
                got_voxels.insert(v);
                CHECK(shell.count(v) != 0, "%s: unexpected voxel (%d,%d,%d)",
                      label, std::get<0>(v), std::get<1>(v), std::get<2>(v));
                const int want_intensity = seeds.count(v) != 0 ? 0 : 1;
                CHECK(std::get<3>(g) == want_intensity,
                      "%s: voxel (%d,%d,%d) intensity %d, expected %d", label,
                      std::get<0>(v), std::get<1>(v), std::get<2>(v),
                      std::get<3>(g), want_intensity);
              }
              for (const Voxel &v : shell) {
                CHECK(got_voxels.count(v) != 0, "%s: missing voxel (%d,%d,%d)",
                      label, std::get<0>(v), std::get<1>(v), std::get<2>(v));
              }

              CHECK(shellIsWatertight(solid, got_voxels),
                    "%s: the interior is reachable through the shell", label);

              /* At this radius the dilated blobs are solid enough to have an
               * interior, so a shell equal to the map would mean the peel did
               * nothing and every check above passed vacuously. */
              if (rxy >= 0.45) {
                CHECK(shell.size() < solid.size(),
                      "%s: the shell dropped nothing from %zu voxels", label,
                      solid.size());
              }

              /* The seed cloud is unaffected: it is the voxelised input. */
              const auto seed_out = readBack(inf, p, true);
              CHECK(seed_out.size() == seeds.size(),
                    "%s: %zu seed voxels, expected %zu", label, seed_out.size(),
                    seeds.size());
              ++shell_cases;
            }
          }
        }
      }
    }
  }
  std::printf("surface-shell cases: %d across %d backend(s)\n", shell_cases,
              backends);

  /* --- a shell thicker than the map is the map ---------------------------- */
  {
    InflationParams p;
    p.resolution = 0.1;
    p.radius_xy = 0.2;
    p.radius_z = 0.1;
    p.incremental = false;
    p.publish_surface_only = true;
    p.surface_thickness = 64;  // wider than anything this cloud dilates to

    PointCloudInflater inf;
    std::string error;
    CHECK(inf.configure(p, &error), "configure failed: %s", error.c_str());

    const std::vector<Point> pts = randomPoints(rng, 200, 1.0f);
    VectorReader reader{&pts};
    InflationStats stats;
    inf.inflate(reader, &stats);
    CHECK(stats.ok, "thick-shell inflate reported not ok");
    CHECK(inf.occupiedCount() == inf.inflatedCount(),
          "a shell wider than the map dropped %zu voxels",
          inf.inflatedCount() - inf.occupiedCount());

    /* And an empty map leaves an empty shell rather than a stale one. */
    std::vector<Point> none;
    VectorReader empty{&none};
    inf.inflate(empty, &stats);
    CHECK(stats.ok, "emptying the map with a shell reported not ok");
    CHECK(inf.occupiedCount() == 0, "empty map published %zu shell voxels",
          inf.occupiedCount());
    CHECK(stats.output_voxels == 0, "empty map reported %zu output voxels",
          stats.output_voxels);
  }

  /* --- surface_thickness below 1 is refused rather than silently clamped --- */
  {
    InflationParams p;
    p.publish_surface_only = true;
    p.surface_thickness = 0;
    PointCloudInflater inf;
    std::string error;
    CHECK(!inf.configure(p, &error), "surface_thickness 0 was accepted");
  }

  /* --- incremental accumulation over a map that keeps growing --- */
  for (InflationShape shape : shapes) {
    for (int threads : {1, 4}) {
      InflationParams p;
      p.resolution = 0.1;
      p.radius_xy = 0.25;
      p.radius_z = 0.2;
      p.shape = shape;
      p.incremental = true;
      p.num_threads = threads;
      p.origin = Eigen::Vector3d(-0.05, 0.02, 0.0);

      PointCloudInflater inf;
      std::string error;
      CHECK(inf.configure(p, &error), "configure failed: %s", error.c_str());

      std::set<Voxel> seeds;
      float span = 1.0f;
      for (int msg = 0; msg < 6; ++msg) {
        /* Each message covers a wider area than the last, which is what forces
         * the persistent grids to grow and relocate their contents. */
        span += 0.9f;
        const std::vector<Point> pts = randomPoints(rng, 300, span);
        VectorReader reader{&pts};
        InflationStats stats;
        inf.inflate(reader, &stats);
        CHECK(stats.ok, "incremental inflate reported not ok");

        const std::size_t before = seeds.size();
        referenceSeeds(p, pts, &seeds);
        CHECK(stats.new_seed_voxels == seeds.size() - before,
              "new seed count %zu, expected %zu", stats.new_seed_voxels,
              seeds.size() - before);
        CHECK(stats.changed == (seeds.size() != before),
              "changed flag %d on message %d", static_cast<int>(stats.changed),
              msg);

        char label[64];
        std::snprintf(label, sizeof(label), "incremental shape=%d msg=%d",
                      static_cast<int>(shape), msg);
        compare(label, p, seeds, inf);
      }

      /* Replaying a message the map has already absorbed must be a no-op. */
      const std::vector<Point> pts = randomPoints(rng, 50, 0.5f);
      VectorReader reader{&pts};
      InflationStats first;
      inf.inflate(reader, &first);
      InflationStats again;
      inf.inflate(reader, &again);
      CHECK(!again.changed, "replaying a seen cloud reported a change");
      CHECK(again.new_seed_voxels == 0, "replay added %zu voxels",
            again.new_seed_voxels);
      CHECK(again.inflated_voxels == first.inflated_voxels,
            "replay changed the voxel count");
    }
  }

  /* --- an unchanged cloud in one-shot mode is also a no-op --- */
  {
    InflationParams p;
    p.incremental = false;
    PointCloudInflater inf;
    std::string error;
    CHECK(inf.configure(p, &error), "configure failed: %s", error.c_str());

    const std::vector<Point> pts = randomPoints(rng, 200, 1.5f);
    VectorReader reader{&pts};
    InflationStats a, b;
    inf.inflate(reader, &a);
    inf.inflate(reader, &b);
    CHECK(a.changed, "first cloud reported no change");
    CHECK(!b.changed, "identical cloud reported a change");

    /* Dropping points must be picked up, unlike in incremental mode. */
    std::vector<Point> fewer(pts.begin(), pts.begin() + 50);
    VectorReader small{&fewer};
    InflationStats c;
    inf.inflate(small, &c);
    CHECK(c.changed, "a shrinking cloud reported no change");
    std::set<Voxel> seeds;
    referenceSeeds(p, fewer, &seeds);
    compare("one-shot shrink", p, seeds, inf);

    /* An empty cloud empties the map. */
    std::vector<Point> none;
    VectorReader empty{&none};
    InflationStats d;
    inf.inflate(empty, &d);
    CHECK(d.changed, "emptying the map reported no change");
    CHECK(d.inflated_voxels == 0, "empty cloud left %zu voxels",
          d.inflated_voxels);
  }

  /* --- a cloud large enough to take the threaded paths --- *
   * The parallel scatter, dilation and serialisation only switch on above a
   * point count and a grid size, so the cases above all run single threaded.
   * This one is sized to cross both thresholds. */
  for (InflationShape shape : shapes) {
    InflationParams p;
    p.resolution = 0.1;
    p.radius_xy = 0.2;
    p.radius_z = 0.2;
    p.shape = shape;
    p.incremental = false;
    p.num_threads = 8;
    p.origin = Eigen::Vector3d(0.017, -0.041, 0.0);

    PointCloudInflater inf;
    std::string error;
    CHECK(inf.configure(p, &error), "configure failed: %s", error.c_str());

    const std::vector<Point> pts = randomPoints(rng, 30000, 6.0f);
    VectorReader reader{&pts};
    InflationStats stats;
    inf.inflate(reader, &stats);
    CHECK(stats.ok, "threaded inflate reported not ok");
    CHECK(stats.filtered_points == 30000, "kept %zu of 30000 points",
          stats.filtered_points);

    std::set<Voxel> seeds;
    referenceSeeds(p, pts, &seeds);
    char label[64];
    std::snprintf(label, sizeof(label), "threaded shape=%d",
                  static_cast<int>(shape));
    compare(label, p, seeds, inf);
  }

  /* --- non-finite points are dropped rather than poisoning the extent --- */
  {
    InflationParams p;
    p.incremental = false;
    PointCloudInflater inf;
    std::string error;
    CHECK(inf.configure(p, &error), "configure failed: %s", error.c_str());

    std::vector<Point> pts = randomPoints(rng, 100, 1.0f);
    const float nan = std::nanf("");
    const float inf_f = std::numeric_limits<float>::infinity();
    pts.push_back(Point{nan, 0.0f, 0.0f});
    pts.push_back(Point{0.0f, inf_f, 0.0f});
    pts.push_back(Point{0.0f, 0.0f, -inf_f});
    VectorReader reader{&pts};
    InflationStats stats;
    inf.inflate(reader, &stats);
    CHECK(stats.ok, "non-finite points broke the inflater");
    CHECK(stats.filtered_points == 100, "kept %zu points, expected 100",
          stats.filtered_points);

    std::set<Voxel> seeds;
    referenceSeeds(p, pts, &seeds);
    compare("non-finite", p, seeds, inf);
  }

  /* --- a point beyond the lattice is refused, not wrapped into the map --- */
  {
    InflationParams p;
    PointCloudInflater inf;
    std::string error;
    CHECK(inf.configure(p, &error), "configure failed: %s", error.c_str());

    std::vector<Point> pts = {Point{0.0f, 0.0f, 0.0f}, Point{5.0e6f, 0.0f, 0.0f}};
    VectorReader reader{&pts};
    InflationStats stats;
    inf.inflate(reader, &stats);
    CHECK(!stats.ok, "a point off the lattice was accepted");
    CHECK(stats.error == occupancy_inflation::InflationError::kOutOfRange,
          "wrong error for an out-of-range point");
  }

  /* --- the grid guard trips instead of allocating without limit --- */
  {
    InflationParams p;
    p.max_grid_voxels = 4096;
    PointCloudInflater inf;
    std::string error;
    CHECK(inf.configure(p, &error), "configure failed: %s", error.c_str());

    std::vector<Point> pts = {Point{0.0f, 0.0f, 0.0f}, Point{900.0f, 0.0f, 0.0f}};
    VectorReader reader{&pts};
    InflationStats stats;
    inf.inflate(reader, &stats);
    CHECK(!stats.ok, "an oversized grid was allowed through");
    CHECK(stats.error == occupancy_inflation::InflationError::kGridTooLarge,
          "wrong error for an oversized grid");
  }

  /* --- the GPU and CPU dilations agree bit for bit --- *
   * The matrix above checks each backend against the brute-force reference,
   * which is the stronger check but only over grids small enough to enumerate.
   * This one runs the two backends over a map the size of a real one and
   * compares the serialised output byte for byte: the passes are integer bit
   * arithmetic in the same order, so anything short of exact equality is a
   * bug, not a tolerance. */
  if (!have_gpu) {
    std::printf("gpu/cpu equivalence: skipped, no CUDA device\n");
  } else {
    for (InflationShape shape : shapes) {
      InflationParams base;
      base.resolution = 0.1;
      base.radius_xy = 0.45;
      base.radius_z = 0.25;
      base.shape = shape;
      base.num_threads = 4;
      base.origin = Eigen::Vector3d(0.017, -0.041, 0.0);

      /* One-shot over a cloud big enough to clear the size threshold on its
       * own, so the default configuration is what gets exercised. */
      for (int one_shot = 0; one_shot < 2; ++one_shot) {
        InflationParams cpu = base;
        cpu.incremental = one_shot == 0;
        cpu.use_cuda = false;
        InflationParams gpu = cpu;
        gpu.use_cuda = true;

        PointCloudInflater inf_cpu, inf_gpu;
        std::string error;
        CHECK(inf_cpu.configure(cpu, &error), "configure failed: %s", error.c_str());
        CHECK(inf_gpu.configure(gpu, &error), "configure failed: %s", error.c_str());
        CHECK(inf_gpu.cudaEnabled(), "cudaEnabled() false with a device present");
        CHECK(!inf_cpu.cudaEnabled(), "cudaEnabled() true with use_cuda off");

        float span = 5.0f;
        for (int msg = 0; msg < 3; ++msg) {
          span += 1.5f;
          const std::vector<Point> pts = randomPoints(rng, 40000, span);
          VectorReader reader{&pts};
          InflationStats sc, sg;
          inf_cpu.inflate(reader, &sc);
          inf_gpu.inflate(reader, &sg);

          char label[96];
          std::snprintf(label, sizeof(label), "shape=%d incremental=%d msg=%d",
                        static_cast<int>(shape), one_shot == 0, msg);
          CHECK(sc.ok && sg.ok, "%s: inflate reported not ok", label);
          CHECK(!sc.gpu_dilation, "%s: the CPU run used the GPU", label);
          CHECK(sg.gpu_dilation, "%s: the GPU run fell back to the CPU", label);
          CHECK(sg.inflated_voxels == sc.inflated_voxels,
                "%s: gpu %zu voxels, cpu %zu", label, sg.inflated_voxels,
                sc.inflated_voxels);
          CHECK(rawOutput(inf_gpu) == rawOutput(inf_cpu),
                "%s: the two backends produced different maps", label);
        }
      }
    }

    /* A grid too small to pay for the transfer stays on the CPU even with the
     * GPU enabled, which is the whole point of the threshold. */
    InflationParams p;
    p.incremental = false;
    p.use_cuda = true;
    PointCloudInflater inf;
    std::string error;
    CHECK(inf.configure(p, &error), "configure failed: %s", error.c_str());
    const std::vector<Point> pts = randomPoints(rng, 500, 1.0f);
    VectorReader reader{&pts};
    InflationStats stats;
    inf.inflate(reader, &stats);
    CHECK(stats.ok, "small-grid inflate reported not ok");
    CHECK(!stats.gpu_dilation,
          "a grid under cuda_min_grid_voxels was sent to the GPU");
    std::printf("gpu/cpu equivalence: checked\n");
  }

  if (g_failures == 0) {
    std::printf("all inflation checks passed\n");
    return 0;
  }
  std::printf("%d check(s) failed\n", g_failures);
  return 1;
}
