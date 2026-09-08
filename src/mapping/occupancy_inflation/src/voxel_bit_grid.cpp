/**
 * @file voxel_bit_grid.cpp
 * @brief Word-parallel morphology on the voxel bitset.
 */

#include "occupancy_inflation/voxel_bit_grid.h"

#include <algorithm>
#include <cstdlib>
#include <cstring>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace occupancy_inflation {

namespace {

/* Below this much work a parallel region costs more than it saves. */
constexpr std::size_t kParallelWordThreshold = 1u << 14;

/* Extra voxels added on a side that has just grown, so a map that expands a
 * little on every message does not reallocate on every message. */
constexpr int kGrowPadXY = 32;
constexpr int kGrowPadZ = 8;

/** Smallest multiple of @p g that is >= v, for possibly negative v. */
inline int ceilTo(int v, int g) { return floorDiv(v + g - 1, g) * g; }
inline int floorTo(int v, int g) { return floorDiv(v, g) * g; }

__attribute__((unused)) inline bool useThreads(int threads, std::size_t work) {
  return threads > 1 && work >= kParallelWordThreshold;
}

}  // namespace

VoxelWindow snapWindow(const Eigen::Vector3i &lo, const Eigen::Vector3i &hi,
                       int pad) {
  VoxelWindow win;
  const Eigen::Vector3i l = lo.array() - pad;
  const Eigen::Vector3i h = hi.array() + pad;
  if ((l.array() > h.array()).any()) {
    return win;  // empty
  }

  /* x is the packed axis, so it is aligned to whole words; y and z only get a
   * coarse alignment to keep regrowth from happening one voxel at a time. */
  win.base.x() = floorTo(l.x(), kWordBits);
  win.base.y() = floorTo(l.y(), 8);
  win.base.z() = floorTo(l.z(), 8);
  win.dims.x() = ceilTo(h.x() + 1, kWordBits) - win.base.x();
  win.dims.y() = ceilTo(h.y() + 1, 8) - win.base.y();
  win.dims.z() = ceilTo(h.z() + 1, 8) - win.base.z();
  return win;
}

VoxelWindow unionWindow(const VoxelWindow &a, const VoxelWindow &b) {
  if (a.empty()) return b;
  if (b.empty()) return a;
  return snapWindow(a.base.cwiseMin(b.base), a.maxIndex().cwiseMax(b.maxIndex()));
}

VoxelWindow intersectWindow(const VoxelWindow &a, const VoxelWindow &b) {
  VoxelWindow out;
  if (a.empty() || b.empty()) return out;
  const Eigen::Vector3i lo = a.base.cwiseMax(b.base);
  const Eigen::Vector3i hi = a.maxIndex().cwiseMin(b.maxIndex());
  if ((lo.array() > hi.array()).any()) return out;
  out.base = lo;
  out.dims = hi - lo + Eigen::Vector3i::Ones();
  return out;
}

// ---------------------------------------------------------------------------

void orShiftedX(uint64_t *dst, const uint64_t *src, std::size_t words,
                int shift) {
  if (shift == 0) {
    for (std::size_t i = 0; i < words; ++i) dst[i] |= src[i];
    return;
  }

  const std::size_t whole = static_cast<std::size_t>(std::abs(shift)) / kWordBits;
  const unsigned bit = static_cast<unsigned>(std::abs(shift)) % kWordBits;
  if (whole >= words) {
    return;  // everything shifts clear off the row
  }

  if (shift > 0) {
    /* Bit x moves to x+shift, i.e. towards higher words. */
    for (std::size_t i = words; i-- > whole;) {
      uint64_t v = src[i - whole] << bit;
      if (bit != 0 && i > whole) {
        v |= src[i - whole - 1] >> (kWordBits - bit);
      }
      dst[i] |= v;
    }
  } else {
    for (std::size_t i = 0; i + whole < words; ++i) {
      uint64_t v = src[i + whole] >> bit;
      if (bit != 0 && i + whole + 1 < words) {
        v |= src[i + whole + 1] << (kWordBits - bit);
      }
      dst[i] |= v;
    }
  }
}

void orShiftedYZ(uint64_t *dst, const uint64_t *src, const VoxelWindow &win,
                 int dy, int dz, int threads) {
  const int ny = win.dims.y();
  const int nz = win.dims.z();
  const std::size_t nw = win.wordsPerRow();

  const int z_begin = std::max(0, dz);
  const int z_end = std::min(nz, nz + dz);
  const int y_begin = std::max(0, dy);
  const int y_end = std::min(ny, ny + dy);
  if (z_begin >= z_end || y_begin >= y_end) return;

#ifdef _OPENMP
#pragma omp parallel for schedule(static) num_threads(threads) \
    if (useThreads(threads, win.wordCount()))
#endif
  for (int z = z_begin; z < z_end; ++z) {
    for (int y = y_begin; y < y_end; ++y) {
      uint64_t *d = dst + win.rowOffset(y, z);
      const uint64_t *s = src + win.rowOffset(y - dy, z - dz);
      for (std::size_t w = 0; w < nw; ++w) d[w] |= s[w];
    }
  }
}

void andShiftedX(uint64_t *dst, const uint64_t *src, std::size_t words,
                 int shift) {
  if (shift == 0) {
    for (std::size_t i = 0; i < words; ++i) dst[i] &= src[i];
    return;
  }

  const std::size_t whole = static_cast<std::size_t>(std::abs(shift)) / kWordBits;
  const unsigned bit = static_cast<unsigned>(std::abs(shift)) % kWordBits;
  if (whole >= words) {
    /* The whole row shifted in from outside, which is free space. */
    std::memset(dst, 0, words * sizeof(uint64_t));
    return;
  }

  if (shift > 0) {
    for (std::size_t i = words; i-- > whole;) {
      uint64_t v = src[i - whole] << bit;
      if (bit != 0 && i > whole) {
        v |= src[i - whole - 1] >> (kWordBits - bit);
      }
      dst[i] &= v;
    }
    /* Words no source word reached see only the zeros that shifted in. */
    for (std::size_t i = 0; i < whole; ++i) dst[i] = 0;
  } else {
    for (std::size_t i = 0; i + whole < words; ++i) {
      uint64_t v = src[i + whole] >> bit;
      if (bit != 0 && i + whole + 1 < words) {
        v |= src[i + whole + 1] << (kWordBits - bit);
      }
      dst[i] &= v;
    }
    for (std::size_t i = words - whole; i < words; ++i) dst[i] = 0;
  }
}

void andShiftedYZ(uint64_t *dst, const uint64_t *src, const VoxelWindow &win,
                  int dy, int dz, int threads) {
  const int ny = win.dims.y();
  const int nz = win.dims.z();
  const std::size_t nw = win.wordsPerRow();

#ifdef _OPENMP
#pragma omp parallel for schedule(static) num_threads(threads) \
    if (useThreads(threads, win.wordCount()))
#endif
  for (int z = 0; z < nz; ++z) {
    for (int y = 0; y < ny; ++y) {
      uint64_t *d = dst + win.rowOffset(y, z);
      const int sy = y - dy;
      const int sz = z - dz;
      if (sy < 0 || sy >= ny || sz < 0 || sz >= nz) {
        /* Reading from outside the window, which holds no voxels. */
        std::memset(d, 0, nw * sizeof(uint64_t));
        continue;
      }
      const uint64_t *s = src + win.rowOffset(sy, sz);
      for (std::size_t w = 0; w < nw; ++w) d[w] &= s[w];
    }
  }
}

void erodeXInPlace(uint64_t *bits, const VoxelWindow &win, int from_r, int to_r,
                   int threads) {
  if (to_r <= from_r) return;

  const std::size_t nw = win.wordsPerRow();
  const long rows = static_cast<long>(win.dims.y()) * win.dims.z();

#ifdef _OPENMP
#pragma omp parallel num_threads(threads) if (useThreads(threads, win.wordCount()))
#endif
  {
    std::vector<uint64_t> scratch(nw);
#ifdef _OPENMP
#pragma omp for schedule(static)
#endif
    for (long row = 0; row < rows; ++row) {
      uint64_t *r = bits + static_cast<std::size_t>(row) * nw;
      int a = from_r;
      while (a < to_r) {
        const int c = std::min(2 * a + 1, to_r - a);
        std::memcpy(scratch.data(), r, nw * sizeof(uint64_t));
        andShiftedX(r, scratch.data(), nw, c);
        andShiftedX(r, scratch.data(), nw, -c);
        a += c;
      }
    }
  }
}

uint64_t *erodeAxisPingPong(uint64_t *a, uint64_t *b, const VoxelWindow &win,
                            int axis, int radius, int threads) {
  if (radius <= 0) return a;

  const std::size_t n = win.wordCount();
  int achieved = 0;
  while (achieved < radius) {
    const int c = std::min(2 * achieved + 1, radius - achieved);
    std::memcpy(b, a, n * sizeof(uint64_t));
    const int dy = (axis == 1) ? c : 0;
    const int dz = (axis == 2) ? c : 0;
    andShiftedYZ(b, a, win, dy, dz, threads);
    andShiftedYZ(b, a, win, -dy, -dz, threads);
    std::swap(a, b);
    achieved += c;
  }
  return a;
}

std::size_t andNotInto(uint64_t *dst, const uint64_t *src, const uint64_t *mask,
                       std::size_t words, int threads) {
  std::size_t set = 0;
#ifdef _OPENMP
#pragma omp parallel for schedule(static) num_threads(threads) reduction(+ : set) \
    if (useThreads(threads, words))
#endif
  for (long i = 0; i < static_cast<long>(words); ++i) {
    const uint64_t v = src[i] & ~mask[i];
    dst[i] = v;
    set += static_cast<std::size_t>(__builtin_popcountll(v));
  }
  return set;
}

void dilateXInPlace(uint64_t *bits, const VoxelWindow &win, int from_r,
                    int to_r, int threads) {
  if (to_r <= from_r) return;

  const std::size_t nw = win.wordsPerRow();
  const long rows = static_cast<long>(win.dims.y()) * win.dims.z();

#ifdef _OPENMP
#pragma omp parallel num_threads(threads) if (useThreads(threads, win.wordCount()))
#endif
  {
    std::vector<uint64_t> scratch(nw);
#ifdef _OPENMP
#pragma omp for schedule(static)
#endif
    for (long row = 0; row < rows; ++row) {
      uint64_t *r = bits + static_cast<std::size_t>(row) * nw;
      /* Doubling: a set already grown by `a` grows to `a + c` for any
       * c <= 2a+1, so the radius roughly doubles per pass. */
      int a = from_r;
      while (a < to_r) {
        const int c = std::min(2 * a + 1, to_r - a);
        std::memcpy(scratch.data(), r, nw * sizeof(uint64_t));
        orShiftedX(r, scratch.data(), nw, c);
        orShiftedX(r, scratch.data(), nw, -c);
        a += c;
      }
    }
  }
}

uint64_t *dilateAxisPingPong(uint64_t *a, uint64_t *b, const VoxelWindow &win,
                             int axis, int radius, int threads) {
  if (radius <= 0) return a;

  const std::size_t n = win.wordCount();
  int achieved = 0;
  while (achieved < radius) {
    const int c = std::min(2 * achieved + 1, radius - achieved);
    std::memcpy(b, a, n * sizeof(uint64_t));
    const int dy = (axis == 1) ? c : 0;
    const int dz = (axis == 2) ? c : 0;
    orShiftedYZ(b, a, win, dy, dz, threads);
    orShiftedYZ(b, a, win, -dy, -dz, threads);
    std::swap(a, b);
    achieved += c;
  }
  return a;
}

std::size_t popCount(const uint64_t *bits, std::size_t words) {
  std::size_t n = 0;
  for (std::size_t i = 0; i < words; ++i) {
    n += static_cast<std::size_t>(__builtin_popcountll(bits[i]));
  }
  return n;
}

std::size_t orWindowed(uint64_t *dst, const VoxelWindow &dst_win,
                       const uint64_t *src, const VoxelWindow &src_win,
                       const VoxelWindow &clip, int threads) {
  const VoxelWindow c = intersectWindow(intersectWindow(src_win, clip), dst_win);
  if (c.empty()) return 0;

  /* src_win and dst_win are both word aligned in x, so they differ by a whole
   * number of words and only the clip needs partial-word masks. */
  const int lo = c.base.x() - src_win.base.x();
  const int hi = lo + c.dims.x() - 1;
  const std::size_t w0 = static_cast<std::size_t>(lo) / kWordBits;
  const std::size_t w1 = static_cast<std::size_t>(hi) / kWordBits;
  const uint64_t mask0 = ~uint64_t(0) << (static_cast<unsigned>(lo) % kWordBits);
  const unsigned hi_bit = static_cast<unsigned>(hi) % kWordBits;
  const uint64_t mask1 = hi_bit == kWordBits - 1
                             ? ~uint64_t(0)
                             : ((uint64_t(1) << (hi_bit + 1)) - 1);
  const long x_word = (src_win.base.x() - dst_win.base.x()) / kWordBits;

  const int nz = c.dims.z();
  const int ny = c.dims.y();
  std::size_t added = 0;

#ifdef _OPENMP
#pragma omp parallel for schedule(static) num_threads(threads) reduction(+ : added) \
    if (useThreads(threads, static_cast<std::size_t>(nz) * ny * (w1 - w0 + 1)))
#endif
  for (int z = 0; z < nz; ++z) {
    const int gz = c.base.z() + z;
    for (int y = 0; y < ny; ++y) {
      const int gy = c.base.y() + y;
      const uint64_t *s =
          src + src_win.rowOffset(gy - src_win.base.y(), gz - src_win.base.z());
      uint64_t *d =
          dst + dst_win.rowOffset(gy - dst_win.base.y(), gz - dst_win.base.z());
      for (std::size_t w = w0; w <= w1; ++w) {
        uint64_t v = s[w];
        if (w == w0) v &= mask0;
        if (w == w1) v &= mask1;
        uint64_t *t = d + static_cast<std::size_t>(static_cast<long>(w) + x_word);
        /* Counting the bits this merge actually turns on is what lets the
         * caller keep a running occupancy total instead of re-counting the
         * whole map after every message. */
        added += static_cast<std::size_t>(__builtin_popcountll(v & ~*t));
        *t |= v;
      }
    }
  }
  return added;
}

// ---------------------------------------------------------------------------

bool VoxelBitGrid::reset(const VoxelWindow &win, std::size_t max_words) {
  if (win.wordCount() > max_words) return false;
  win_ = win;
  bits_.assign(win.wordCount(), 0);
  return true;
}

void VoxelBitGrid::clearBits() { std::fill(bits_.begin(), bits_.end(), 0); }

void VoxelBitGrid::release() {
  win_ = VoxelWindow();
  std::vector<uint64_t>().swap(bits_);
}

bool VoxelBitGrid::growToInclude(const VoxelWindow &want,
                                 std::size_t max_words) {
  if (want.empty()) return true;
  if (win_.empty()) {
    return reset(snapWindow(want.base, want.maxIndex()), max_words);
  }
  if ((want.base.array() >= win_.base.array()).all() &&
      (want.maxIndex().array() <= win_.maxIndex().array()).all()) {
    return true;
  }

  const VoxelWindow tight = unionWindow(win_, snapWindow(want.base, want.maxIndex()));

  /* Pad only the sides that moved, so the next few messages fit in place. */
  Eigen::Vector3i lo = tight.base;
  Eigen::Vector3i hi = tight.maxIndex();
  const Eigen::Vector3i pad(kGrowPadXY, kGrowPadXY, kGrowPadZ);
  for (int i = 0; i < 3; ++i) {
    if (tight.base(i) < win_.base(i)) lo(i) -= pad(i);
    if (tight.maxIndex()(i) > win_.maxIndex()(i)) hi(i) += pad(i);
  }

  VoxelWindow next = snapWindow(lo, hi);
  if (next.wordCount() > max_words) {
    next = tight;  // drop the padding before giving up
    if (next.wordCount() > max_words) return false;
  }

  std::vector<uint64_t> moved(next.wordCount(), 0);
  const std::size_t src_nw = win_.wordsPerRow();
  const std::size_t x_word = static_cast<std::size_t>(
      (win_.base.x() - next.base.x()) / kWordBits);

  for (int z = 0; z < win_.dims.z(); ++z) {
    const int dz = win_.base.z() + z - next.base.z();
    for (int y = 0; y < win_.dims.y(); ++y) {
      const int dy = win_.base.y() + y - next.base.y();
      std::memcpy(moved.data() + next.rowOffset(dy, dz) + x_word,
                  bits_.data() + win_.rowOffset(y, z),
                  src_nw * sizeof(uint64_t));
    }
  }

  bits_.swap(moved);
  win_ = next;
  return true;
}

bool VoxelBitGrid::testGlobal(const Eigen::Vector3i &g) const {
  if (!win_.containsGlobal(g)) return false;
  const int x = g.x() - win_.base.x();
  const std::size_t w =
      win_.rowOffset(g.y() - win_.base.y(), g.z() - win_.base.z()) +
      static_cast<std::size_t>(x) / kWordBits;
  return (bits_[w] >> (static_cast<unsigned>(x) % kWordBits)) & 1u;
}

void VoxelBitGrid::subtract(const VoxelBitGrid &other, int threads) {
  const VoxelWindow c = intersectWindow(win_, other.win_);
  if (c.empty()) return;

  /* Both windows are word aligned in x, so the overlap is too: no masks. */
  const std::size_t w_src =
      static_cast<std::size_t>(c.base.x() - other.win_.base.x()) / kWordBits;
  const std::size_t w_dst =
      static_cast<std::size_t>(c.base.x() - win_.base.x()) / kWordBits;
  const std::size_t nw = static_cast<std::size_t>(c.dims.x()) / kWordBits;

  const int nz = c.dims.z();
  const int ny = c.dims.y();

#ifdef _OPENMP
#pragma omp parallel for schedule(static) num_threads(threads) \
    if (useThreads(threads, static_cast<std::size_t>(nz) * ny * nw))
#endif
  for (int z = 0; z < nz; ++z) {
    const int gz = c.base.z() + z;
    for (int y = 0; y < ny; ++y) {
      const int gy = c.base.y() + y;
      const uint64_t *s = other.data() +
                          other.win_.rowOffset(gy - other.win_.base.y(),
                                               gz - other.win_.base.z()) +
                          w_src;
      uint64_t *d =
          data() + win_.rowOffset(gy - win_.base.y(), gz - win_.base.z()) + w_dst;
      for (std::size_t w = 0; w < nw; ++w) d[w] &= ~s[w];
    }
  }
}

std::size_t VoxelBitGrid::occupiedExtent(VoxelWindow *tight) const {
  *tight = VoxelWindow();
  if (win_.empty()) return 0;

  const std::size_t nw = win_.wordsPerRow();
  int lo_x = win_.dims.x(), hi_x = -1;
  int lo_y = win_.dims.y(), hi_y = -1;
  int lo_z = win_.dims.z(), hi_z = -1;
  std::size_t count = 0;

  for (int z = 0; z < win_.dims.z(); ++z) {
    for (int y = 0; y < win_.dims.y(); ++y) {
      const uint64_t *row = bits_.data() + win_.rowOffset(y, z);
      for (std::size_t w = 0; w < nw; ++w) {
        const uint64_t v = row[w];
        if (v == 0) continue;
        count += static_cast<std::size_t>(__builtin_popcountll(v));
        const int first =
            static_cast<int>(w * kWordBits) + __builtin_ctzll(v);
        const int last = static_cast<int>(w * kWordBits) + kWordBits - 1 -
                         __builtin_clzll(v);
        if (first < lo_x) lo_x = first;
        if (last > hi_x) hi_x = last;
        if (y < lo_y) lo_y = y;
        if (y > hi_y) hi_y = y;
        if (z < lo_z) lo_z = z;
        if (z > hi_z) hi_z = z;
      }
    }
  }
  if (count == 0) return 0;

  tight->base = win_.base + Eigen::Vector3i(lo_x, lo_y, lo_z);
  tight->dims = Eigen::Vector3i(hi_x - lo_x + 1, hi_y - lo_y + 1, hi_z - lo_z + 1);
  return count;
}

bool VoxelBitGrid::isSubsetOf(const VoxelBitGrid &other) const {
  if (win_.empty()) return true;

  const VoxelWindow c = intersectWindow(win_, other.win_);
  const std::size_t nw = win_.wordsPerRow();
  const std::size_t w_lo =
      c.empty() ? 0
                : static_cast<std::size_t>(c.base.x() - win_.base.x()) / kWordBits;
  const std::size_t w_hi =
      c.empty() ? 0 : w_lo + static_cast<std::size_t>(c.dims.x()) / kWordBits;

  for (int z = 0; z < win_.dims.z(); ++z) {
    const int gz = win_.base.z() + z;
    for (int y = 0; y < win_.dims.y(); ++y) {
      const int gy = win_.base.y() + y;
      const uint64_t *d = data() + win_.rowOffset(y, z);

      const bool row_overlaps =
          !c.empty() && gz >= c.base.z() && gz <= c.maxIndex().z() &&
          gy >= c.base.y() && gy <= c.maxIndex().y();
      if (!row_overlaps) {
        for (std::size_t w = 0; w < nw; ++w) {
          if (d[w] != 0) return false;
        }
        continue;
      }

      const uint64_t *s = other.data() +
                          other.win_.rowOffset(gy - other.win_.base.y(),
                                               gz - other.win_.base.z()) +
                          static_cast<std::size_t>(c.base.x() -
                                                   other.win_.base.x()) /
                              kWordBits;
      for (std::size_t w = 0; w < nw; ++w) {
        if (w < w_lo || w >= w_hi) {
          if (d[w] != 0) return false;
        } else if ((d[w] & ~s[w - w_lo]) != 0) {
          return false;
        }
      }
    }
  }
  return true;
}

}  // namespace occupancy_inflation
