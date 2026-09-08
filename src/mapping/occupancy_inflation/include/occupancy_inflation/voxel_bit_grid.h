/**
 * @file voxel_bit_grid.h
 * @brief A dense bitset over a window of the voxel lattice, and the word
 *        parallel morphology that runs on it.
 *
 * The inflation used to live in a hash set: one insert per (seed, kernel cell)
 * pair, so a 200k voxel map with a 5x5x5 kernel cost 25M hashed inserts. Here
 * one voxel is one bit, 64 voxels share a word, and dilation becomes shifted
 * ORs over those words. A whole pass over a 1.5M voxel map touches 24k words.
 *
 * Layout: x is the packed axis, then y, then z. Row (y, z) is a run of
 * wordsPerRow() words, so a shift along x is a bit shift inside a row while a
 * shift along y or z is just a different row pointer.
 *
 * This header is deliberately free of ROS, PCL and message types so the
 * morphology can be unit tested on its own.
 */

#ifndef OCCUPANCY_INFLATION_VOXEL_BIT_GRID_H_
#define OCCUPANCY_INFLATION_VOXEL_BIT_GRID_H_

#include <Eigen/Core>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

namespace occupancy_inflation {

/** Voxels per word, and therefore the alignment every window keeps in x. */
static constexpr int kWordBits = 64;

/** Floor division, used because voxel indices go negative. */
inline int floorDiv(int a, int b) {
  const int q = a / b;
  return (a % b != 0 && ((a < 0) != (b < 0))) ? q - 1 : q;
}

/**
 * @brief An axis-aligned block of the voxel lattice, in global voxel indices.
 *
 * base is the global index of local voxel (0,0,0). Windows produced by
 * snapWindow() always have a base.x and a dims.x that are multiples of
 * kWordBits, which is what makes every grid-to-grid copy word aligned: two
 * windows can only ever differ by a whole number of words along x.
 */
struct VoxelWindow {
  Eigen::Vector3i base = Eigen::Vector3i::Zero();
  Eigen::Vector3i dims = Eigen::Vector3i::Zero();

  bool empty() const { return dims.x() <= 0 || dims.y() <= 0 || dims.z() <= 0; }

  std::size_t wordsPerRow() const {
    return (static_cast<std::size_t>(dims.x()) + kWordBits - 1) / kWordBits;
  }
  std::size_t wordsPerPlane() const {
    return wordsPerRow() * static_cast<std::size_t>(dims.y());
  }
  std::size_t wordCount() const {
    return empty() ? 0 : wordsPerPlane() * static_cast<std::size_t>(dims.z());
  }
  /** Offset of the first word of row (y, z), in local coordinates. */
  std::size_t rowOffset(int y, int z) const {
    return (static_cast<std::size_t>(z) * static_cast<std::size_t>(dims.y()) +
            static_cast<std::size_t>(y)) *
           wordsPerRow();
  }

  Eigen::Vector3i maxIndex() const { return base + dims - Eigen::Vector3i::Ones(); }

  bool containsGlobal(const Eigen::Vector3i &g) const {
    return (g.array() >= base.array()).all() &&
           (g.array() < (base + dims).array()).all();
  }
};

/**
 * @brief Smallest word-aligned window covering the inclusive index box.
 * @param pad extra voxels on every side, e.g. the inflation reach
 */
VoxelWindow snapWindow(const Eigen::Vector3i &lo, const Eigen::Vector3i &hi,
                       int pad = 0);

/** @brief Smallest word-aligned window covering both inputs. */
VoxelWindow unionWindow(const VoxelWindow &a, const VoxelWindow &b);

/** @brief Overlap of two windows, or an empty window. Not snapped. */
VoxelWindow intersectWindow(const VoxelWindow &a, const VoxelWindow &b);

// ---------------------------------------------------------------------------
// Word-level primitives. All of these assume the caller has already sized the
// buffers; they are the inner loops, so they do no checking of their own.
// ---------------------------------------------------------------------------

/** @brief dst |= src moved so bit x lands on bit x+shift. Bits leaving drop. */
void orShiftedX(uint64_t *dst, const uint64_t *src, std::size_t words, int shift);

/** @brief dst |= src translated by (0, dy, dz). */
void orShiftedYZ(uint64_t *dst, const uint64_t *src, const VoxelWindow &win,
                 int dy, int dz, int threads);

/**
 * @brief Grow every row from radius @p from_r to radius @p to_r along x.
 *
 * Works in place with a per-row scratch, doubling the radius each step
 * (r -> 2r+1), so a radius of R costs ceil(log2(R+1)) passes instead of R.
 */
void dilateXInPlace(uint64_t *bits, const VoxelWindow &win, int from_r, int to_r,
                    int threads);

/**
 * @brief dst &= src moved so bit x lands on bit x+shift. Bits entering are 0.
 *
 * The AND counterpart of orShiftedX(). Zeros shift in rather than being
 * dropped, which is exactly the semantics erosion needs: a voxel off the end of
 * a row is outside the map, i.e. free, so it must clear the bit it lands on.
 */
void andShiftedX(uint64_t *dst, const uint64_t *src, std::size_t words,
                 int shift);

/**
 * @brief dst &= src translated by (0, dy, dz), with rows from outside cleared.
 */
void andShiftedYZ(uint64_t *dst, const uint64_t *src, const VoxelWindow &win,
                  int dy, int dz, int threads);

/**
 * @brief Erode every row from radius @p from_r to radius @p to_r along x.
 *
 * The mirror of dilateXInPlace(), doubling the same way: eroding by a and then
 * by c erodes by a + c, because the interval [-a,a] dilated by [-c,c] is
 * [-(a+c), a+c].
 */
void erodeXInPlace(uint64_t *bits, const VoxelWindow &win, int from_r, int to_r,
                   int threads);

/**
 * @brief Erode along y (axis 1) or z (axis 2) by @p radius. See the dilate
 *        counterpart for why this ping-pongs between the two buffers.
 */
uint64_t *erodeAxisPingPong(uint64_t *a, uint64_t *b, const VoxelWindow &win,
                            int axis, int radius, int threads);

/**
 * @brief dst = src & ~mask over @p words words; returns the bits left set.
 *
 * With @p mask an erosion of @p src this is the boundary shell: the voxels of
 * src that have at least one neighbour the erosion's element could not cover.
 */
std::size_t andNotInto(uint64_t *dst, const uint64_t *src, const uint64_t *mask,
                       std::size_t words, int threads);

/**
 * @brief One (dy, dz) column of a structuring element and its x half-extent.
 *
 * A non-separable element (the ellipsoid) is a stack of these: at a fixed
 * (dy, dz) the admissible dx form one symmetric interval. Both the CPU
 * dilation and the CUDA one consume the element in this form, which is why it
 * sits here beside the primitives rather than inside the inflater.
 */
struct KernelRun {
  int dy;
  int dz;
  int rx;
};

/**
 * @brief Dilate along y (axis 1) or z (axis 2) by @p radius, doubling as above.
 *
 * Ping-pongs between @p a and @p b because a row's new value depends on rows
 * that later rows still need to read. Returns whichever buffer holds the
 * result; the other is left as scratch.
 */
uint64_t *dilateAxisPingPong(uint64_t *a, uint64_t *b, const VoxelWindow &win,
                             int axis, int radius, int threads);

/** @brief Number of set bits. */
std::size_t popCount(const uint64_t *bits, std::size_t words);

// ---------------------------------------------------------------------------

/**
 * @brief OR the part of @p src that falls inside @p clip into @p dst.
 *
 * @p dst_win and @p src_win must be word aligned against each other, which
 * snapWindow() guarantees; @p clip need not be, so the partial words at its x
 * edges are masked. This is how a dilated block is merged into the map and
 * clipped to output_bounds in a single pass.
 */
std::size_t orWindowed(uint64_t *dst, const VoxelWindow &dst_win,
                       const uint64_t *src, const VoxelWindow &src_win,
                       const VoxelWindow &clip, int threads);

/**
 * @brief A bitset plus the window it covers, able to grow without losing bits.
 */
class VoxelBitGrid {
 public:
  /** @brief Resize to @p win and clear. Returns false if it would exceed @p max_words. */
  bool reset(const VoxelWindow &win, std::size_t max_words);

  void clearBits();
  void release();

  const VoxelWindow &window() const { return win_; }
  bool empty() const { return win_.empty(); }
  std::size_t wordCount() const { return bits_.size(); }
  uint64_t *data() { return bits_.data(); }
  const uint64_t *data() const { return bits_.data(); }

  /**
   * @brief Expand so @p want is covered, keeping the bits already set.
   *
   * Growth is padded so a steadily expanding map reallocates a handful of
   * times rather than on every message.
   */
  bool growToInclude(const VoxelWindow &want, std::size_t max_words);

  /** @brief Set a voxel by local coordinates. The caller guarantees the range. */
  void setLocal(int x, int y, int z) {
    bits_[win_.rowOffset(y, z) + static_cast<std::size_t>(x) / kWordBits] |=
        uint64_t(1) << (static_cast<unsigned>(x) % kWordBits);
  }

  bool testGlobal(const Eigen::Vector3i &g) const;

  std::size_t count() const { return popCount(bits_.data(), bits_.size()); }

  /** @brief orWindowed() into this grid; see the free function above. */
  std::size_t orFromRaw(const uint64_t *src, const VoxelWindow &src_win,
                        const VoxelWindow &clip, int threads) {
    return orWindowed(bits_.data(), win_, src, src_win, clip, threads);
  }

  std::size_t orFrom(const VoxelBitGrid &src, int threads) {
    return orFromRaw(src.data(), src.window(), src.window(), threads);
  }

  /**
   * @brief Set bits, and the smallest window holding them, in one scan.
   *
   * Dilating a block costs whatever the block covers, so the difference between
   * the window a grid was allocated with and the window its bits actually
   * occupy is the difference between inflating the whole map and inflating only
   * what just changed.
   */
  std::size_t occupiedExtent(VoxelWindow *tight) const;

  /** @brief this &= ~other, i.e. keep only the voxels @p other does not hold. */
  void subtract(const VoxelBitGrid &other, int threads);

  /** @brief True if every voxel of this grid is also set in @p other. */
  bool isSubsetOf(const VoxelBitGrid &other) const;

  void swap(VoxelBitGrid &other) {
    std::swap(win_, other.win_);
    bits_.swap(other.bits_);
  }

 private:
  VoxelWindow win_;
  std::vector<uint64_t> bits_;
};

}  // namespace occupancy_inflation

#endif  // OCCUPANCY_INFLATION_VOXEL_BIT_GRID_H_
