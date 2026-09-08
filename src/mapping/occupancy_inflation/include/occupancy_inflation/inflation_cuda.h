/**
 * @file inflation_cuda.h
 * @brief Optional CUDA backend for the dilation, and the CPU-only stubs.
 *
 * The dilation is the half of inflate() that does not depend on the input
 * cloud: a window of the bit grid goes in, the same window grown by the
 * structuring element comes out. That makes it the piece worth handing to a
 * GPU, and it is the only piece this header covers -- the voxelisation, the
 * merges into the persistent grids and the serialisation all stay on the CPU.
 *
 * Everything here is integer bit arithmetic, the same shifted ORs the CPU
 * path performs in the same order, so the two backends do not merely agree to
 * within a tolerance: they produce bit-identical grids. That is what lets the
 * fallback be silent.
 *
 * In a build without a CUDA compiler the declarations below become inline
 * stubs that decline every request, so callers need no #ifdef of their own.
 */

#ifndef OCCUPANCY_INFLATION_INFLATION_CUDA_H_
#define OCCUPANCY_INFLATION_INFLATION_CUDA_H_

#include <cstddef>
#include <cstdint>

#include "occupancy_inflation/voxel_bit_grid.h"

namespace occupancy_inflation {
namespace cuda {

/** Outcome of a dilation request. */
enum class Status {
  kOk,        //!< the grid was dilated on the device and copied back
  kDeclined,  //!< this piece of work was refused; run it on the CPU and ask again next time
  kFailed,    //!< the device errored; the caller should stop asking
};

#ifdef OCCUPANCY_INFLATION_WITH_CUDA

/** @brief Whether this build was compiled with a CUDA compiler. */
inline bool built() { return true; }

/** @brief Whether a usable device answered. Probed once, then cached. */
bool available();

/**
 * @brief Create the device context now.
 *
 * Doing it lazily would put a hundred milliseconds or more of driver setup
 * inside whichever message happened to be first, which on a node running at
 * 20 Hz is a dropped frame for no reason. A no-op without a device.
 */
void warmUp();

/**
 * @brief Dilate @p bits in place by a box of the given half-extents.
 *
 * @param bits host buffer of win.wordCount() words, read and overwritten
 * @param win  the window those words describe; its x extent is a whole number
 *             of words, which snapWindow() guarantees
 *
 * A box is separable, so this is three axis passes, each doubling its reach
 * per launch exactly as dilateXInPlace()/dilateAxisPingPong() do.
 *
 * On kDeclined @p bits is untouched. On kFailed it may have been left partly
 * written: the download is the only step that writes it and the only one that
 * can fail halfway, so a caller falling back to the CPU has to put the seeds
 * back first rather than dilate whatever survived.
 */
Status dilateBox(uint64_t *bits, const VoxelWindow &win, int rx, int ry, int rz);

/**
 * @brief Dilate @p bits in place by an element given as x runs.
 *
 * @param runs  must be sorted by rx ascending, as buildKernel() leaves them:
 *              the rows are grown along x once per distinct half-extent and
 *              every run at that extent is OR-ed into place from it.
 *
 * @p bits follows the same contract as dilateBox().
 */
Status dilateRuns(uint64_t *bits, const VoxelWindow &win, const KernelRun *runs,
                  std::size_t count);

#else  // CPU-only build

inline bool built() { return false; }
inline bool available() { return false; }
inline void warmUp() {}

inline Status dilateBox(uint64_t *, const VoxelWindow &, int, int, int) {
  return Status::kDeclined;
}
inline Status dilateRuns(uint64_t *, const VoxelWindow &, const KernelRun *,
                         std::size_t) {
  return Status::kDeclined;
}

#endif  // OCCUPANCY_INFLATION_WITH_CUDA

}  // namespace cuda
}  // namespace occupancy_inflation

#endif  // OCCUPANCY_INFLATION_INFLATION_CUDA_H_
