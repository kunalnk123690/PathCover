#ifndef VOXELGRID_HPP
#define VOXELGRID_HPP

#include <iostream>
#include <Eigen/Dense>
#include <vector>

namespace VoxelGrid {

    template<typename T, int Dim>
    inline Eigen::Matrix<int, Dim, 1> computePointIndex(const Eigen::Matrix<T, Dim, 1> &point, 
                                                        const Eigen::Matrix<T, Dim, 1> &min_bounds, 
                                                        const T resolution) {
        static_assert(Dim == 2 || Dim == 3, "Dim must be 2 or 3.");
        return ((point - min_bounds) / resolution).array().floor().template cast<int>();
    }    

    template<int Dim>
    inline int getLinearIndex(const Eigen::Matrix<int, Dim, 1> &index, 
                              const Eigen::Matrix<int, Dim, 1> &grid_size) {
        static_assert(Dim == 2 || Dim == 3, "Dim must be 2 or 3.");
        if constexpr (Dim == 2) {
            return index(0) + index(1) * grid_size(0);
        } else {
            return index(0) + index(1) * grid_size(0) + index(2) * grid_size(0) * grid_size(1);
        }
    }

    template<typename T, int Dim>
    inline Eigen::Matrix<int, Dim, 1> initializeGrid(const Eigen::Matrix<T, Dim, 1> &min_bounds, 
                                                     const Eigen::Matrix<T, Dim, 1> &max_bounds,
                                                     T resolution, std::vector<signed char> &grid) {
        static_assert(Dim == 2 || Dim == 3, "Dim must be 2 or 3.");

        if ((max_bounds.array() <= min_bounds.array()).any()) {
            throw std::invalid_argument("max_bounds must be greater than min_bounds in all dimensions.");
        }

        Eigen::Matrix<int, Dim, 1> grid_size = ((max_bounds - min_bounds) / resolution).array().ceil().template cast<int>();

        if ((grid_size.array() == 0).any()) {
            throw std::invalid_argument("Grid size must be non-zero in all dimensions.");
        }

        int total_size = (Dim == 2) ? (grid_size(0) * grid_size(1)) 
                                    : (grid_size(0) * grid_size(1) * grid_size(2));
        grid.assign(total_size, 0); 

        return grid_size;
    }

    // ---------------------------------------------------------------------
    // Fast per-frame reset: only the cells that were marked occupied on the
    // previous frame are cleared, so reset is O(occupied) instead of
    // O(total_voxels). Use together with the setOccupancy overload below that
    // records the occupied linear indices. This avoids re-zeroing (and the
    // original code's per-frame re-allocation of) the entire grid every frame,
    // which matters when the map volume is large relative to occupancy.
    // ---------------------------------------------------------------------
    inline void resetOccupancy(std::vector<signed char> &grid, 
                               std::vector<int> &occupied) {
        for (int linear_idx : occupied) {
            grid[linear_idx] = 0;
        }
        occupied.clear();
    }

    // Original signature (kept for backward compatibility).
    template<typename T, int Dim>
    inline void setOccupancy(const std::vector<Eigen::Matrix<T, Dim, 1>> &pointCloud, 
                             const Eigen::Matrix<T, Dim, 1> &min_bounds, 
                             const T resolution, 
                             std::vector<signed char> &grid, 
                             const Eigen::Matrix<int, Dim, 1> &grid_size) {
        
        static_assert(Dim == 2 || Dim == 3, "Dim must be 2 or 3.");
        Eigen::Matrix<int, Dim, 1> idx;
        
        for (const auto& point : pointCloud) {
            idx = computePointIndex<T, Dim>(point, min_bounds, resolution);

            if ((idx.array() >= 0).all() && (idx.array() < grid_size.array()).all()) {
                int linear_idx = getLinearIndex<Dim>(idx, grid_size);
                grid[linear_idx] = 100;
            }
        }
    }

    // ---------------------------------------------------------------------
    // Overload that records every distinct occupied cell into `occupied`, so
    // the next frame can be cleared cheaply via resetOccupancy(). The
    // `grid[linear_idx] == 0` guard both de-duplicates the occupied list
    // (multiple points in the same voxel are recorded once) and skips
    // redundant stores. Designed to run once per cloud against the FULL cloud
    // (up to ~500k points) before JPS/DMP planning.
    // ---------------------------------------------------------------------
    template<typename T, int Dim>
    inline void setOccupancy(const std::vector<Eigen::Matrix<T, Dim, 1>> &pointCloud, 
                             const Eigen::Matrix<T, Dim, 1> &min_bounds, 
                             const T resolution, 
                             std::vector<signed char> &grid, 
                             const Eigen::Matrix<int, Dim, 1> &grid_size,
                             std::vector<int> &occupied) {
        
        static_assert(Dim == 2 || Dim == 3, "Dim must be 2 or 3.");
        occupied.reserve(pointCloud.size());
        Eigen::Matrix<int, Dim, 1> idx;
        
        for (const auto& point : pointCloud) {
            idx = computePointIndex<T, Dim>(point, min_bounds, resolution);

            if ((idx.array() >= 0).all() && (idx.array() < grid_size.array()).all()) {
                int linear_idx = getLinearIndex<Dim>(idx, grid_size);
                if (grid[linear_idx] == 0) {
                    grid[linear_idx] = 100;
                    occupied.push_back(linear_idx);
                }
            }
        }
    }

    template<typename T, int Dim>
    inline bool checkOccupancy(const Eigen::Matrix<T, Dim, 1> &point,
                               const std::vector<signed char> &grid, 
                               const Eigen::Matrix<T, Dim, 1> &min_bounds, 
                               const T resolution, 
                               const Eigen::Matrix<int, Dim, 1> &grid_size) {
        static_assert(Dim == 2 || Dim == 3, "Dim must be 2 or 3.");

        // FIX: template arguments were previously swapped (<Dim, T>), which
        // fails to compile the moment this function is instantiated.
        Eigen::Matrix<int, Dim, 1> idx = computePointIndex<T, Dim>(point, min_bounds, resolution);

        if ((idx.array() >= 0).all() && (idx.array() < grid_size.array()).all()) {
            int linear_idx = getLinearIndex<Dim>(idx, grid_size);
            return grid[linear_idx] > 0;
        }

        return false;
    }

}

#endif