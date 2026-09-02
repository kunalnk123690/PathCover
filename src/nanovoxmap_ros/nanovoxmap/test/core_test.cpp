#include "nanovoxmap/nanovoxmap.hpp"

#include <iostream>
#include <vector>

int main() {
    using Map = NanoVoxMap::OccupancyMap<double>;
    using Vec3 = Map::Vec3;
    Map map(1.0, 0.9, 0.1, 0.1192, 0.971);
    Map forced_cpu(1.0, 0.9, 0.1, 0.1192, 0.971,
                   NanoVoxMap::Backend::kCpu, false);
    if (forced_cpu.activeBackend() != NanoVoxMap::Backend::kCpu ||
        forced_cpu.allowsCudaFallback()) {
        std::cerr << "CPU backend policy was not honored\n";
        return 1;
    }

    // A diagonal clearing ray must visit a face-connected intermediate cell.
    const Vec3 diagonal_ghost(1.5, 0.5, 0.5);
    map.setOccupied(diagonal_ghost);
    const std::vector<Vec3> diagonal_end{Vec3(2.5, 2.5, 0.5)};
    map.insertPointCloud(Vec3(0.5, 0.5, 0.5), diagonal_end, false);
    map.insertPointCloud(Vec3(0.5, 0.5, 0.5), diagonal_end, false);
    if (map.isOccupied(diagonal_ghost)) {
        std::cerr << "diagonal traversal skipped an intermediate voxel\n";
        return 1;
    }

    // Miss-only rays include their finite/clamped endpoint in observed free space.
    const Vec3 endpoint_ghost(4.5, 0.5, 0.5);
    map.setOccupied(endpoint_ghost);
    const std::vector<Vec3> endpoint{endpoint_ghost};
    map.insertPointCloud(Vec3(0.5, 0.5, 0.5), endpoint, false);
    map.insertPointCloud(Vec3(0.5, 0.5, 0.5), endpoint, false);
    if (map.isOccupied(endpoint_ghost)) {
        std::cerr << "clearing endpoint was not updated\n";
        return 1;
    }

    const auto occupied = map.getOccupiedVoxels();
    if (occupied.size() != map.numOccupied()) {
        std::cerr << "block occupancy masks/count disagree\n";
        return 1;
    }

    std::cout << "nanovoxmap core clearing tests passed\n";
    return 0;
}
