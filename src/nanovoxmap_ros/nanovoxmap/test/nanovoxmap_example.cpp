/**
 * @file nanovoxmap_example.cpp
 * @brief Minimal NanoVoxMap demo: load a point cloud with PCL, voxelize it into
 *        a sparse log-odds occupancy map, and show the occupied voxels.
 *        Resolution 0.05 m; [-6,-6,0] -> [6,6,4] is used only to frame the
 *        viewer (height coloring, reference wireframe) since the map itself
 *        is unbounded.
 *
 * Usage: nanovoxmap_example [cloud.ply]
 */
#include <nanovoxmap/nanovoxmap.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <algorithm>
#include <iostream>
#include <string>

int main(int argc, char** argv) {
    using Vec3 = NanoVoxMap::OccupancyMapd::Vec3;
    const std::string path =
        argc > 1 ? argv[1] : std::string(NANOVOXMAP_DATA_DIR) + "/cloud.ply";

    // 1. Load the cloud.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPLYFile(path, *cloud) != 0) {
        std::cerr << "Failed to load " << path << "\n";
        return 1;
    }

    // 2. Voxelize it into the occupancy map.
    const Vec3 lower(-6, -6, 0), upper(6, 6, 4);
    NanoVoxMap::OccupancyMapd map(0.05);
    for (const auto& p : *cloud) map.setOccupied(Vec3(p.x, p.y, p.z));
    std::cout << "Loaded " << cloud->size() << " points -> "
              << map.numOccupied() << " occupied voxels\n";

    // 3. Turn the occupied voxels into a cloud coloured by height.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr occ(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (const Vec3& c : map.getOccupiedVoxels()) {
        const double t = std::clamp((c.z() - lower.z()) / (upper.z() - lower.z()), 0.0, 1.0);
        pcl::PointXYZRGB pt;
        pt.x = c.x(); pt.y = c.y(); pt.z = c.z();
        pt.r = static_cast<uint8_t>(255 * t);
        pt.g = 80;
        pt.b = static_cast<uint8_t>(255 * (1 - t));
        occ->push_back(pt);
    }

    // 4. Show it.
    pcl::visualization::PCLVisualizer viewer("NanoVoxMap occupancy");
    viewer.setBackgroundColor(0.05, 0.05, 0.05);
    viewer.addPointCloud<pcl::PointXYZRGB>(occ, "occupied");
    viewer.setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "occupied");
    viewer.addCube(lower.x(), upper.x(), lower.y(), upper.y(), lower.z(), upper.z(),
                   1, 1, 1, "box");
    viewer.setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
        pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "box");
    viewer.spin();  // spin(), not spinOnce(): the latter crashes on PCL+VTK 9.1
    return 0;
}
