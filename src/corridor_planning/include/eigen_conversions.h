/**
 * @file eigen_conversions.h
 * @brief Conversions between ROS sensor_msgs point cloud types and Eigen matrices/vectors.
 */

#ifndef EIGEN_CONVERSIONS_H
#define EIGEN_CONVERSIONS_H

#include <iostream>
#include <stdexcept>
#include <Eigen/Dense>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

/// @brief Helpers for converting between sensor_msgs point clouds and Eigen types.
namespace EigenPointCloudConversions {

/**
 * @brief Locate the x/y/z fields of a PointCloud2 message.
 *
 * Validates that all three fields exist and are FLOAT32 (the type the
 * conversions below read via `reinterpret_cast`). Throws rather than
 * silently reading out of bounds or misinterpreting bytes, since both a
 * missing field and a non-FLOAT32 field would otherwise corrupt or crash on
 * attacker- or driver-malformed input.
 * @param cloud_msg Source point cloud message.
 * @param[out] x_offset Byte offset of the "x" field within each point.
 * @param[out] y_offset Byte offset of the "y" field within each point.
 * @param[out] z_offset Byte offset of the "z" field within each point.
 * @throws std::runtime_error if x/y/z fields are missing or not FLOAT32.
 */
inline void findXYZFloat32Fields(const sensor_msgs::msg::PointCloud2& cloud_msg,
                                  int& x_offset, int& y_offset, int& z_offset) {
    const sensor_msgs::msg::PointField *fx = nullptr, *fy = nullptr, *fz = nullptr;
    for (const auto& field : cloud_msg.fields) {
        if (field.name == "x") fx = &field;
        else if (field.name == "y") fy = &field;
        else if (field.name == "z") fz = &field;
    }
    if (!fx || !fy || !fz) {
        throw std::runtime_error("PointCloud2 message is missing x/y/z fields");
    }
    if (fx->datatype != sensor_msgs::msg::PointField::FLOAT32 ||
        fy->datatype != sensor_msgs::msg::PointField::FLOAT32 ||
        fz->datatype != sensor_msgs::msg::PointField::FLOAT32) {
        throw std::runtime_error("PointCloud2 x/y/z fields must be FLOAT32");
    }
    x_offset = fx->offset;
    y_offset = fy->offset;
    z_offset = fz->offset;
}

/**
 * @brief Convert a 3xN Eigen matrix of points to a sensor_msgs::msg::PointCloud.
 * @tparam T Scalar type of the input matrix; cast to float on write.
 * @param inputCloud 3xN matrix, one point per column.
 * @param[out] cloud Resized to N points and filled in-place.
 */
template <typename T>
inline void MatrixToPointCloud(const Eigen::Matrix<T, 3, -1>& inputCloud, sensor_msgs::msg::PointCloud& cloud) {
    // Resize msg.points to match the number of columns in the Eigen matrix
    cloud.points.resize(inputCloud.cols());

    // Create a map to write directly into msg.points, converting Scalar to float
    Eigen::Map<Eigen::Matrix<float, 3, -1>> map(reinterpret_cast<float*>(cloud.points.data()), 3, inputCloud.cols());

    // Assign the Eigen matrix to mapped memory, casting to float if necessary
    map = inputCloud.template cast<float>();
}


/**
 * @brief Convert a sensor_msgs::msg::PointCloud to a 3xN Eigen matrix.
 * @tparam T Scalar type of the output matrix; cast from the message's float storage.
 * @param cloud Source point cloud.
 * @param[out] outputCloud 3xN matrix, one point per column.
 */
template <typename T>
inline void PointCloudToMatrix(const sensor_msgs::msg::PointCloud& cloud, Eigen::Matrix<T, 3, -1>& outputCloud) {

    // Create a map from the cloud's data
    const float* points = reinterpret_cast<const float*>(cloud.points.data());
    Eigen::Map<const Eigen::Matrix<float, 3, -1>> mat(points, 3, cloud.points.size());

    // Cast the data to the desired Scalar type if necessary
    outputCloud = mat.template cast<T>();
}



/**
 * @brief Convert a sensor_msgs::msg::PointCloud to a std::vector of Eigen 3-vectors.
 * @tparam T Scalar type of the output vectors; cast from the message's float storage.
 * @param cloud Source point cloud.
 * @param[out] outputCloud Resized to match `cloud.points.size()` and filled in-place.
 */
template <typename T>
inline void PointCloudToVector(const sensor_msgs::msg::PointCloud& cloud, std::vector<Eigen::Matrix<T, 3, 1>>& outputCloud) {

    // Map the raw data into an Eigen::Matrix (sensor_msgs::msg::PointCloud uses float internally)
    Eigen::Map<const Eigen::Matrix<float, 3, -1>> points_map(reinterpret_cast<const float*>(cloud.points.data()), 3, cloud.points.size());

    // Resize the output vector to hold all points
    outputCloud.resize(points_map.cols());

    // Map the Eigen matrix directly into the vector's memory, with type casting
    Eigen::Map<Eigen::Matrix<T, 3, -1>>(reinterpret_cast<T*>(outputCloud.data()), 3, points_map.cols()) = points_map.template cast<T>();
}


/**
 * @brief Convert a std::vector of Eigen 3-vectors to a sensor_msgs::msg::PointCloud.
 * @tparam T Scalar type of the input vectors; cast to float on write.
 * @param inputCloud Source points.
 * @param[out] cloud Resized to match `inputCloud.size()` and filled in-place.
 */
template <typename T>
inline void VectorToPointCloud(const std::vector<Eigen::Matrix<T, 3, 1>>& inputCloud, sensor_msgs::msg::PointCloud& cloud) {

    // Resize the sensor_msgs::msg::PointCloud to match the input vector
    cloud.points.resize(inputCloud.size());

    // Map the vector memory into an Eigen::Matrix
    Eigen::Map<const Eigen::Matrix<T, 3, -1>> points_map(reinterpret_cast<const T*>(inputCloud.data()), 3, inputCloud.size());

    // Map the sensor_msgs::msg::PointCloud memory
    Eigen::Map<Eigen::Matrix<float, 3, -1>> cloud_map(reinterpret_cast<float*>(cloud.points.data()), 3, inputCloud.size());

    // Copy data from the input vector to the sensor_msgs::msg::PointCloud with type casting
    cloud_map = points_map.template cast<float>();
}



/**
 * @brief Convert a sensor_msgs::msg::PointCloud2 to a 3xN Eigen matrix.
 * @tparam T Scalar type of the output matrix; cast from the message's FLOAT32 x/y/z fields.
 * @param cloud_msg Source point cloud; x/y/z fields must be FLOAT32 (see findXYZFloat32Fields()).
 * @param[out] result Resized to 3 x (height*width) and filled in-place.
 * @throws std::runtime_error if x/y/z fields are missing/wrong type, or the
 *         data buffer is smaller than `height*width*point_step`.
 */
template <typename T>
inline void PointCloud2ToMatrix(const sensor_msgs::msg::PointCloud2& cloud_msg, Eigen::Matrix<T, 3, Eigen::Dynamic>& result) {

    // Extract the number of points in the PointCloud2 message
    uint32_t num_points = cloud_msg.height * cloud_msg.width;

    // Initialize the result matrix with 3 rows and num_points columns
    result.resize(3, num_points);
    if (num_points == 0) return;

    int x_offset, y_offset, z_offset;
    findXYZFloat32Fields(cloud_msg, x_offset, y_offset, z_offset);

    if (cloud_msg.data.size() < static_cast<size_t>(num_points) * cloud_msg.point_step) {
        throw std::runtime_error("PointCloud2ToMatrix: data buffer smaller than height*width*point_step");
    }

    // Extract point data
    const uint8_t* data_ptr = cloud_msg.data.data();
    for (size_t i = 0; i < num_points; ++i) {
        const uint8_t* point_ptr = data_ptr + i * cloud_msg.point_step;
        result(0, i) = *reinterpret_cast<const float*>(point_ptr + x_offset);
        result(1, i) = *reinterpret_cast<const float*>(point_ptr + y_offset);
        result(2, i) = *reinterpret_cast<const float*>(point_ptr + z_offset);
    }

}


/**
 * @brief Convert a sensor_msgs::msg::PointCloud2 to a std::vector of Eigen 3-vectors.
 * @tparam T Scalar type of the output vectors; cast from the message's FLOAT32 x/y/z fields.
 * @param cloud_msg Source point cloud; x/y/z fields must be FLOAT32 (see findXYZFloat32Fields()).
 * @param[out] result Resized to `height*width` and filled in-place.
 * @throws std::runtime_error if x/y/z fields are missing/wrong type, or the
 *         data buffer is smaller than `height*width*point_step`.
 */
template <typename T>
inline void PointCloud2ToVector(const sensor_msgs::msg::PointCloud2& cloud_msg, std::vector<Eigen::Matrix<T, 3, 1>>& result) {

    // Extract the number of points in the PointCloud2 message
    uint32_t num_points = cloud_msg.height * cloud_msg.width;
    result.resize(num_points);
    if (num_points == 0) return;

    int x_offset, y_offset, z_offset;
    findXYZFloat32Fields(cloud_msg, x_offset, y_offset, z_offset);

    if (cloud_msg.data.size() < static_cast<size_t>(num_points) * cloud_msg.point_step) {
        throw std::runtime_error("PointCloud2ToVector: data buffer smaller than height*width*point_step");
    }

    // Extract point data
    const uint8_t* data_ptr = cloud_msg.data.data();
    for (size_t i = 0; i < num_points; ++i) {
        const uint8_t* point_ptr = data_ptr + i * cloud_msg.point_step;
        Eigen::Matrix<T, 3, 1> point;
        point(0) = *reinterpret_cast<const float*>(point_ptr + x_offset);
        point(1) = *reinterpret_cast<const float*>(point_ptr + y_offset);
        point(2) = *reinterpret_cast<const float*>(point_ptr + z_offset);
        result[i] = point;
    }

}


/**
 * @brief Fused PointCloud2 -> Eigen conversion + circular (XY-plane) radius filter.
 *
 * Single pass over the raw PointCloud2 bytes: points within `radius` of
 * `start` in the XY plane are dropped, everything else is appended to `out`.
 * Avoids materializing the full converted cloud before filtering.
 * @param msg Source point cloud; x/y/z fields must be FLOAT32 (see findXYZFloat32Fields()).
 * @param start Center of the exclusion disk (only x/y are used).
 * @param radius Exclusion radius, in the same units as the cloud.
 * @param[out] out Cleared and filled with the surviving points.
 * @throws std::runtime_error if x/y/z fields are missing/wrong type.
 */
template <typename T>
void convertAndFilter(const sensor_msgs::msg::PointCloud2 &msg,
                      const Eigen::Matrix<T, 3, 1> &start,
                      const T radius,
                      std::vector<Eigen::Matrix<T, 3, 1>> &out) {

  const uint32_t num_points = msg.height * msg.width;
  if (num_points == 0) {
    out.clear();
    return;
  }

  // 1. Compile-time constant offsets for standard packed PointCloud2 layouts.
  // In ROS, standard XYZ clouds typically have x=0, y=4, z=8. We use a fast-path
  // check to bypass the dynamic field lookup if it matches standard packing.
  int x_offset = 0, y_offset = 4, z_offset = 8;

  if (msg.fields.size() < 3 ||
      msg.fields[0].name != "x" || msg.fields[0].offset != 0 ||
      msg.fields[1].name != "y" || msg.fields[1].offset != 4 ||
      msg.fields[2].name != "z" || msg.fields[2].offset != 8) {

    // Fallback to dynamic lookup if the layout is non-standard; throws if
    // x/y/z are missing or not FLOAT32.
    findXYZFloat32Fields(msg, x_offset, y_offset, z_offset);
  }

  // 2. Pre-allocate memory to prevent costly vector resizing reallocations
  out.clear();
  out.reserve(num_points);

  const uint8_t * __restrict__ data_ptr = msg.data.data();
  const uint32_t point_step = msg.point_step;

  // Cache start positions and pre-calculate squared radius
  const T sx = start.x();
  const T sy = start.y();
  const T r2 = radius * radius;

  // 3. Loop optimization: Hoist invariant operations and help the compiler autovectorize
  for (uint32_t i = 0; i < num_points; ++i) {
    const uint8_t * __restrict__ p = data_ptr + (static_cast<size_t>(i) * point_step);

    // ROS PointCloud2 data is almost always float (32-bit).
    // We cast to float first to read correctly, then implicitly convert to template type T.
    const T x = static_cast<T>(*reinterpret_cast<const float*>(p + x_offset));
    const T y = static_cast<T>(*reinterpret_cast<const float*>(p + y_offset));
    const T z = static_cast<T>(*reinterpret_cast<const float*>(p + z_offset));

    const T dx = x - sx;
    const T dy = y - sy;

    if ((dx * dx) + (dy * dy) > r2) {
      out.emplace_back(x, y, z);
    }
  }
}


} // namespace EigenPointCloudConversions

#endif // EIGEN_CONVERSIONS_H
