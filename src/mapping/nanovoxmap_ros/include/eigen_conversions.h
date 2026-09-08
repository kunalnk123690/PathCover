/**
 * @file eigen_conversions.h
 * @brief Conversions between ROS sensor_msgs point cloud types and Eigen matrices/vectors.
 */

#ifndef EIGEN_CONVERSIONS_H
#define EIGEN_CONVERSIONS_H

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <Eigen/Dense>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>

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
inline void findXYZFloat32Fields(const sensor_msgs::PointCloud2& cloud_msg,
                                  int& x_offset, int& y_offset, int& z_offset) {
    const sensor_msgs::PointField *fx = nullptr, *fy = nullptr, *fz = nullptr;
    for (const auto& field : cloud_msg.fields) {
        if (field.name == "x") fx = &field;
        else if (field.name == "y") fy = &field;
        else if (field.name == "z") fz = &field;
    }
    if (!fx || !fy || !fz) {
        throw std::runtime_error("PointCloud2 message is missing x/y/z fields");
    }
    if (fx->datatype != sensor_msgs::PointField::FLOAT32 ||
        fy->datatype != sensor_msgs::PointField::FLOAT32 ||
        fz->datatype != sensor_msgs::PointField::FLOAT32) {
        throw std::runtime_error("PointCloud2 x/y/z fields must be FLOAT32");
    }
    x_offset = fx->offset;
    y_offset = fy->offset;
    z_offset = fz->offset;
    // A point must actually contain the three floats the offsets point at.
    // Without this a message declaring point_step = 4 and z at offset 8 would
    // read past the end of every point.
    const size_t last = static_cast<size_t>(
        std::max(x_offset, std::max(y_offset, z_offset)));
    if (static_cast<size_t>(cloud_msg.point_step) < last + sizeof(float)) {
        throw std::runtime_error("PointCloud2 point_step is too small for its x/y/z offsets");
    }
    if (cloud_msg.is_bigendian) {
        throw std::runtime_error("big-endian PointCloud2 is not supported");
    }
}

/**
 * @brief Read one FLOAT32 field out of a point record.
 *
 * Via memcpy rather than a `reinterpret_cast` dereference: PointCloud2 packs
 * points at whatever `point_step` the producer chose, so nothing guarantees a
 * field lands on a 4-byte boundary. The cast version is undefined behaviour on
 * every platform and a real fault risk on the ARM targets this runs on; memcpy
 * compiles to the same single load whenever the address happens to be aligned.
 */
inline float readFloat32(const uint8_t* point_ptr, int offset) {
    float value;
    std::memcpy(&value, point_ptr + offset, sizeof(float));
    return value;
}

/**
 * @brief Convert a 3xN Eigen matrix of points to a sensor_msgs::PointCloud.
 * @tparam T Scalar type of the input matrix; cast to float on write.
 * @param inputCloud 3xN matrix, one point per column.
 * @param[out] cloud Resized to N points and filled in-place.
 */
template <typename T>
inline void MatrixToPointCloud(const Eigen::Matrix<T, 3, -1>& inputCloud, sensor_msgs::PointCloud& cloud) {
    // Resize msg.points to match the number of columns in the Eigen matrix
    cloud.points.resize(inputCloud.cols());

    // Create a map to write directly into msg.points, converting Scalar to float
    Eigen::Map<Eigen::Matrix<float, 3, -1>> map(reinterpret_cast<float*>(cloud.points.data()), 3, inputCloud.cols());

    // Assign the Eigen matrix to mapped memory, casting to float if necessary
    map = inputCloud.template cast<float>();
}


/**
 * @brief Convert a sensor_msgs::PointCloud to a 3xN Eigen matrix.
 * @tparam T Scalar type of the output matrix; cast from the message's float storage.
 * @param cloud Source point cloud.
 * @param[out] outputCloud 3xN matrix, one point per column.
 */
template <typename T>
inline void PointCloudToMatrix(const sensor_msgs::PointCloud& cloud, Eigen::Matrix<T, 3, -1>& outputCloud) {

    // Create a map from the cloud's data
    const float* points = reinterpret_cast<const float*>(cloud.points.data());
    Eigen::Map<const Eigen::Matrix<float, 3, -1>> mat(points, 3, cloud.points.size());

    // Cast the data to the desired Scalar type if necessary
    outputCloud = mat.template cast<T>();
}



/**
 * @brief Convert a sensor_msgs::PointCloud to a std::vector of Eigen 3-vectors.
 * @tparam T Scalar type of the output vectors; cast from the message's float storage.
 * @param cloud Source point cloud.
 * @param[out] outputCloud Resized to match `cloud.points.size()` and filled in-place.
 */
template <typename T>
inline void PointCloudToVector(const sensor_msgs::PointCloud& cloud, std::vector<Eigen::Matrix<T, 3, 1>>& outputCloud) {

    // Map the raw data into an Eigen::Matrix (sensor_msgs::PointCloud uses float internally)
    Eigen::Map<const Eigen::Matrix<float, 3, -1>> points_map(reinterpret_cast<const float*>(cloud.points.data()), 3, cloud.points.size());

    // Resize the output vector to hold all points
    outputCloud.resize(points_map.cols());

    // Map the Eigen matrix directly into the vector's memory, with type casting
    Eigen::Map<Eigen::Matrix<T, 3, -1>>(reinterpret_cast<T*>(outputCloud.data()), 3, points_map.cols()) = points_map.template cast<T>();
}


/**
 * @brief Convert a std::vector of Eigen 3-vectors to a sensor_msgs::PointCloud.
 * @tparam T Scalar type of the input vectors; cast to float on write.
 * @param inputCloud Source points.
 * @param[out] cloud Resized to match `inputCloud.size()` and filled in-place.
 */
template <typename T>
inline void VectorToPointCloud(const std::vector<Eigen::Matrix<T, 3, 1>>& inputCloud, sensor_msgs::PointCloud& cloud) {

    // Resize the sensor_msgs::PointCloud to match the input vector
    cloud.points.resize(inputCloud.size());

    // Map the vector memory into an Eigen::Matrix
    Eigen::Map<const Eigen::Matrix<T, 3, -1>> points_map(reinterpret_cast<const T*>(inputCloud.data()), 3, inputCloud.size());

    // Map the sensor_msgs::PointCloud memory
    Eigen::Map<Eigen::Matrix<float, 3, -1>> cloud_map(reinterpret_cast<float*>(cloud.points.data()), 3, inputCloud.size());

    // Copy data from the input vector to the sensor_msgs::PointCloud with type casting
    cloud_map = points_map.template cast<float>();
}



/**
 * @brief Convert a sensor_msgs::PointCloud2 to a 3xN Eigen matrix.
 * @tparam T Scalar type of the output matrix; cast from the message's FLOAT32 x/y/z fields.
 * @param cloud_msg Source point cloud; x/y/z fields must be FLOAT32 (see findXYZFloat32Fields()).
 * @param[out] result Resized to 3 x (height*width) and filled in-place.
 * @throws std::runtime_error if x/y/z fields are missing/wrong type, or the
 *         data buffer is smaller than `height*width*point_step`.
 */
template <typename T>
inline void PointCloud2ToMatrix(const sensor_msgs::PointCloud2& cloud_msg, Eigen::Matrix<T, 3, Eigen::Dynamic>& result) {

    // size_t, not uint32_t: height * width is a uint32 product that wraps on a
    // large organized cloud, and the wrapped count would size the output too
    // small for the loop that fills it.
    const size_t num_points = static_cast<size_t>(cloud_msg.height) * cloud_msg.width;
    if (num_points == 0) {
        result.resize(3, 0);
        return;
    }

    // Validate before allocating: a malformed header should not first cost a
    // multi-gigabyte allocation.
    int x_offset, y_offset, z_offset;
    findXYZFloat32Fields(cloud_msg, x_offset, y_offset, z_offset);
    const size_t required = (static_cast<size_t>(cloud_msg.height) - 1) * cloud_msg.row_step +
                            static_cast<size_t>(cloud_msg.width) * cloud_msg.point_step;
    if (cloud_msg.row_step < cloud_msg.width * cloud_msg.point_step ||
        cloud_msg.data.size() < required) {
        throw std::runtime_error("PointCloud2ToMatrix: invalid row_step/data buffer");
    }

    result.resize(3, static_cast<Eigen::Index>(num_points));

    const uint8_t* data_ptr = cloud_msg.data.data();
    for (size_t i = 0; i < num_points; ++i) {
        const size_t row = i / cloud_msg.width, col = i % cloud_msg.width;
        const uint8_t* point_ptr = data_ptr + row * cloud_msg.row_step + col * cloud_msg.point_step;
        result(0, static_cast<Eigen::Index>(i)) = readFloat32(point_ptr, x_offset);
        result(1, static_cast<Eigen::Index>(i)) = readFloat32(point_ptr, y_offset);
        result(2, static_cast<Eigen::Index>(i)) = readFloat32(point_ptr, z_offset);
    }
}


/**
 * @brief Convert a sensor_msgs::PointCloud2 to a std::vector of Eigen 3-vectors.
 * @tparam T Scalar type of the output vectors; cast from the message's FLOAT32 x/y/z fields.
 * @param cloud_msg Source point cloud; x/y/z fields must be FLOAT32 (see findXYZFloat32Fields()).
 * @param[out] result Resized to `height*width` and filled in-place.
 * @throws std::runtime_error if x/y/z fields are missing/wrong type, or the
 *         data buffer is smaller than `height*width*point_step`.
 */
template <typename T>
inline void PointCloud2ToVector(const sensor_msgs::PointCloud2& cloud_msg, std::vector<Eigen::Matrix<T, 3, 1>>& result) {

    // size_t, not uint32_t: see PointCloud2ToMatrix.
    const size_t num_points = static_cast<size_t>(cloud_msg.height) * cloud_msg.width;
    if (num_points == 0) {
        result.clear();
        return;
    }

    // Validate before allocating, so a malformed header costs nothing.
    int x_offset, y_offset, z_offset;
    findXYZFloat32Fields(cloud_msg, x_offset, y_offset, z_offset);
    const size_t required = (static_cast<size_t>(cloud_msg.height) - 1) * cloud_msg.row_step +
                            static_cast<size_t>(cloud_msg.width) * cloud_msg.point_step;
    if (cloud_msg.row_step < cloud_msg.width * cloud_msg.point_step ||
        cloud_msg.data.size() < required) {
        throw std::runtime_error("PointCloud2ToVector: invalid row_step/data buffer");
    }

    result.resize(num_points);

    const uint8_t* data_ptr = cloud_msg.data.data();
    for (size_t i = 0; i < num_points; ++i) {
        const size_t row = i / cloud_msg.width, col = i % cloud_msg.width;
        const uint8_t* point_ptr = data_ptr + row * cloud_msg.row_step + col * cloud_msg.point_step;
        result[i] << static_cast<T>(readFloat32(point_ptr, x_offset)),
                     static_cast<T>(readFloat32(point_ptr, y_offset)),
                     static_cast<T>(readFloat32(point_ptr, z_offset));
    }
}



} // namespace EigenPointCloudConversions

#endif // EIGEN_CONVERSIONS_H
