#ifndef EIGEN_CONVERSIONS_H
#define EIGEN_CONVERSIONS_H

#include <iostream>
#include <Eigen/Dense>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

using namespace std;

namespace EigenPointCloudConversions {

template <typename T>
inline void MatrixToPointCloud(const Eigen::Matrix<T, 3, -1>& inputCloud, 
                               sensor_msgs::PointCloud &cloud) {
    // Resize msg.points to match the number of columns in the Eigen matrix
    cloud.points.resize(inputCloud.cols());

    // Create a map to write directly into msg.points, converting Scalar to float
    Eigen::Map<Eigen::Matrix<float, 3, -1>> map(reinterpret_cast<float*>(cloud.points.data()), 3, inputCloud.cols());
    
    // Assign the Eigen matrix to mapped memory, casting to float if necessary
    map = inputCloud.template cast<float>();
}    


template <typename T>
inline void PointCloudToMatrix(const sensor_msgs::PointCloud &cloud, 
                               Eigen::Matrix<T, 3, -1> &outputCloud) {
    
    // Create a map from the cloud's data
    const float* points = reinterpret_cast<const float*>(cloud.points.data());
    Eigen::Map<const Eigen::Matrix<float, 3, -1>> mat(points, 3, cloud.points.size());

    // Cast the data to the desired Scalar type if necessary
    outputCloud = mat.template cast<T>();
}



template <typename T>
inline void PointCloudToVector(const sensor_msgs::PointCloud &cloud, 
                               std::vector<Eigen::Matrix<T, 3, 1>> &outputCloud) {

    // Map the raw data into an Eigen::Matrix (sensor_msgs::PointCloud uses float internally)
    Eigen::Map<const Eigen::Matrix<float, 3, -1>> points_map(reinterpret_cast<const float*>(cloud.points.data()), 3, cloud.points.size());

    // Resize the output vector to hold all points
    outputCloud.resize(points_map.cols());

    // Map the Eigen matrix directly into the vector's memory, with type casting
    Eigen::Map<Eigen::Matrix<T, 3, -1>>(reinterpret_cast<T*>(outputCloud.data()), 3, points_map.cols()) = points_map.template cast<T>();
}


template <typename T>
inline void VectorToPointCloud(const std::vector<Eigen::Matrix<T, 3, 1>> &inputCloud, 
                               sensor_msgs::PointCloud &cloud) {

    // Resize the sensor_msgs::PointCloud to match the input vector
    cloud.points.resize(inputCloud.size());

    // Map the vector memory into an Eigen::Matrix
    Eigen::Map<const Eigen::Matrix<T, 3, -1>> points_map(reinterpret_cast<const T*>(inputCloud.data()), 3, inputCloud.size());

    // Map the sensor_msgs::PointCloud memory
    Eigen::Map<Eigen::Matrix<float, 3, -1>> cloud_map(reinterpret_cast<float*>(cloud.points.data()), 3, inputCloud.size());

    // Copy data from the input vector to the sensor_msgs::PointCloud with type casting
    cloud_map = points_map.template cast<float>();
}


template <typename T>
inline void PointCloud2ToMatrix(const sensor_msgs::PointCloud2 &cloud_msg, 
                                Eigen::Matrix<T, 3, Eigen::Dynamic> &result) {

    // Extract the number of points in the PointCloud2 message
    uint32_t num_points = cloud_msg.height * cloud_msg.width;

    // Initialize the result matrix with 3 rows and num_points columns
    result.resize(3, num_points);

    // Locate the x, y, z fields and ensure they exist
    int x_offset = -1, y_offset = -1, z_offset = -1;
    for (const auto& field : cloud_msg.fields) {
        if (field.name == "x") x_offset = field.offset;
        else if (field.name == "y") y_offset = field.offset;
        else if (field.name == "z") z_offset = field.offset;
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


template <typename T>
inline void PointCloud2ToVector(const sensor_msgs::PointCloud2 &cloud_msg, 
                                std::vector<Eigen::Matrix<T, 3, 1>> &result) {

    // Extract the number of points in the PointCloud2 message
    uint32_t num_points = cloud_msg.height * cloud_msg.width;
    result.resize(num_points);

    // Get the pointer to the raw point cloud data (e.g., the x, y, z coordinates)
    const uint8_t* point_data = cloud_msg.data.data();

    // Locate the x, y, z fields and ensure they exist
    int x_offset = -1, y_offset = -1, z_offset = -1;
    for (const auto& field : cloud_msg.fields) {
        if (field.name == "x") x_offset = field.offset;
        else if (field.name == "y") y_offset = field.offset;
        else if (field.name == "z") z_offset = field.offset;
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



template <typename T>
void convertAndFilter(const sensor_msgs::PointCloud2 &msg,
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
    
    // Fallback to dynamic lookup if the layout is non-standard
    x_offset = -1; y_offset = -1; z_offset = -1;
    for (const auto &field : msg.fields) {
      if (field.name == "x") x_offset = field.offset;
      else if (field.name == "y") y_offset = field.offset;
      else if (field.name == "z") z_offset = field.offset;
    }
    
    if (x_offset < 0 || y_offset < 0 || z_offset < 0) {
      ROS_ERROR("convertAndFilter: cloud is missing x/y/z fields.");
      out.clear();
      return;
    }
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


} // namespace EigenCloudConversions

#endif // EIGEN_CONVERSIONS_H