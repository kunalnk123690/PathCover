/**
 * @file eigen_conversions.h
 * @brief PointCloud <-> Eigen helpers, plus the zero-copy PointCloud2 readers
 *        and writers this package uses instead of PCL.
 *
 * The four templates at the top keep the original API of the shared
 * eigen_conversions.h so this file stays a drop-in replacement. They all
 * materialise a copy of the cloud, which is fine for one-shot use but too
 * expensive for a node that runs on every incoming map.
 *
 * The second half is what occupancy_inflation actually runs on: XyzLayout
 * resolves the x/y/z field offsets once per message, and PointCloud2XyzReader
 * hands out points straight from the message buffer with no intermediate
 * container at all.
 */

#ifndef OCCUPANCY_INFLATION_EIGEN_CONVERSIONS_H_
#define OCCUPANCY_INFLATION_EIGEN_CONVERSIONS_H_

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>

#include <Eigen/Core>
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace EigenPointCloudConversions {

/* geometry_msgs::Point32 is three floats with no padding, which is what lets
 * the sensor_msgs::PointCloud helpers below map its storage directly. */
static_assert(sizeof(geometry_msgs::Point32) == 3 * sizeof(float),
              "geometry_msgs::Point32 is expected to be three packed floats");

// ---------------------------------------------------------------------------
// sensor_msgs::PointCloud (the legacy message) <-> Eigen
// ---------------------------------------------------------------------------

template <typename T>
inline void EigenToPointCloud(const Eigen::Matrix<T, 3, -1> &inputCloud,
                              sensor_msgs::PointCloud &cloud) {
  cloud.points.resize(inputCloud.cols());
  if (inputCloud.cols() == 0) {
    return;
  }
  Eigen::Map<Eigen::Matrix<float, 3, -1>> map(
      reinterpret_cast<float *>(cloud.points.data()), 3, inputCloud.cols());
  map = inputCloud.template cast<float>();
}

template <typename T>
inline void PointCloudToEigen(const sensor_msgs::PointCloud &cloud,
                              Eigen::Matrix<T, 3, -1> &outputCloud) {
  outputCloud.resize(3, cloud.points.size());
  if (cloud.points.empty()) {
    return;
  }
  Eigen::Map<const Eigen::Matrix<float, 3, -1>> mat(
      reinterpret_cast<const float *>(cloud.points.data()), 3,
      cloud.points.size());
  outputCloud = mat.template cast<T>();
}

template <typename T>
inline void PointCloudToVector(const sensor_msgs::PointCloud &cloud,
                               std::vector<Eigen::Matrix<T, 3, 1>> &outputCloud) {
  outputCloud.resize(cloud.points.size());
  if (cloud.points.empty()) {
    return;
  }
  Eigen::Map<const Eigen::Matrix<float, 3, -1>> points_map(
      reinterpret_cast<const float *>(cloud.points.data()), 3,
      cloud.points.size());
  Eigen::Map<Eigen::Matrix<T, 3, -1>>(
      reinterpret_cast<T *>(outputCloud.data()), 3, points_map.cols()) =
      points_map.template cast<T>();
}

template <typename T>
inline void VectorToPointCloud(const std::vector<Eigen::Matrix<T, 3, 1>> &inputCloud,
                               sensor_msgs::PointCloud &cloud) {
  cloud.points.resize(inputCloud.size());
  if (inputCloud.empty()) {
    return;
  }
  Eigen::Map<const Eigen::Matrix<T, 3, -1>> points_map(
      reinterpret_cast<const T *>(inputCloud.data()), 3, inputCloud.size());
  Eigen::Map<Eigen::Matrix<float, 3, -1>> cloud_map(
      reinterpret_cast<float *>(cloud.points.data()), 3, inputCloud.size());
  cloud_map = points_map.template cast<float>();
}

// ---------------------------------------------------------------------------
// sensor_msgs::PointCloud2 field layout
// ---------------------------------------------------------------------------

/**
 * @brief Where x, y and z live inside a PointCloud2, resolved once per message.
 *
 * Resolving the offsets per message rather than per point is the difference
 * between a linear scan of the field list for every point and a single pass.
 */
struct XyzLayout {
  bool valid = false;
  uint8_t datatype = 0;  //!< sensor_msgs::PointField::FLOAT32 or FLOAT64
  uint32_t x_offset = 0;
  uint32_t y_offset = 0;
  uint32_t z_offset = 0;
  uint32_t point_step = 0;
  std::size_t size = 0;              //!< number of points
  const uint8_t *data = nullptr;     //!< first point, owned by the message
  std::string error;                 //!< why valid is false
};

/**
 * @brief Resolve the x/y/z layout of a PointCloud2 without copying anything.
 *
 * Rejects clouds this package cannot read straight out of the buffer: foreign
 * endianness, missing coordinates, mixed or non-floating-point coordinate
 * types, and a point_step too small to hold the fields it advertises.
 */
inline XyzLayout InspectXyz(const sensor_msgs::PointCloud2 &msg) {
  XyzLayout layout;

  if (msg.is_bigendian != static_cast<uint8_t>(false)) {
    layout.error = "big-endian point clouds are not supported";
    return layout;
  }

  const sensor_msgs::PointField *fx = nullptr;
  const sensor_msgs::PointField *fy = nullptr;
  const sensor_msgs::PointField *fz = nullptr;
  for (const auto &field : msg.fields) {
    if (field.name == "x") {
      fx = &field;
    } else if (field.name == "y") {
      fy = &field;
    } else if (field.name == "z") {
      fz = &field;
    }
  }
  if (fx == nullptr || fy == nullptr || fz == nullptr) {
    layout.error = "cloud has no x/y/z fields";
    return layout;
  }
  if (fx->datatype != fy->datatype || fx->datatype != fz->datatype) {
    layout.error = "x, y and z must share one datatype";
    return layout;
  }
  if (fx->datatype != sensor_msgs::PointField::FLOAT32 &&
      fx->datatype != sensor_msgs::PointField::FLOAT64) {
    layout.error = "x/y/z must be FLOAT32 or FLOAT64";
    return layout;
  }

  const std::size_t scalar_size =
      fx->datatype == sensor_msgs::PointField::FLOAT32 ? 4u : 8u;
  const std::size_t reach =
      std::max(std::max(fx->offset, fy->offset), fz->offset) + scalar_size;
  if (msg.point_step < reach) {
    layout.error = "point_step is smaller than the x/y/z fields it declares";
    return layout;
  }

  const std::size_t count =
      static_cast<std::size_t>(msg.height) * static_cast<std::size_t>(msg.width);
  if (msg.point_step != 0 && count > msg.data.size() / msg.point_step) {
    layout.error = "data buffer is shorter than width * height points";
    return layout;
  }

  layout.valid = true;
  layout.datatype = fx->datatype;
  layout.x_offset = fx->offset;
  layout.y_offset = fy->offset;
  layout.z_offset = fz->offset;
  layout.point_step = msg.point_step;
  layout.size = count;
  layout.data = msg.data.data();
  return layout;
}

/**
 * @brief Random-access view of the x/y/z of a PointCloud2, with no copy.
 *
 * @tparam Storage the type the coordinates are stored as in the message.
 *
 * Templating on the storage type keeps the datatype branch out of the inner
 * loop, and the index-based access lets callers split the cloud across threads.
 * Coordinates are read with memcpy because point_step gives no alignment
 * guarantee; every compiler folds that back into a single load.
 */
template <typename Storage>
class PointCloud2XyzReader {
 public:
  PointCloud2XyzReader() = default;

  explicit PointCloud2XyzReader(const XyzLayout &layout)
      : data_(layout.data),
        size_(layout.size),
        step_(layout.point_step),
        x_offset_(layout.x_offset),
        y_offset_(layout.y_offset),
        z_offset_(layout.z_offset) {}

  std::size_t size() const { return size_; }

  void point(std::size_t i, float &x, float &y, float &z) const {
    const uint8_t *p = data_ + i * step_;
    Storage sx, sy, sz;
    std::memcpy(&sx, p + x_offset_, sizeof(Storage));
    std::memcpy(&sy, p + y_offset_, sizeof(Storage));
    std::memcpy(&sz, p + z_offset_, sizeof(Storage));
    x = static_cast<float>(sx);
    y = static_cast<float>(sy);
    z = static_cast<float>(sz);
  }

 private:
  const uint8_t *data_ = nullptr;
  std::size_t size_ = 0;
  std::size_t step_ = 0;
  std::size_t x_offset_ = 0;
  std::size_t y_offset_ = 0;
  std::size_t z_offset_ = 0;
};

// ---------------------------------------------------------------------------
// Writing a PointCloud2
// ---------------------------------------------------------------------------

/** Byte stride of the clouds PrepareXyziCloud lays out. */
inline std::size_t XyziPointStep(bool with_intensity) {
  return with_intensity ? 16u : 12u;
}

/**
 * @brief Lay out a dense XYZ(I) PointCloud2 and size its buffer for @p points.
 *
 * The caller then writes the payload straight into msg.data, so the points are
 * only ever materialised once.
 */
inline void PrepareXyziCloud(sensor_msgs::PointCloud2 &msg, std::size_t points,
                             bool with_intensity) {
  const uint32_t step = static_cast<uint32_t>(XyziPointStep(with_intensity));

  msg.fields.clear();
  msg.fields.reserve(with_intensity ? 4 : 3);
  const char *names[4] = {"x", "y", "z", "intensity"};
  for (int i = 0; i < (with_intensity ? 4 : 3); ++i) {
    sensor_msgs::PointField field;
    field.name = names[i];
    field.offset = static_cast<uint32_t>(4 * i);
    field.datatype = sensor_msgs::PointField::FLOAT32;
    field.count = 1;
    msg.fields.push_back(field);
  }

  msg.height = 1;
  msg.width = static_cast<uint32_t>(points);
  msg.is_bigendian = false;
  msg.point_step = step;
  msg.row_step = static_cast<uint32_t>(step * points);
  msg.is_dense = true;
  msg.data.resize(step * points);
}

}  // namespace EigenPointCloudConversions

#endif  // OCCUPANCY_INFLATION_EIGEN_CONVERSIONS_H_
