/**
 * @file NanoVoxMapNode.hpp
 * @brief ROS node that builds a NanoVoxMap::OccupancyMap from a synchronized
 *        point cloud + odometry stream and publishes the occupied voxel set.
 */

#ifndef NanoVoxMapNode_H
#define NanoVoxMapNode_H

#include <iostream>
#include <math.h>
#include <mutex>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <cstdint>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "nanovoxmap/nanovoxmap.hpp"
#include "eigen_conversions.h"

namespace NanoVoxMap {

/**
 * @struct Parameters
 * @brief Reads and validates nanovoxmap_node's ROS parameters from the node.
 *
 * Required parameters throw `std::runtime_error` (after an `RCLCPP_FATAL`)
 * if missing or invalid; optional tuning parameters fall back to the
 * defaults below if absent so existing parameter YAMLs keep working.
 */
struct Parameters {
    std::string cloudTopic;     ///< Input point cloud topic name (param `CloudTopic`).
    std::string OdometryTopic;  ///< Input odometry topic name; publishes nav_msgs/Odometry (param `OdometryTopic`).
    std::string OutputTopic;    ///< Output occupied-voxel topic name (param `OutputTopic`).
    double VoxelResolution;     ///< Voxel edge length, in metres (param `VoxelResolution`).

    /// @name Optional tuning parameters
    /// @{
    std::string WorldFrameId   = "world"; ///< Frame stamped on published clouds; must match the frame OdometryTopic reports in.
    double PublishRate    = 5.0;   ///< Publish rate (Hz); decouples publish from sensor rate. `<=0` disables the timer.
    bool   DownsampleCloud = true; ///< Voxel-downsample the input cloud at map resolution before raycasting.
    int    SyncQueueSize   = 20;   ///< Approximate-time synchronizer queue size.
    int    SubQueueSize    = 5;    ///< Per-topic subscriber queue size.
    double MaxRayLength    = 30.0; ///< Clip returns farther than this from the sensor, in metres; `<=0` disables.
    /// @}

    /// @name No-return frustum clearing
    /// Optional, off by default. A depth pixel with no return means its beam
    /// reached MaxRayLength without hitting anything, so the ray out to
    /// MaxRayLength is free; reconstructing those rays from the camera
    /// intrinsics sweeps out ghost voxels a plain surface return never
    /// reaches. Requires an organized depth cloud (CameraInfo width/height
    /// matching the cloud) and MaxRayLength > 0.
    /// @{
    std::string DepthCameraInfoTopic;    ///< CameraInfo topic for the depth sensor; empty disables clearing.
    int         ClearingRayStride = 1;   ///< Pixel stride (both axes) for the clearing-ray fan; 1 = every empty pixel.
    bool        DepthCloudOpticalFrame = true; ///< true: +Z forward; false: Gazebo +X forward convention.
    /// @}

    /// @name Log-odds occupancy model parameters
    /// OctoMap-standard defaults. A single hit or miss nudges a cell's
    /// log-odds by ln(p/(1-p)) instead of overwriting its state outright, so
    /// a few spurious returns can't flip a cell - repeated, consistent
    /// observations are needed to cross the occupied/free threshold. Tune
    /// ProbHit/ProbMiss for noisier or cleaner sensors.
    /// @{
    double ProbHit         = 0.7;    ///< P(occupied | hit).
    double ProbMiss        = 0.4;    ///< P(occupied | miss).
    double ProbMin         = 0.1192; ///< Clamp floor (most-free a cell can get).
    double ProbMax         = 0.971;  ///< Clamp ceiling (most-occupied a cell can get).
    /// @}

    /// @name Signed Euclidean distance field (ESDF)
    /// The map maintains this incrementally regardless of publishing; these
    /// fields only control whether/how it is published. Leave EsdfTopic
    /// empty to publish nothing.
    /// @{
    std::string EsdfTopic;              ///< Output topic for the ESDF point cloud; empty disables publishing.
    double      EsdfMaxDistance  = 1.0; ///< Truncation distance (m): only this band around a surface is stored/updated.
    double      EsdfPublishRate  = 2.0; ///< ESDF publish rate (Hz); `<=0` falls back to `PublishRate`.
    bool        EsdfPublishSlice = true;///< Publish one horizontal slice (cheap) instead of the full 3-D band.
    double      EsdfSliceHeight  = 0.0; ///< World-frame Z (m) of the published slice, when EsdfPublishSlice is set.
    /// @}

    /**
     * @brief Read and validate all parameters from @p node.
     * @param node Node to declare/read parameters on (e.g. the NanoVoxMapNode itself).
     * @throws std::runtime_error if a required parameter is missing, or if
     *         VoxelResolution, EsdfMaxDistance, or ClearingRayStride are invalid.
     */
    explicit Parameters(rclcpp::Node &node) {
        auto fail = [&](const std::string &msg) {
            RCLCPP_FATAL(node.get_logger(), "%s", msg.c_str());
            throw std::runtime_error(msg);
        };

        auto requireParam = [&](const std::string& name, auto& value) {
            using ParamT = std::decay_t<decltype(value)>;
            try {
                node.declare_parameter<ParamT>(name);
                if (!node.get_parameter(name, value)) {
                    fail("nanovoxmap: required parameter '" + name + "' is missing");
                }
            } catch (const std::exception&) {
                fail("nanovoxmap: required parameter '" + name + "' is missing");
            }
        };

        requireParam("CloudTopic",      cloudTopic);
        requireParam("OdometryTopic",   OdometryTopic);
        requireParam("OutputTopic",     OutputTopic);
        requireParam("VoxelResolution", VoxelResolution);

        if (!(VoxelResolution > 0.0)) {
            fail("nanovoxmap: VoxelResolution must be > 0, got " + std::to_string(VoxelResolution));
        }

        // Optional parameters
        WorldFrameId    = node.declare_parameter("WorldFrameId",    WorldFrameId);
        PublishRate     = node.declare_parameter("PublishRate",     PublishRate);
        DownsampleCloud = node.declare_parameter("DownsampleCloud", DownsampleCloud);
        SyncQueueSize   = node.declare_parameter("SyncQueueSize",   SyncQueueSize);
        SubQueueSize    = node.declare_parameter("SubQueueSize",    SubQueueSize);
        MaxRayLength    = node.declare_parameter("MaxRayLength",    MaxRayLength);

        DepthCameraInfoTopic = node.declare_parameter("DepthCameraInfoTopic", DepthCameraInfoTopic);
        ClearingRayStride    = node.declare_parameter("ClearingRayStride",    ClearingRayStride);
        DepthCloudOpticalFrame = node.declare_parameter(
            "DepthCloudOpticalFrame", DepthCloudOpticalFrame);
        if (ClearingRayStride < 1) {
            fail("nanovoxmap: ClearingRayStride must be >= 1, got " + std::to_string(ClearingRayStride));
        }

        ProbHit         = node.declare_parameter("ProbHit",         ProbHit);
        ProbMiss        = node.declare_parameter("ProbMiss",        ProbMiss);
        ProbMin         = node.declare_parameter("ProbMin",         ProbMin);
        ProbMax         = node.declare_parameter("ProbMax",         ProbMax);

        EsdfTopic        = node.declare_parameter("EsdfTopic",        EsdfTopic);
        EsdfMaxDistance  = node.declare_parameter("EsdfMaxDistance",  EsdfMaxDistance);
        EsdfPublishRate  = node.declare_parameter("EsdfPublishRate",  EsdfPublishRate);
        EsdfPublishSlice = node.declare_parameter("EsdfPublishSlice", EsdfPublishSlice);
        EsdfSliceHeight  = node.declare_parameter("EsdfSliceHeight",  EsdfSliceHeight);
        if (!(EsdfMaxDistance > 0.0)) {
            fail("nanovoxmap: EsdfMaxDistance must be > 0, got " + std::to_string(EsdfMaxDistance));
        }
    }
};


/**
 * @class NanoVoxMapNode
 * @brief ROS node wiring a synchronized point cloud + odometry stream into a NanoVoxMap::OccupancyMap.
 *
 * Subscribes to the configured cloud/odometry topics via an
 * approximate-time message_filters::Synchronizer, transforms each scan into
 * the world frame using the odometry pose directly (the odometry topic is
 * expected to already report the point cloud sensor's pose in the world
 * frame - no separate sensor->base calibration is applied), and raycasts
 * it into an internal OccupancyMap on the subscriber thread
 * (SynchronizedCallback()). A separate rclcpp::TimerBase (publishTimerCallback())
 * publishes the occupied voxel set at a fixed rate, decoupled from sensor
 * rate; map_mutex_ guards all access to map_ shared between the two
 * callbacks.
 */
class NanoVoxMapNode : public rclcpp::Node {
    public:
        /**
         * @brief Construct the node: configure the map, publisher, subscribers, sync, and publish timer.
         * @param options Node options, forwarded to rclcpp::Node.
         */
        explicit NanoVoxMapNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
        ~NanoVoxMapNode(){};

    private:
        /**
         * @brief Synchronized callback: integrates one (cloud, odom) pair into the map.
         *
         * Hot path - converts the cloud to Eigen points, optionally
         * voxel-downsamples it, transforms it to world frame (clipping rays
         * longer than `MaxRayLength`), then raycasts the whole batch into
         * map_ under map_mutex_. Malformed point clouds are caught and the
         * frame is dropped (rate-limited `RCLCPP_ERROR`) rather than crashing
         * the node.
         * @param CloudMsg Sensor-frame point cloud.
         * @param OdomMsg  World-frame pose of the point cloud sensor at the same time.
         */
        void SynchronizedCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& CloudMsg,
                                  const nav_msgs::msg::Odometry::ConstSharedPtr& OdomMsg);

        /**
         * @brief Caches the depth sensor's intrinsics for no-return frustum clearing.
         *
         * Only subscribed when `DepthCameraInfoTopic` is set. Guarded by
         * camera_info_mutex_ since it runs on a different callback than
         * SynchronizedCallback(), which reads the cached intrinsics.
         * @param msg Camera intrinsics for the depth sensor.
         */
        void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg);

        /**
         * @brief Timer callback: publishes the current occupied voxel set as a PointCloud2.
         *
         * Skips all work if there are no subscribers. Holds map_mutex_ only
         * long enough to snapshot the occupied-index set (OccupancyMap::snapshotOccupied()),
         * then builds and publishes the message outside the lock so publish
         * latency never blocks SynchronizedCallback().
         */
        void publishTimerCallback();

        /**
         * @brief Timer callback: publishes the signed ESDF as a PointCloud2 (intensity = distance, m).
         *
         * Only created/active when `EsdfTopic` is set. Holds map_mutex_ for
         * OccupancyMap::forEachEsdfVoxel(), which brings the incremental block
         * ESDF up to date and visits its materialized voxels; the message is
         * then built and published outside the lock. When `EsdfPublishSlice`
         * is set, only voxels within half a resolution of `EsdfSliceHeight`
         * are kept, otherwise the full truncated band is published. No-op if
         * there are no subscribers.
         */
        void publishEsdfTimerCallback();

        Parameters params_;          ///< Validated node configuration.
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;  ///< Publishes the occupied voxel set (params_.OutputTopic).
        rclcpp::TimerBase::SharedPtr publish_timer_;    ///< Drives publishTimerCallback() at params_.PublishRate.
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr esdf_pub_;  ///< Publishes the ESDF point cloud (params_.EsdfTopic); null unless EsdfTopic is set.
        rclcpp::TimerBase::SharedPtr esdf_timer_;       ///< Drives publishEsdfTimerCallback(); null unless EsdfTopic is set.

        /// @name Message filter subscribers and synchronizer
        /// @{
        message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_sub_;
        message_filters::Subscriber<nav_msgs::msg::Odometry>       odom_sub_;

        typedef message_filters::sync_policies::ApproximateTime<
            sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry> MySyncPolicy;
        typedef message_filters::Synchronizer<MySyncPolicy> Synchronizer;
        std::unique_ptr<Synchronizer> sync_;
        /// @}

        /// @name No-return frustum clearing (optional)
        /// camera_info_mutex_ guards the cached intrinsics, which are written
        /// by cameraInfoCallback() and read by SynchronizedCallback() - two
        /// different callbacks, potentially on different executor threads.
        /// @{
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
        std::mutex camera_info_mutex_;
        bool     camera_info_valid_ = false;
        double   cam_fx_ = 0.0, cam_fy_ = 0.0, cam_cx_ = 0.0, cam_cy_ = 0.0;
        uint32_t cam_width_ = 0, cam_height_ = 0;
        /// @}

        /// @name Map state, protected by map_mutex_
        /// @{
        std::mutex map_mutex_;
        std::mutex sensor_mutex_;  ///< Serializes shared per-scan scratch under the multithreaded executor.
        std::unique_ptr<OccupancyMap<double>> map_;
        /// @}

        /// @name Scratch buffers reused across callbacks
        /// SynchronizedCallback() is the sole writer, so reuse across calls
        /// avoids per-frame allocation without needing extra locking.
        /// @{
        std::vector<Eigen::Vector3d> inputCloud_;
        std::vector<Eigen::Vector3d> filteredCloud_;
        std::vector<Eigen::Vector3d> worldPoints_;
        std::vector<Eigen::Vector3d> clearingPoints_;  ///< World-frame no-return ray endpoints for this frame.
        /// @}

        rclcpp::Time last_msg_stamp_;  ///< Stamp of the most recently integrated cloud; used as the published map's header stamp.
};

} // namespace NanoVoxMap

#endif
