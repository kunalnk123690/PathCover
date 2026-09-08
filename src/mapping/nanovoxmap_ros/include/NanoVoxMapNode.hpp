/**
 * @file NanoVoxMapNode.hpp
 * @brief ROS node that builds a NanoVoxMap::OccupancyMap from a synchronized
 *        point cloud + odometry stream and publishes the occupied voxel set.
 */

#ifndef NanoVoxMapNode_H
#define NanoVoxMapNode_H

#include <atomic>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
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
 * @brief Reads and validates nanovoxmap_node's ROS parameters from a private node handle.
 *
 * Required parameters throw `std::runtime_error` (after an `ROS_FATAL_STREAM`)
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
    std::string WorldFrameId = "world";  ///< `frame_id` stamped on every published cloud.
    double PublishRate    = 5.0;   ///< Publish rate (Hz); decouples publish from sensor rate. `<=0` disables the timer.
    bool   DownsampleCloud = true; ///< Voxel-downsample the input cloud at map resolution before raycasting.
    int    SyncQueueSize   = 20;   ///< Approximate-time synchronizer queue size.
    int    SubQueueSize    = 5;    ///< Per-topic subscriber queue size.
    double MaxRayLength    = 30.0; ///< Clip returns farther than this from the sensor, in metres; `<=0` disables.

    /// @name No-return frustum clearing (organized depth clouds)
    /// A depth pixel that returns nothing means its beam left the sensor and
    /// hit nothing in range, so the whole beam out to MaxRayLength is free.
    /// Reconstructing those rays from the camera intrinsics lets the map sweep
    /// out ghost voxels in open air, where surface returns never send a ray.
    /// @{
    std::string DepthCameraInfoTopic;   ///< CameraInfo topic for the depth cloud; empty disables frustum clearing.
    int ClearingRayStride = 4;          ///< Subsample factor for no-return clearing rays (1 = every pixel).
    bool DepthCloudOpticalFrame = true; ///< true: ROS optical +Z forward; false: Gazebo +X forward.
    /// @}

    /// @name Signed Euclidean distance field (ESDF)
    /// The map maintains a signed distance field incrementally alongside
    /// occupancy; these parameters only control whether and how it is
    /// *published*. Querying it from C++ works regardless (see
    /// OccupancyMap::getSignedDistanceWithGradient()).
    /// @{
    std::string EsdfTopic;            ///< Output ESDF topic; empty disables ESDF publishing entirely.
    double EsdfMaxDistance = 1.0;     ///< ESDF truncation distance, in metres. Bounds both memory and update cost.
    double EsdfPublishRate = 2.0;     ///< ESDF publish rate (Hz); `<=0` disables the timer.
    bool   EsdfPublishSlice = true;   ///< Publish one horizontal slice (cheap, RViz-friendly) instead of the whole band.
    double EsdfSliceHeight = 0.0;     ///< World z of that slice, in metres.
    /// @}
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

    /**
     * @brief Read and validate all parameters from @p nh_priv.
     * @param nh_priv Private node handle (e.g. `ros::NodeHandle("~")`) to read parameters from.
     * @throws std::runtime_error if a required parameter is missing, or if
     *         VoxelResolution is invalid.
     */
    Parameters(const ros::NodeHandle &nh_priv) {
        auto fail = [](const std::string& msg) {
            ROS_FATAL_STREAM(msg);
            throw std::runtime_error(msg);
        };
        auto requireParam = [&](const std::string& name, auto& value) {
            if (!nh_priv.getParam(name, value)) {
                fail("nanovoxmap: required parameter '" + name + "' is missing");
            }
        };
        auto requirePositive = [&](const char* name, double value) {
            if (!(value > 0.0)) {
                fail(std::string("nanovoxmap: ") + name + " must be > 0, got " +
                     std::to_string(value));
            }
        };

        requireParam("CloudTopic",      cloudTopic);
        requireParam("OdometryTopic",   OdometryTopic);
        requireParam("OutputTopic",     OutputTopic);
        requireParam("VoxelResolution", VoxelResolution);
        requirePositive("VoxelResolution", VoxelResolution);

        // Optional parameters
        nh_priv.param("WorldFrameId",     WorldFrameId,     WorldFrameId);
        nh_priv.param("PublishRate",      PublishRate,      PublishRate);
        nh_priv.param("DownsampleCloud",  DownsampleCloud,  DownsampleCloud);
        nh_priv.param("SyncQueueSize",    SyncQueueSize,    SyncQueueSize);
        nh_priv.param("SubQueueSize",     SubQueueSize,     SubQueueSize);
        nh_priv.param("MaxRayLength",     MaxRayLength,     MaxRayLength);
        nh_priv.param("DepthCameraInfoTopic", DepthCameraInfoTopic, std::string(""));
        nh_priv.param("ClearingRayStride",    ClearingRayStride,    ClearingRayStride);
        nh_priv.param("DepthCloudOpticalFrame", DepthCloudOpticalFrame, DepthCloudOpticalFrame);
        nh_priv.param("ProbHit",          ProbHit,          ProbHit);
        nh_priv.param("ProbMiss",         ProbMiss,         ProbMiss);
        nh_priv.param("ProbMin",          ProbMin,          ProbMin);
        nh_priv.param("ProbMax",          ProbMax,          ProbMax);
        nh_priv.param("EsdfTopic",        EsdfTopic,        std::string(""));
        nh_priv.param("EsdfMaxDistance",  EsdfMaxDistance,  EsdfMaxDistance);
        nh_priv.param("EsdfPublishRate",  EsdfPublishRate,  EsdfPublishRate);
        nh_priv.param("EsdfPublishSlice", EsdfPublishSlice, EsdfPublishSlice);
        nh_priv.param("EsdfSliceHeight",  EsdfSliceHeight,  EsdfSliceHeight);

        requirePositive("EsdfMaxDistance", EsdfMaxDistance);
        // Queue sizes reach roscpp as uint32_t, so a negative value here would
        // silently become a multi-billion-message queue rather than an error.
        if (SyncQueueSize <= 0) fail("nanovoxmap: SyncQueueSize must be >= 1");
        if (SubQueueSize <= 0) fail("nanovoxmap: SubQueueSize must be >= 1");
        if (ClearingRayStride <= 0) fail("nanovoxmap: ClearingRayStride must be >= 1");
        if (WorldFrameId.empty()) fail("nanovoxmap: WorldFrameId must not be empty");
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
 * (SynchronizedCallback()). A separate ros::Timer (publishTimerCallback())
 * publishes the occupied voxel set at a fixed rate, decoupled from sensor
 * rate; map_mutex_ guards all access to map_ shared between the two
 * callbacks.
 */
class NanoVoxMapNode {
    public:
        /**
         * @brief Construct the node: configure the map, publisher, subscribers, sync, and publish timer.
         * @param conf Validated parameters (see Parameters), typically constructed from `ros::NodeHandle("~")`.
         */
        NanoVoxMapNode(const Parameters &conf);
        ~NanoVoxMapNode(){};

    private:
        /**
         * @brief Synchronized callback: integrates one (cloud, odom) pair into the map.
         *
         * Hot path - converts the cloud to Eigen points, optionally
         * voxel-downsamples it, transforms it to world frame (clipping rays
         * longer than `MaxRayLength`), then raycasts the whole batch into
         * map_ under map_mutex_. Malformed point clouds are caught and the
         * frame is dropped (rate-limited `ROS_ERROR`) rather than crashing
         * the node.
         * @param CloudMsg Sensor-frame point cloud.
         * @param OdomMsg  World-frame pose of the point cloud sensor at the same time.
         */
        void SynchronizedCallback(const sensor_msgs::PointCloud2::ConstPtr& CloudMsg,
                                  const nav_msgs::Odometry::ConstPtr& OdomMsg);

        /**
         * @brief Timer callback: publishes the current occupied voxel set as a PointCloud2.
         *
         * Skips all work if there are no subscribers. Holds map_mutex_ only
         * long enough to snapshot the occupied-index set (OccupancyMap::snapshotOccupied()),
         * then builds and publishes the message outside the lock so publish
         * latency never blocks SynchronizedCallback().
         */
        void publishTimerCallback(const ros::TimerEvent&);

        /**
         * @brief Timer callback: brings the signed ESDF up to date and publishes it.
         *
         * Separate from publishTimerCallback() because the two have different
         * costs and useful rates: the occupied set is a cheap snapshot, while
         * the ESDF update is the expensive part and is deliberately kept off
         * the sensor thread. The incremental update runs here under
         * map_mutex_, so integration latency in SynchronizedCallback() is
         * unaffected by field maintenance.
         *
         * Published as a `sensor_msgs/PointCloud2` with an `intensity` field
         * carrying the **signed** distance in metres (negative inside
         * obstacles), which RViz can colour directly.
         */
        void esdfTimerCallback(const ros::TimerEvent&);

        /**
         * @brief CameraInfo callback: caches depth intrinsics and builds the
         *        per-pixel unit-ray table used for no-return frustum clearing.
         *
         * Intrinsics are static, so the table is built once on the first valid
         * message and then published to the sensor thread via a release store
         * on intrinsics_ready_; subsequent messages are ignored.
         */
        void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& info);

        Parameters params_;          ///< Validated node configuration.
        ros::NodeHandle n_;           ///< Public node handle used for the publisher/subscribers/timer.
        ros::Publisher pub_;          ///< Publishes the occupied voxel set (params_.OutputTopic).
        ros::Publisher esdf_pub_;     ///< Publishes the signed distance field (params_.EsdfTopic).
        ros::Timer publish_timer_;    ///< Drives publishTimerCallback() at params_.PublishRate.
        ros::Timer esdf_timer_;       ///< Drives esdfTimerCallback() at params_.EsdfPublishRate.

        /// @name Message filter subscribers and synchronizer
        /// @{
        message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_;
        message_filters::Subscriber<nav_msgs::Odometry>       odom_sub_;

        typedef message_filters::sync_policies::ApproximateTime<
            sensor_msgs::PointCloud2, nav_msgs::Odometry> MySyncPolicy;
        typedef message_filters::Synchronizer<MySyncPolicy> Synchronizer;
        std::unique_ptr<Synchronizer> sync_;
        ros::Subscriber caminfo_sub_;  ///< Depth CameraInfo subscription (enables no-return frustum clearing).
        /// @}

        /// @name Depth intrinsics for no-return frustum clearing
        /// Built once by cameraInfoCallback(); read lock-free by the sensor
        /// thread, guarded by the release/acquire ordering on intrinsics_ready_.
        /// intrinsics_claimed_ is the write side of that handshake: the node
        /// spins a multi-threaded spinner, so two CameraInfo messages can be
        /// dispatched at once and exactly one of them must get to build the
        /// table.
        /// @{
        std::atomic<bool> intrinsics_claimed_{false};
        std::atomic<bool> intrinsics_ready_{false};
        uint32_t cam_width_ = 0;
        uint32_t cam_height_ = 0;
        std::vector<Eigen::Vector3d> ray_table_;  ///< Per-pixel unit ray in the depth optical frame.
        /// @}

        /// @name Map state, protected by map_mutex_
        /// @{
        std::mutex map_mutex_;
        std::unique_ptr<OccupancyMap<double>> map_;
        ros::Time last_msg_stamp_;  ///< Stamp of the most recently integrated cloud; used as the published map's header stamp.
        /// @}

        /// @name Scratch buffers reused across callbacks, protected by sensor_mutex_
        /// A ros::MultiThreadedSpinner will dispatch two synchronized (cloud,
        /// odom) pairs concurrently, so "one writer" holds only if the callback
        /// body is serialized. sensor_mutex_ does that; it is held for the whole
        /// callback and is separate from map_mutex_ so the publish timers still
        /// only ever contend on the map itself.
        /// @{
        std::mutex sensor_mutex_;
        std::vector<Eigen::Vector3d> inputCloud_;
        std::vector<Eigen::Vector3d> filteredCloud_;
        std::vector<Eigen::Vector3d> worldPoints_;
        std::vector<Eigen::Vector3d> clearingPoints_;  ///< Clamped endpoints of over-range beams; carved as miss-only clearing rays.
        std::unordered_set<int64_t> downsampleSeen_;   ///< Voxel keys already emitted by the input downsampler.
        /// @}

        /// Scratch for the ESDF publisher: (world centre, signed distance in m).
        /// esdf_publish_mutex_ serializes the callback for the same reason
        /// sensor_mutex_ does: nothing guarantees a timer callback has returned
        /// before the next tick is dispatched to another spinner thread.
        std::mutex esdf_publish_mutex_;
        std::vector<std::pair<Eigen::Vector3f, float>> esdfSamples_;
};

} // namespace NanoVoxMap

#endif
