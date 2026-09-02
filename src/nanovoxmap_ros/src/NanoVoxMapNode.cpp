/**
 * @file NanoVoxMapNode.cpp
 * @brief Implementation of NanoVoxMapNode (see NanoVoxMapNode.hpp for the documented API).
 */
#include "NanoVoxMapNode.hpp"

namespace NanoVoxMap {

NanoVoxMapNode::NanoVoxMapNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("nanovoxmap_node", options), params_(*this) {

    // ---- Map (unbounded sparse block-hash) ---------------------------------
    map_ = std::make_unique<OccupancyMap<double>>(params_.VoxelResolution,
                                                   params_.ProbHit, params_.ProbMiss,
                                                   params_.ProbMin, params_.ProbMax);
    // Governs both storage/update cost of the incremental ESDF and query
    // results, whether or not it's published, so this is set unconditionally.
    map_->setEsdfMaxDistance(params_.EsdfMaxDistance);

    // ---- Publisher --------------------------------------------------------
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(params_.OutputTopic, 1);

    // ---- Subscribers --------------------------------------------------------
    const auto sub_qos = rclcpp::QoS(rclcpp::KeepLast(params_.SubQueueSize)).get_rmw_qos_profile();
    cloud_sub_.subscribe(this, params_.cloudTopic, sub_qos);
    odom_sub_.subscribe(this, params_.OdometryTopic, sub_qos);

    // ---- Approximate-time synchronizer ------------------------------------
    sync_ = std::make_unique<Synchronizer>(
        MySyncPolicy(params_.SyncQueueSize), cloud_sub_, odom_sub_);

    sync_->registerCallback(
        std::bind(&NanoVoxMapNode::SynchronizedCallback, this, std::placeholders::_1, std::placeholders::_2));

    // ---- Decoupled publishing timer ---------------------------------------
    if (params_.PublishRate > 0.0) {
        publish_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / params_.PublishRate),
            std::bind(&NanoVoxMapNode::publishTimerCallback, this));
    }

    // ---- Optional no-return frustum-clearing subscriber --------------------
    if (!params_.DepthCameraInfoTopic.empty()) {
        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            params_.DepthCameraInfoTopic, rclcpp::SensorDataQoS(),
            std::bind(&NanoVoxMapNode::cameraInfoCallback, this, std::placeholders::_1));
    }

    // ---- Optional ESDF publisher -------------------------------------------
    if (!params_.EsdfTopic.empty()) {
        esdf_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(params_.EsdfTopic, 1);

        const double esdf_rate = params_.EsdfPublishRate > 0.0 ? params_.EsdfPublishRate
                                                               : params_.PublishRate;
        if (esdf_rate > 0.0) {
            esdf_timer_ = this->create_wall_timer(
                std::chrono::duration<double>(1.0 / esdf_rate),
                std::bind(&NanoVoxMapNode::publishEsdfTimerCallback, this));
        }
    }

    RCLCPP_INFO(this->get_logger(), "nanovoxmap initialized.");
    RCLCPP_INFO(this->get_logger(), "  cloud topic   : %s", params_.cloudTopic.c_str());
    RCLCPP_INFO(this->get_logger(), "  odom topic    : %s", params_.OdometryTopic.c_str());
    RCLCPP_INFO(this->get_logger(), "  output topic  : %s", params_.OutputTopic.c_str());
    RCLCPP_INFO(this->get_logger(), "  world frame   : %s", params_.WorldFrameId.c_str());
    RCLCPP_INFO(this->get_logger(), "  resolution    : %.3f m", params_.VoxelResolution);
    RCLCPP_INFO(this->get_logger(), "  publish rate  : %.1f Hz", params_.PublishRate);
    RCLCPP_INFO(this->get_logger(), "  downsample    : %s", params_.DownsampleCloud ? "on" : "off");
    RCLCPP_INFO(this->get_logger(), "  max ray length: %.2f m", params_.MaxRayLength);
    RCLCPP_INFO(this->get_logger(), "  log-odds      : hit=%.2f miss=%.2f min=%.4f max=%.4f",
                params_.ProbHit, params_.ProbMiss, params_.ProbMin, params_.ProbMax);
    if (!params_.DepthCameraInfoTopic.empty()) {
        RCLCPP_INFO(this->get_logger(), "  no-return clear: on, camera_info=%s stride=%d",
                    params_.DepthCameraInfoTopic.c_str(), params_.ClearingRayStride);
    } else {
        RCLCPP_INFO(this->get_logger(), "  no-return clear: off");
    }
    if (!params_.EsdfTopic.empty()) {
        RCLCPP_INFO(this->get_logger(), "  esdf          : on, topic=%s max_dist=%.2fm slice=%s z=%.2fm rate=%.1fHz",
                    params_.EsdfTopic.c_str(), params_.EsdfMaxDistance,
                    params_.EsdfPublishSlice ? "on" : "off", params_.EsdfSliceHeight,
                    params_.EsdfPublishRate > 0.0 ? params_.EsdfPublishRate : params_.PublishRate);
    } else {
        RCLCPP_INFO(this->get_logger(), "  esdf          : off (publishing)");
    }
}

void NanoVoxMapNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &msg) {
    if (!(msg->k[0] > 0.0) || !(msg->k[4] > 0.0)) return;  // degenerate intrinsics
    std::lock_guard<std::mutex> lock(camera_info_mutex_);
    cam_fx_ = msg->k[0];
    cam_fy_ = msg->k[4];
    cam_cx_ = msg->k[2];
    cam_cy_ = msg->k[5];
    cam_width_  = msg->width;
    cam_height_ = msg->height;
    camera_info_valid_ = true;
}

// ---------------------------------------------------------------------------
// Sensor callback: integrates a single (cloud, odom) pair into the map.
// Hot path - keep allocations and Eigen temporaries out of the inner loop.
// ---------------------------------------------------------------------------
void NanoVoxMapNode::SynchronizedCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &CloudMsg,
    const nav_msgs::msg::Odometry::ConstSharedPtr       &OdomMsg) {

    std::lock_guard<std::mutex> sensor_lock(sensor_mutex_);

    // ---- 1) Pose: sensor -> world (OdomMsg already reports the cloud
    //         sensor's pose directly in the world frame; no separate
    //         sensor->base calibration to compose here). ------------------
    const Eigen::Vector3d pose(OdomMsg->pose.pose.position.x,
                               OdomMsg->pose.pose.position.y,
                               OdomMsg->pose.pose.position.z);
    Eigen::Quaterniond q(OdomMsg->pose.pose.orientation.w,
                        OdomMsg->pose.pose.orientation.x,
                        OdomMsg->pose.pose.orientation.y,
                        OdomMsg->pose.pose.orientation.z);
    q.normalize();

    // Cache R once - applying R*v is much cheaper than q*v inside the loop,
    // and we avoid recomputing the rotation matrix per point.
    const Eigen::Matrix3d R = q.toRotationMatrix();

    // ---- 2) Convert PointCloud2 -> std::vector<Eigen::Vector3d> -----------
    // A malformed/unexpected message (missing x/y/z fields, wrong datatype,
    // truncated buffer) throws rather than reading out of bounds; caught
    // here so one bad message drops a frame instead of killing the node.
    inputCloud_.clear();
    try {
        EigenPointCloudConversions::PointCloud2ToVector<double>(*CloudMsg, inputCloud_);
    } catch (const std::exception& e) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                              "nanovoxmap: dropping malformed point cloud: %s", e.what());
        return;
    }

    // ---- 3) (Optional) voxel downsample at map resolution -----------------
    // Many returns land in the same voxel; deduplicating here removes
    // redundant rays without changing the map outcome.
    const std::vector<Eigen::Vector3d>* cloud_ptr = &inputCloud_;
    if (params_.DownsampleCloud && !inputCloud_.empty()) {
        filteredCloud_.clear();
        filteredCloud_.reserve(inputCloud_.size());

        // Open-addressed set of voxel keys; reserve to avoid rehashing.
        std::unordered_set<int64_t> seen;
        seen.reserve(inputCloud_.size() * 2);

        const double inv_res = 1.0 / params_.VoxelResolution;
        for (const auto &p : inputCloud_) {
            // Filter non-finite returns (NaN/Inf) that some drivers emit.
            if (!p.allFinite()) continue;

            constexpr int64_t kBias = int64_t(1) << 20;
            const int64_t kx = static_cast<int64_t>(std::floor(p.x() * inv_res));
            const int64_t ky = static_cast<int64_t>(std::floor(p.y() * inv_res));
            const int64_t kz = static_cast<int64_t>(std::floor(p.z() * inv_res));
            // Exact 21-bit-per-axis key. Points outside the representable
            // sensor-frame range are retained rather than aliased.
            if (std::abs(kx) >= kBias || std::abs(ky) >= kBias || std::abs(kz) >= kBias) {
                filteredCloud_.push_back(p);
                continue;
            }
            const int64_t key = (kx + kBias) |
                                ((ky + kBias) << 21) |
                                ((kz + kBias) << 42);
            if (seen.insert(key).second) filteredCloud_.push_back(p);
        }
        cloud_ptr = &filteredCloud_;
    }

    // ---- 4) Transform (sensor-frame) points to world frame, clipping
    //         long returns, then raycast the whole batch in one call so it
    //         can run as a single parallel pass (CPU SoA or CUDA kernel -
    //         see OccupancyMap::insertPointCloud).
    const double max_len_sq = params_.MaxRayLength > 0.0
                              ? params_.MaxRayLength * params_.MaxRayLength
                              : std::numeric_limits<double>::infinity();
    worldPoints_.clear();
    clearingPoints_.clear();
    worldPoints_.reserve(cloud_ptr->size());
    clearingPoints_.reserve(cloud_ptr->size() / 8);
    for (const auto &point : *cloud_ptr) {
        if (!point.allFinite()) continue;
        const double len_sq = point.squaredNorm();
        if (len_sq > max_len_sq) {
            // A finite return beyond the integration range still proves that
            // the beam is free up to MaxRayLength.  Treat it as a clipped
            // miss-only ray instead of dropping it; otherwise an old occupied
            // voxel on that beam can remain forever when the newly visible
            // background lies beyond MaxRayLength.
            clearingPoints_.push_back(
                R * (point * (params_.MaxRayLength / std::sqrt(len_sq))) + pose);
            continue;
        }
        worldPoints_.push_back(R * point + pose);
    }

    // ---- 5) (Optional) no-return frustum clearing --------------------------
    // A depth pixel with no return means its beam reached MaxRayLength without
    // hitting anything, so the whole ray out to MaxRayLength is free. This
    // reads inputCloud_ (not the downsampled/filtered cloud_ptr) because the
    // raw buffer keeps its pixel-grid layout: element (v*width+u) is pixel
    // (u,v), which is what lets a no-return element be paired with a ray
    // direction from the camera intrinsics.
    if (!params_.DepthCameraInfoTopic.empty() && params_.MaxRayLength > 0.0 &&
        CloudMsg->height > 1) {
        double fx = 0.0, fy = 0.0, cx = 0.0, cy = 0.0;
        bool have_intrinsics = false;
        {
            std::lock_guard<std::mutex> lock(camera_info_mutex_);
            have_intrinsics = camera_info_valid_ &&
                              cam_width_ == CloudMsg->width && cam_height_ == CloudMsg->height;
            fx = cam_fx_; fy = cam_fy_; cx = cam_cx_; cy = cam_cy_;
        }
        if (have_intrinsics) {
            const uint32_t width  = CloudMsg->width;
            const uint32_t height = CloudMsg->height;
            const uint32_t stride = static_cast<uint32_t>(params_.ClearingRayStride);
            clearingPoints_.reserve((height / stride + 1) * (width / stride + 1));
            for (uint32_t v = 0; v < height; v += stride) {
                const size_t row = static_cast<size_t>(v) * width;
                for (uint32_t u = 0; u < width; u += stride) {
                    if (inputCloud_[row + u].allFinite()) continue;  // has a real return
                    // gz-sensors publishes RGB-D points in the Gazebo camera
                    // frame (+X forward, +Y left, +Z up).  Merely setting
                    // gz_frame_id to a name ending in "optical_link" does not
                    // rotate the samples into ROS optical convention
                    // (+Z forward, +X right, +Y down).  Keep these synthetic
                    // no-return rays in the same frame as the real points;
                    // otherwise they clear above/behind the camera and ghosts
                    // in the visible frustum survive until the robot moves.
                    Eigen::Vector3d dir;
                    if (params_.DepthCloudOpticalFrame) {
                        dir = Eigen::Vector3d((static_cast<double>(u) - cx) / fx,
                                              (static_cast<double>(v) - cy) / fy, 1.0);
                    } else {
                        dir = Eigen::Vector3d(1.0,
                                              -(static_cast<double>(u) - cx) / fx,
                                              -(static_cast<double>(v) - cy) / fy);
                    }
                    dir.normalize();
                    clearingPoints_.push_back(R * (dir * params_.MaxRayLength) + pose);
                }
            }
        }
    }

    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        map_->insertPointCloud(pose, worldPoints_);
        if (!clearingPoints_.empty()) {
            map_->insertPointCloud(pose, clearingPoints_, /*apply_hits=*/false);
        }
        last_msg_stamp_ = CloudMsg->header.stamp;
    }
}

// ---------------------------------------------------------------------------
// Timer-driven publisher: publishes the occupied set without holding the map
// mutex during the (allocating, serialising) heavy work. The mutex is held
// only long enough to snapshot the occupied-index set, which is the minimum
// required to keep the sensor callback latency bounded.
// ---------------------------------------------------------------------------
void NanoVoxMapNode::publishTimerCallback() {

    if (pub_->get_subscription_count() == 0) return;  // skip work if nobody listens

    // ---- 1) Take a fast snapshot of the occupied indices under the lock --
    std::vector<int64_t> snapshot;
    Eigen::Vector3d       lower_bound;
    double                resolution;
    int64_t               size_x, size_xy;
    rclcpp::Time          stamp;
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        map_->snapshotOccupied(snapshot, lower_bound, resolution, size_x, size_xy);
        stamp = last_msg_stamp_;
    }
    if (stamp.nanoseconds() == 0) stamp = this->get_clock()->now();

    // ---- 2) Build the PointCloud2 OUTSIDE the lock -----------------------
    // Pre-size everything; avoid sensor_msgs::PointCloud round-trip which
    // allocates twice. We write floats directly into the PointCloud2 byte
    // buffer.
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp    = stamp;
    msg.header.frame_id = params_.WorldFrameId;
    msg.height          = 1;
    msg.width           = static_cast<uint32_t>(snapshot.size());
    msg.is_dense        = true;
    msg.is_bigendian    = false;
    msg.point_step      = 12;                 // 3 * float32
    msg.row_step        = msg.point_step * msg.width;
    msg.fields.resize(3);
    msg.fields[0].name = "x"; msg.fields[0].offset = 0;
    msg.fields[1].name = "y"; msg.fields[1].offset = 4;
    msg.fields[2].name = "z"; msg.fields[2].offset = 8;
    for (int i = 0; i < 3; ++i) {
        msg.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
        msg.fields[i].count    = 1;
    }
    msg.data.resize(static_cast<size_t>(msg.row_step));

    float* out = reinterpret_cast<float*>(msg.data.data());
    const float half_res = static_cast<float>(0.5 * resolution);
    const float lx = static_cast<float>(lower_bound.x());
    const float ly = static_cast<float>(lower_bound.y());
    const float lz = static_cast<float>(lower_bound.z());
    const float res_f = static_cast<float>(resolution);

    for (size_t i = 0; i < snapshot.size(); ++i) {
        const int64_t f   = snapshot[i];
        const int64_t z   = f / size_xy;
        const int64_t rem = f - z * size_xy;
        const int64_t y   = rem / size_x;
        const int64_t x   = rem - y * size_x;
        out[3 * i + 0] = lx + (static_cast<float>(x) * res_f) + half_res;
        out[3 * i + 1] = ly + (static_cast<float>(y) * res_f) + half_res;
        out[3 * i + 2] = lz + (static_cast<float>(z) * res_f) + half_res;
    }

    pub_->publish(msg);
}

// ---------------------------------------------------------------------------
// Signed ESDF publisher: intensity carries the signed distance (m) - positive
// in free space (out to the surface), negative inside obstacles.
// ---------------------------------------------------------------------------
void NanoVoxMapNode::publishEsdfTimerCallback() {

    if (esdf_pub_->get_subscription_count() == 0) return;  // skip work if nobody listens

    // ---- 1) Bring the ESDF up to date and collect its voxels, under the lock -
    // forEachEsdfVoxel() internally calls updateEsdf() (incremental - only
    // blocks touched since the last update are recomputed) before visiting
    // the materialized band, so this stays cheap on frames with no new hits.
    std::vector<Eigen::Vector3d> points;
    std::vector<double>          dists;
    rclcpp::Time                 stamp;
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        const bool   slice    = params_.EsdfPublishSlice;
        const double slice_z  = params_.EsdfSliceHeight;
        const double half_res = 0.5 * params_.VoxelResolution;
        map_->forEachEsdfVoxel([&](const Eigen::Vector3d &center, double dist) {
            if (slice && std::abs(center.z() - slice_z) > half_res) return;
            points.push_back(center);
            dists.push_back(dist);
        });
        stamp = last_msg_stamp_;
    }
    if (stamp.nanoseconds() == 0) stamp = this->get_clock()->now();

    // ---- 2) Build the PointCloud2 OUTSIDE the lock -------------------------
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp    = stamp;
    msg.header.frame_id = params_.WorldFrameId;
    msg.height          = 1;
    msg.width           = static_cast<uint32_t>(points.size());
    msg.is_dense        = true;
    msg.is_bigendian    = false;
    msg.point_step      = 16;                 // 4 * float32 (x, y, z, intensity)
    msg.row_step        = msg.point_step * msg.width;
    msg.fields.resize(4);
    msg.fields[0].name = "x"; msg.fields[0].offset = 0;
    msg.fields[1].name = "y"; msg.fields[1].offset = 4;
    msg.fields[2].name = "z"; msg.fields[2].offset = 8;
    msg.fields[3].name = "intensity"; msg.fields[3].offset = 12;
    for (int i = 0; i < 4; ++i) {
        msg.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
        msg.fields[i].count    = 1;
    }
    msg.data.resize(static_cast<size_t>(msg.row_step));

    float* out = reinterpret_cast<float*>(msg.data.data());
    for (size_t i = 0; i < points.size(); ++i) {
        out[4 * i + 0] = static_cast<float>(points[i].x());
        out[4 * i + 1] = static_cast<float>(points[i].y());
        out[4 * i + 2] = static_cast<float>(points[i].z());
        out[4 * i + 3] = static_cast<float>(dists[i]);
    }

    esdf_pub_->publish(msg);
}

} // namespace NanoVoxMap
