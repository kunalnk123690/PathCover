/**
 * @file NanoVoxMapNode.cpp
 * @brief Implementation of NanoVoxMapNode (see NanoVoxMapNode.hpp for the documented API).
 */
#include "NanoVoxMapNode.hpp"

namespace NanoVoxMap {

NanoVoxMapNode::NanoVoxMapNode(const Parameters &conf) : params_(conf) {

    // ---- Map resolution & occupancy model ---------------------------------
    // The map is an unbounded sparse block-hash, so it needs no world bounds:
    // storage grows only where rays are actually observed.
    map_ = std::make_unique<OccupancyMap<double>>(params_.VoxelResolution,
                                                   params_.ProbHit, params_.ProbMiss,
                                                   params_.ProbMin, params_.ProbMax);
    // ESDF truncation: bounds the materialized band and, because a change can
    // only move distances within one band of it, the incremental update cost.
    map_->setEsdfMaxDistance(params_.EsdfMaxDistance);

    // ---- Publisher --------------------------------------------------------
    pub_ = n_.advertise<sensor_msgs::PointCloud2>(params_.OutputTopic, 1);
    if (!params_.EsdfTopic.empty()) {
        esdf_pub_ = n_.advertise<sensor_msgs::PointCloud2>(params_.EsdfTopic, 1);
    }

    // ---- Subscribers (tcp_nodelay reduces per-msg latency tens of ms) -----
    cloud_sub_.subscribe(n_, params_.cloudTopic, params_.SubQueueSize,
                         ros::TransportHints().tcpNoDelay());
    odom_sub_.subscribe(n_, params_.OdometryTopic, params_.SubQueueSize,
                        ros::TransportHints().tcpNoDelay());

    // ---- Approximate-time synchronizer ------------------------------------
    sync_ = std::make_unique<Synchronizer>(
        MySyncPolicy(params_.SyncQueueSize), cloud_sub_, odom_sub_);

    // boost::placeholders:: rather than the global _1/_2, which Boost >= 1.73
    // no longer defines by default.
    sync_->registerCallback(
        boost::bind(&NanoVoxMapNode::SynchronizedCallback, this,
                    boost::placeholders::_1, boost::placeholders::_2));

    // ---- Depth intrinsics (optional; enables no-return frustum clearing) --
    // Not part of the cloud/odom sync: intrinsics are static, so a plain
    // subscriber that latches the first message is enough.
    if (!params_.DepthCameraInfoTopic.empty()) {
        caminfo_sub_ = n_.subscribe(params_.DepthCameraInfoTopic, 1,
                                    &NanoVoxMapNode::cameraInfoCallback, this);
    }

    // ---- Decoupled publishing timers --------------------------------------
    if (params_.PublishRate > 0.0) {
        publish_timer_ = n_.createTimer(
            ros::Duration(1.0 / params_.PublishRate),
            &NanoVoxMapNode::publishTimerCallback, this);
    }
    if (!params_.EsdfTopic.empty() && params_.EsdfPublishRate > 0.0) {
        esdf_timer_ = n_.createTimer(
            ros::Duration(1.0 / params_.EsdfPublishRate),
            &NanoVoxMapNode::esdfTimerCallback, this);
    }

    ROS_INFO("nanovoxmap initialized.");
    ROS_INFO("  cloud topic   : %s", params_.cloudTopic.c_str());
    ROS_INFO("  odom topic    : %s", params_.OdometryTopic.c_str());
    ROS_INFO("  output topic  : %s", params_.OutputTopic.c_str());
    ROS_INFO("  world frame   : %s", params_.WorldFrameId.c_str());
    ROS_INFO("  resolution    : %.3f m", params_.VoxelResolution);
    ROS_INFO("  publish rate  : %.1f Hz", params_.PublishRate);
    ROS_INFO("  downsample    : %s", params_.DownsampleCloud ? "on" : "off");
    ROS_INFO("  max ray length: %.2f m", params_.MaxRayLength);
    ROS_INFO("  frustum clear : %s (stride %d)",
             params_.DepthCameraInfoTopic.empty() ? "off"
                                                   : params_.DepthCameraInfoTopic.c_str(),
             params_.ClearingRayStride);
    ROS_INFO("  log-odds      : hit=%.2f miss=%.2f min=%.4f max=%.4f",
             params_.ProbHit, params_.ProbMiss, params_.ProbMin, params_.ProbMax);
    if (params_.EsdfTopic.empty()) {
        ROS_INFO("  esdf          : not published (library queries still available)");
    } else {
        ROS_INFO("  esdf          : %s @ %.1f Hz, truncation %.2f m, %s, backend %s",
                 params_.EsdfTopic.c_str(), params_.EsdfPublishRate,
                 map_->esdfMaxDistance(),
                 params_.EsdfPublishSlice ? "horizontal slice" : "full band",
                 map_->esdfUsesGpu() ? "cuda (cpu fallback)" : "cpu");
    }
}

// ---------------------------------------------------------------------------
// ESDF timer: runs the incremental distance-field update and publishes the
// result. Deliberately on its own timer rather than in the sensor callback -
// field maintenance is the expensive half, and there is no reason for it to
// sit in the integration latency path.
// ---------------------------------------------------------------------------
void NanoVoxMapNode::esdfTimerCallback(const ros::TimerEvent&) {

    if (esdf_pub_.getNumSubscribers() == 0) return;  // skip work if nobody listens

    // Serialize the callback against itself: a slow update must not have a
    // second spinner thread enter here and rewrite esdfSamples_ underneath it.
    std::lock_guard<std::mutex> publish_lock(esdf_publish_mutex_);

    // ---- 1) Update + collect under the lock ------------------------------
    // forEachEsdfVoxel() brings the field up to date first, so the update
    // itself happens here rather than lazily inside some later query.
    const double half_res = 0.5 * params_.VoxelResolution;
    const bool slice = params_.EsdfPublishSlice;
    const double slice_z = params_.EsdfSliceHeight;
    ros::Time stamp;
    esdfSamples_.clear();
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        map_->forEachEsdfVoxel([&](const Eigen::Vector3d& centre, double distance) {
            if (slice && std::fabs(centre.z() - slice_z) > half_res) return;
            esdfSamples_.emplace_back(centre.cast<float>(), static_cast<float>(distance));
        });
        stamp = last_msg_stamp_;
    }
    if (stamp.isZero()) stamp = ros::Time::now();

    // ---- 2) Serialize outside the lock -----------------------------------
    // x/y/z plus intensity = the SIGNED distance in metres, so RViz's
    // intensity colouring shows obstacle interiors (negative) distinctly from
    // free space, and a downstream consumer gets the real value rather than a
    // magnitude.
    sensor_msgs::PointCloud2 msg;
    msg.header.stamp    = stamp;
    msg.header.frame_id = params_.WorldFrameId;
    msg.height          = 1;
    msg.width           = static_cast<uint32_t>(esdfSamples_.size());
    msg.is_dense        = true;
    msg.is_bigendian    = false;
    msg.point_step      = 16;                 // 4 * float32
    msg.row_step        = msg.point_step * msg.width;
    const size_t esdf_bytes = static_cast<size_t>(msg.point_step) * msg.width;
    msg.fields.resize(4);
    msg.fields[0].name = "x";         msg.fields[0].offset = 0;
    msg.fields[1].name = "y";         msg.fields[1].offset = 4;
    msg.fields[2].name = "z";         msg.fields[2].offset = 8;
    msg.fields[3].name = "intensity"; msg.fields[3].offset = 12;
    for (int i = 0; i < 4; ++i) {
        msg.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
        msg.fields[i].count    = 1;
    }
    msg.data.resize(esdf_bytes);

    float* out = reinterpret_cast<float*>(msg.data.data());
    for (size_t i = 0; i < esdfSamples_.size(); ++i) {
        out[4 * i + 0] = esdfSamples_[i].first.x();
        out[4 * i + 1] = esdfSamples_[i].first.y();
        out[4 * i + 2] = esdfSamples_[i].first.z();
        out[4 * i + 3] = esdfSamples_[i].second;
    }

    esdf_pub_.publish(msg);
}

// ---------------------------------------------------------------------------
// CameraInfo callback: build the per-pixel unit-ray table once. Intrinsics are
// static, so after the first valid message we publish the table to the sensor
// thread with a release store and ignore further messages.
// ---------------------------------------------------------------------------
void NanoVoxMapNode::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& info) {
    if (intrinsics_ready_.load(std::memory_order_acquire)) return;

    const double fx = info->K[0], fy = info->K[4];
    const double cx = info->K[2], cy = info->K[5];
    if (!(fx > 0.0) || !(fy > 0.0) || info->width == 0 || info->height == 0) return;

    // Claim the build before touching anything: under a multi-threaded spinner
    // two CameraInfo messages can reach here at once, and both would otherwise
    // write ray_table_ concurrently. The loser simply drops its message -- the
    // intrinsics are static, so any valid message produces the same table.
    if (intrinsics_claimed_.exchange(true, std::memory_order_acq_rel)) return;

    // Unit ray per pixel in the depth OPTICAL frame (x right, y down, z fwd) -
    // the same frame the organized cloud's points live in, so the sensor
    // thread can transform a reconstructed endpoint exactly like a real return.
    std::vector<Eigen::Vector3d> table(static_cast<size_t>(info->width) * info->height);
    for (uint32_t v = 0; v < info->height; ++v) {
        for (uint32_t u = 0; u < info->width; ++u) {
            const Eigen::Vector3d dir = params_.DepthCloudOpticalFrame
                ? Eigen::Vector3d((static_cast<double>(u) - cx) / fx,
                                  (static_cast<double>(v) - cy) / fy, 1.0)
                : Eigen::Vector3d(1.0, -(static_cast<double>(u) - cx) / fx,
                                  -(static_cast<double>(v) - cy) / fy);
            table[static_cast<size_t>(v) * info->width + u] = dir.normalized();
        }
    }

    cam_width_  = info->width;
    cam_height_ = info->height;
    ray_table_  = std::move(table);
    intrinsics_ready_.store(true, std::memory_order_release);
    ROS_INFO("nanovoxmap: depth intrinsics received (%ux%u); no-return frustum clearing enabled.",
             cam_width_, cam_height_);
}

// ---------------------------------------------------------------------------
// Sensor callback: integrates a single (cloud, odom) pair into the map.
// Hot path - keep allocations and Eigen temporaries out of the inner loop.
// ---------------------------------------------------------------------------
void NanoVoxMapNode::SynchronizedCallback(
    const sensor_msgs::PointCloud2::ConstPtr &CloudMsg,
    const nav_msgs::Odometry::ConstPtr       &OdomMsg) {

    // ---- 0) Serialize the callback against itself -------------------------
    // main() spins a ros::MultiThreadedSpinner, which will dispatch two
    // synchronized pairs to two threads at once. Every scratch buffer below is
    // reused across frames, so without this the second frame clears and
    // reallocates vectors the first is still iterating. This lock is what makes
    // "SynchronizedCallback is the sole writer" actually true; the publish
    // timers are unaffected because they only ever take map_mutex_.
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
        ROS_ERROR_THROTTLE(5.0, "nanovoxmap: dropping malformed point cloud: %s", e.what());
        return;
    }

    // ---- 3) (Optional) voxel downsample at map resolution -----------------
    // Many returns land in the same voxel; deduplicating here removes
    // redundant rays without changing the map outcome.
    const std::vector<Eigen::Vector3d>* cloud_ptr = &inputCloud_;
    if (params_.DownsampleCloud && !inputCloud_.empty()) {
        filteredCloud_.clear();
        filteredCloud_.reserve(inputCloud_.size());

        // Set of voxel keys, reused across frames so a scan pays no allocation.
        downsampleSeen_.clear();
        downsampleSeen_.reserve(inputCloud_.size() * 2);

        // The key is an exact packing of the voxel index, not a hash of it.
        // A hash (the previous `kx*p1 ^ ky*p2 ^ kz*p3`) collides, and every
        // collision silently discards a real return from a *different* voxel --
        // a beam that is never cast and a surface that is never mapped.  21 bits
        // per axis covers +/-2^20 voxels around the sensor, far past any sensor
        // range at a sane resolution; anything beyond that keeps the point
        // rather than risking a wrapped key aliasing onto another voxel.
        constexpr int64_t kBias = int64_t(1) << 20;
        const double inv_res = 1.0 / params_.VoxelResolution;
        for (const auto &p : inputCloud_) {
            // Filter non-finite returns (NaN/Inf) that some drivers emit.
            if (!p.allFinite()) continue;

            const double fx = std::floor(p.x() * inv_res);
            const double fy = std::floor(p.y() * inv_res);
            const double fz = std::floor(p.z() * inv_res);
            if (std::fabs(fx) >= kBias || std::fabs(fy) >= kBias || std::fabs(fz) >= kBias) {
                filteredCloud_.push_back(p);
                continue;
            }
            const int64_t key =
                ((static_cast<int64_t>(fx) + kBias)) |
                ((static_cast<int64_t>(fy) + kBias) << 21) |
                ((static_cast<int64_t>(fz) + kBias) << 42);
            if (downsampleSeen_.insert(key).second) filteredCloud_.push_back(p);
        }
        cloud_ptr = &filteredCloud_;
    }

    // ---- 4) Transform (sensor-frame) points to world frame, clipping
    //         long returns, then raycast the whole batch in one call so it
    //         can run as a single parallel pass (CPU SoA or CUDA kernel -
    //         see OccupancyMap::insertPointCloud).
    const bool clip = params_.MaxRayLength > 0.0;
    const double max_len_sq = clip ? params_.MaxRayLength * params_.MaxRayLength
                                   : std::numeric_limits<double>::infinity();
    worldPoints_.clear();
    clearingPoints_.clear();
    worldPoints_.reserve(cloud_ptr->size());
    for (const auto &point : *cloud_ptr) {
        // Non-finite returns carry no direction, so they cannot even seed a
        // clearing ray; drop them (the downsample path already filters these).
        if (!point.allFinite()) continue;
        const double range_sq = point.squaredNorm();
        if (clip && range_sq > max_len_sq) {
            // Over-range return: there is no surface here, but everything from
            // the sensor out to MaxRayLength along this beam is free space. Cast
            // a clearing ray to a clamped endpoint instead of dropping the beam,
            // so voxels a now-removed obstacle used to occupy get swept clear
            // rather than lingering as ghosts.
            const Eigen::Vector3d dir = point / std::sqrt(range_sq);
            clearingPoints_.push_back(R * (dir * params_.MaxRayLength) + pose);
        } else {
            worldPoints_.push_back(R * point + pose);
        }
    }

    // ---- 4b) No-return frustum clearing ----------------------------------
    // Surface returns only ever land on obstacles, so nothing sweeps the open
    // air a removed object used to occupy above the floor line - those beams
    // simply return nothing (NaN pixels in the organized depth cloud). Recover
    // each empty pixel's ray from the cached intrinsics and cast a miss-only
    // clearing ray to MaxRayLength. Subsampled by ClearingRayStride because
    // neighbouring rays overlap heavily, so a coarse fan clears the volume for
    // a fraction of the cost. Runs on the original organized cloud
    // (inputCloud_), independent of the downsampled hit path above.
    if (clip && intrinsics_ready_.load(std::memory_order_acquire) &&
        cam_height_ > 1 && CloudMsg->height == cam_height_ &&
        CloudMsg->width == cam_width_ && inputCloud_.size() == ray_table_.size()) {
        const uint32_t W = cam_width_, H = cam_height_;
        const uint32_t stride = static_cast<uint32_t>(params_.ClearingRayStride);  // validated >= 1
        for (uint32_t v = 0; v < H; v += stride) {
            for (uint32_t u = 0; u < W; u += stride) {
                const size_t i = static_cast<size_t>(v) * W + u;
                if (inputCloud_[i].allFinite()) continue;  // real return -> a surface, not empty
                clearingPoints_.push_back(R * (ray_table_[i] * params_.MaxRayLength) + pose);
            }
        }
    }

    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        // Surface returns: carve free space along each ray and stamp the hit.
        map_->insertPointCloud(pose, worldPoints_);
        // Over-range beams: carve free space only (miss-only, no endpoint hit).
        // Skipped entirely when empty, so in-range scans pay nothing extra.
        if (!clearingPoints_.empty()) {
            map_->insertPointCloud(pose, clearingPoints_, /*apply_hits=*/false);
        }
        // Under the same lock as the insert it describes: the publish timers
        // read this to stamp the map, and ros::Time is not atomic.
        last_msg_stamp_ = CloudMsg->header.stamp;
    }
}

// ---------------------------------------------------------------------------
// Timer-driven publisher: publishes the occupied set without holding the map
// mutex during the (allocating, serialising) heavy work. The mutex is held
// only long enough to snapshot the occupied-index set, which is the minimum
// required to keep the sensor callback latency bounded.
// ---------------------------------------------------------------------------
void NanoVoxMapNode::publishTimerCallback(const ros::TimerEvent&) {

    if (pub_.getNumSubscribers() == 0) return;  // skip work if nobody listens

    // ---- 1) Take a fast snapshot of the occupied indices under the lock --
    std::vector<int64_t> snapshot;
    Eigen::Vector3d       lower_bound;
    double                resolution;
    int64_t               size_x, size_xy;
    ros::Time             stamp;
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        map_->snapshotOccupied(snapshot, lower_bound, resolution, size_x, size_xy);
        stamp = last_msg_stamp_;
    }
    if (stamp.isZero()) stamp = ros::Time::now();

    // ---- 2) Build the PointCloud2 OUTSIDE the lock -----------------------
    // Pre-size everything; avoid sensor_msgs::PointCloud round-trip which
    // allocates twice. We write floats directly into the PointCloud2 byte
    // buffer.
    sensor_msgs::PointCloud2 msg;
    msg.header.stamp    = stamp;
    msg.header.frame_id = params_.WorldFrameId;
    msg.height          = 1;
    msg.width           = static_cast<uint32_t>(snapshot.size());
    msg.is_dense        = true;
    msg.is_bigendian    = false;
    msg.point_step      = 12;                 // 3 * float32
    msg.row_step        = msg.point_step * msg.width;
    // Sized in size_t: row_step is uint32 and would wrap on a very large map,
    // handing the write loop below a buffer far shorter than it believes.
    const size_t map_bytes = static_cast<size_t>(msg.point_step) * msg.width;
    msg.fields.resize(3);
    msg.fields[0].name = "x"; msg.fields[0].offset = 0;
    msg.fields[1].name = "y"; msg.fields[1].offset = 4;
    msg.fields[2].name = "z"; msg.fields[2].offset = 8;
    for (int i = 0; i < 3; ++i) {
        msg.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
        msg.fields[i].count    = 1;
    }
    msg.data.resize(map_bytes);

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

    pub_.publish(msg);
}

} // namespace NanoVoxMap
