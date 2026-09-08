/**
 * @file occupancy_inflation_node.cpp
 * @brief Subscribes to a PointCloud2, inflates the occupied voxels and
 *        republishes them. Every setting comes from config/inflation.yaml.
 *
 * The node holds no cloud of its own: points are read straight out of the
 * incoming message and the outgoing message is filled in place, so a map only
 * ever exists once, as bits in the inflater's grids.
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>

#include <string>
#include <vector>

#include "occupancy_inflation/eigen_conversions.h"
#include "occupancy_inflation/point_cloud_inflater.h"

namespace {

/**
 * @brief Read a 3-element vector parameter, keeping the default if unset.
 * @return false if the parameter exists but is not a 3-element list of numbers
 */
bool readVector3(const ros::NodeHandle &nh, const std::string &name,
                 Eigen::Vector3d *out) {
  if (!nh.hasParam(name)) {
    return true;
  }
  std::vector<double> values;
  if (!nh.getParam(name, values)) {
    /* Set, but not a list of numbers. Silently keeping the default here would
     * hide a typo in the config file. */
    ROS_ERROR_STREAM("Parameter '" << nh.resolveName(name) << "' must be a list "
                                   << "of 3 numbers");
    return false;
  }
  if (values.size() != 3) {
    ROS_ERROR_STREAM("Parameter '" << nh.resolveName(name) << "' needs exactly "
                                   << "3 elements, got " << values.size());
    return false;
  }
  *out = Eigen::Vector3d(values[0], values[1], values[2]);
  return true;
}

class InflationNode {
 public:
  InflationNode(ros::NodeHandle &nh, ros::NodeHandle &pnh) : pnh_(pnh) {
    if (!loadParams()) {
      ros::shutdown();
      return;
    }

    cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>(output_topic_,
                                                        queue_size_, latch_);
    if (publish_seed_cloud_) {
      seed_pub_ = nh.advertise<sensor_msgs::PointCloud2>(seed_topic_,
                                                         queue_size_, latch_);
    }
    cloud_sub_ = nh.subscribe(input_topic_, queue_size_,
                              &InflationNode::cloudCallback, this,
                              ros::TransportHints().tcpNoDelay());
    reset_srv_ = pnh_.advertiseService("reset", &InflationNode::resetService, this);

    /* Fixed-rate publishing. The timer shares the global callback queue with
     * the subscriber, and main() spins that queue on one thread, so the two
     * callbacks are serialised against each other by construction: the timer
     * can never read the grids while a cloud is being folded into them. That
     * is what keeps this lock free, so it has to stay a single-threaded spin. */
    if (publish_rate_hz_ > 0.0) {
      publish_timer_ = nh.createTimer(ros::Duration(1.0 / publish_rate_hz_),
                                      &InflationNode::publishTimerCallback, this);
    }

    ROS_INFO_STREAM("occupancy_inflation ready"
                    << "\n  input topic:       " << nh.resolveName(input_topic_)
                    << "\n  output topic:      " << nh.resolveName(output_topic_)
                    << "\n  resolution:        " << params_.resolution << " m"
                    << "\n  radius_xy:         " << params_.radius_xy << " m"
                    << "\n  radius_z:          " << params_.radius_z << " m"
                    << "\n  shape:             " << shape_name_
                    << "\n  kernel voxels:     " << inflater_.kernelSize()
                    << "\n  incremental:       " << (params_.incremental ? "yes" : "no")
                    << "\n  published map:     "
                    << (params_.publish_surface_only
                            ? "boundary shell, " +
                                  std::to_string(params_.surface_thickness) +
                                  " voxel(s) thick"
                            : std::string("solid"))
                    << "\n  publish rate:      "
                    << (publish_rate_hz_ > 0.0
                            ? std::to_string(publish_rate_hz_) + " Hz (timer)"
                            : std::string("on every input message"))
                    << "\n  threads:           " << params_.num_threads
                    << "\n  dilation backend:  "
                    << (inflater_.cudaEnabled()
                            ? "CUDA"
                            : (occupancy_inflation::cuda::built()
                                   ? (params_.use_cuda ? "CPU (no device found)"
                                                       : "CPU (use_cuda off)")
                                   : "CPU (built without CUDA)"))
                    << "\n  reset service:     " << pnh_.resolveName("reset"));
  }

 private:
  bool loadParams() {
    pnh_.param<std::string>("input_topic", input_topic_,
                            "/map_generator/global_cloud");
    pnh_.param<std::string>("output_topic", output_topic_,
                            "occupancy_inflation/inflated_cloud");
    pnh_.param<std::string>("seed_topic", seed_topic_,
                            "occupancy_inflation/seed_cloud");
    pnh_.param("publish_seed_cloud", publish_seed_cloud_, false);
    pnh_.param("publish_intensity", publish_intensity_, true);
    pnh_.param("publish_on_change_only", publish_on_change_only_, true);
    pnh_.param("publish_rate_hz", publish_rate_hz_, 0.0);
    pnh_.param("queue_size", queue_size_, 1);
    pnh_.param("latch", latch_, true);
    pnh_.param("max_rate_hz", max_rate_hz_, 0.0);
    pnh_.param("verbose", verbose_, true);

    /* Empty means "reuse the frame of the incoming cloud". */
    pnh_.param<std::string>("frame_id", frame_id_, "");

    if (queue_size_ < 1) {
      ROS_ERROR("queue_size must be >= 1");
      return false;
    }
    if (max_rate_hz_ < 0.0) {
      ROS_ERROR("max_rate_hz must be >= 0 (0 disables throttling)");
      return false;
    }
    if (publish_rate_hz_ < 0.0) {
      ROS_ERROR("publish_rate_hz must be >= 0 (0 publishes on every input message)");
      return false;
    }
    /* A fixed output rate and "only when it changed" are contradictory: an
     * unchanged map is exactly what the timer still has to republish to hold
     * the rate up. Say so rather than quietly publishing at some other rate. */
    if (publish_rate_hz_ > 0.0 && publish_on_change_only_) {
      ROS_WARN("publish_on_change_only is ignored while publish_rate_hz > 0; "
               "the timer publishes the current map whether or not it changed");
      publish_on_change_only_ = false;
    }

    pnh_.param("resolution", params_.resolution, 0.1);
    pnh_.param("radius_xy", params_.radius_xy, 0.2);
    pnh_.param("radius_z", params_.radius_z, 0.2);
    pnh_.param("incremental", params_.incremental, true);
    pnh_.param("num_threads", params_.num_threads, 4);

    pnh_.param("publish_surface_only", params_.publish_surface_only, false);
    pnh_.param("surface_thickness", params_.surface_thickness, 1);
    if (params_.publish_surface_only && params_.surface_thickness < 1) {
      ROS_ERROR("surface_thickness must be >= 1");
      return false;
    }

    pnh_.param("use_cuda", params_.use_cuda, true);
    int cuda_min_grid_voxels = static_cast<int>(params_.cuda_min_grid_voxels);
    pnh_.param("cuda_min_grid_voxels", cuda_min_grid_voxels,
               cuda_min_grid_voxels);
    if (cuda_min_grid_voxels < 0) {
      ROS_ERROR("cuda_min_grid_voxels must be >= 0");
      return false;
    }
    params_.cuda_min_grid_voxels =
        static_cast<std::size_t>(cuda_min_grid_voxels);
    if (params_.use_cuda && !occupancy_inflation::cuda::built()) {
      ROS_WARN("use_cuda is set but this build has no CUDA backend; "
               "the dilation will run on the CPU");
    }

    int max_grid_mib = 64;
    pnh_.param("max_grid_mib", max_grid_mib, 64);
    if (max_grid_mib < 1) {
      ROS_ERROR("max_grid_mib must be >= 1");
      return false;
    }
    /* One voxel is one bit, so a mebibyte of grid is 8 Mi voxels. */
    params_.max_grid_voxels =
        static_cast<std::size_t>(max_grid_mib) * 8u * 1024u * 1024u;

    pnh_.param<std::string>("shape", shape_name_, "box");
    if (!occupancy_inflation::PointCloudInflater::parseShape(shape_name_,
                                                             &params_.shape)) {
      ROS_ERROR_STREAM("Unknown shape '" << shape_name_
                                         << "', expected 'box' or 'ellipsoid'");
      return false;
    }

    pnh_.param("input_filter/enable", params_.input_filter_enable, false);
    pnh_.param("output_bounds/enable", params_.output_bounds_enable, false);

    if (!readVector3(pnh_, "origin", &params_.origin) ||
        !readVector3(pnh_, "input_filter/min", &params_.input_min) ||
        !readVector3(pnh_, "input_filter/max", &params_.input_max) ||
        !readVector3(pnh_, "output_bounds/min", &params_.output_min) ||
        !readVector3(pnh_, "output_bounds/max", &params_.output_max)) {
      return false;
    }

    std::string error;
    if (!inflater_.configure(params_, &error)) {
      ROS_ERROR_STREAM("Invalid inflation parameters: " << error);
      return false;
    }
    return true;
  }

  bool resetService(std_srvs::Empty::Request &, std_srvs::Empty::Response &) {
    inflater_.reset();
    map_frame_.clear();
    ROS_INFO("occupancy_inflation: accumulated map cleared");
    return true;
  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
    if (max_rate_hz_ > 0.0) {
      const ros::Time now = ros::Time::now();
      const ros::Duration min_period(1.0 / max_rate_hz_);
      if (!last_processed_.isZero() && (now - last_processed_) < min_period) {
        return;
      }
      last_processed_ = now;
    }

    const auto layout = EigenPointCloudConversions::InspectXyz(*msg);
    if (!layout.valid) {
      ROS_ERROR_STREAM_THROTTLE(5.0, "Cannot read the incoming cloud: "
                                         << layout.error);
      return;
    }
    if (layout.size == 0) {
      ROS_WARN_THROTTLE(5.0, "Received an empty point cloud, nothing to inflate");
      return;
    }

    /* Voxels accumulated in one frame mean nothing in another, so a frame
     * change starts the map over rather than mixing the two. */
    if (params_.incremental && msg->header.frame_id != map_frame_) {
      if (!map_frame_.empty()) {
        ROS_WARN_STREAM("Input frame changed from '"
                        << map_frame_ << "' to '" << msg->header.frame_id
                        << "'; clearing the accumulated map");
        inflater_.reset();
      }
      map_frame_ = msg->header.frame_id;
    }

    occupancy_inflation::InflationStats stats;
    if (layout.datatype == sensor_msgs::PointField::FLOAT32) {
      const EigenPointCloudConversions::PointCloud2XyzReader<float> reader(layout);
      inflater_.inflate(reader, &stats);
    } else {
      const EigenPointCloudConversions::PointCloud2XyzReader<double> reader(layout);
      inflater_.inflate(reader, &stats);
    }

    if (!stats.ok) {
      switch (stats.error) {
        case occupancy_inflation::InflationError::kGridTooLarge:
          ROS_ERROR_STREAM_THROTTLE(
              5.0, "Inflation aborted: the map needs more than max_grid_mib ("
                       << (params_.max_grid_voxels >> 23)
                       << " MiB). Enable output_bounds, raise max_grid_mib, or "
                          "look for stray points far from the map.");
          break;
        case occupancy_inflation::InflationError::kOutOfRange:
          ROS_ERROR_THROTTLE(5.0,
                             "Inflation aborted: the cloud reaches beyond the "
                             "voxel lattice (about 1e6 voxels from the origin). "
                             "Enable input_filter to drop the outliers.");
          break;
        default:
          ROS_ERROR_THROTTLE(5.0, "Inflation aborted: the inflater is not configured");
          break;
      }
      return;
    }

    /* What the timer needs to publish the map without an input message in
     * hand. The stamp stays the input's, not the tick's: it is the time the
     * map content is valid for, and a fixed-rate republish does not make it
     * any newer. */
    map_stamp_ = msg->header.stamp;
    map_publish_frame_ = frame_id_.empty() ? msg->header.frame_id : frame_id_;
    have_map_ = true;

    const bool publish = publish_rate_hz_ <= 0.0 &&
                         (stats.changed || !publish_on_change_only_ || !published_);
    if (publish) {
      publishCurrentMap();
    }

    if (verbose_) {
      const double growth =
          stats.seed_voxels > 0
              ? static_cast<double>(stats.inflated_voxels) / stats.seed_voxels
              : 0.0;
      ROS_INFO_STREAM("inflated " << stats.seed_voxels << " -> "
                                  << stats.inflated_voxels << " voxels (x"
                                  << growth << ") from " << stats.input_points
                                  << " points, +" << stats.new_seed_voxels
                                  << " new"
                                  << (params_.publish_surface_only
                                          ? ", publishing " +
                                                std::to_string(stats.output_voxels) +
                                                " on the shell"
                                          : std::string())
                                  << ", " << stats.duration_ms << " ms"
                                  << (stats.changed ? "" : ", unchanged")
                                  << (stats.gpu_dilation ? ", gpu" : "")
                                  << ", grids " << (inflater_.memoryBytes() >> 10)
                                  << " KiB");
    }
  }

  /**
   * @brief Publish the current map on a fixed schedule.
   *
   * The point of the timer is that the output rate is a property of this node
   * rather than of how often the mapper happens to send, or of whether the map
   * changed since it last did. Serialising the grid again per tick costs about
   * a millisecond on a room-sized map, so the tick is cheap next to its period.
   */
  void publishTimerCallback(const ros::TimerEvent &) {
    if (!have_map_) return;  // nothing folded in yet
    /* Nobody listening means the serialisation and the megabytes behind it are
     * pure waste; a latched publisher still hands the last map to whoever
     * subscribes later. */
    if (cloud_pub_.getNumSubscribers() == 0 &&
        (!publish_seed_cloud_ || seed_pub_.getNumSubscribers() == 0)) {
      return;
    }
    publishCurrentMap();
  }

  void publishCurrentMap() {
    publishGrid(cloud_pub_, false, map_publish_frame_, map_stamp_);
    if (publish_seed_cloud_) {
      publishGrid(seed_pub_, true, map_publish_frame_, map_stamp_);
    }
    published_ = true;
  }

  /**
   * @brief Serialise a grid into a fresh message and publish it.
   *
   * Publishing through a shared pointer hands the buffer to the subscriber
   * instead of copying it, which matters once the map is a few megabytes.
   */
  void publishGrid(const ros::Publisher &pub, bool seeds,
                   const std::string &frame, const ros::Time &stamp) {
    const std::size_t count =
        seeds ? inflater_.seedCount() : inflater_.occupiedCount();
    const std::size_t stride =
        EigenPointCloudConversions::XyziPointStep(publish_intensity_);

    sensor_msgs::PointCloud2Ptr msg(new sensor_msgs::PointCloud2);
    EigenPointCloudConversions::PrepareXyziCloud(*msg, count, publish_intensity_);
    msg->header.frame_id = frame;
    msg->header.stamp = stamp;

    if (count != 0) {
      const std::size_t written =
          seeds ? inflater_.serializeSeeds(msg->data.data(), stride,
                                           publish_intensity_)
                : inflater_.serialize(msg->data.data(), stride,
                                      publish_intensity_);
      ROS_ASSERT(written == count);
    }
    pub.publish(msg);
  }

  ros::NodeHandle pnh_;
  ros::Subscriber cloud_sub_;
  ros::Publisher cloud_pub_;
  ros::Publisher seed_pub_;
  ros::ServiceServer reset_srv_;
  ros::Timer publish_timer_;

  occupancy_inflation::PointCloudInflater inflater_;
  occupancy_inflation::InflationParams params_;

  std::string input_topic_;
  std::string output_topic_;
  std::string seed_topic_;
  std::string frame_id_;
  std::string shape_name_;
  std::string map_frame_;
  /* Frame and stamp the timer publishes the map under, from the last cloud. */
  std::string map_publish_frame_;
  ros::Time map_stamp_;
  bool have_map_ = false;
  bool publish_seed_cloud_ = false;
  bool publish_intensity_ = true;
  bool publish_on_change_only_ = true;
  bool published_ = false;
  bool latch_ = true;
  bool verbose_ = true;
  int queue_size_ = 1;
  double max_rate_hz_ = 0.0;
  double publish_rate_hz_ = 0.0;
  ros::Time last_processed_;
};

}  // namespace

int main(int argc, char **argv) {
  ros::init(argc, argv, "occupancy_inflation_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  InflationNode node(nh, pnh);
  ros::spin();
  return 0;
}
