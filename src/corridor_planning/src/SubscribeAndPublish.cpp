#include "SubscribeAndPublish.hpp"

SubscribeAndPublish::SubscribeAndPublish(const rclcpp::NodeOptions &options)
    : rclcpp::Node("corridor_planning_node", options), config_(*this), running_(true) {
  isFirstRun_ = true;

  generalCbGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  transformCbGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  visualizer_ = std::make_unique<Visualizer>(this);
  polyhedraPublisher_ = std::make_unique<PolyhedraPublisher<double, 3>>(this, "/Polyhedra/mesh", "/Polyhedra/edge");


  resolution_ = config_.VoxelResolution;
  deflation_factor_ = config_.DeflationFactor; // Tolerance for polyhedra computation
  horizon_ = config_.horizon;
  filter_radius_ = config_.filterRadius;

  dmp_ptr->setPotentialRadius(Vec3f(config_.DmPotentialRadius[0], config_.DmPotentialRadius[1], config_.DmPotentialRadius[2]));
  dmp_ptr->setSearchRadius(Vec3f(config_.DmPSearchRadius[0], config_.DmPSearchRadius[1], config_.DmPSearchRadius[2]));

  origin_ = Vec3f(config_.MapLowerBound[0], config_.MapLowerBound[1], config_.MapLowerBound[2]);
  max_bounds_ = Vec3f(config_.MapUpperBound[0], config_.MapUpperBound[1], config_.MapUpperBound[2]);

  start_ = Eigen::Vector3d(config_.InitialPose[0], config_.InitialPose[1], config_.InitialPose[2]);
  goal_ = Eigen::Vector3d(config_.GoalPose[0], config_.GoalPose[1], config_.GoalPose[2]);

  A_bound_ << -Eigen::MatrixXd::Identity(3, 3), Eigen::MatrixXd::Identity(3, 3);
  b_bound_ << -origin_(0), -origin_(1), -origin_(2), max_bounds_(0), max_bounds_(1), max_bounds_(2);

  // Grid geometry is fixed by config (origin / bounds / resolution), so allocate
  // and zero the occupancy grid ONCE here. Per-frame we only reset the cells
  // touched on the previous frame (see PlanCorridor -> resetOccupancy).
  grid_size_ = VoxelGrid::initializeGrid<double, 3>(origin_, max_bounds_, resolution_, grid_);

  // // Topic you want to publish
  // pubFilteredCloud_ = this->create_publisher<sensor_msgs::msg::PointCloud>(config_.FilteredCloudTopic, 1);

  // Polytope publisher
  polytopePub_ = this->create_publisher<polytope_msgs::msg::PolytopeArray>("/quadrotor/polytopes", 1);

  // Remaining points publisher
  remainingPtsPub_ = this->create_publisher<std_msgs::msg::Int64MultiArray>("/benchmark/remaining_points", 1);

  // Computation time publisher (now reports PathCover time in isolation)
  ComputationTimePub_ = this->create_publisher<std_msgs::msg::Float64>("/benchmark/computation_time_ms", 1);

  // Topic you want to subscribe
  rclcpp::SubscriptionOptions generalSubOptions;
  generalSubOptions.callback_group = generalCbGroup_;
  PointCloudSub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      config_.mapTopic, rclcpp::QoS(1),
      std::bind(&SubscribeAndPublish::PointCloudCallback, this, std::placeholders::_1),
      generalSubOptions);

  rclcpp::SubscriptionOptions transformSubOptions;
  transformSubOptions.callback_group = transformCbGroup_;
  groundTruthSub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      config_.GroundTruthTopic, rclcpp::QoS(1),
      std::bind(&SubscribeAndPublish::groundTruthCallback, this, std::placeholders::_1),
      transformSubOptions);
  targetSub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      config_.goalPoseTopic, rclcpp::QoS(1),
      std::bind(&SubscribeAndPublish::targetCallback, this, std::placeholders::_1),
      transformSubOptions);

  // Spin up the planner thread. All heavy work (conversion, grid, JPS/DMP,
  // PathCover, publishing) runs here so the subscriber callbacks stay responsive.
  worker_thread_ = std::thread(&SubscribeAndPublish::plannerLoop, this);

  RCLCPP_INFO(this->get_logger(), "corridor_planning initialized.");
  RCLCPP_INFO(this->get_logger(), "  map topic        : %s", config_.mapTopic.c_str());
  RCLCPP_INFO(this->get_logger(), "  ground truth topic: %s", config_.GroundTruthTopic.c_str());
  RCLCPP_INFO(this->get_logger(), "  goal pose topic   : %s", config_.goalPoseTopic.c_str());
  RCLCPP_INFO(this->get_logger(), "  initial goal      : [%.2f, %.2f, %.2f]", goal_.x(), goal_.y(), goal_.z());
}


SubscribeAndPublish::~SubscribeAndPublish() {
  running_.store(false);
  cloud_cv_.notify_all();
  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }
}


// ---------------------------------------------------------------------------
// Cloud callback: do the absolute minimum. Just stash the newest message and
// wake the planner thread. If clouds arrive faster than we can plan (likely at
// 500k points with full-cloud RISP), older unprocessed clouds are simply
// overwritten -> "process-latest, drop-stale". This keeps the queue (size 1)
// from backing up and bounds end-to-end latency.
// ---------------------------------------------------------------------------
inline void SubscribeAndPublish::PointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &CloudMsg) {
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    latest_cloud_ = CloudMsg;   // overwrite any stale, unprocessed cloud
  }
  cloud_cv_.notify_one();
}


// Planner thread main loop.
void SubscribeAndPublish::plannerLoop() {
  while (running_.load()) {
    sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud;
    {
      std::unique_lock<std::mutex> lock(cloud_mutex_);
      cloud_cv_.wait(lock, [this] { return latest_cloud_ != nullptr || !running_.load(); });
      if (!running_.load()) break;
      cloud.swap(latest_cloud_);   // take the freshest; leave latest_cloud_ null
    }
    if (cloud) {
      processCloud(cloud);
    }
  }
}


// One full planning iteration against a single (freshest) cloud.
void SubscribeAndPublish::processCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &CloudMsg) {
  // Snapshot start/goal ONCE under a single mutex. (Previously start_ was read
  // under transform_mutex_ but written under start_goal_mutex_ -> data race.)
  Eigen::Vector3d start, goal;
  {
    std::lock_guard<std::mutex> lock(start_goal_mutex_);
    start = start_;
    goal = goal_;
  }

  // Fused conversion + filter (single pass over the raw PointCloud2 bytes).
  auto tic = std::chrono::high_resolution_clock::now();
  try {
    EigenPointCloudConversions::convertAndFilter(*CloudMsg, start, filter_radius_, filteredCloud_);
  } catch (const std::exception &e) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                          "corridor_planning: dropping malformed point cloud: %s", e.what());
    return;
  }
  auto toc = std::chrono::high_resolution_clock::now();
  double convert_ms = std::chrono::duration<double, std::milli>(toc - tic).count();
  // RCLCPP_INFO(this->get_logger(), "convert+filter (%zu pts): %f ms", filteredCloud_.size(), convert_ms);

  // Plan path + corridors. PathCover still receives the FULL filtered cloud.
  bool plan_success = PlanCorridor(filteredCloud_, start, goal);

  // Guard: if planning failed (path/corridor empty) skip downstream publishing,
  // which would otherwise dereference A_.at(0) etc. on an empty corridor set.
  if (!plan_success) {
    return;
  }

  VisualizePolyhedra(filteredCloud_, path_, start, goal);
}


inline bool SubscribeAndPublish::PlanCorridor(std::vector<Eigen::Vector3d> &filteredCloud,
                                              Eigen::Vector3d &start,
                                              Eigen::Vector3d &goal) {
  // NOTE: no internal locking here. Only the planner thread calls this, and it
  // operates on the start/goal snapshot passed in by processCloud().
  path_.clear();
  A_.clear();
  b_.clear();
  seeds_.clear();
  if ((start - goal).norm() <= 1e-3) {
    path_.push_back(start);
  }
  else {
    auto tg0 = std::chrono::high_resolution_clock::now();

    // Reset only previously-occupied cells, then re-mark (records new occupancy).
    VoxelGrid::resetOccupancy(grid_, occupied_cells_);
    VoxelGrid::setOccupancy<double, 3>(filteredCloud, origin_, resolution_, grid_, grid_size_, occupied_cells_);

    map_util->setMap(origin_, grid_size_, grid_, resolution_);
    planner_ptr->setMapUtil(map_util);
    planner_ptr->updateMap();
    auto tg1 = std::chrono::high_resolution_clock::now();

    bool plan_success = planner_ptr->plan(start, goal, resolution_, true); // Plan a path
    vec_Vecf<3> path_jps = planner_ptr->getRawPath();

    dmp_ptr->setMap(map_util, start);

    bool dmp_success = dmp_ptr->computePath(start, goal, path_jps); // Compute the path given the jps path
    vec_Vecf<3> path_dmp = dmp_ptr->getRawPath();
    auto tg2 = std::chrono::high_resolution_clock::now();

    if (!plan_success || !dmp_success) {
      bool startOcc = VoxelGrid::checkOccupancy<double, 3>(start, grid_, origin_, resolution_, grid_size_);
      bool goalOcc = VoxelGrid::checkOccupancy<double, 3>(goal, grid_, origin_, resolution_, grid_size_);
      RCLCPP_WARN(this->get_logger(),
                  "JPS Planning failed! plan_success=%d dmp_success=%d start=[%.2f,%.2f,%.2f](occ=%d) goal=[%.2f,%.2f,%.2f](occ=%d) jps_path_len=%zu",
                  plan_success, dmp_success, start.x(), start.y(), start.z(), startOcc,
                  goal.x(), goal.y(), goal.z(), goalOcc, path_jps.size());
      path_.clear();
      A_.clear();
      b_.clear();
      seeds_.clear();
      return false;
    }

    path_dmp.insert(path_dmp.begin(), start);
    path_dmp.push_back(goal); // Ensure start and goal are included in the path

    // Convert path to Eigen::Vector3d
    path_.assign(path_dmp.begin(), path_dmp.end());

    double grid_ms   = std::chrono::duration<double, std::milli>(tg1 - tg0).count();
    double search_ms = std::chrono::duration<double, std::milli>(tg2 - tg1).count();
    // RCLCPP_INFO(this->get_logger(), "grid+map: %f ms | JPS+DMP: %f ms", grid_ms, search_ms);
  }


  std::vector<std::vector<int>> all_pts_remaining;

  // -------- The benchmark target: PathCover on the FULL cloud (unchanged) ----
  auto tp0 = std::chrono::high_resolution_clock::now();
  try {
    PathCover::pathCover<double, 3>(filteredCloud, path_, A_bound_, b_bound_, A_, b_, seeds_, all_pts_remaining, deflation_factor_, horizon_);
  } catch (const std::exception &e) {
    // PathCover throws on degenerate numeric edge cases (e.g. a seed landing
    // exactly on a corridor boundary). That must never take the whole node
    // down: an uncaught exception here previously propagated out of this
    // worker thread and called std::terminate(), silently killing
    // corridor_planning_node -- after which trajectory_server has no way to
    // know the corridor/goal it's still flying toward will never update
    // again. Treat it like any other failed planning cycle instead.
    RCLCPP_WARN(this->get_logger(), "PathCover failed: %s", e.what());
    path_.clear();
    A_.clear();
    b_.clear();
    seeds_.clear();
    return false;
  }
  auto tp1 = std::chrono::high_resolution_clock::now();
  double pathcover_ms = std::chrono::duration<double, std::milli>(tp1 - tp0).count();
  // RCLCPP_INFO(this->get_logger(), "PathCover (%zu pts): %f ms", filteredCloud.size(), pathcover_ms);

  // Publish PathCover time in isolation as the benchmark metric.
  std_msgs::msg::Float64 elapsedMsg;
  elapsedMsg.data = pathcover_ms;
  ComputationTimePub_->publish(elapsedMsg);

  // Publish remaining points for benchmarking
  if (!all_pts_remaining.empty()) {
    std_msgs::msg::Int64MultiArray remainingPtsMsg;
    remainingPtsMsg.data = std::vector<int64_t>(all_pts_remaining.at(0).begin(), all_pts_remaining.at(0).end());
    remainingPtsPub_->publish(remainingPtsMsg);
  }

  return true;
}


inline void SubscribeAndPublish::VisualizePolyhedra(std::vector<Eigen::Vector3d> &filteredCloud,
                                                    std::vector<Eigen::Vector3d> &path,
                                                    const Eigen::Vector3d &start,
                                                    const Eigen::Vector3d &goal) {
  polyhedraPublisher_->publishMesh(A_, b_, seeds_, horizon_);
  visualizer_->visualize_seeds(seeds_);
  visualizer_->visualize_path(path);


  // Plan the path and safe corridors
  visualizer_->visualizeStart(start, 0.25, Eigen::Vector4d(1, 0, 0, 1));
  visualizer_->visualizeGoal(goal, 0.25, Eigen::Vector4d(0, 1, 0, 1));


  // Publish polyhedra as polytopes
  polytope_msgs::msg::PolytopeArray polytopes_msg;
  for (size_t i = 0; i < A_.size(); ++i) {
    polytope_msgs::msg::Polytope polytope_msg;
    this->convertConstraints(A_.at(i), b_.at(i), seeds_.at(i), polytope_msg);
    polytopes_msg.polytopes.push_back(polytope_msg);
  }
  polytopes_msg.header.frame_id = "world";
  polytopes_msg.header.stamp = this->now();
  if (seeds_.size() > 1) {
    std::vector<double> goal_msg(seeds_.back().data(), seeds_.back().data() + seeds_.back().size());
    polytopes_msg.goal = goal_msg;
    if ((seeds_.back() - goal).norm() > 1e-3) {
      RCLCPP_WARN(this->get_logger(),
                  "corridor_planning: published goal [%.2f,%.2f,%.2f] differs from true goal [%.2f,%.2f,%.2f] (horizon=%d, A_.size()=%zu)",
                  seeds_.back().x(), seeds_.back().y(), seeds_.back().z(),
                  goal.x(), goal.y(), goal.z(), horizon_, A_.size());
    }
  }
  polytopePub_->publish(polytopes_msg);


  // // Publish filtered point cloud
  // sensor_msgs::msg::PointCloud filteredCloudMsg;
  // EigenPointCloudConversions::VectorToPointCloud(filteredCloud, filteredCloudMsg);
  // filteredCloudMsg.header.frame_id = "world";
  // filteredCloudMsg.header.stamp = this->now();
  // pubFilteredCloud_->publish(filteredCloudMsg);
}



inline void SubscribeAndPublish::groundTruthCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg) {
  std::lock_guard<std::mutex> lock(start_goal_mutex_);
  start_ = Eigen::Vector3d(msg->pose.pose.position.x,
                           msg->pose.pose.position.y,
                           msg->pose.pose.position.z);
}


inline void SubscribeAndPublish::targetCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg) {
  std::lock_guard<std::mutex> lock(start_goal_mutex_);
  goal_ = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, 0.75);
  RCLCPP_INFO(this->get_logger(), "corridor_planning: new goal received: [%.2f, %.2f, %.2f]",
              goal_.x(), goal_.y(), goal_.z());
}


inline void SubscribeAndPublish::convertConstraints(Eigen::Matrix<double, -1, 3> &A,
                                                    Eigen::VectorXd &b,
                                                    Eigen::Vector3d& seed,
                                                    polytope_msgs::msg::Polytope &msg) {
  Eigen::Matrix<double, -1, 3, Eigen::RowMajor> A_row_major = A;
  std::vector<double> A_msg(A_row_major.data(), A_row_major.data() + A_row_major.size());
  std::vector<double> b_msg(b.data(), b.data() + b.size());
  std::vector<double> seed_msg(seed.data(), seed.data() + seed.size());
  msg.a = A_msg;
  msg.b = b_msg;
  msg.seed = seed_msg;
}
