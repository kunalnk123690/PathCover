#ifndef SUBSCRIBEANDPUBLISH_H
#define SUBSCRIBEANDPUBLISH_H


#include <iostream>
#include <math.h>
#include <chrono>
#include <mutex>
#include <thread>
#include <atomic>
#include <condition_variable>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <std_msgs/Int64MultiArray.h>
#include <std_msgs/Float64.h>
#include <Eigen/Core>
#include "polytope_msgs/Polytopes.h"
#include "polytope_msgs/Polytope.h"
#include "eigen_conversions.h"
#include "jps_basis/data_utils.h"
#include "jps_planner/jps_planner.h"
#include "distance_map_planner/distance_map_planner.h"
#include "visual/visualizer.hpp"
#include "planner/VoxelGrid.hpp"
#include "planner/PathCover.hpp"
#include "geometry/polytope_utils.h"
#include "visual/polytope_publisher.hpp"


using namespace std;
using namespace chrono;
using namespace JPS;

struct Config {
    std::string mapTopic;
    std::string GroundTruthTopic;
    std::string FilteredCloudTopic;
    double DeflationFactor;
    double VoxelResolution;
    double filterRadius;
    int horizon;
    std::vector<double> MapLowerBound;
    std::vector<double> MapUpperBound;
    std::vector<double> InitialPose;
    std::vector<double> GoalPose;
    std::vector<double> DmPotentialRadius;
    std::vector<double> DmPSearchRadius;


    Config(const ros::NodeHandle &nh_priv) {
        nh_priv.getParam("MapTopic", mapTopic);
        nh_priv.getParam("DeflationFactor", DeflationFactor);
        nh_priv.getParam("VoxelResolution", VoxelResolution);
        nh_priv.getParam("MapLowerBound", MapLowerBound);
        nh_priv.getParam("MapUpperBound", MapUpperBound);
        nh_priv.getParam("InitialPose", InitialPose);
        nh_priv.getParam("GoalPose", GoalPose);
        nh_priv.getParam("FilterRadius", filterRadius);
        nh_priv.getParam("DmPotentialRadius", DmPotentialRadius);
        nh_priv.getParam("DmPSearchRadius", DmPSearchRadius);
        nh_priv.getParam("GroundTruthTopic", GroundTruthTopic);
        nh_priv.getParam("FilteredCloudTopic", FilteredCloudTopic);
        nh_priv.getParam("horizon", horizon);
    }
};


class SubscribeAndPublish {
    public:
        SubscribeAndPublish(const Config &conf, ros::NodeHandle nhg, ros::NodeHandle nht);
        ~SubscribeAndPublish();

        // --- Callbacks (kept cheap; they never block on planning) ---
        inline void PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);
        inline void groundTruthCallback(const nav_msgs::Odometry::ConstPtr &msg);
        inline void targetCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

        // --- Heavy work, executed on the dedicated planner thread ---
        void plannerLoop();
        void processCloud(const sensor_msgs::PointCloud2::ConstPtr &msg);

        inline bool PlanCorridor(std::vector<Eigen::Vector3d> &PointCloud, 
                                 Eigen::Vector3d &start,
                                 Eigen::Vector3d &goal);
        inline void VisualizePolyhedra(std::vector<Eigen::Vector3d> &PointCloud, 
                                       std::vector<Eigen::Vector3d> &path,
                                       const Eigen::Vector3d &start,
                                       const Eigen::Vector3d &goal);
        inline void convertConstraints(Eigen::Matrix<double, -1, 3> &A, 
                                       Eigen::VectorXd &b, 
                                       Eigen::Vector3d& seed, 
                                       polytope_msgs::Polytope &msg);


    private:
        Config config_;
        ros::NodeHandle n_;
        ros::Subscriber PointCloudSub_;
        ros::Subscriber subTransform_;
        ros::Subscriber groundTruthSub_;
        ros::Subscriber targetSub_;
        // ros::Publisher pubFilteredCloud_;
        ros::Publisher polytopePub_;
        ros::Publisher remainingPtsPub_;
        ros::Publisher ComputationTimePub_;
        std::vector<Eigen::Vector3d> inputCloud_;

        std::vector<signed char> grid_;
        Eigen::Vector3i grid_size_;
        std::vector<int> occupied_cells_;   // dirty-cell list for fast grid reset
        Vec3f origin_;
        Vec3f max_bounds_;
        double resolution_;
        double filter_radius_;
        std::vector<Eigen::Vector3d> PointCloud_;
        std::vector<Eigen::Vector3d> startGoal_;

        std::vector<Eigen::Vector3d> route_;
        std::vector<Eigen::Vector3d> path_;
      

        /****** Create JPS Map ******/
        std::shared_ptr<VoxelMapUtil> map_util = std::make_shared<VoxelMapUtil>();

        /****** Declare a planner pointer ******/
        std::unique_ptr<JPSPlanner3D> planner_ptr = std::make_unique<JPSPlanner3D>(false); 
        std::unique_ptr<DMPlanner3D> dmp_ptr = std::make_unique<DMPlanner3D>(false);
        std::vector<Eigen::Vector3d> filteredCloud_;
        bool isFirstRun_;

        // Eigen::Quaterniond q_;
        // Eigen::Vector3d pose_;
        
        std::unique_ptr<Visualizer> visualizer_ = std::make_unique<Visualizer>(n_);

        Eigen::Matrix<double, 6, 3> A_bound_;
        Eigen::Matrix<double, 6, 1> b_bound_;
        std::vector<Eigen::Matrix<double, -1, 3>> A_;
        std::vector<Eigen::VectorXd> b_;
        std::vector<Eigen::Vector3d> seeds_;
        std::unique_ptr<PolyhedraPublisher<double, 3>> polyhedraPublisher_;
        double deflation_factor_;
        int horizon_;
        Eigen::Vector3d start_;
        Eigen::Vector3d goal_;

        std::mutex start_goal_mutex_;

        // --- Planner-thread plumbing (process-latest, drop-stale) ---
        std::thread worker_thread_;
        std::mutex cloud_mutex_;                          // guards latest_cloud_
        std::condition_variable cloud_cv_;
        sensor_msgs::PointCloud2::ConstPtr latest_cloud_; // newest unprocessed cloud
        std::atomic<bool> running_;

};

#endif