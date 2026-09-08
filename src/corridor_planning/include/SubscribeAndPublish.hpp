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

#ifndef PATHCOVER_DIM
#define PATHCOVER_DIM 3
#endif

using VectorDim = Eigen::Matrix<double, PATHCOVER_DIM, 1>;
using GridIndex = Eigen::Matrix<int, PATHCOVER_DIM, 1>;
using ConstraintMatrix = Eigen::Matrix<double, Eigen::Dynamic, PATHCOVER_DIM>;
using BoundaryMatrix = Eigen::Matrix<double, 2 * PATHCOVER_DIM, PATHCOVER_DIM>;
using BoundaryVector = Eigen::Matrix<double, 2 * PATHCOVER_DIM, 1>;

#if PATHCOVER_DIM == 2
using JpsVector = Vec2f;
using MapUtilType = OccMapUtil;
using JpsPlanner = JPSPlanner2D;
using DmPlannerType = DMPlanner2D;
#elif PATHCOVER_DIM == 3
using JpsVector = Vec3f;
using MapUtilType = VoxelMapUtil;
using JpsPlanner = JPSPlanner3D;
using DmPlannerType = DMPlanner3D;
#else
#error "corridor_planning supports only PATHCOVER_DIM=2 or 3"
#endif

// Shared corridor node. Aerial builds operate directly in 3D; planar builds
// slice the mapped cloud by height and project it to XY before planning.
struct Config {
    std::string mapTopic;
    std::string GroundTruthTopic;
    std::string FilteredCloudTopic;
    std::string PolytopeTopic;
    double VoxelResolution;
    double filterRadius;
    double ObstacleZMin;
    double ObstacleZMax;
    double VizHeight;
    double JpsHeuristicWeight;
    int horizon;
    std::vector<double> MapLowerBound;
    std::vector<double> MapUpperBound;
    std::vector<double> InitialPose;
    std::vector<double> GoalPose;
    std::vector<double> DmPotentialRadius;
    std::vector<double> DmPSearchRadius;


    Config(const ros::NodeHandle &nh_priv) {
        nh_priv.getParam("MapTopic", mapTopic);
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

        PolytopeTopic = PATHCOVER_DIM == 2 ? "/jackal/polytopes" : "/quadrotor/polytopes";
        nh_priv.getParam("PolytopeTopic", PolytopeTopic);
        ObstacleZMin = 0.05;
        nh_priv.getParam("ObstacleZMin", ObstacleZMin);
        ObstacleZMax = 0.8;
        nh_priv.getParam("ObstacleZMax", ObstacleZMax);
        VizHeight = 0.15;
        nh_priv.getParam("VizHeight", VizHeight);
        JpsHeuristicWeight = VoxelResolution;
        nh_priv.getParam("JpsHeuristicWeight", JpsHeuristicWeight);
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

        inline bool PlanCorridor(std::vector<VectorDim> &PointCloud,
                                 VectorDim &start,
                                 VectorDim &goal);
        inline void VisualizePolyhedra(std::vector<VectorDim> &PointCloud,
                                       std::vector<VectorDim> &path,
                                       const VectorDim &start,
                                       const VectorDim &goal);
        inline void convertConstraints(ConstraintMatrix &A,
                                       Eigen::VectorXd &b,
                                       VectorDim& seed,
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
        std::vector<VectorDim> inputCloud_;

        std::vector<signed char> grid_;
        GridIndex grid_size_;
        std::vector<int> occupied_cells_;   // dirty-cell list for fast grid reset
        JpsVector origin_;
        JpsVector max_bounds_;
        double resolution_;
        double filter_radius_;
        double obstacle_z_min_;
        double obstacle_z_max_;
        double jps_heuristic_weight_;
        std::vector<VectorDim> PointCloud_;
        std::vector<VectorDim> startGoal_;

        std::vector<VectorDim> route_;
        std::vector<VectorDim> path_;


        /****** Create JPS Map ******/
        std::shared_ptr<MapUtilType> map_util = std::make_shared<MapUtilType>();

        /****** Declare a planner pointer ******/
        std::unique_ptr<JpsPlanner> planner_ptr = std::make_unique<JpsPlanner>(false);
        std::unique_ptr<DmPlannerType> dmp_ptr = std::make_unique<DmPlannerType>(false);
        std::vector<VectorDim> filteredCloud_;
        bool isFirstRun_;

        std::unique_ptr<Visualizer> visualizer_;

        BoundaryMatrix A_bound_;
        BoundaryVector b_bound_;
        std::vector<ConstraintMatrix> A_;
        std::vector<Eigen::VectorXd> b_;
        std::vector<VectorDim> seeds_;
        std::unique_ptr<PolyhedraPublisher<double, PATHCOVER_DIM>> polyhedraPublisher_;
        int horizon_;
        VectorDim start_;
        VectorDim goal_;

        std::mutex start_goal_mutex_;

        // --- Planner-thread plumbing (process-latest, drop-stale) ---
        std::thread worker_thread_;
        std::mutex cloud_mutex_;                          // guards latest_cloud_
        std::condition_variable cloud_cv_;
        sensor_msgs::PointCloud2::ConstPtr latest_cloud_; // newest unprocessed cloud
        std::atomic<bool> running_;

};

#endif
