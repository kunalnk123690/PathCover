#ifndef SUBSCRIBEANDPUBLISH_H
#define SUBSCRIBEANDPUBLISH_H


#include <iostream>
#include <math.h>
#include <chrono>
#include <mutex>
#include <thread>
#include <atomic>
#include <condition_variable>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/int64_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <Eigen/Core>
#include "polytope_msgs/msg/polytope_array.hpp"
#include "polytope_msgs/msg/polytope.hpp"
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
    std::string goalPoseTopic;


    explicit Config(rclcpp::Node &node) {
        auto fail = [&](const std::string &msg) {
            RCLCPP_FATAL(node.get_logger(), "%s", msg.c_str());
            throw std::runtime_error(msg);
        };

        auto requireParam = [&](const std::string &name, auto &value) {
            using ParamT = std::decay_t<decltype(value)>;
            try {
                node.declare_parameter<ParamT>(name);
                if (!node.get_parameter(name, value)) {
                    fail("corridor_planning: required parameter '" + name + "' is missing");
                }
            } catch (const std::exception &) {
                fail("corridor_planning: required parameter '" + name + "' is missing");
            }
        };

        requireParam("MapTopic", mapTopic);
        requireParam("DeflationFactor", DeflationFactor);
        requireParam("VoxelResolution", VoxelResolution);
        requireParam("MapLowerBound", MapLowerBound);
        requireParam("MapUpperBound", MapUpperBound);
        requireParam("InitialPose", InitialPose);
        requireParam("GoalPose", GoalPose);
        requireParam("FilterRadius", filterRadius);
        requireParam("DmPotentialRadius", DmPotentialRadius);
        requireParam("DmPSearchRadius", DmPSearchRadius);
        requireParam("GroundTruthTopic", GroundTruthTopic);
        requireParam("FilteredCloudTopic", FilteredCloudTopic);
        requireParam("horizon", horizon);

        // Optional: defaults to rviz_default_plugins/SetGoal's default topic
        // ("/goal_pose"), NOT ROS1 move_base's "/move_base_simple/goal".
        goalPoseTopic = node.declare_parameter("GoalPoseTopic", std::string("/goal_pose"));
    }
};


class SubscribeAndPublish : public rclcpp::Node {
    public:
        explicit SubscribeAndPublish(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
        ~SubscribeAndPublish();

        // --- Callbacks (kept cheap; they never block on planning) ---
        inline void PointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);
        inline void groundTruthCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg);
        inline void targetCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg);

        // --- Heavy work, executed on the dedicated planner thread ---
        void plannerLoop();
        void processCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);

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
                                       polytope_msgs::msg::Polytope &msg);


    private:
        Config config_;
        rclcpp::CallbackGroup::SharedPtr generalCbGroup_;
        rclcpp::CallbackGroup::SharedPtr transformCbGroup_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr PointCloudSub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr groundTruthSub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr targetSub_;
        // rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pubFilteredCloud_;
        rclcpp::Publisher<polytope_msgs::msg::PolytopeArray>::SharedPtr polytopePub_;
        rclcpp::Publisher<std_msgs::msg::Int64MultiArray>::SharedPtr remainingPtsPub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr ComputationTimePub_;
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

        std::unique_ptr<Visualizer> visualizer_;

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
        sensor_msgs::msg::PointCloud2::ConstSharedPtr latest_cloud_; // newest unprocessed cloud
        std::atomic<bool> running_;

};

#endif
