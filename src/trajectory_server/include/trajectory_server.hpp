#ifndef TRAJECTORY_SERVER_HPP
#define TRAJECTORY_SERVER_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <polytope_msgs/msg/polytope_array.hpp>
#include <quadrotor_msgs/msg/trajectory_command.hpp>

#include <Eigen/Eigen>
#include <memory>
#include <mutex>
#include <vector>

#include "gcopter_solver.hpp"
#include "trajectory_interface.hpp"

namespace trajectory_server
{

    // Receding-horizon trajectory generator built on GCOPTER (MINCO + safe
    // flight corridor). Subscribes/publishes on the exact same topics as the
    // old Bezier-QP based trajectory_server node, so it is a drop-in swap.
    class TrajOptNode : public rclcpp::Node
    {
    public:
        explicit TrajOptNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    private:
        void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg);
        void polytopesCallback(const polytope_msgs::msg::PolytopeArray::ConstSharedPtr &msg);
        void controlLoopCallback();

        bool replan(const Eigen::Vector3d &pos,
                    const Eigen::Vector3d &vel,
                    const Eigen::Vector3d &acc,
                    const std::vector<Eigen::MatrixX4d> &hPolys,
                    const Eigen::Vector3d &goal);

        // S = 3 -> MINCO_S3NU, degree-5, minimum jerk (boundary: P,V,A)
        // S = 4 -> MINCO_S4NU, degree-7, minimum snap (boundary: P,V,A,J)
        template <int S>
        bool replanImpl(const Eigen::Vector3d &pos,
                        const Eigen::Vector3d &vel,
                        const Eigen::Vector3d &acc,
                        const std::vector<Eigen::MatrixX4d> &hPolys,
                        const Eigen::Vector3d &goal);

        void publishHoverCommand(const Eigen::Vector3d &pos);
        void publishTrajectoryCommand(const Eigen::Vector3d &pos,
                                       const Eigen::Vector3d &vel,
                                       const Eigen::Vector3d &acc,
                                       const Eigen::Vector3d &jer,
                                       double yaw,
                                       double yawDot);
        void publishVisualization();

        std::pair<double, double> calculateYaw(const Eigen::Vector3d &velDes,
                                                double dt);

        // --- ROS interface ---
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub_;
        rclcpp::Subscription<polytope_msgs::msg::PolytopeArray>::SharedPtr polySub_;
        rclcpp::Publisher<quadrotor_msgs::msg::TrajectoryCommand>::SharedPtr trajPub_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr vizPub_;
        rclcpp::TimerBase::SharedPtr controlTimer_;

        // --- Topic params ---
        std::string odomTopic_;
        std::string trajTopic_;
        std::string polyTopic_;

        // --- GCOPTER tuning params ---
        double vMax_, omgMax_, thetaMax_, thrustMin_, thrustMax_;
        Eigen::VectorXd magnitudeBounds_;   // [vMax, omgMax, thetaMax, thrustMin, thrustMax]
        Eigen::VectorXd penaltyWeights_;    // [pos, vel, omg, theta, thrust]
        Eigen::VectorXd physicalParams_;    // [mass, grav, dh, dv, cp, speedEps]
        double weightT_;
        double smoothingEps_;
        int integralRes_;
        double lengthPerPiece_;
        double relCostTol_;

        // Which MINCO order to solve with: 3 = minimum jerk / degree-5
        // (default), 4 = minimum snap / degree-7.
        int costOrder_;

        // --- Corridor safety margin ---
        // Shrinks each half-space constraint inward by this distance (m)
        // before handing the corridor to GCOPTER, same role as
        // trajectory_server's SafetyMargin.
        double safetyMargin_;

        // --- Receding horizon params ---
        double replanPeriod_;
        double goalReachedThreshold_;

        // --- Corridor staleness watchdog ---
        // If corridor_planning stops publishing (crash, killed, never
        // restarted) without this node knowing, hPolys_/goal_ would
        // otherwise stay frozen at their last received values forever, and
        // this node would keep flying toward that stale goal indefinitely.
        double polyhedraTimeout_;

        // --- Yaw params ---
        double yawDotMax_;
        double lowSpeedThreshold_;

        // --- Shared state (locked by dataMutex_) ---
        std::mutex dataMutex_;
        bool hasOdom_;
        Eigen::Vector3d odomPos_;
        Eigen::Vector3d odomVel_;
        bool hasPoly_;
        std::vector<Eigen::MatrixX4d> hPolys_;
        Eigen::Vector3d goal_;
        rclcpp::Time lastPolyRecvTime_;

        // --- Planning state (owned by control loop thread only) ---
        bool isFirstRun_;
        rclcpp::Time lastReplanTime_;
        bool trajValid_;
        std::unique_ptr<ITrajectory> traj_;
        rclcpp::Time trajStartTime_;

        // --- Yaw state ---
        double lastYaw_;
        double lastYawDot_;
        rclcpp::Time lastYawTime_;
    };

} // namespace trajectory_server

#endif // TRAJECTORY_SERVER_HPP
