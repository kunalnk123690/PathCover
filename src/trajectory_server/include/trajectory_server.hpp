#ifndef TRAJECTORY_SERVER_HPP
#define TRAJECTORY_SERVER_HPP

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <polytope_msgs/Polytopes.h>
#include <quadrotor_msgs/TrajectoryCommand.h>

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
    class TrajOptNode
    {
    public:
        explicit TrajOptNode(ros::NodeHandle &nh);

    private:
        void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
        void polytopesCallback(const polytope_msgs::Polytopes::ConstPtr &msg);
        void controlLoopCallback(const ros::TimerEvent &event);

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
        ros::NodeHandle nh_;
        ros::Subscriber odomSub_;
        ros::Subscriber polySub_;
        ros::Publisher trajPub_;
        ros::Publisher vizPub_;
        ros::Timer controlTimer_;

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

        // --- Planning state (owned by control loop thread only) ---
        bool isFirstRun_;
        ros::Time lastReplanTime_;
        bool trajValid_;
        std::unique_ptr<ITrajectory> traj_;
        ros::Time trajStartTime_;

        // --- Yaw state ---
        double lastYaw_;
        double lastYawDot_;
        ros::Time lastYawTime_;
    };

} // namespace trajectory_server

#endif // TRAJECTORY_SERVER_HPP
