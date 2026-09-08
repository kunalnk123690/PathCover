#ifndef PATHCOVER_SELECTED_TRAJECTORY_SERVER_HPP
#define PATHCOVER_SELECTED_TRAJECTORY_SERVER_HPP

#ifndef PATHCOVER_DIM
#define PATHCOVER_DIM 3
#endif

#if PATHCOVER_DIM == 2
#ifndef TRAJECTORY_SERVER_HPP
#define TRAJECTORY_SERVER_HPP

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <polytope_msgs/Polytopes.h>

#include <Eigen/Eigen>
#include <memory>
#include <mutex>
#include <vector>

#include "gcopter_solver.hpp"
#include "trajectory_interface.hpp"

namespace trajectory_server
{

    // Receding-horizon planar trajectory generator built on GCOPTER (MINCO +
    // safe corridor), driving a differential-drive base.
    //
    // Ported from the quadrotor version: the corridor is a chain of 2D
    // half-plane polytopes, the optimizer works in (x, y), and the output is a
    // geometry_msgs/Twist command rather than a flat-output trajectory message
    // -- a Jackal cannot be commanded with (position, velocity, acceleration,
    // jerk, yaw), it takes (v, omega). The MINCO trajectory is therefore
    // tracked by a unicycle feedback law (see calculateCommand) instead of
    // being handed to an external flatness-based controller.
    class TrajOptNode
    {
    public:
        explicit TrajOptNode(ros::NodeHandle &nh);

    private:
        void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
        void polytopesCallback(const polytope_msgs::Polytopes::ConstPtr &msg);
        void controlLoopCallback(const ros::TimerEvent &event);

        bool replan(const Eigen::Vector2d &pos,
                    const Eigen::Vector2d &vel,
                    const Eigen::Vector2d &acc,
                    const std::vector<Eigen::MatrixX3d> &hPolys,
                    const Eigen::Vector2d &goal);

        // S = 3 -> MINCO_S3NU<2>, degree-5, minimum jerk (boundary: P,V,A)
        // S = 4 -> MINCO_S4NU<2>, degree-7, minimum snap (boundary: P,V,A,J)
        template <int S>
        bool replanImpl(const Eigen::Vector2d &pos,
                        const Eigen::Vector2d &vel,
                        const Eigen::Vector2d &acc,
                        const std::vector<Eigen::MatrixX3d> &hPolys,
                        const Eigen::Vector2d &goal);

        // Unicycle tracking law turning a reference (pos, vel, acc) sample and
        // the measured pose into a (v, omega) command.
        std::pair<double, double> calculateCommand(const Eigen::Vector2d &refPos,
                                                   const Eigen::Vector2d &refVel,
                                                   const Eigen::Vector2d &refAcc,
                                                   const Eigen::Vector2d &pos,
                                                   double yaw);

        void publishStopCommand();
        void publishTwistCommand(double v, double omega);
        void publishVisualization();

        // --- ROS interface ---
        ros::NodeHandle nh_;
        ros::Subscriber odomSub_;
        ros::Subscriber polySub_;
        ros::Publisher cmdPub_;
        ros::Publisher vizPub_;
        ros::Timer controlTimer_;

        // --- Topic params ---
        std::string odomTopic_;
        std::string cmdVelTopic_;
        std::string polyTopic_;

        // --- GCOPTER tuning params ---
        double vMax_, aMax_, omgMax_;
        Eigen::VectorXd magnitudeBounds_;   // [vMax, aMax, omgMax]
        Eigen::VectorXd penaltyWeights_;    // [pos, vel, acc, omg]
        double curvatureEps_;
        double weightT_;
        double smoothingEps_;
        int integralRes_;
        double lengthPerPiece_;
        double relCostTol_;

        // Which MINCO order to solve with: 3 = minimum jerk / degree-5
        // (default), 4 = minimum snap / degree-7.
        int costOrder_;

        // --- Corridor safety margin ---
        // Shrinks each half-plane constraint inward by this distance (m)
        // before handing the corridor to GCOPTER. For a ground robot this is
        // where the footprint radius belongs.
        double safetyMargin_;

        // World z at which a 3-D corridor is sliced into the planar corridor
        // the optimizer works with. Only used when the incoming polytopes
        // carry three coefficients per row (which is what corridor_planning
        // publishes); a natively planar corridor ignores it.
        double corridorSliceHeight_;

        // --- Receding horizon params ---
        double replanPeriod_;
        double goalReachedThreshold_;

        // --- Tracking-law gains ---
        double kX_;                 // along-track error gain
        double kY_;                 // cross-track error gain
        double kTheta_;             // heading error gain
        double headingAlignTol_;    // rotate in place above this heading error
        double kAlign_;             // in-place rotation gain
        double lowSpeedThreshold_;  // below this |v_ref|, hold the last heading

        // True when the odometry twist is expressed in the base frame (as
        // diff_drive_controller reports it) rather than the world frame (as
        // the Gazebo p3d ground-truth plugin reports it).
        bool odomTwistInBodyFrame_;

        // --- Shared state (locked by dataMutex_) ---
        std::mutex dataMutex_;
        bool hasOdom_;
        Eigen::Vector2d odomPos_;
        Eigen::Vector2d odomVel_;
        double odomYaw_;
        bool hasPoly_;
        std::vector<Eigen::MatrixX3d> hPolys_;
        Eigen::Vector2d goal_;

        // --- Planning state (owned by control loop thread only) ---
        bool isFirstRun_;
        ros::Time lastReplanTime_;
        bool trajValid_;
        std::unique_ptr<ITrajectory> traj_;
        ros::Time trajStartTime_;

        // --- Tracking state ---
        double lastRefYaw_;
        bool hasLastRefYaw_;

        // Height (world z) the planar trajectory is drawn at in RViz.
        double vizHeight_;
    };

} // namespace trajectory_server

#endif // TRAJECTORY_SERVER_HPP

#elif PATHCOVER_DIM == 3
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

#else
#error "trajectory_server supports only PATHCOVER_DIM=2 or 3"
#endif

#endif
