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

        // Index of the corridor polytope the replan should start from, given
        // where the robot actually is. Operates on the RAW (unshrunk)
        // corridor -- see the note on hPolys_.
        int findStartPolytope(const Eigen::Vector2d &pos,
                              const std::vector<Eigen::MatrixX3d> &hPolys) const;

        // Reports which polytope or which consecutive overlap has no interior,
        // so a failed replan names the cause instead of leaving GCOPTER's
        // generic setup() warning to be guessed at.
        void diagnoseCorridor(const std::vector<Eigen::MatrixX3d> &corridor) const;

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
        // > 0 pins each corridor polytope to exactly this many MINCO pieces;
        // 0 falls back to allocating by path length via lengthPerPiece_.
        int piecesPerPolytope_;
        double relCostTol_;
        // Bounds a single L-BFGS solve so one pathological corridor cannot
        // stall the replan cycle; 0 leaves it uncapped.
        int maxIterations_;

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
        // RAW corridor (sliced to the plane, but NOT margin-shrunk): rows are
        // [n^T, d] with n.x + d <= 0 and d = -b. The safety margin is
        // deliberately not baked in here -- findStartPolytope() has to test
        // containment against the corridor the robot is really driving in, and
        // a margin-shrunk polytope can easily exclude a robot that is
        // comfortably inside the true one. The margin is applied at replan
        // time, after truncation.
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
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <polytope_msgs/Polytopes.h>
#include <quadrotor_msgs/State.h>
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
    // flight corridor). Publishes the same TrajectoryCommand on the same topic
    // as the old Bezier-QP based trajectory_server node, but takes its vehicle
    // state from quadrotor_msgs::State rather than nav_msgs::Odometry, since
    // the initial boundary condition of each replan needs a measured
    // acceleration and Odometry carries none.
    class TrajOptNode
    {
    public:
        explicit TrajOptNode(ros::NodeHandle &nh);

    private:
        void stateCallback(const quadrotor_msgs::State::ConstPtr &msg);
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

        // Index of the corridor polytope the replan should start from, given
        // where the vehicle actually is. Operates on the RAW (unshrunk)
        // corridor -- see the note on hPolys_.
        int findStartPolytope(const Eigen::Vector3d &pos,
                              const std::vector<Eigen::MatrixX4d> &hPolys) const;

        // Reports which polytope or which consecutive overlap has no interior,
        // so a failed replan names the cause instead of leaving GCOPTER's
        // generic setup() warning to be guessed at.
        void diagnoseCorridor(const std::vector<Eigen::MatrixX4d> &corridor) const;

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
        ros::Subscriber stateSub_;
        ros::Subscriber polySub_;
        ros::Publisher trajPub_;
        ros::Publisher vizPub_;
        ros::Timer controlTimer_;

        // --- Topic params ---
        std::string stateTopic_;
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
        // > 0 pins each corridor polytope to exactly this many MINCO pieces;
        // 0 falls back to allocating by path length via lengthPerPiece_.
        int piecesPerPolytope_;
        double relCostTol_;
        // Bounds a single L-BFGS solve so one pathological corridor cannot
        // stall the replan cycle; 0 leaves it uncapped.
        int maxIterations_;

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
        bool hasState_;
        // Measured state of the vehicle, world frame, straight off the state
        // topic. These are the only seed for every replan's initial boundary
        // condition -- nothing here is ever re-sampled from a previous solve.
        Eigen::Vector3d statePos_;
        Eigen::Vector3d stateVel_;
        Eigen::Vector3d stateAcc_;
        bool hasPoly_;
        // RAW corridor, exactly as received: rows are [n^T, d] with n.x + d <= 0
        // and d = -b. The safety margin is deliberately NOT baked in here --
        // findStartPolytope() has to test containment against the corridor the
        // vehicle is really flying in, and a margin-shrunk polytope can easily
        // exclude a vehicle that is comfortably inside the true one. The margin
        // is applied at replan time, after truncation.
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
