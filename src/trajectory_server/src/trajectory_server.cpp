#ifndef PATHCOVER_DIM
#define PATHCOVER_DIM 3
#endif

#if PATHCOVER_DIM == 2
#include "trajectory_server.hpp"

#include <tf/transform_datatypes.h>

#include <algorithm>
#include <cmath>

namespace trajectory_server
{

    namespace
    {
        double wrapAngle(double angle)
        {
            while (angle > M_PI)
            {
                angle -= 2.0 * M_PI;
            }
            while (angle < -M_PI)
            {
                angle += 2.0 * M_PI;
            }
            return angle;
        }

        double clamp(double v, double lo, double hi)
        {
            return std::max(lo, std::min(hi, v));
        }

        template <typename T>
        T getRequiredParam(ros::NodeHandle &nh, const std::string &name)
        {
            T value;
            if (!nh.getParam(name, value))
            {
                ROS_FATAL("trajectory_server_node: required parameter '~%s' not set", name.c_str());
                ros::shutdown();
                std::exit(1);
            }
            return value;
        }
    } // namespace

    TrajOptNode::TrajOptNode(ros::NodeHandle &nh)
        : nh_(nh),
          hasOdom_(false),
          odomPos_(Eigen::Vector2d::Zero()),
          odomVel_(Eigen::Vector2d::Zero()),
          odomYaw_(0.0),
          hasPoly_(false),
          goal_(Eigen::Vector2d::Zero()),
          isFirstRun_(true),
          trajValid_(false),
          lastRefYaw_(0.0),
          hasLastRefYaw_(false)
    {
        // --- Topics ---
        odomTopic_ = getRequiredParam<std::string>(nh_, "OdometryTopic");
        cmdVelTopic_ = getRequiredParam<std::string>(nh_, "CmdVelTopic");
        polyTopic_ = getRequiredParam<std::string>(nh_, "PolyhedraTopic");

        // --- Dynamics / GCOPTER bounds ---
        vMax_ = getRequiredParam<double>(nh_, "v_max");
        aMax_ = nh_.param("a_max", 1.0);
        omgMax_ = nh_.param("omg_max", 1.5);

        magnitudeBounds_.resize(3);
        magnitudeBounds_ << vMax_, aMax_, omgMax_;

        double weightPos = nh_.param("weight_pos", 1.0e4);
        double weightVel = nh_.param("weight_vel", 1.0e4);
        double weightAcc = nh_.param("weight_acc", 1.0e4);
        double weightOmg = nh_.param("weight_omg", 1.0e4);
        penaltyWeights_.resize(4);
        penaltyWeights_ << weightPos, weightVel, weightAcc, weightOmg;

        curvatureEps_ = nh_.param("curvature_eps", 1.0e-2);

        weightT_ = nh_.param("weight_time", 100.0);
        smoothingEps_ = nh_.param("smoothing_eps", 0.01);
        integralRes_ = nh_.param("integral_resolution", 16);
        lengthPerPiece_ = nh_.param("length_per_piece", 1.0);
        relCostTol_ = nh_.param("rel_cost_tol", 1.0e-4);

        costOrder_ = nh_.param("cost_order", 3);
        if (costOrder_ != 3 && costOrder_ != 4)
        {
            ROS_FATAL("trajectory_server_node: cost_order must be 3 (minimum jerk, degree-5) "
                      "or 4 (minimum snap, degree-7), got %d",
                      costOrder_);
            ros::shutdown();
            std::exit(1);
        }

        safetyMargin_ = nh_.param("SafetyMargin", 0.3);

        replanPeriod_ = nh_.param("ReplanPeriod", 0.5);
        goalReachedThreshold_ = nh_.param("GoalReachedThreshold", 0.2);

        kX_ = nh_.param("k_x", 1.0);
        kY_ = nh_.param("k_y", 4.0);
        kTheta_ = nh_.param("k_theta", 2.0);
        headingAlignTol_ = nh_.param("HeadingAlignTolerance", 0.6);
        kAlign_ = nh_.param("k_align", 1.5);
        lowSpeedThreshold_ = nh_.param("LowSpeedThreshold", 0.05);

        odomTwistInBodyFrame_ = nh_.param("OdomTwistInBodyFrame", false);
        vizHeight_ = nh_.param("VizHeight", 0.0);

        // --- Publishers & Subscribers ---
        cmdPub_ = nh_.advertise<geometry_msgs::Twist>(cmdVelTopic_, 1);
        vizPub_ = nh_.advertise<nav_msgs::Path>("/colored_trajectory", 1);
        polySub_ = nh_.subscribe(polyTopic_, 1, &TrajOptNode::polytopesCallback, this);
        odomSub_ = nh_.subscribe(odomTopic_, 1, &TrajOptNode::odomCallback, this);

        ros::Time now = ros::Time::now();
        lastReplanTime_ = now;

        controlTimer_ = nh_.createTimer(ros::Duration(1.0 / 50.0), &TrajOptNode::controlLoopCallback, this);

        ROS_INFO("trajectory_server_node (planar GCOPTER receding-horizon, cost_order=%d) is running...", costOrder_);
    }

    void TrajOptNode::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        std::lock_guard<std::mutex> lock(dataMutex_);
        odomPos_ << msg->pose.pose.position.x,
            msg->pose.pose.position.y;

        odomYaw_ = tf::getYaw(msg->pose.pose.orientation);

        const double vx = msg->twist.twist.linear.x;
        const double vy = msg->twist.twist.linear.y;
        if (odomTwistInBodyFrame_)
        {
            const double c = std::cos(odomYaw_);
            const double s = std::sin(odomYaw_);
            odomVel_ << c * vx - s * vy,
                s * vx + c * vy;
        }
        else
        {
            odomVel_ << vx, vy;
        }
        hasOdom_ = true;
    }

    void TrajOptNode::polytopesCallback(const polytope_msgs::Polytopes::ConstPtr &msg)
    {
        std::lock_guard<std::mutex> lock(dataMutex_);

        if (msg->polytope.empty())
        {
            hasPoly_ = false;
            hPolys_.clear();
            return;
        }

        std::vector<Eigen::MatrixX3d> hPolys;
        hPolys.reserve(msg->polytope.size());

        for (const auto &poly : msg->polytope)
        {
            // corridor_planning publishes planar polytopes: A is a row-major
            // (rows x 2) flattening, so each row is two coefficients.
            const int rows = static_cast<int>(poly.A.size() / 2);
            Eigen::MatrixX3d hPoly(rows, 3);
            // Ax <= b  <=>  n.dot(x) + d <= 0 with n = A_row, d = -b_row.
            // Shrink b by safetyMargin_ * ||A_row|| (not just safetyMargin_)
            // so the inward shift is exactly safetyMargin_ metres regardless
            // of whether A_row is normalized; GCOPTER itself normalizes each
            // row by ||A_row|| in setup().
            for (int i = 0; i < rows; i++)
            {
                const double ax = poly.A[2 * i + 0];
                const double ay = poly.A[2 * i + 1];
                const double normA = std::sqrt(ax * ax + ay * ay);
                const double safeB = poly.b[i] - safetyMargin_ * normA;

                hPoly(i, 0) = ax;
                hPoly(i, 1) = ay;
                hPoly(i, 2) = -safeB;
            }
            hPolys.push_back(hPoly);
        }

        if (msg->goal.size() < 2)
        {
            ROS_WARN_THROTTLE(2.0, "trajectory_server_node: received Polytopes message without a valid goal, ignoring.");
            hasPoly_ = false;
            hPolys_.clear();
            return;
        }

        hPolys_ = std::move(hPolys);
        goal_ << msg->goal[0], msg->goal[1];
        hasPoly_ = true;
    }

    template <int S>
    bool TrajOptNode::replanImpl(const Eigen::Vector2d &pos,
                                  const Eigen::Vector2d &vel,
                                  const Eigen::Vector2d &acc,
                                  const std::vector<Eigen::MatrixX3d> &hPolys,
                                  const Eigen::Vector2d &goal)
    {
        using BoundaryT = typename MincoTraits<S>::Boundary;
        constexpr int Degree = MincoTraits<S>::Degree;

        // Boundary conditions are seeded purely from the robot's actual
        // current state (position + velocity from odometry) every replan.
        // Acceleration (and jerk, for the snap-cost case) has no direct sensor
        // measurement here, so it is always zero rather than carried forward
        // from the previous solve's own evaluated trajectory -- carrying it
        // forward was feeding each solve's own artifacts back in as the next
        // replan's hard boundary condition, compounding any distortion across
        // replan cycles.
        BoundaryT initialState, terminalState;
        initialState.col(0) = pos;
        initialState.col(1) = vel;
        initialState.col(2) = acc;
        if constexpr (S == 4)
        {
            initialState.col(3).setZero();
        }

        // Terminal velocity/acceleration(/jerk) are zero. The corridor's
        // published goal can be a horizon-truncated point rather than the true
        // mission destination, but since we replan every replanPeriod_ seconds
        // and only execute the near-term part of each solve, the terminal
        // condition mostly just shapes the unexecuted tail -- it should stay
        // simple/neutral rather than guessing a flow-through direction that
        // may fight the corridor's actual curvature near its end.
        terminalState.col(0) = goal;
        terminalState.col(1).setZero();
        terminalState.col(2).setZero();
        if constexpr (S == 4)
        {
            terminalState.col(3).setZero();
        }

        GcopterSolver<S> solver;
        if (!solver.setup(weightT_, initialState, terminalState, hPolys,
                          lengthPerPiece_, smoothingEps_, integralRes_,
                          magnitudeBounds_, penaltyWeights_, curvatureEps_))
        {
            ROS_WARN_THROTTLE(2.0, "trajectory_server_node: GCOPTER setup failed (corridor likely degenerate/non-overlapping).");
            return false;
        }

        Trajectory<Degree, 2> candidate;
        const double cost = solver.optimize(candidate, relCostTol_);

        if (!std::isfinite(cost) || candidate.getPieceNum() == 0)
        {
            ROS_WARN_THROTTLE(2.0, "trajectory_server_node: GCOPTER optimization failed to find a feasible trajectory.");
            return false;
        }

        traj_ = std::make_unique<TrajectoryWrapper<Degree>>(candidate);
        trajValid_ = true;
        return true;
    }

    bool TrajOptNode::replan(const Eigen::Vector2d &pos,
                              const Eigen::Vector2d &vel,
                              const Eigen::Vector2d &acc,
                              const std::vector<Eigen::MatrixX3d> &hPolys,
                              const Eigen::Vector2d &goal)
    {
        if (costOrder_ == 4)
        {
            return replanImpl<4>(pos, vel, acc, hPolys, goal);
        }
        return replanImpl<3>(pos, vel, acc, hPolys, goal);
    }

    // Explicit instantiations since replanImpl is defined here, not in the header.
    template bool TrajOptNode::replanImpl<3>(const Eigen::Vector2d &,
                                              const Eigen::Vector2d &,
                                              const Eigen::Vector2d &,
                                              const std::vector<Eigen::MatrixX3d> &,
                                              const Eigen::Vector2d &);
    template bool TrajOptNode::replanImpl<4>(const Eigen::Vector2d &,
                                              const Eigen::Vector2d &,
                                              const Eigen::Vector2d &,
                                              const std::vector<Eigen::MatrixX3d> &,
                                              const Eigen::Vector2d &);

    void TrajOptNode::publishStopCommand()
    {
        publishTwistCommand(0.0, 0.0);
    }

    void TrajOptNode::publishTwistCommand(double v, double omega)
    {
        geometry_msgs::Twist cmd;
        cmd.linear.x = clamp(v, -vMax_, vMax_);
        cmd.linear.y = 0.0;
        cmd.linear.z = 0.0;
        cmd.angular.x = 0.0;
        cmd.angular.y = 0.0;
        cmd.angular.z = clamp(omega, -omgMax_, omgMax_);
        cmdPub_.publish(cmd);
    }

    void TrajOptNode::publishVisualization()
    {
        nav_msgs::Path path;
        path.header.frame_id = "world";
        path.header.stamp = ros::Time::now();

        const double totalDur = traj_->getTotalDuration();
        const double dt = 0.05;
        for (double t = 0.0; t <= totalDur; t += dt)
        {
            const Eigen::Vector2d p = traj_->getPos(t);
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = p.x();
            pose.pose.position.y = p.y();
            pose.pose.position.z = vizHeight_;
            pose.pose.orientation.w = 1.0;
            path.poses.push_back(pose);
        }

        vizPub_.publish(path);
    }

    // Unicycle trajectory tracking (Kanayama-style feedback).
    //
    //   e_x =  cos(yaw)*dx + sin(yaw)*dy      (along-track error, body frame)
    //   e_y = -sin(yaw)*dx + cos(yaw)*dy      (cross-track error, body frame)
    //   e_t = wrap(yaw_ref - yaw)
    //
    //   v     = |v_ref| * cos(e_t) + k_x * e_x
    //   omega = omega_ref + |v_ref| * (k_y * e_y + k_theta * sin(e_t))
    //
    // The cross-track term is scaled by the reference speed because a
    // stationary reference gives no lever arm to correct lateral error with --
    // that case is handled by the in-place rotation branch instead.
    std::pair<double, double> TrajOptNode::calculateCommand(const Eigen::Vector2d &refPos,
                                                            const Eigen::Vector2d &refVel,
                                                            const Eigen::Vector2d &refAcc,
                                                            const Eigen::Vector2d &pos,
                                                            double yaw)
    {
        const double refSpeed = refVel.norm();

        double refYaw;
        if (refSpeed < lowSpeedThreshold_)
        {
            // No meaningful direction of travel to align to; keep the last one
            // so the robot doesn't spin chasing numerical noise.
            refYaw = hasLastRefYaw_ ? lastRefYaw_ : yaw;
        }
        else
        {
            refYaw = std::atan2(refVel.y(), refVel.x());
            lastRefYaw_ = refYaw;
            hasLastRefYaw_ = true;
        }

        const double eTheta = wrapAngle(refYaw - yaw);

        // Reference yaw rate from the path curvature: omega = (v x a) / |v|^2.
        double refOmega = 0.0;
        if (refSpeed > lowSpeedThreshold_)
        {
            refOmega = (refVel.x() * refAcc.y() - refVel.y() * refAcc.x()) /
                       (refSpeed * refSpeed);
        }

        // Pointing the wrong way: rotate in place first. Driving forward with a
        // large heading error would take the robot away from the corridor
        // before the feedback could pull it back.
        if (std::fabs(eTheta) > headingAlignTol_)
        {
            return {0.0, kAlign_ * eTheta};
        }

        const Eigen::Vector2d delta = refPos - pos;
        const double c = std::cos(yaw);
        const double s = std::sin(yaw);
        const double eX = c * delta.x() + s * delta.y();
        const double eY = -s * delta.x() + c * delta.y();

        const double v = refSpeed * std::cos(eTheta) + kX_ * eX;
        const double omega = refOmega +
                             refSpeed * (kY_ * eY + kTheta_ * std::sin(eTheta));

        return {v, omega};
    }

    void TrajOptNode::controlLoopCallback(const ros::TimerEvent & /*event*/)
    {
        bool hasOdom, hasPoly;
        Eigen::Vector2d pos, vel, goal;
        double yaw;
        std::vector<Eigen::MatrixX3d> hPolys;
        {
            std::lock_guard<std::mutex> lock(dataMutex_);
            hasOdom = hasOdom_;
            hasPoly = hasPoly_;
            pos = odomPos_;
            vel = odomVel_;
            yaw = odomYaw_;
            goal = goal_;
            hPolys = hPolys_;
        }

        if (!hasOdom)
        {
            ROS_WARN_THROTTLE(2.0, "trajectory_server_node: waiting for Odometry data...");
            return;
        }

        if (!hasPoly)
        {
            ROS_INFO_THROTTLE(2.0, "trajectory_server_node: waiting for Polyhedrons. Holding position.");
            publishStopCommand();
            return;
        }

        if ((goal - pos).norm() <= goalReachedThreshold_)
        {
            ROS_INFO_THROTTLE(2.0, "trajectory_server_node: Reached goal!");
            publishStopCommand();
            return;
        }

        const ros::Time now = ros::Time::now();
        const bool timeToReplan = isFirstRun_ || (now - lastReplanTime_).toSec() >= replanPeriod_;

        if (timeToReplan)
        {
            // Always re-seed from the robot's actual current state; see the
            // comment in replanImpl() for why acceleration is zero rather
            // than carried forward from the previous solve.
            const Eigen::Vector2d initAcc = Eigen::Vector2d::Zero();

            if (replan(pos, vel, initAcc, hPolys, goal))
            {
                trajStartTime_ = now;
                isFirstRun_ = false;
            }
            else if (!trajValid_)
            {
                // No prior valid trajectory to fall back on; stop.
                publishStopCommand();
                return;
            }
            lastReplanTime_ = now;
        }

        if (!trajValid_)
        {
            publishStopCommand();
            return;
        }

        const double t = std::max(0.0, (now - trajStartTime_).toSec());
        const double totalDur = traj_->getTotalDuration();
        const double tClamped = std::min(t, totalDur);

        const Eigen::Vector2d desPos = traj_->getPos(tClamped);
        const Eigen::Vector2d desVel = traj_->getVel(tClamped);
        const Eigen::Vector2d desAcc = traj_->getAcc(tClamped);

        const std::pair<double, double> cmd = calculateCommand(desPos, desVel, desAcc, pos, yaw);
        publishTwistCommand(cmd.first, cmd.second);

        publishVisualization();
    }

} // namespace trajectory_server



#elif PATHCOVER_DIM == 3
#include "trajectory_server.hpp"

#include <algorithm>
#include <cmath>

namespace trajectory_server
{

    namespace
    {
        double wrapAngle(double angle)
        {
            while (angle > M_PI)
            {
                angle -= 2.0 * M_PI;
            }
            while (angle < -M_PI)
            {
                angle += 2.0 * M_PI;
            }
            return angle;
        }

        template <typename T>
        T getRequiredParam(ros::NodeHandle &nh, const std::string &name)
        {
            T value;
            if (!nh.getParam(name, value))
            {
                ROS_FATAL("traj_opt_node: required parameter '~%s' not set", name.c_str());
                ros::shutdown();
                std::exit(1);
            }
            return value;
        }
    } // namespace

    TrajOptNode::TrajOptNode(ros::NodeHandle &nh)
        : nh_(nh),
          hasOdom_(false),
          odomPos_(Eigen::Vector3d::Zero()),
          odomVel_(Eigen::Vector3d::Zero()),
          hasPoly_(false),
          goal_(Eigen::Vector3d::Zero()),
          isFirstRun_(true),
          trajValid_(false),
          lastYaw_(0.0),
          lastYawDot_(0.0)
    {
        // --- Topics (same names/semantics as trajectory_server) ---
        odomTopic_ = getRequiredParam<std::string>(nh_, "OdometryTopic");
        trajTopic_ = getRequiredParam<std::string>(nh_, "TrajectoryTopic");
        polyTopic_ = getRequiredParam<std::string>(nh_, "PolyhedraTopic");

        // --- Dynamics / GCOPTER bounds ---
        vMax_ = getRequiredParam<double>(nh_, "v_max");
        omgMax_ = nh_.param("omg_max", 3.0);
        thetaMax_ = nh_.param("theta_max", 0.7854);
        thrustMin_ = nh_.param("thrust_min", 2.0);
        thrustMax_ = nh_.param("thrust_max", 20.0);

        magnitudeBounds_.resize(5);
        magnitudeBounds_ << vMax_, omgMax_, thetaMax_, thrustMin_, thrustMax_;

        double weightPos = nh_.param("weight_pos", 1.0e4);
        double weightVel = nh_.param("weight_vel", 1.0e4);
        double weightOmg = nh_.param("weight_omg", 1.0e4);
        double weightTheta = nh_.param("weight_theta", 1.0e4);
        double weightThrust = nh_.param("weight_thrust", 1.0e4);
        penaltyWeights_.resize(5);
        penaltyWeights_ << weightPos, weightVel, weightOmg, weightTheta, weightThrust;

        double mass = nh_.param("vehicle_mass", 1.0);
        double gravAcc = nh_.param("grav_acc", 9.81);
        double horizDrag = nh_.param("horiz_drag", 0.0);
        double vertDrag = nh_.param("vert_drag", 0.0);
        double parasDrag = nh_.param("paras_drag", 0.0);
        double speedEps = nh_.param("speed_eps", 1.0e-3);
        physicalParams_.resize(6);
        physicalParams_ << mass, gravAcc, horizDrag, vertDrag, parasDrag, speedEps;

        weightT_ = nh_.param("weight_time", 100.0);
        smoothingEps_ = nh_.param("smoothing_eps", 0.01);
        integralRes_ = nh_.param("integral_resolution", 16);
        lengthPerPiece_ = nh_.param("length_per_piece", 2.0);
        relCostTol_ = nh_.param("rel_cost_tol", 1.0e-4);

        costOrder_ = nh_.param("cost_order", 3);
        if (costOrder_ != 3 && costOrder_ != 4)
        {
            ROS_FATAL("traj_opt_node: cost_order must be 3 (minimum jerk, degree-5) "
                      "or 4 (minimum snap, degree-7), got %d",
                      costOrder_);
            ros::shutdown();
            std::exit(1);
        }

        safetyMargin_ = nh_.param("SafetyMargin", 0.3);

        replanPeriod_ = nh_.param("ReplanPeriod", 0.5);
        goalReachedThreshold_ = nh_.param("GoalReachedThreshold", 0.2);

        yawDotMax_ = nh_.param("YawDotMax", M_PI);
        lowSpeedThreshold_ = nh_.param("LowSpeedThreshold", 0.1);

        // --- Publishers & Subscribers (identical topics to trajectory_server) ---
        trajPub_ = nh_.advertise<quadrotor_msgs::TrajectoryCommand>(trajTopic_, 1, true);
        vizPub_ = nh_.advertise<nav_msgs::Path>("/colored_trajectory", 1);
        polySub_ = nh_.subscribe(polyTopic_, 1, &TrajOptNode::polytopesCallback, this);
        odomSub_ = nh_.subscribe(odomTopic_, 1, &TrajOptNode::odomCallback, this);

        ros::Time now = ros::Time::now();
        lastReplanTime_ = now;
        lastYawTime_ = now;

        controlTimer_ = nh_.createTimer(ros::Duration(1.0 / 100.0), &TrajOptNode::controlLoopCallback, this);

        ROS_INFO("traj_opt_node (GCOPTER receding-horizon, cost_order=%d) is running...", costOrder_);
    }

    void TrajOptNode::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        std::lock_guard<std::mutex> lock(dataMutex_);
        odomPos_ << msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z;
        odomVel_ << msg->twist.twist.linear.x,
            msg->twist.twist.linear.y,
            msg->twist.twist.linear.z;
        hasOdom_ = true;
    }

    void TrajOptNode::polytopesCallback(const polytope_msgs::Polytopes::ConstPtr &msg)
    {
        std::lock_guard<std::mutex> lock(dataMutex_);

        if (msg->polytope.empty())
        {
            hasPoly_ = false;
            hPolys_.clear();
            return;
        }

        std::vector<Eigen::MatrixX4d> hPolys;
        hPolys.reserve(msg->polytope.size());

        for (const auto &poly : msg->polytope)
        {
            const int rows = static_cast<int>(poly.A.size() / 3);
            Eigen::MatrixX4d hPoly(rows, 4);
            // A is row-major flattened (rows x 3): Ax <= b  <=>  n.dot(x) + d <= 0
            // with n = A_row, d = -b_row. Shrink b by safetyMargin_ * ||A_row||
            // (not just safetyMargin_) so the inward shift is exactly
            // safetyMargin_ meters regardless of whether A_row is normalized;
            // GCOPTER itself normalizes each row by ||A_row|| in setup().
            for (int i = 0; i < rows; i++)
            {
                const double ax = poly.A[3 * i + 0];
                const double ay = poly.A[3 * i + 1];
                const double az = poly.A[3 * i + 2];
                const double normA = std::sqrt(ax * ax + ay * ay + az * az);
                const double safeB = poly.b[i] - safetyMargin_ * normA;

                hPoly(i, 0) = ax;
                hPoly(i, 1) = ay;
                hPoly(i, 2) = az;
                hPoly(i, 3) = -safeB;
            }
            hPolys.push_back(hPoly);
        }

        if (msg->goal.size() < 3)
        {
            ROS_WARN_THROTTLE(2.0, "traj_opt_node: received Polytopes message without a valid goal, ignoring.");
            hasPoly_ = false;
            hPolys_.clear();
            return;
        }

        hPolys_ = std::move(hPolys);
        goal_ << msg->goal[0], msg->goal[1], msg->goal[2];
        hasPoly_ = true;
    }

    template <int S>
    bool TrajOptNode::replanImpl(const Eigen::Vector3d &pos,
                                  const Eigen::Vector3d &vel,
                                  const Eigen::Vector3d &acc,
                                  const std::vector<Eigen::MatrixX4d> &hPolys,
                                  const Eigen::Vector3d &goal)
    {
        using BoundaryT = typename MincoTraits<S>::Boundary;
        constexpr int Degree = MincoTraits<S>::Degree;

        // Boundary conditions are seeded purely from the drone's actual
        // current state (position + velocity from odometry) every replan,
        // exactly like trajectory_server. Acceleration (and jerk, for the
        // snap-cost case) has no direct sensor measurement here, so it is
        // always zero rather than carried forward from the previous solve's
        // own evaluated trajectory -- carrying it forward was feeding each
        // solve's own artifacts back in as the next replan's hard boundary
        // condition, compounding any distortion across replan cycles.
        BoundaryT initialState, terminalState;
        initialState.col(0) = pos;
        initialState.col(1) = vel;
        initialState.col(2) = acc;
        if constexpr (S == 4)
        {
            initialState.col(3).setZero();
        }

        // Terminal velocity/acceleration(/jerk) are zero, matching
        // trajectory_server exactly. The corridor's published goal can be a
        // horizon-truncated point rather than the true mission destination,
        // but since we replan every replanPeriod_ seconds and only execute
        // the near-term part of each solve, the terminal condition mostly
        // just shapes the unexecuted tail -- it should stay simple/neutral
        // rather than guessing a flow-through direction that may fight the
        // corridor's actual curvature near its end.
        terminalState.col(0) = goal;
        terminalState.col(1).setZero();
        terminalState.col(2).setZero();
        if constexpr (S == 4)
        {
            terminalState.col(3).setZero();
        }

        GcopterSolver<S> solver;
        if (!solver.setup(weightT_, initialState, terminalState, hPolys,
                          lengthPerPiece_, smoothingEps_, integralRes_,
                          magnitudeBounds_, penaltyWeights_, physicalParams_))
        {
            ROS_WARN_THROTTLE(2.0, "traj_opt_node: GCOPTER setup failed (corridor likely degenerate/non-overlapping).");
            return false;
        }

        Trajectory<Degree> candidate;
        const double cost = solver.optimize(candidate, relCostTol_);

        if (!std::isfinite(cost) || candidate.getPieceNum() == 0)
        {
            ROS_WARN_THROTTLE(2.0, "traj_opt_node: GCOPTER optimization failed to find a feasible trajectory.");
            return false;
        }

        traj_ = std::make_unique<TrajectoryWrapper<Degree>>(candidate);
        trajValid_ = true;
        return true;
    }

    bool TrajOptNode::replan(const Eigen::Vector3d &pos,
                              const Eigen::Vector3d &vel,
                              const Eigen::Vector3d &acc,
                              const std::vector<Eigen::MatrixX4d> &hPolys,
                              const Eigen::Vector3d &goal)
    {
        if (costOrder_ == 4)
        {
            return replanImpl<4>(pos, vel, acc, hPolys, goal);
        }
        return replanImpl<3>(pos, vel, acc, hPolys, goal);
    }

    // Explicit instantiations since replanImpl is defined here, not in the header.
    template bool TrajOptNode::replanImpl<3>(const Eigen::Vector3d &,
                                              const Eigen::Vector3d &,
                                              const Eigen::Vector3d &,
                                              const std::vector<Eigen::MatrixX4d> &,
                                              const Eigen::Vector3d &);
    template bool TrajOptNode::replanImpl<4>(const Eigen::Vector3d &,
                                              const Eigen::Vector3d &,
                                              const Eigen::Vector3d &,
                                              const std::vector<Eigen::MatrixX4d> &,
                                              const Eigen::Vector3d &);

    void TrajOptNode::publishHoverCommand(const Eigen::Vector3d &pos)
    {
        quadrotor_msgs::TrajectoryCommand traj;
        traj.header.stamp = ros::Time::now();
        traj.header.frame_id = "world";

        traj.position.x = pos.x();
        traj.position.y = pos.y();
        traj.position.z = pos.z();

        traj.velocity.x = traj.velocity.y = traj.velocity.z = 0.0;
        traj.acceleration.x = traj.acceleration.y = traj.acceleration.z = 0.0;
        traj.jerk.x = traj.jerk.y = traj.jerk.z = 0.0;
        traj.snap.x = traj.snap.y = traj.snap.z = 0.0;

        traj.yaw = lastYaw_;
        traj.yaw_velocity = 0.0;
        traj.yaw_acceleration = 0.0;

        trajPub_.publish(traj);
    }

    void TrajOptNode::publishTrajectoryCommand(const Eigen::Vector3d &pos,
                                                const Eigen::Vector3d &vel,
                                                const Eigen::Vector3d &acc,
                                                const Eigen::Vector3d &jer,
                                                double yaw,
                                                double yawDot)
    {
        quadrotor_msgs::TrajectoryCommand traj;
        traj.header.stamp = ros::Time::now();
        traj.header.frame_id = "world";

        traj.position.x = pos.x();
        traj.position.y = pos.y();
        traj.position.z = pos.z();

        traj.velocity.x = vel.x();
        traj.velocity.y = vel.y();
        traj.velocity.z = vel.z();

        traj.acceleration.x = acc.x();
        traj.acceleration.y = acc.y();
        traj.acceleration.z = acc.z();

        traj.jerk.x = jer.x();
        traj.jerk.y = jer.y();
        traj.jerk.z = jer.z();

        traj.snap.x = traj.snap.y = traj.snap.z = 0.0;

        traj.yaw = yaw;
        traj.yaw_velocity = yawDot;
        traj.yaw_acceleration = 0.0;

        trajPub_.publish(traj);
    }

    void TrajOptNode::publishVisualization()
    {
        nav_msgs::Path path;
        path.header.frame_id = "world";
        path.header.stamp = ros::Time::now();

        const double totalDur = traj_->getTotalDuration();
        const double dt = 0.05;
        for (double t = 0.0; t <= totalDur; t += dt)
        {
            const Eigen::Vector3d p = traj_->getPos(t);
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = p.x();
            pose.pose.position.y = p.y();
            pose.pose.position.z = p.z();
            pose.pose.orientation.w = 1.0;
            path.poses.push_back(pose);
        }

        vizPub_.publish(path);
    }

    std::pair<double, double> TrajOptNode::calculateYaw(const Eigen::Vector3d &velDes, double dt)
    {
        const double vx = velDes.x();
        const double vy = velDes.y();
        const double horizSpeed = std::sqrt(vx * vx + vy * vy);

        double yawTarget;
        if (horizSpeed < lowSpeedThreshold_)
        {
            yawTarget = lastYaw_;
        }
        else
        {
            yawTarget = std::atan2(vy, vx);
        }

        const double maxYawChange = dt > 1e-6 ? yawDotMax_ * dt : 0.0;
        const double diff = wrapAngle(yawTarget - lastYaw_);

        double yaw, yawDot;
        if (diff > maxYawChange)
        {
            yaw = wrapAngle(lastYaw_ + maxYawChange);
            yawDot = yawDotMax_;
        }
        else if (diff < -maxYawChange)
        {
            yaw = wrapAngle(lastYaw_ - maxYawChange);
            yawDot = -yawDotMax_;
        }
        else
        {
            yaw = wrapAngle(lastYaw_ + diff);
            yawDot = dt > 1e-6 ? diff / dt : 0.0;
        }

        if (std::fabs(diff) <= maxYawChange)
        {
            yaw = 0.5 * lastYaw_ + 0.5 * yaw;
        }
        yawDot = 0.5 * lastYawDot_ + 0.5 * yawDot;

        return {yaw, yawDot};
    }

    void TrajOptNode::controlLoopCallback(const ros::TimerEvent & /*event*/)
    {
        bool hasOdom, hasPoly;
        Eigen::Vector3d pos, vel, goal;
        std::vector<Eigen::MatrixX4d> hPolys;
        {
            std::lock_guard<std::mutex> lock(dataMutex_);
            hasOdom = hasOdom_;
            hasPoly = hasPoly_;
            pos = odomPos_;
            vel = odomVel_;
            goal = goal_;
            hPolys = hPolys_;
        }

        if (!hasOdom)
        {
            ROS_WARN_THROTTLE(2.0, "traj_opt_node: waiting for Odometry data...");
            return;
        }

        if (!hasPoly)
        {
            ROS_INFO_THROTTLE(2.0, "traj_opt_node: waiting for Polyhedrons. Holding position.");
            publishHoverCommand(pos);
            return;
        }

        const ros::Time now = ros::Time::now();
        const bool timeToReplan = isFirstRun_ || (now - lastReplanTime_).toSec() >= replanPeriod_;

        if (timeToReplan)
        {
            // Always re-seed from the drone's actual current state; see the
            // comment in replanImpl() for why acceleration is zero rather
            // than carried forward from the previous solve.
            const Eigen::Vector3d initAcc = Eigen::Vector3d::Zero();

            if (replan(pos, vel, initAcc, hPolys, goal))
            {
                trajStartTime_ = now;
                isFirstRun_ = false;
            }
            else if (!trajValid_)
            {
                // No prior valid trajectory to fall back on; hover.
                publishHoverCommand(pos);
                return;
            }
            lastReplanTime_ = now;
        }

        if (!trajValid_)
        {
            publishHoverCommand(pos);
            return;
        }

        const double t = std::max(0.0, (now - trajStartTime_).toSec());
        const double totalDur = traj_->getTotalDuration();
        const double tClamped = std::min(t, totalDur);

        const Eigen::Vector3d desPos = traj_->getPos(tClamped);
        const Eigen::Vector3d desVel = traj_->getVel(tClamped);
        const Eigen::Vector3d desAcc = traj_->getAcc(tClamped);
        const Eigen::Vector3d desJer = traj_->getJer(tClamped);

        const double dtYaw = (now - lastYawTime_).toSec();
        lastYawTime_ = now;
        const std::pair<double, double> yawResult = calculateYaw(desVel, dtYaw);
        const double yaw = yawResult.first;
        const double yawDot = yawResult.second;
        lastYaw_ = yaw;
        lastYawDot_ = yawDot;

        if ((goal - pos).norm() > goalReachedThreshold_)
        {
            publishTrajectoryCommand(desPos, desVel, desAcc, desJer, yaw, yawDot);
        }
        else
        {
            ROS_INFO_THROTTLE(2.0, "traj_opt_node: Reached goal!");
        }

        publishVisualization();
    }

} // namespace traj_opt



#else
#error "trajectory_server supports only PATHCOVER_DIM=2 or 3"
#endif
