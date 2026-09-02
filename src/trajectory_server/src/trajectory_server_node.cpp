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

        // Declares and reads a required parameter, throwing std::runtime_error
        // (after an RCLCPP_FATAL) if it isn't set -- caught in main().
        template <typename T>
        T getRequiredParam(rclcpp::Node &node, const std::string &name)
        {
            T value;
            try
            {
                node.declare_parameter<T>(name);
                if (node.get_parameter(name, value))
                {
                    return value;
                }
            }
            catch (const std::exception &)
            {
                // fall through to the fatal error below
            }
            const std::string msg = "traj_opt_node: required parameter '" + name + "' not set";
            RCLCPP_FATAL(node.get_logger(), "%s", msg.c_str());
            throw std::runtime_error(msg);
        }
    } // namespace

    TrajOptNode::TrajOptNode(const rclcpp::NodeOptions &options)
        : rclcpp::Node("traj_opt_node", options),
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
        odomTopic_ = getRequiredParam<std::string>(*this, "OdometryTopic");
        trajTopic_ = getRequiredParam<std::string>(*this, "TrajectoryTopic");
        polyTopic_ = getRequiredParam<std::string>(*this, "PolyhedraTopic");

        // --- Dynamics / GCOPTER bounds ---
        vMax_ = getRequiredParam<double>(*this, "v_max");
        omgMax_ = this->declare_parameter("omg_max", 3.0);
        thetaMax_ = this->declare_parameter("theta_max", 0.7854);
        thrustMin_ = this->declare_parameter("thrust_min", 2.0);
        thrustMax_ = this->declare_parameter("thrust_max", 20.0);

        magnitudeBounds_.resize(5);
        magnitudeBounds_ << vMax_, omgMax_, thetaMax_, thrustMin_, thrustMax_;

        double weightPos = this->declare_parameter("weight_pos", 1.0e4);
        double weightVel = this->declare_parameter("weight_vel", 1.0e4);
        double weightOmg = this->declare_parameter("weight_omg", 1.0e4);
        double weightTheta = this->declare_parameter("weight_theta", 1.0e4);
        double weightThrust = this->declare_parameter("weight_thrust", 1.0e4);
        penaltyWeights_.resize(5);
        penaltyWeights_ << weightPos, weightVel, weightOmg, weightTheta, weightThrust;

        double mass = this->declare_parameter("vehicle_mass", 1.0);
        double gravAcc = this->declare_parameter("grav_acc", 9.81);
        double horizDrag = this->declare_parameter("horiz_drag", 0.0);
        double vertDrag = this->declare_parameter("vert_drag", 0.0);
        double parasDrag = this->declare_parameter("paras_drag", 0.0);
        double speedEps = this->declare_parameter("speed_eps", 1.0e-3);
        physicalParams_.resize(6);
        physicalParams_ << mass, gravAcc, horizDrag, vertDrag, parasDrag, speedEps;

        weightT_ = this->declare_parameter("weight_time", 100.0);
        smoothingEps_ = this->declare_parameter("smoothing_eps", 0.01);
        integralRes_ = this->declare_parameter("integral_resolution", 16);
        lengthPerPiece_ = this->declare_parameter("length_per_piece", 2.0);
        relCostTol_ = this->declare_parameter("rel_cost_tol", 1.0e-4);

        costOrder_ = this->declare_parameter("cost_order", 3);
        if (costOrder_ != 3 && costOrder_ != 4)
        {
            const std::string msg = "traj_opt_node: cost_order must be 3 (minimum jerk, degree-5) "
                                     "or 4 (minimum snap, degree-7), got " + std::to_string(costOrder_);
            RCLCPP_FATAL(this->get_logger(), "%s", msg.c_str());
            throw std::runtime_error(msg);
        }

        safetyMargin_ = this->declare_parameter("SafetyMargin", 0.3);

        replanPeriod_ = this->declare_parameter("ReplanPeriod", 0.5);
        goalReachedThreshold_ = this->declare_parameter("GoalReachedThreshold", 0.2);

        polyhedraTimeout_ = this->declare_parameter("PolyhedraTimeout", 1.0);

        yawDotMax_ = this->declare_parameter("YawDotMax", M_PI);
        lowSpeedThreshold_ = this->declare_parameter("LowSpeedThreshold", 0.1);

        // --- Publishers & Subscribers (identical topics to trajectory_server) ---
        trajPub_ = this->create_publisher<quadrotor_msgs::msg::TrajectoryCommand>(trajTopic_, rclcpp::QoS(1).transient_local());
        vizPub_ = this->create_publisher<nav_msgs::msg::Path>("/colored_trajectory", 1);
        polySub_ = this->create_subscription<polytope_msgs::msg::PolytopeArray>(
            polyTopic_, rclcpp::QoS(1),
            std::bind(&TrajOptNode::polytopesCallback, this, std::placeholders::_1));
        odomSub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odomTopic_, rclcpp::QoS(1),
            std::bind(&TrajOptNode::odomCallback, this, std::placeholders::_1));

        rclcpp::Time now = this->now();
        lastReplanTime_ = now;
        lastYawTime_ = now;

        controlTimer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / 100.0),
            std::bind(&TrajOptNode::controlLoopCallback, this));

        RCLCPP_INFO(this->get_logger(), "traj_opt_node (GCOPTER receding-horizon, cost_order=%d) is running...", costOrder_);
    }

    void TrajOptNode::odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg)
    {
        std::lock_guard<std::mutex> lock(dataMutex_);
        odomPos_ << msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z;

        // nav_msgs/Odometry.twist is expressed in the body (child_frame_id)
        // frame per REP-105 -- which is what gz-sim's OdometryPublisher
        // plugin publishes here (robot_base_frame=base_link) -- while GCOPTER's
        // boundary conditions are all in the world frame the rest of this node
        // works in. Feeding the raw body-frame twist straight in as a
        // world-frame velocity biases every replan's initial-velocity
        // constraint toward the body +X axis (roughly the travel direction,
        // since calculateYaw keeps yaw aligned with velocity), which is what
        // was causing the persistent +X drift. Rotate it into world frame first.
        const Eigen::Quaterniond q(msg->pose.pose.orientation.w,
                                    msg->pose.pose.orientation.x,
                                    msg->pose.pose.orientation.y,
                                    msg->pose.pose.orientation.z);
        const Eigen::Vector3d bodyVel(msg->twist.twist.linear.x,
                                       msg->twist.twist.linear.y,
                                       msg->twist.twist.linear.z);
        odomVel_ = q * bodyVel;
        hasOdom_ = true;
    }

    void TrajOptNode::polytopesCallback(const polytope_msgs::msg::PolytopeArray::ConstSharedPtr &msg)
    {
        std::lock_guard<std::mutex> lock(dataMutex_);

        // Mark freshness on every received message, valid or not -- this is
        // what the staleness watchdog in controlLoopCallback() checks to
        // detect a corridor_planning that has stopped publishing entirely
        // (crashed, killed), as opposed to one that's merely reporting "no
        // corridor yet".
        lastPolyRecvTime_ = this->now();

        if (msg->polytopes.empty())
        {
            hasPoly_ = false;
            hPolys_.clear();
            return;
        }

        std::vector<Eigen::MatrixX4d> hPolys;
        hPolys.reserve(msg->polytopes.size());

        for (const auto &poly : msg->polytopes)
        {
            const int rows = static_cast<int>(poly.a.size() / 3);
            Eigen::MatrixX4d hPoly(rows, 4);
            // a is row-major flattened (rows x 3): Ax <= b  <=>  n.dot(x) + d <= 0
            // with n = a_row, d = -b_row. Shrink b by safetyMargin_ * ||a_row||
            // (not just safetyMargin_) so the inward shift is exactly
            // safetyMargin_ meters regardless of whether a_row is normalized;
            // GCOPTER itself normalizes each row by ||a_row|| in setup().
            for (int i = 0; i < rows; i++)
            {
                const double ax = poly.a[3 * i + 0];
                const double ay = poly.a[3 * i + 1];
                const double az = poly.a[3 * i + 2];
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
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                  "traj_opt_node: received Polytopes message without a valid goal, ignoring.");
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
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                  "traj_opt_node: GCOPTER setup failed (corridor likely degenerate/non-overlapping).");
            return false;
        }

        Trajectory<Degree> candidate;
        const double cost = solver.optimize(candidate, relCostTol_);

        if (!std::isfinite(cost) || candidate.getPieceNum() == 0)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                  "traj_opt_node: GCOPTER optimization failed to find a feasible trajectory.");
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
        quadrotor_msgs::msg::TrajectoryCommand traj;
        traj.header.stamp = this->now();
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

        trajPub_->publish(traj);
    }

    void TrajOptNode::publishTrajectoryCommand(const Eigen::Vector3d &pos,
                                                const Eigen::Vector3d &vel,
                                                const Eigen::Vector3d &acc,
                                                const Eigen::Vector3d &jer,
                                                double yaw,
                                                double yawDot)
    {
        quadrotor_msgs::msg::TrajectoryCommand traj;
        traj.header.stamp = this->now();
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

        trajPub_->publish(traj);
    }

    void TrajOptNode::publishVisualization()
    {
        nav_msgs::msg::Path path;
        path.header.frame_id = "world";
        path.header.stamp = this->now();

        const double totalDur = traj_->getTotalDuration();
        const double dt = 0.05;
        for (double t = 0.0; t <= totalDur; t += dt)
        {
            const Eigen::Vector3d p = traj_->getPos(t);
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = p.x();
            pose.pose.position.y = p.y();
            pose.pose.position.z = p.z();
            pose.pose.orientation.w = 1.0;
            path.poses.push_back(pose);
        }

        vizPub_->publish(path);
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

    void TrajOptNode::controlLoopCallback()
    {
        bool hasOdom, hasPoly;
        Eigen::Vector3d pos, vel, goal;
        std::vector<Eigen::MatrixX4d> hPolys;
        rclcpp::Time lastPolyRecvTime;
        {
            std::lock_guard<std::mutex> lock(dataMutex_);
            hasOdom = hasOdom_;
            hasPoly = hasPoly_;
            pos = odomPos_;
            vel = odomVel_;
            goal = goal_;
            hPolys = hPolys_;
            lastPolyRecvTime = lastPolyRecvTime_;
        }

        if (!hasOdom)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                  "traj_opt_node: waiting for Odometry data...");
            return;
        }

        // Watchdog: if corridor_planning stops publishing entirely (crash,
        // killed, never restarted), hPolys_/goal_ would otherwise stay frozen
        // at their last received values, and this node would keep flying
        // toward that stale goal forever with no way to notice. Treat a
        // too-old "last received" timestamp the same as never having
        // received anything.
        if (hasPoly && lastPolyRecvTime.nanoseconds() > 0 &&
            (this->now() - lastPolyRecvTime).seconds() > polyhedraTimeout_)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                  "traj_opt_node: no Polyhedra message received in over %.1fs "
                                  "(corridor_planning may have died) -- discarding stale corridor/goal "
                                  "and holding position.",
                                  polyhedraTimeout_);
            hasPoly = false;
            std::lock_guard<std::mutex> lock(dataMutex_);
            hasPoly_ = false;
            hPolys_.clear();
        }

        if (!hasPoly)
        {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                  "traj_opt_node: waiting for Polyhedrons. Holding position.");
            publishHoverCommand(pos);
            return;
        }

        const rclcpp::Time now = this->now();
        const bool timeToReplan = isFirstRun_ || (now - lastReplanTime_).seconds() >= replanPeriod_;

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

        const double t = std::max(0.0, (now - trajStartTime_).seconds());
        const double totalDur = traj_->getTotalDuration();
        const double tClamped = std::min(t, totalDur);

        const Eigen::Vector3d desPos = traj_->getPos(tClamped);
        const Eigen::Vector3d desVel = traj_->getVel(tClamped);
        const Eigen::Vector3d desAcc = traj_->getAcc(tClamped);
        const Eigen::Vector3d desJer = traj_->getJer(tClamped);

        const double dtYaw = (now - lastYawTime_).seconds();
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
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                  "traj_opt_node: Reached goal!");
        }

        publishVisualization();
    }

} // namespace trajectory_server


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<trajectory_server::TrajOptNode>();
        rclcpp::spin(node);
    } catch (const std::exception &e) {
        RCLCPP_FATAL(rclcpp::get_logger("traj_opt_node"), "%s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
