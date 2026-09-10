#ifndef PATHCOVER_DIM
#define PATHCOVER_DIM 3
#endif

#if PATHCOVER_DIM == 2
#include "trajectory_server.hpp"

#include <tf/transform_datatypes.h>

#include <algorithm>
#include <cmath>
#include <limits>

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
        piecesPerPolytope_ = nh_.param("PiecesPerPolytope", 1);
        relCostTol_ = nh_.param("rel_cost_tol", 1.0e-4);
        maxIterations_ = nh_.param("max_iterations", 1000);

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
        corridorSliceHeight_ = nh_.param("CorridorSliceHeight", 0.0);

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
            // A is a row-major (rows x stride) flattening and b has one entry
            // per row, so the stride tells us the dimension the corridor was
            // generated in. Deriving it (rather than assuming 2) is what lets
            // the planar build consume the 3-D corridor that corridor_planning
            // actually publishes: that node is hardcoded to
            // Eigen::Matrix<double, -1, 3>, so its rows carry three
            // coefficients even when the robot is a ground vehicle.
            if (poly.b.empty() || poly.A.size() % poly.b.size() != 0)
            {
                ROS_WARN_THROTTLE(2.0,
                                  "trajectory_server_node: malformed Polytope (A has %zu entries, "
                                  "b has %zu); ignoring corridor.",
                                  poly.A.size(), poly.b.size());
                hasPoly_ = false;
                hPolys_.clear();
                return;
            }

            const int rows = static_cast<int>(poly.b.size());
            const int stride = static_cast<int>(poly.A.size() / poly.b.size());
            if (stride != 2 && stride != 3)
            {
                ROS_WARN_THROTTLE(2.0,
                                  "trajectory_server_node: Polytope rows have %d coefficients; "
                                  "the planar build understands only 2 (planar corridor) or "
                                  "3 (3-D corridor, sliced at CorridorSliceHeight).",
                                  stride);
                hasPoly_ = false;
                hPolys_.clear();
                return;
            }

            // Ax <= b  <=>  n.dot(x) + d <= 0 with n = A_row, d = -b_row.
            // Stored unshrunk, and it stays that way all the way to the
            // solver: GcopterSolver::setup() applies the safety margin to its
            // own penalty copy only.
            //
            // For a 3-D corridor we take the planar cross-section at
            // z = corridorSliceHeight_: substituting that z into
            // ax*x + ay*y + az*z <= b leaves ax*x + ay*y <= b - az*z, which is
            // the exact horizontal slice of the polytope at the height the
            // base drives at. The rows are left unnormalized; GCOPTER's
            // setup() normalizes its own copy by the planar row norm, which
            // after the substitution is the row's true normal length, so the
            // safety margin it then applies is exactly safetyMargin_ metres
            // rather than over-shrinking every face that was tilted in z.
            //
            // A face whose normal is purely vertical (a floor or ceiling
            // plane) slices to a constraint with no x/y dependence at all. It
            // carries no planar information and, worse, its zero row norm
            // would become a division by zero in GcopterSolver::setup(), so it
            // is dropped -- unless it is *violated* at the slice height, which
            // means the polytope simply does not reach that height and the
            // whole corridor has to be rejected.
            Eigen::MatrixX3d hPoly(rows, 3);
            int kept = 0;
            bool emptySlice = false;
            for (int i = 0; i < rows; i++)
            {
                const double ax = poly.A[stride * i + 0];
                const double ay = poly.A[stride * i + 1];
                const double az = stride == 3 ? poly.A[stride * i + 2] : 0.0;

                const double normA = std::sqrt(ax * ax + ay * ay);
                const double bSliced = poly.b[i] - az * corridorSliceHeight_;

                if (normA < 1.0e-9)
                {
                    if (bSliced < 0.0)
                    {
                        emptySlice = true;
                        break;
                    }
                    continue;
                }

                hPoly(kept, 0) = ax;
                hPoly(kept, 1) = ay;
                hPoly(kept, 2) = -bSliced;
                kept++;
            }

            if (emptySlice || kept < 3)
            {
                ROS_WARN_THROTTLE(2.0,
                                  "trajectory_server_node: corridor polytope is empty or degenerate "
                                  "at z = %.2f (CorridorSliceHeight); ignoring corridor.",
                                  corridorSliceHeight_);
                hasPoly_ = false;
                hPolys_.clear();
                return;
            }

            hPolys.push_back(hPoly.topRows(kept));
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

    namespace
    {
        // Signed distance by which pos sits outside hPoly, in metres: the worst
        // per-row (n.x + d)/||n||. <= 0 means inside, so this is an exact
        // containment test. Rows are not assumed normalized -- the corridor
        // arrives however the planner produced it, and GCOPTER only normalizes
        // its own copy inside setup().
        double polytopeViolation(const Eigen::Vector2d &pos, const Eigen::MatrixX3d &hPoly)
        {
            double worst = -std::numeric_limits<double>::infinity();
            for (int i = 0; i < hPoly.rows(); i++)
            {
                const Eigen::Vector2d n = hPoly.row(i).head<2>().transpose();
                const double normN = n.norm();
                if (normN < 1.0e-9)
                {
                    continue;
                }
                worst = std::max(worst, (n.dot(pos) + hPoly(i, 2)) / normN);
            }
            return worst;
        }

        // How far outside hPoly pos is, for RANKING polytopes that all fail the
        // containment test: the Euclidean norm of the per-row overshoot.
        //
        // The plain max above is useless for ranking. A robot that has drifted
        // sideways out of a straight corridor overshoots every polytope
        // laterally by the same amount, so the max ties across all of them and
        // the choice falls to whichever happens to come first -- which can be a
        // polytope far behind or far ahead. Summing in quadrature keeps the
        // along-track terms, which are what actually distinguish the polytope
        // beside the robot from one 50 m down the corridor.
        double polytopeExcess(const Eigen::Vector2d &pos, const Eigen::MatrixX3d &hPoly)
        {
            double sumSq = 0.0;
            for (int i = 0; i < hPoly.rows(); i++)
            {
                const Eigen::Vector2d n = hPoly.row(i).head<2>().transpose();
                const double normN = n.norm();
                if (normN < 1.0e-9)
                {
                    continue;
                }
                const double over = (n.dot(pos) + hPoly(i, 2)) / normN;
                if (over > 0.0)
                {
                    sumSq += over * over;
                }
            }
            return std::sqrt(sumSq);
        }
    } // namespace

    int TrajOptNode::findStartPolytope(const Eigen::Vector2d &pos,
                                       const std::vector<Eigen::MatrixX3d> &hPolys) const
    {
        // Pick the polytope FURTHEST along the corridor that still contains the
        // robot, and let the caller drop everything before it.
        //
        // This is what untangles the knots. The corridor is anchored wherever
        // the robot was when corridor_planning built it, but we replan on our
        // own timer and reuse that corridor until the next one arrives -- by
        // then the robot has driven into the middle of it. Keeping the
        // already-passed polytopes forces MINCO to pin its first waypoints into
        // corridor that is now BEHIND the robot, so the trajectory doubles
        // back before turning for the goal. On a diff-drive base that is worse
        // than on a quadrotor: the tracking law sees a reference heading
        // pointing backwards and spins the robot in place to chase it.
        //
        // Consecutive polytopes overlap, so when the robot sits in an overlap
        // the later one is the right answer: it is the one that makes progress.
        // Skipping ahead stays safe even if the corridor loops back near itself,
        // because each polytope is convex and collision-free -- if the robot
        // is inside polytope k then the start of the trajectory lies in a free
        // convex set no matter how many polytopes were dropped.
        int lastContaining = -1;
        int nearest = 0;
        double nearestExcess = std::numeric_limits<double>::infinity();

        for (int i = 0; i < static_cast<int>(hPolys.size()); i++)
        {
            if (polytopeViolation(pos, hPolys[i]) <= 0.0)
            {
                lastContaining = i;
            }
            // <= so that ties resolve to the polytope furthest along, matching
            // the containment rule above.
            const double excess = polytopeExcess(pos, hPolys[i]);
            if (excess <= nearestExcess)
            {
                nearestExcess = excess;
                nearest = i;
            }
        }

        if (lastContaining >= 0)
        {
            return lastContaining;
        }

        // Outside every polytope: tracking error, or a corridor stale enough
        // that the robot has left it. Start from the closest one and let the
        // corridor penalty pull the trajectory back in -- refusing to plan here
        // would strand the robot exactly when it most needs a way back.
        ROS_WARN_THROTTLE(2.0,
                          "trajectory_server_node: robot is %.2f m outside every corridor polytope; "
                          "starting from the nearest (index %d of %zu).",
                          nearestExcess, nearest, hPolys.size());
        return nearest;
    }


    void TrajOptNode::diagnoseCorridor(const std::vector<Eigen::MatrixX3d> &corridor) const
    {
        // GCOPTER's setup() rejects a corridor when processCorridor() cannot
        // enumerate the vertices of some polytope, or of some CONSECUTIVE PAIR's
        // intersection; both need a strictly positive inscribed radius.
        //
        // The safety margin can no longer cause that -- setup() shrinks only its
        // penalty copy -- so anything reported here is a genuine property of the
        // corridor as published, and the generic "degenerate/non-overlapping"
        // warning is now telling the truth. Name the element and its radius so
        // the corridor generator can be pointed at the right place.
        Eigen::Vector2d centre;

        for (size_t i = 0; i < corridor.size(); i++)
        {
            const double r = geo_utils::inscribedRadius2d(corridor[i], centre);
            if (r <= 0.0)
            {
                ROS_WARN_THROTTLE(2.0,
                                  "trajectory_server_node: corridor polytope %zu of %zu has no interior "
                                  "(inscribed radius %.3f m). This is the corridor as "
                                  "published, before any margin.",
                                  i, corridor.size(), r);
                return;
            }
        }

        for (size_t i = 0; i + 1 < corridor.size(); i++)
        {
            const double r = geo_utils::overlapRadius2d(corridor[i], corridor[i + 1], centre);
            if (r <= 0.0)
            {
                ROS_WARN_THROTTLE(2.0,
                                  "trajectory_server_node: corridor polytopes %zu and %zu of %zu do not "
                                  "overlap (overlap radius %.3f m). This is the corridor as "
                                  "published, before any margin.",
                                  i, i + 1, corridor.size(), r);
                return;
            }
        }

        ROS_WARN_THROTTLE(2.0,
                          "trajectory_server_node: corridor of %zu polytopes is geometrically sound "
                          "(every polytope and overlap has an interior), so the replan failed "
                          "inside the optimizer rather than in setup().",
                          corridor.size());
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
                          safetyMargin_, lengthPerPiece_, piecesPerPolytope_,
                          smoothingEps_, integralRes_,
                          magnitudeBounds_, penaltyWeights_, curvatureEps_))
        {
            ROS_WARN_THROTTLE(2.0, "trajectory_server_node: GCOPTER setup failed (corridor likely degenerate/non-overlapping).");
            return false;
        }

        // setup() caps the safety margin wherever a polytope or one of its
        // overlaps could not afford the full amount. Say so, since a corridor
        // that keeps reporting a capped margin is a corridor whose narrow
        // sections leave the robot less standoff than SafetyMargin asks for --
        // a property of the corridor generator, not of the optimizer.
        const Eigen::VectorXd &applied = solver.getAppliedMargins();
        if (applied.size() > 0 && applied.minCoeff() < safetyMargin_ - 1.0e-6)
        {
            Eigen::Index tightest = 0;
            const double smallest = applied.minCoeff(&tightest);
            ROS_INFO_THROTTLE(2.0,
                              "trajectory_server_node: safety margin capped to %.3f m (requested %.3f m) "
                              "at polytope %ld of %ld -- the corridor is too narrow there to "
                              "give up the full margin without collapsing the polytope or its "
                              "overlap.",
                              smallest, safetyMargin_,
                              static_cast<long>(tightest),
                              static_cast<long>(applied.size()));
        }

        Trajectory<Degree, 2> candidate;
        const double cost = solver.optimize(candidate, relCostTol_, maxIterations_);

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
            // Drop the corridor the robot has already driven past, then apply
            // the safety margin to what is left. Order matters: containment has
            // to be tested against the true corridor, since a margin-shrunk
            // polytope can exclude a robot that is well inside the real one --
            // and with a footprint-sized margin on a ground vehicle that is the
            // common case, not the corner case.
            // Drop the corridor the robot has already driven past. The safety
            // margin is NOT applied here any more: it goes to
            // GcopterSolver::setup(), which shrinks only its penalty copy of
            // the corridor and leaves the decomposition on the true one.
            const int startIdx = findStartPolytope(pos, hPolys);
            const std::vector<Eigen::MatrixX3d> corridor(hPolys.begin() + startIdx, hPolys.end());

            if (startIdx > 0)
            {
                ROS_INFO_THROTTLE(2.0,
                                  "trajectory_server_node: starting replan from polytope %d of %zu "
                                  "(%d already passed).",
                                  startIdx, hPolys.size(), startIdx);
            }

            // Always re-seed from the robot's actual current state; see the
            // comment in replanImpl() for why acceleration is zero rather
            // than carried forward from the previous solve.
            const Eigen::Vector2d initAcc = Eigen::Vector2d::Zero();

            if (replan(pos, vel, initAcc, corridor, goal))
            {
                trajStartTime_ = now;
                isFirstRun_ = false;
            }
            else
            {
                // Say which polytope or which overlap actually failed, and
                // whether the margin caused it.
                diagnoseCorridor(corridor);

                if (!trajValid_)
                {
                    // No prior valid trajectory to fall back on; stop.
                    publishStopCommand();
                    return;
                }
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
#include <limits>

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
          hasState_(false),
          statePos_(Eigen::Vector3d::Zero()),
          stateVel_(Eigen::Vector3d::Zero()),
          stateAcc_(Eigen::Vector3d::Zero()),
          hasPoly_(false),
          goal_(Eigen::Vector3d::Zero()),
          isFirstRun_(true),
          trajValid_(false),
          lastYaw_(0.0),
          lastYawDot_(0.0)
    {
        // --- Topics ---
        stateTopic_ = getRequiredParam<std::string>(nh_, "StateTopic");
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
        piecesPerPolytope_ = nh_.param("PiecesPerPolytope", 1);
        relCostTol_ = nh_.param("rel_cost_tol", 1.0e-4);
        maxIterations_ = nh_.param("max_iterations", 1000);

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
        stateSub_ = nh_.subscribe(stateTopic_, 1, &TrajOptNode::stateCallback, this);

        ros::Time now = ros::Time::now();
        lastReplanTime_ = now;
        lastYawTime_ = now;

        controlTimer_ = nh_.createTimer(ros::Duration(1.0 / 100.0), &TrajOptNode::controlLoopCallback, this);

        ROS_INFO("traj_opt_node (GCOPTER receding-horizon, cost_order=%d) is running...", costOrder_);
    }

    void TrajOptNode::stateCallback(const quadrotor_msgs::State::ConstPtr &msg)
    {
        std::lock_guard<std::mutex> lock(dataMutex_);
        // All three are world-frame and measured; quadrotor_msgs::State carries
        // acceleration precisely so a replan never has to invent one.
        statePos_ << msg->pose.position.x,
            msg->pose.position.y,
            msg->pose.position.z;
        stateVel_ << msg->velocity.x,
            msg->velocity.y,
            msg->velocity.z;
        stateAcc_ << msg->acceleration.x,
            msg->acceleration.y,
            msg->acceleration.z;
        hasState_ = true;
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
            // with n = A_row, d = -b_row. Stored unshrunk; the safety margin
            // is applied inside GcopterSolver::setup(), to its penalty copy of
            // the corridor only.
            for (int i = 0; i < rows; i++)
            {
                hPoly(i, 0) = poly.A[3 * i + 0];
                hPoly(i, 1) = poly.A[3 * i + 1];
                hPoly(i, 2) = poly.A[3 * i + 2];
                hPoly(i, 3) = -poly.b[i];
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

    namespace
    {
        // Signed distance by which pos sits outside hPoly, in meters: the worst
        // per-row (n.x + d)/||n||. <= 0 means inside, so this is an exact
        // containment test. Rows are not assumed normalized -- the corridor
        // arrives however the planner produced it, and GCOPTER only normalizes
        // its own copy inside setup().
        double polytopeViolation(const Eigen::Vector3d &pos, const Eigen::MatrixX4d &hPoly)
        {
            double worst = -std::numeric_limits<double>::infinity();
            for (int i = 0; i < hPoly.rows(); i++)
            {
                const Eigen::Vector3d n = hPoly.row(i).head<3>().transpose();
                const double normN = n.norm();
                if (normN < 1.0e-9)
                {
                    continue;
                }
                worst = std::max(worst, (n.dot(pos) + hPoly(i, 3)) / normN);
            }
            return worst;
        }

        // How far outside hPoly pos is, for RANKING polytopes that all fail the
        // containment test: the Euclidean norm of the per-row overshoot.
        //
        // The plain max above is useless for ranking. A vehicle that has drifted
        // sideways out of a straight corridor overshoots every polytope
        // laterally by the same amount, so the max ties across all of them and
        // the choice falls to whichever happens to come first -- which can be a
        // polytope far behind or far ahead. Summing in quadrature keeps the
        // along-track terms, which are what actually distinguish the polytope
        // beside the vehicle from one 50 m down the corridor.
        double polytopeExcess(const Eigen::Vector3d &pos, const Eigen::MatrixX4d &hPoly)
        {
            double sumSq = 0.0;
            for (int i = 0; i < hPoly.rows(); i++)
            {
                const Eigen::Vector3d n = hPoly.row(i).head<3>().transpose();
                const double normN = n.norm();
                if (normN < 1.0e-9)
                {
                    continue;
                }
                const double over = (n.dot(pos) + hPoly(i, 3)) / normN;
                if (over > 0.0)
                {
                    sumSq += over * over;
                }
            }
            return std::sqrt(sumSq);
        }
    } // namespace

    int TrajOptNode::findStartPolytope(const Eigen::Vector3d &pos,
                                       const std::vector<Eigen::MatrixX4d> &hPolys) const
    {
        // Pick the polytope FURTHEST along the corridor that still contains the
        // vehicle, and let the caller drop everything before it.
        //
        // This is what untangles the knots. The corridor is anchored wherever
        // the vehicle was when corridor_planning built it, but we replan on our
        // own timer and reuse that corridor until the next one arrives -- by
        // then the vehicle has flown into the middle of it. Keeping the
        // already-passed polytopes forces MINCO to pin its first waypoints into
        // corridor that is now BEHIND the vehicle, so the trajectory doubles
        // back before turning for the goal.
        //
        // Consecutive polytopes overlap, so when the vehicle sits in an overlap
        // the later one is the right answer: it is the one that makes progress.
        // Skipping ahead stays safe even if the corridor loops back near itself,
        // because each polytope is convex and collision-free -- if the vehicle
        // is inside polytope k then the start of the trajectory lies in a free
        // convex set no matter how many polytopes were dropped.
        int lastContaining = -1;
        int nearest = 0;
        double nearestExcess = std::numeric_limits<double>::infinity();

        for (int i = 0; i < static_cast<int>(hPolys.size()); i++)
        {
            if (polytopeViolation(pos, hPolys[i]) <= 0.0)
            {
                lastContaining = i;
            }
            // <= so that ties resolve to the polytope furthest along, matching
            // the containment rule above.
            const double excess = polytopeExcess(pos, hPolys[i]);
            if (excess <= nearestExcess)
            {
                nearestExcess = excess;
                nearest = i;
            }
        }

        if (lastContaining >= 0)
        {
            return lastContaining;
        }

        // Outside every polytope: tracking error, or a corridor stale enough
        // that the vehicle has left it. Start from the closest one and let the
        // corridor penalty pull the trajectory back in -- refusing to plan here
        // would strand the vehicle exactly when it most needs a way back.
        ROS_WARN_THROTTLE(2.0,
                          "traj_opt_node: vehicle is %.2f m outside every corridor polytope; "
                          "starting from the nearest (index %d of %zu).",
                          nearestExcess, nearest, hPolys.size());
        return nearest;
    }


    void TrajOptNode::diagnoseCorridor(const std::vector<Eigen::MatrixX4d> &corridor) const
    {
        // GCOPTER's setup() rejects a corridor when processCorridor() cannot
        // enumerate the vertices of some polytope, or of some CONSECUTIVE PAIR's
        // intersection; both need a strictly positive inscribed radius.
        //
        // The safety margin can no longer cause that -- setup() shrinks only its
        // penalty copy -- so anything reported here is a genuine property of the
        // corridor as published, and the generic "degenerate/non-overlapping"
        // warning is now telling the truth. Name the element and its radius so
        // the corridor generator can be pointed at the right place.
        Eigen::Vector3d centre;

        for (size_t i = 0; i < corridor.size(); i++)
        {
            const double r = geo_utils::inscribedRadius(corridor[i], centre);
            if (r <= 0.0)
            {
                ROS_WARN_THROTTLE(2.0,
                                  "traj_opt_node: corridor polytope %zu of %zu has no interior "
                                  "(inscribed radius %.3f m). This is the corridor as "
                                  "published, before any margin.",
                                  i, corridor.size(), r);
                return;
            }
        }

        for (size_t i = 0; i + 1 < corridor.size(); i++)
        {
            const double r = geo_utils::overlapRadius(corridor[i], corridor[i + 1], centre);
            if (r <= 0.0)
            {
                ROS_WARN_THROTTLE(2.0,
                                  "traj_opt_node: corridor polytopes %zu and %zu of %zu do not "
                                  "overlap (overlap radius %.3f m). This is the corridor as "
                                  "published, before any margin.",
                                  i, i + 1, corridor.size(), r);
                return;
            }
        }

        ROS_WARN_THROTTLE(2.0,
                          "traj_opt_node: corridor of %zu polytopes is geometrically sound "
                          "(every polytope and overlap has an interior), so the replan failed "
                          "inside the optimizer rather than in setup().",
                          corridor.size());
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

        // Boundary conditions are seeded entirely from the vehicle's measured
        // state -- position, velocity AND acceleration all come from the state
        // topic. Nothing is re-sampled from the previous solve's own evaluated
        // trajectory: doing that fed each solve's artifacts back in as the next
        // replan's hard boundary condition, compounding any distortion across
        // replan cycles. Seeding the true acceleration (rather than assuming
        // zero) also means the new trajectory starts out tangent to the motion
        // the vehicle is really executing, so the handover at each replan no
        // longer demands an instantaneous acceleration step from the
        // controller.
        //
        // Because these are measurements they can land outside what the vehicle
        // can fly (a gust, an overshoot, a differentiation spike), and a
        // boundary condition is hard rather than penalized. setup() therefore
        // projects them back onto the feasible envelope -- see
        // GcopterSolver::clampBoundaryState().
        BoundaryT initialState, terminalState;
        initialState.col(0) = pos;
        initialState.col(1) = vel;
        initialState.col(2) = acc;
        if constexpr (S == 4)
        {
            // Jerk is the one boundary term with no measurement behind it --
            // differentiating the measured acceleration would be too noisy to
            // use as a hard constraint -- so the minimum-snap case still starts
            // from zero jerk.
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
                          safetyMargin_, lengthPerPiece_, piecesPerPolytope_,
                          smoothingEps_, integralRes_,
                          magnitudeBounds_, penaltyWeights_, physicalParams_))
        {
            ROS_WARN_THROTTLE(2.0, "traj_opt_node: GCOPTER setup failed (corridor likely degenerate/non-overlapping).");
            return false;
        }

        // setup() caps the safety margin wherever a polytope or one of its
        // overlaps could not afford the full amount. Say so, since a corridor
        // that keeps reporting a capped margin is a corridor whose narrow
        // sections leave the robot less standoff than SafetyMargin asks for --
        // a property of the corridor generator, not of the optimizer.
        const Eigen::VectorXd &applied = solver.getAppliedMargins();
        if (applied.size() > 0 && applied.minCoeff() < safetyMargin_ - 1.0e-6)
        {
            Eigen::Index tightest = 0;
            const double smallest = applied.minCoeff(&tightest);
            ROS_INFO_THROTTLE(2.0,
                              "traj_opt_node: safety margin capped to %.3f m (requested %.3f m) "
                              "at polytope %ld of %ld -- the corridor is too narrow there to "
                              "give up the full margin without collapsing the polytope or its "
                              "overlap.",
                              smallest, safetyMargin_,
                              static_cast<long>(tightest),
                              static_cast<long>(applied.size()));
        }

        Trajectory<Degree> candidate;
        const double cost = solver.optimize(candidate, relCostTol_, maxIterations_);

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
        bool hasState, hasPoly;
        Eigen::Vector3d pos, vel, acc, goal;
        std::vector<Eigen::MatrixX4d> hPolys;
        {
            std::lock_guard<std::mutex> lock(dataMutex_);
            hasState = hasState_;
            hasPoly = hasPoly_;
            pos = statePos_;
            vel = stateVel_;
            acc = stateAcc_;
            goal = goal_;
            hPolys = hPolys_;
        }

        if (!hasState)
        {
            ROS_WARN_THROTTLE(2.0, "traj_opt_node: waiting for vehicle state data...");
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
            // Drop the corridor the vehicle has already flown past, then apply
            // the safety margin to what is left. Order matters: containment has
            // to be tested against the true corridor, since a margin-shrunk
            // polytope can exclude a vehicle that is well inside the real one.
            // Drop the corridor the vehicle has already flown past. The
            // safety margin is NOT applied here any more: it goes to
            // GcopterSolver::setup(), which shrinks only its penalty copy of
            // the corridor and leaves the decomposition on the true one.
            const int startIdx = findStartPolytope(pos, hPolys);
            const std::vector<Eigen::MatrixX4d> corridor(hPolys.begin() + startIdx, hPolys.end());

            if (startIdx > 0)
            {
                ROS_INFO_THROTTLE(2.0,
                                  "traj_opt_node: starting replan from polytope %d of %zu "
                                  "(%d already passed).",
                                  startIdx, hPolys.size(), startIdx);
            }

            // Re-seed from the vehicle's measured position, velocity and
            // acceleration; see the comment in replanImpl().
            if (replan(pos, vel, acc, corridor, goal))
            {
                trajStartTime_ = now;
                isFirstRun_ = false;
            }
            else
            {
                // Say which polytope or which overlap actually failed, and
                // whether the margin caused it, instead of leaving the generic
                // setup() warning to be guessed at.
                diagnoseCorridor(corridor);

                if (!trajValid_)
                {
                    // No prior valid trajectory to fall back on; hover.
                    publishHoverCommand(pos);
                    return;
                }
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
