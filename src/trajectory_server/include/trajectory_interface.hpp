#ifndef TRAJECTORY_SERVER_TRAJECTORY_INTERFACE_HPP
#define TRAJECTORY_SERVER_TRAJECTORY_INTERFACE_HPP

#include "gcopter/trajectory.hpp"
#include <Eigen/Eigen>

namespace trajectory_server
{

#ifndef PATHCOVER_DIM
#define PATHCOVER_DIM 3
#endif

    using TrajectoryVector = Eigen::Matrix<double, PATHCOVER_DIM, 1>;

    // Type-erased view over Trajectory<5> (jerk) / Trajectory<7> (snap) so the
    // rest of TrajOptNode doesn't need to know which polynomial degree the
    // active cost_order produced.
    class ITrajectory
    {
    public:
        virtual ~ITrajectory() = default;
        virtual TrajectoryVector getPos(double t) const = 0;
        virtual TrajectoryVector getVel(double t) const = 0;
        virtual TrajectoryVector getAcc(double t) const = 0;
        virtual TrajectoryVector getJer(double t) const = 0;
        virtual double getTotalDuration() const = 0;
        virtual int getPieceNum() const = 0;
    };

    template <int Degree>
    class TrajectoryWrapper : public ITrajectory
    {
    public:
        explicit TrajectoryWrapper(const Trajectory<Degree, PATHCOVER_DIM> &traj) : traj_(traj) {}

        TrajectoryVector getPos(double t) const override { return traj_.getPos(t); }
        TrajectoryVector getVel(double t) const override { return traj_.getVel(t); }
        TrajectoryVector getAcc(double t) const override { return traj_.getAcc(t); }
        TrajectoryVector getJer(double t) const override { return traj_.getJer(t); }
        double getTotalDuration() const override { return traj_.getTotalDuration(); }
        int getPieceNum() const override { return traj_.getPieceNum(); }

    private:
        Trajectory<Degree, PATHCOVER_DIM> traj_;
    };

} // namespace trajectory_server

#endif // TRAJECTORY_SERVER_TRAJECTORY_INTERFACE_HPP
