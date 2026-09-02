#ifndef TRAJ_OPT_TRAJECTORY_INTERFACE_HPP
#define TRAJ_OPT_TRAJECTORY_INTERFACE_HPP

#include "gcopter/trajectory.hpp"
#include <Eigen/Eigen>

namespace trajectory_server
{

    // Type-erased view over Trajectory<5> (jerk) / Trajectory<7> (snap) so the
    // rest of TrajOptNode doesn't need to know which polynomial degree the
    // active cost_order produced.
    class ITrajectory
    {
    public:
        virtual ~ITrajectory() = default;
        virtual Eigen::Vector3d getPos(double t) const = 0;
        virtual Eigen::Vector3d getVel(double t) const = 0;
        virtual Eigen::Vector3d getAcc(double t) const = 0;
        virtual Eigen::Vector3d getJer(double t) const = 0;
        virtual double getTotalDuration() const = 0;
        virtual int getPieceNum() const = 0;
    };

    template <int Degree>
    class TrajectoryWrapper : public ITrajectory
    {
    public:
        explicit TrajectoryWrapper(const Trajectory<Degree> &traj) : traj_(traj) {}

        Eigen::Vector3d getPos(double t) const override { return traj_.getPos(t); }
        Eigen::Vector3d getVel(double t) const override { return traj_.getVel(t); }
        Eigen::Vector3d getAcc(double t) const override { return traj_.getAcc(t); }
        Eigen::Vector3d getJer(double t) const override { return traj_.getJer(t); }
        double getTotalDuration() const override { return traj_.getTotalDuration(); }
        int getPieceNum() const override { return traj_.getPieceNum(); }

    private:
        Trajectory<Degree> traj_;
    };

} // namespace traj_opt

#endif // TRAJ_OPT_TRAJECTORY_INTERFACE_HPP
