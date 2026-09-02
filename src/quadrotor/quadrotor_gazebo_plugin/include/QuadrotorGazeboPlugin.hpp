#ifndef QUADROTOR_GAZEBO_PLUGIN_HPP
#define QUADROTOR_GAZEBO_PLUGIN_HPP

/**
 * @file QuadrotorGazeboPlugin.hpp
 * @brief Gazebo system plugin that applies a geometric SE(3) controller to a quadrotor model.
 *
 * @details
 * - Subscribes to a trajectory command topic and feeds a GeometricController.
 * - Reads model/link state from the EntityComponentManager during PreUpdate.
 * - Intended for Gazebo Harmonic and newer.
 *
 * @note Coordinate frames and units should be consistent with the SDF and controller assumptions
 * (meters, radians, world frame right-handed).
 */

#include <memory>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

#include <gz/sim/System.hh>
#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/components/Inertial.hh>

#include "GeometricController.hpp"
#include "quadrotor_msgs/msg/trajectory_command.hpp"

namespace gz {
  namespace sim {
    namespace systems {

      /// \brief Forward declaration of the private implementation (pImpl).
      class QuadrotorGazeboPluginPrivate;

      /**
       * @class QuadrotorGazeboPlugin
       * @brief Gazebo system that runs a geometric SE(3) controller on a quadrotor model.
       *
       * @details
       * Responsibilities:
       * - Parse SDF and initialize controller parameters in Configure().
       * - On each simulation step (PreUpdate), read the model state from components
       * (pose, linear and angular velocity), compute control wrench/forces with
       * GeometricController, and apply them to the appropriate link(s).
       * - Receive high-level commands via @ref TrajectoryCallback and update
       * controller setpoints.
       *
       * Expected SDF parameters (typical):
       * - Model/link names to control.
       * - Mass/inertia or link from which to read them.
       * - ROS 2 topic names for trajectory commands.
       */
      class QuadrotorGazeboPlugin
          : public System,
            public ISystemConfigure,
            public ISystemPreUpdate {
      public:
        /// @brief Default constructor.
        QuadrotorGazeboPlugin();

        /// @brief Virtual destructor.
        ~QuadrotorGazeboPlugin() override = default;

        /**
         * @brief Gazebo callback: configure the system after creation.
         *
         * @param[in] _entity Root entity of the model to which this system is attached.
         * @param[in] _sdf SDF element containing system/plugin parameters.
         * @param[in,out] _ecm EntityComponentManager used to query and create components.
         * @param[in,out] _eventMgr Event manager for registering event-based hooks.
         *
         * @post Initializes internal state, locates model/link entities, and sets up ROS 2 interfaces.
         */
        void Configure(const Entity &_entity,
                       const std::shared_ptr<const sdf::Element> &_sdf,
                       EntityComponentManager &_ecm,
                       EventManager &_eventMgr) override;

        /**
         * @brief Gazebo callback: called every simulation iteration before physics update.
         *
         * @param[in] _info Simulation timing and stepping info.
         * @param[in,out] _ecm EntityComponentManager to read/update components.
         *
         * @details
         * Typical flow:
         * 1) Read current pose/velocities from components.
         * 2) Evaluate control law via GeometricController.
         * 3) Write forces/torques (components or Link API) for physics to apply this step.
         */
        void PreUpdate(const UpdateInfo &_info,
                       EntityComponentManager &_ecm) override;

        /**
         * @brief ROS 2 subscriber callback for trajectory commands.
         *
         * @param[in] msg Shared pointer to the received TrajectoryCommand message.
         * @details Updates the controller's desired position/velocity/acceleration/yaw setpoints.
         */
        void TrajectoryCallback(const quadrotor_msgs::msg::TrajectoryCommand::SharedPtr msg);

      private:
        /// @brief Opaque pointer to implementation details.
        std::unique_ptr<QuadrotorGazeboPluginPrivate> dataPtr_;

        /// @brief Geometric SE(3) controller instance used to compute control inputs.
        std::unique_ptr<GeometricController> controller_;
      };
    }  // namespace systems
  }    // namespace sim
}      // namespace gz

#endif // QUADROTOR_GAZEBO_PLUGIN_HPP