#include <thread>
#include <iostream>
#include <cmath>
#include <mutex>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Console.hh>
#include <gazebo/physics/physics.hh>
#include <boost/bind.hpp>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include "quadrotor_msgs/TrajectoryCommand.h"
#include "GeometricController.hpp"

/**
 * @file QuadrotorGazeboPlugin.cpp
 * @brief A Gazebo plugin for simulating a quadrotor with a geometric controller.
 * @author Kunal Narkhede <kunalnk@udel.edu>
 */

namespace gazebo {

/**
 * @class QuadrotorGazeboPlugin
 * @brief A Gazebo plugin to simulate a quadrotor using a geometric controller.
 * @details This plugin interfaces a quadrotor model in the Gazebo simulator with the Robot Operating System (ROS).
 * It inherits from `gazebo::ModelPlugin` to interact with the simulation environment and `GeometricController`
 * to implement the control logic. The plugin subscribes to `quadrotor_msgs::TrajectoryCommand` messages to receive
 * desired trajectory setpoints. It then calculates the required motor forces and body torques and applies them to the
 * quadrotor's base link. Additionally, it publishes the ground truth odometry of the vehicle.
 */
class QuadrotorGazeboPlugin : public ModelPlugin {
    public:
        /**
         * @brief Constructor for the QuadrotorGazeboPlugin.
         */
        QuadrotorGazeboPlugin(){};

        /**
         * @brief Destructor for the QuadrotorGazeboPlugin.
         * @details Cleans up ROS-related resources, specifically shutting down the custom callback queue and joining the ROS thread.
         */
        ~QuadrotorGazeboPlugin();

        /**
         * @brief The load function is called by Gazebo when the plugin is inserted into simulation
         * @param[in] model A pointer to the model that this plugin is attached to.
         * @param[in] sdf A pointer to the plugin's SDF element.
         * @details This function initializes the plugin. It sets up the ROS node, subscribes to the trajectory topic,
         * reads parameters from the SDF file, and connects the `OnUpdate` function to Gazebo's simulation update event.
         */
        void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override;

    private:
        physics::ModelPtr model_;                                       ///< Pointer to the model in Gazebo.
        std::shared_ptr<ros::NodeHandle> rosNode_;                      ///< ROS node handle.
        ros::Subscriber trajectorySub_;                                 ///< ROS subscriber for pose commands.
        ros::CallbackQueue rosQueue_;                                   ///< Custom ROS callback queue.
        std::thread rosQueueThread_;                                    ///< Thread for processing the ROS callback queue.
        event::ConnectionPtr update_connection_;                        ///< Gazebo update connection pointer.

        // --- Configuration Parameters from SDF ---
        std::string frame_;                                             ///< Name of the link to apply forces/torques.
        std::string trajectoryTopicName_;                               ///< Topic name for trajectory commands.
        std::string groundTruthTopicName_;                              ///< Topic name for ground truth odometry.

        // --- State & Synchronization ---
        quadrotor_msgs::TrajectoryCommand latest_cmd_;                        ///< Stores the latest received pose command.
        std::mutex mutex_;                                              ///< Mutex to protect access to shared data like `latest_cmd_`.
        bool has_cmd_ = false;                                          ///< Flag to indicate if a command has been received.

        /**
         * @brief Callback function for handling incoming pose commands.
         * @param[in] msg A constant pointer to the received `quadrotor_msgs::TrajectoryCommand` message.
         * @details This function is called whenever a new message is received on the trajectory topic. It
         * stores the command in `latest_cmd_` for the controller to use. Access is synchronized with a mutex.
         */
        void TrajectoryCommandCallback(const quadrotor_msgs::TrajectoryCommand::ConstPtr& msg);

        /**
         * @brief The update function, called at every simulation step.
         * @param[in] info Gazebo update information structure.
         * @details This is the main control loop. It reads the current state of the quadrotor from the simulator,
         * computes the control inputs (total thrust and body torques) using the geometric controller logic,
         * and applies these forces and torques to the quadrotor model. It also publishes the ground truth state.
         */
        void OnUpdate(const common::UpdateInfo &info);

        /**
         * @brief Thread function for the ROS callback queue.
         * @details This function runs in a separate thread and continuously processes the `rosQueue_`.
         * This prevents the Gazebo simulation thread from being blocked by ROS message handling.
         */
        void QueueThread();

        // --- Controller Parameters ---
        ignition::math::Vector3d Kp_;                                       ///< Proportional gains for position error ($K_p$).
        ignition::math::Vector3d Kd_;                                       ///< Derivative gains for velocity error ($K_d$).
        ignition::math::Vector3d KR_;                                       ///< Proportional gains for attitude error ($K_R$).
        ignition::math::Vector3d KW_;                                       ///< Proportional gains for angular velocity error ($K_W$).        

        // --- Physical Properties ---
        double mass_;                                                       ///< Mass of the quadrotor ($m$).
        Eigen::Matrix3d inertia_;                                           ///< Inertia matrix of the quadrotor ($J$).

        // --- Desired States (from TrajectoryCommand) ---
        Eigen::Vector3d xDes_;                                              ///< Desired position ($x_d$).
        Eigen::Vector3d vDes_;                                              ///< Desired velocity ($v_d$).
        Eigen::Vector3d aDes_;                                              ///< Desired acceleration ($a_d$).
        Eigen::Vector3d jerkDes_;                                           ///< Desired jerk ($j_d$).
        Eigen::Vector3d snapDes_;                                           ///< Desired snap ($s_d$).
        double yaw_;                                                        ///< Desired yaw angle.
        double yaw_dot_;                                                    ///< Desired yaw rate.
        double yaw_ddot_;                                                   ///< Desired yaw acceleration.

        std::unique_ptr<GeometricController> controller_;

    };

    /**
     * @brief Registers this plugin with the Gazebo simulator.
     * @details This macro is necessary for Gazebo to be able to find and load the plugin.
     */
    GZ_REGISTER_MODEL_PLUGIN(QuadrotorGazeboPlugin)

} // namespace gazebo