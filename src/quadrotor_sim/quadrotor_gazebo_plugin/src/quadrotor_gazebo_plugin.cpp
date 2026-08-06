#include "quadrotor_gazebo_plugin.hpp"

/**
 * @file quadrotor_gazebo_plugin.cpp
 * @brief Implementation of the QuadrotorGazeboPlugin class.
 */

namespace gazebo {
    /**
     * @brief Destructor for the QuadrotorGazeboPlugin.
     * @details Gracefully shuts down the ROS node and joins the ROS queue processing thread
     * to ensure a clean exit.
     */
    QuadrotorGazeboPlugin::~QuadrotorGazeboPlugin() {
        // Shutdown the ROS node and join the thread
        rosNode_->shutdown();
        rosQueueThread_.join();
    }


    /**
     * @brief The load function called by Gazebo when the plugin is initialized.
     * @param[in] model A pointer to the model that this plugin is attached to.
     * @param[in] sdf A pointer to the plugin's SDF element in the world file.
     * @details This function performs all the initial setup for the plugin. It reads configuration
     * parameters (like ROS topics and controller gains) from the SDF file, retrieves the physical
     * properties (mass and inertia) of the quadrotor, initializes the geometric controller,
     * sets up ROS publishers and subscribers, and connects the main update loop to the
     * Gazebo simulation event system.
     */
    void QuadrotorGazeboPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf) {
        model_ = model;

        // Read required and optional parameters from the SDF file
        if (sdf->HasElement("wrenchFrame")) {
            frame_ = sdf->Get<std::string>("wrenchFrame");
        }
        if (sdf->HasElement("trajectoryTopic")) {
            trajectoryTopicName_ = sdf->Get<std::string>("trajectoryTopic");
        }
        if (sdf->HasElement("KP")) {
            Kp_ = sdf->Get<ignition::math::Vector3d>("KP");
        }
        if (sdf->HasElement("KD")) {
            Kd_ = sdf->Get<ignition::math::Vector3d>("KD");
        }
        if (sdf->HasElement("KR")) {
            KR_ = sdf->Get<ignition::math::Vector3d>("KR");
        }
        if (sdf->HasElement("KW")) {
            KW_ = sdf->Get<ignition::math::Vector3d>("KW");
        }
        
        // Ensure essential parameters are loaded
        if (frame_.empty() || trajectoryTopicName_.empty()) {
            gzerr << "Required SDF parameters (wrenchFrame, trajectoryTopic) not found.\n";
            return;
        }

        // Get physical properties from the specified link's inertial component
        auto link = model_->GetLink(frame_);
        if (!link) {
            gzerr << "Link [" << frame_ << "] not found.\n";
            return;
        }
        auto inertial = link->GetInertial();
        mass_ = inertial->Mass();
        inertia_ << inertial->PrincipalMoments().X(), 0, 0,
                    0, inertial->PrincipalMoments().Y(), 0,
                    0, 0, inertial->PrincipalMoments().Z();

        // Initialize the base geometric controller with the physical and gain parameters
        controller_ = std::make_unique<GeometricController>(mass_,
                                                            inertia_,
                                                            Eigen::Vector3d(Kp_.X(), Kp_.Y(), Kp_.Z()),
                                                            Eigen::Vector3d(Kd_.X(), Kd_.Y(), Kd_.Z()),
                                                            Eigen::Vector3d(KR_.X(), KR_.Y(), KR_.Z()),
                                                            Eigen::Vector3d(KW_.X(), KW_.Y(), KW_.Z()));

        // Initialize ROS node if it's not already running
        if (!ros::isInitialized()) {
            ROS_FATAL("A ROS node for Gazebo has not been initialized, unable to load plugin.\n");
            return;
        }
        rosNode_ = std::make_shared<ros::NodeHandle>("quadrotor_gazebo_plugin");        

        // Set up the ROS subscriber for trajectory commands, using a custom callback queue
        ros::SubscribeOptions sub = ros::SubscribeOptions::create<quadrotor_msgs::TrajectoryCommand>(
                                        trajectoryTopicName_,
                                        1,
                                        boost::bind(&QuadrotorGazeboPlugin::TrajectoryCommandCallback, this, _1),
                                        ros::VoidPtr(), &rosQueue_);
        trajectorySub_ = rosNode_->subscribe(sub);
        

        // Connect the OnUpdate method to Gazebo's world update event
        update_connection_ = event::Events::ConnectWorldUpdateBegin(std::bind(&QuadrotorGazeboPlugin::OnUpdate, this, std::placeholders::_1));        

        // Start a separate thread to process the custom ROS callback queue
        rosQueueThread_ = std::thread(std::bind(&QuadrotorGazeboPlugin::QueueThread, this));        
    }


    /**
     * @brief The main update loop, called at every simulation step by Gazebo.
     * @param[in] info Contains information about the current simulation time and step size.
     * @details This function is the core of the plugin. On each simulation update, it:
     * 1. Checks if a valid command has been received.
     * 2. If a command is available, it extracts the current state (position, orientation, velocities) from Gazebo.
     * 3. It calls the geometric controller to calculate the required total thrust and body torques.
     * 4. It applies the calculated wrench (force and torque) to the quadrotor model.
     */
    void QuadrotorGazeboPlugin::OnUpdate(const common::UpdateInfo &info) {
        auto link = model_->GetLink(frame_);
        if (!link) return;
        
        // Lock the mutex and apply control inputs
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!has_cmd_) {
                return; // Do nothing if no command has been received yet
            }
              
            // Current state variables
            Eigen::Vector3d x;
            Eigen::Matrix3d R;
            Eigen::Vector3d v;
            Eigen::Vector3d w;

            // Get current state from Gazebo and convert to Eigen types
            ignition::math::Pose3d currentPose = link->WorldPose();
            ignition::math::Vector3d linear_vel = link->WorldLinearVel();
            ignition::math::Vector3d angular_vel = link->WorldAngularVel();
            
            x << currentPose.Pos().X(), currentPose.Pos().Y(), currentPose.Pos().Z();
            R = Eigen::Quaterniond(currentPose.Rot().W(), currentPose.Rot().X(), currentPose.Rot().Y(), currentPose.Rot().Z()).toRotationMatrix();
            v << linear_vel.X(), linear_vel.Y(), linear_vel.Z();
            
            // Convert angular velocity to body frame (EXTREMELY IMPORTANT)
            w = R.transpose() * Eigen::Vector3d(angular_vel.X(), angular_vel.Y(), angular_vel.Z());
            
            // Update desired states from the latest received command
            quadrotor_msgs::TrajectoryCommand cmd = latest_cmd_;
            xDes_ << cmd.position.x, cmd.position.y, cmd.position.z;
            vDes_ << cmd.velocity.x, cmd.velocity.y, cmd.velocity.z;
            aDes_ << cmd.acceleration.x, cmd.acceleration.y, cmd.acceleration.z;
            jerkDes_ << cmd.jerk.x, cmd.jerk.y, cmd.jerk.z;
            snapDes_ << cmd.snap.x, cmd.snap.y, cmd.snap.z;
            yaw_ = cmd.yaw;
            yaw_dot_ = cmd.yaw_velocity;
            yaw_ddot_ = cmd.yaw_acceleration;

            // Calculate wrench (force and torque) using the geometric controller
            // Eigen::Vector4d wrench = controller_->Controller(x, v, R, w, xDes_, vDes_, aDes_, jerkDes_, snapDes_, yaw_, yaw_dot_, yaw_ddot_);
            Eigen::Vector4d wrench = controller_->Controller(x, v, R, w, xDes_, vDes_, aDes_, yaw_, yaw_dot_);

            // Apply the calculated wrench to the quadrotor model
            ignition::math::Vector3d force(0, 0, wrench(0));
            ignition::math::Vector3d torque(wrench(1), wrench(2), wrench(3));
            link->AddRelativeForce(force);
            link->AddRelativeTorque(torque);
        }
    }


    /**
     * @brief Callback function for the pose command subscriber.
     * @param[in] msg The received `quadrotor_msgs::TrajectoryCommand` message.
     * @details This function is executed whenever a new command is received. It copies the message
     * data into the `latest_cmd_` member variable and sets the `has_cmd_` flag to true.
     * A mutex is used to ensure thread-safe access to these shared variables.
     */
    void QuadrotorGazeboPlugin::TrajectoryCommandCallback(const quadrotor_msgs::TrajectoryCommand::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_cmd_ = *msg;
        has_cmd_ = true;
    }    


    /**
     * @brief The function for the dedicated ROS message processing thread.
     * @details This function runs in a loop, processing callbacks in `rosQueue_` with a short timeout.
     * Using a separate thread for ROS callbacks prevents the main Gazebo simulation thread from being
     * blocked, ensuring smooth simulation performance.
     */
    void QuadrotorGazeboPlugin::QueueThread() {
        static const double timeout = 0.01;
        while (rosNode_->ok()) {
            rosQueue_.callAvailable(ros::WallDuration(timeout));
        }
    }

} // namespace gazebo