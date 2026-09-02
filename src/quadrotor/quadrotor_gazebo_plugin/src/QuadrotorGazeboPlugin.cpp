#include "QuadrotorGazeboPlugin.hpp"

#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>

using namespace gz;
using namespace sim;
using namespace systems;

/**
 * @brief Private implementation for QuadrotorGazeboPlugin.
 *
 * Holds model/link ids, controller params, state, desired setpoints, and ROS 2 interfaces.
 */
class gz::sim::systems::QuadrotorGazeboPluginPrivate {

  /// \brief Joint Entity
  public:
    Entity baseEntity;
    Model model{kNullEntity};
    double timestep;
    std::string baseLink;
    std::string topicName;

    // Parameters
    double m;
    Eigen::Matrix3d J;
    Eigen::Vector3d KP;
    Eigen::Vector3d KD;
    Eigen::Vector3d KR;
    Eigen::Vector3d KW;
    
    // States
    Eigen::Vector3d x;
    Eigen::Matrix3d R;
    Eigen::Vector3d v;
    Eigen::Vector3d w;

    // Desired states
    Eigen::Vector3d xDes;             ///< Desired position ($x_d$).
    Eigen::Vector3d vDes;             ///< Desired velocity ($v_d$).
    Eigen::Vector3d aDes;             ///< Desired acceleration ($a_d$).
    Eigen::Vector3d jerkDes;          ///< Desired jerk ($j_d$).
    Eigen::Vector3d snapDes;          ///< Desired snap ($s_d$).
    double yaw;                       ///< Desired yaw angle.
    double yaw_dot;                   ///< Desired yaw rate.
    double yaw_ddot;                  ///< Desired yaw acceleration.


    quadrotor_msgs::msg::TrajectoryCommand latestCommand;
    std::mutex commandMtx;
    bool hasCmd = false;
    rclcpp::Node::SharedPtr rosNode;
    rclcpp::Subscription<quadrotor_msgs::msg::TrajectoryCommand>::SharedPtr trajSub;
};


//////////////////////////////////////////////////
QuadrotorGazeboPlugin::QuadrotorGazeboPlugin() : dataPtr_(std::make_unique<QuadrotorGazeboPluginPrivate>())
{
}


//////////////////////////////////////////////////
void QuadrotorGazeboPlugin::Configure(const Entity &_entity,
                                      const std::shared_ptr<const sdf::Element> &_sdf,
                                      EntityComponentManager &_ecm,
                                      EventManager & /*_eventMgr*/) {
  
  this->dataPtr_->model = Model(_entity);

  if (!this->dataPtr_->model.Valid(_ecm)) {
    gzerr << "[QuadrotorGazeboPlugin] plugin should be attached to a model "
           << "entity. Failed to initialize.\n";
    return;
  }

  /// SDF Parameters
  if (_sdf->HasElement("frame_name")) {
    this->dataPtr_->baseLink = _sdf->Get<std::string>("frame_name");
    gzmsg << "Applying wrench to " << this->dataPtr_->baseLink << "\n";
  }
  else {
    gzerr << "[QuadrotorGazeboPlugin] Missing <frame_name> parameter.\n";
    return;
  }

  if (_sdf->HasElement("trajectory_topic")) {
    this->dataPtr_->topicName = _sdf->Get<std::string>("trajectory_topic");
    gzmsg << "Subscribing to trajectory commands on topic " << this->dataPtr_->topicName << "\n";
  }
  else {
    gzerr << "[QuadrotorGazeboPlugin] Missing <trajectory_topic> parameter.\n";
    return;
  }

  if (_sdf->HasElement("KP")) {
    this->dataPtr_->KP << _sdf->Get<gz::math::Vector3d>("KP").X(),
                          _sdf->Get<gz::math::Vector3d>("KP").Y(),
                          _sdf->Get<gz::math::Vector3d>("KP").Z();
  }
  else {
    gzerr << "[QuadrotorGazeboPlugin] Missing <KP> parameter.\n";
    return;
  }

  if (_sdf->HasElement("KD")) {
    this->dataPtr_->KD << _sdf->Get<gz::math::Vector3d>("KD").X(),
                          _sdf->Get<gz::math::Vector3d>("KD").Y(),
                          _sdf->Get<gz::math::Vector3d>("KD").Z();
  }
  else {
    gzerr << "[QuadrotorGazeboPlugin] Missing <KD> parameter.\n";
    return;
  }

  if (_sdf->HasElement("KR")) {
    this->dataPtr_->KR << _sdf->Get<gz::math::Vector3d>("KR").X(),
                          _sdf->Get<gz::math::Vector3d>("KR").Y(),
                          _sdf->Get<gz::math::Vector3d>("KR").Z();
  }
  else {
    gzerr << "[QuadrotorGazeboPlugin] Missing <KR> parameter.\n";
    return;
  }  

  if (_sdf->HasElement("KW")) {
    this->dataPtr_->KW << _sdf->Get<gz::math::Vector3d>("KW").X(),
                          _sdf->Get<gz::math::Vector3d>("KW").Y(),
                          _sdf->Get<gz::math::Vector3d>("KW").Z();
  }
  else {
    gzerr << "[QuadrotorGazeboPlugin] Missing <KW> parameter.\n";
    return;
  }


  /// Find the Base frame of the quadrotor
  if (this->dataPtr_->baseEntity == kNullEntity) {
    auto entities = entitiesFromScopedName(this->dataPtr_->baseLink, _ecm, this->dataPtr_->model.Entity());
    if (!entities.empty()) {
      if (entities.size() > 1) {
        gzwarn << "Multiple base entities found. Using the first one.\n";
        throw std::runtime_error("Multiple base entities found");
      }
      this->dataPtr_->baseEntity = *entities.begin();
      gzmsg << "Base entity found: " << this->dataPtr_->baseLink << "\n";
    }
    else {
      gzwarn << "Failed to find base entity\n";
      return;
    }
  }


  // 1. Get the Inertial component associated with your baseEntity
  auto inertialComp = _ecm.Component<components::Inertial>(this->dataPtr_->baseEntity);
  
  // 2. Always check if the component was found
  if (nullptr == inertialComp) {
    gzerr << "Entity [" << this->dataPtr_->baseEntity << "] does not have an Inertial component. "
           << "Check your SDF file to ensure the link has an <inertial> tag." << std::endl;
    return; // Stop configuration if inertial properties are missing
  }  

  this->dataPtr_->m = inertialComp->Data().MassMatrix().Mass();
  gz::math::Matrix3d inertiaMatrix = inertialComp->Data().MassMatrix().Moi();
  this->dataPtr_->J << inertiaMatrix(0, 0), inertiaMatrix(0, 1), inertiaMatrix(0, 2),
                       inertiaMatrix(1, 0), inertiaMatrix(1, 1), inertiaMatrix(1, 2),
                       inertiaMatrix(2, 0), inertiaMatrix(2, 1), inertiaMatrix(2, 2);  

  gzmsg << "Quadrotor mass: " << this->dataPtr_->m << "\n";
  gzmsg << "Quadrotor inertia:\n" << this->dataPtr_->J << "\n";


  /// Initilize the controller
  this->controller_ = std::make_unique<GeometricController>(this->dataPtr_->m,
                                                            this->dataPtr_->J,
                                                            this->dataPtr_->KP,
                                                            this->dataPtr_->KD,
                                                            this->dataPtr_->KR,
                                                            this->dataPtr_->KW);


  /// Initilize ROS node
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  this->dataPtr_->rosNode = std::make_shared<rclcpp::Node>("quadrotor_gazebo_plugin");
  this->dataPtr_->trajSub = this->dataPtr_->rosNode->create_subscription<quadrotor_msgs::msg::TrajectoryCommand>(
                            this->dataPtr_->topicName,
                            1,
                            std::bind(&QuadrotorGazeboPlugin::TrajectoryCallback, this, std::placeholders::_1));

}

//////////////////////////////////////////////////
/**
 * @brief Simulation loop: read base link state, update setpoints, compute wrench, and apply.
 *
 * @details
 * - Pose from base link (world frame).
 * - Velocities provided by Gazebo are world-frame; convert to body with Rᵀ.
 * - Controller returns body-frame thrust & moments; rotate to world and apply as world wrench.
 */
void QuadrotorGazeboPlugin::PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm) {
  GZ_PROFILE("QuadrotorGazeboPlugin::PreUpdate");

  /// Spin the ROS node to process incoming messages
  rclcpp::spin_some(this->dataPtr_->rosNode);

  if (kNullEntity == this->dataPtr_->model.Entity()) {
    return;
  }

  if (!this->dataPtr_->model.Valid(_ecm)) {
    gzwarn << "QuadrotorGazeboPlugin model no longer valid.\n Disabling plugin.\n";
    this->dataPtr_->model = Model(kNullEntity);
    return;
  }
  
  // Extracting the states of the base link
  auto basePoseComp = _ecm.Component<components::Pose>(this->dataPtr_->model.Entity());               // In world frame
  auto baseLinVelComp = _ecm.Component<components::LinearVelocity>(this->dataPtr_->baseEntity);       // In body frame
  auto baseAngVelComp = _ecm.Component<components::AngularVelocity>(this->dataPtr_->baseEntity);      // In body frame

  if (basePoseComp == nullptr || baseLinVelComp == nullptr || baseAngVelComp == nullptr) {
    // create components
    if (basePoseComp == nullptr) {
      _ecm.CreateComponent(this->dataPtr_->model.Entity(), components::Pose());
    }
    if (baseLinVelComp == nullptr) {
      _ecm.CreateComponent(this->dataPtr_->baseEntity, components::LinearVelocity());
    }
    if (baseAngVelComp == nullptr) {
      _ecm.CreateComponent(this->dataPtr_->baseEntity, components::AngularVelocity());
    }
    gzwarn << "Base pose, linear velocity or angular velocity components were missing,\n returning early.\n";
    return;
  }

  
  Eigen::Quaterniond q(basePoseComp->Data().Rot().W(),
                       basePoseComp->Data().Rot().X(),
                       basePoseComp->Data().Rot().Y(),
                       basePoseComp->Data().Rot().Z());
  this->dataPtr_->R = q.toRotationMatrix();
  this->dataPtr_->x = Eigen::Vector3d(basePoseComp->Data().Pos().X(), basePoseComp->Data().Pos().Y(), basePoseComp->Data().Pos().Z());
  this->dataPtr_->v = this->dataPtr_->R * Eigen::Vector3d(baseLinVelComp->Data().X(), baseLinVelComp->Data().Y(), baseLinVelComp->Data().Z());
  this->dataPtr_->w = Eigen::Vector3d(baseAngVelComp->Data().X(), baseAngVelComp->Data().Y(), baseAngVelComp->Data().Z());

  // ------> NEW: Lock and update desired states
  {
    std::lock_guard<std::mutex> lock(this->dataPtr_->commandMtx);
    if (this->dataPtr_->hasCmd) {
      this->dataPtr_->xDes << this->dataPtr_->latestCommand.position.x,
                              this->dataPtr_->latestCommand.position.y,
                              this->dataPtr_->latestCommand.position.z;
      
      this->dataPtr_->vDes << this->dataPtr_->latestCommand.velocity.x,
                              this->dataPtr_->latestCommand.velocity.y,
                              this->dataPtr_->latestCommand.velocity.z;

      this->dataPtr_->aDes << this->dataPtr_->latestCommand.acceleration.x,
                              this->dataPtr_->latestCommand.acceleration.y,
                              this->dataPtr_->latestCommand.acceleration.z;

      this->dataPtr_->jerkDes << this->dataPtr_->latestCommand.jerk.x,
                                 this->dataPtr_->latestCommand.jerk.y,
                                 this->dataPtr_->latestCommand.jerk.z;

      this->dataPtr_->snapDes << this->dataPtr_->latestCommand.snap.x,
                                 this->dataPtr_->latestCommand.snap.y,
                                 this->dataPtr_->latestCommand.snap.z;

      this->dataPtr_->yaw = this->dataPtr_->latestCommand.yaw;
      this->dataPtr_->yaw_dot = this->dataPtr_->latestCommand.yaw_velocity;
      this->dataPtr_->yaw_ddot = this->dataPtr_->latestCommand.yaw_acceleration;

    }
    else {
      return; // No command received yet
    }
  }
  // <------ end of locking  


  /// Call the controller
  Eigen::Vector4d wrench = this->controller_->Controller(this->dataPtr_->x,
                                                         this->dataPtr_->v,
                                                         this->dataPtr_->R,
                                                         this->dataPtr_->w,
                                                         this->dataPtr_->xDes,
                                                         this->dataPtr_->vDes,
                                                         this->dataPtr_->aDes,
                                                         this->dataPtr_->yaw,
                                                         this->dataPtr_->yaw_dot);

  /// Convert force to world frame if needed (wrench(0) is in body fixed Z)
  const gz::math::Vector3d force_world = basePoseComp->Data().Rot().RotateVector(gz::math::Vector3d(0, 0, wrench(0)));

  /// Convert moment to world frame if needed
  const gz::math::Vector3d moment_world = basePoseComp->Data().Rot().RotateVector(gz::math::Vector3d(wrench(1), wrench(2), wrench(3)));

  /// Publish the wrench
  Link link(this->dataPtr_->baseEntity);
  link.AddWorldWrench(_ecm, force_world, moment_world);


}

/**
 * @brief Receive trajectory command and update the latest setpoints (thread-safe).
 */
void QuadrotorGazeboPlugin::TrajectoryCallback(const quadrotor_msgs::msg::TrajectoryCommand::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(this->dataPtr_->commandMtx);
  this->dataPtr_->latestCommand = *msg;
  this->dataPtr_->hasCmd = true;
}


GZ_ADD_PLUGIN(QuadrotorGazeboPlugin,
                    System,
                    QuadrotorGazeboPlugin::ISystemConfigure,
                    QuadrotorGazeboPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(QuadrotorGazeboPlugin, "QuadrotorGazeboPlugin")