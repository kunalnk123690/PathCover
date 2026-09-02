#include "SubscribeAndPublish.hpp"

/**
 * @brief Construct a new SubscribeAndPublish node.
 *
 * Initializes publishers for LiDAR and camera ground truth, creates a subscription
 * to the quadrotor ground truth odometry, and sets up a TF broadcaster for RViz visualization.
 */
SubscribeAndPublish::SubscribeAndPublish() : Node("ground_truth_node") {

    // Create TF broadcaster for publishing transformations
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Publishers for LiDAR and camera ground truth transforms
    lidarOdometryPub_ = this->create_publisher<nav_msgs::msg::Odometry>(
                        "/quadrotor/velodyne/ground_truth", 1);
    cameraOdometryPub_ = this->create_publisher<nav_msgs::msg::Odometry>(
                         "/quadrotor/realsense/ground_truth", 1);

    // Subscription to ground truth odometry
    sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/quadrotor/ground_truth", 1,
        std::bind(&SubscribeAndPublish::odomCallback, this, std::placeholders::_1));
}

/**
 * @brief Callback function for processing ground truth odometry.
 *
 * This function:
 * 1. Publishes a TF transform for the base link (`T_world_base`).
 * 2. Computes and publishes the LiDAR pose (`T_world_lidar`).
 * 3. Computes and publishes the camera pose (`T_world_camera`).
 *
 * All transforms are published both to TF (where applicable) and as
 * `geometry_msgs::msg::TransformStamped` messages.
 *
 * @param msg Shared pointer to incoming `nav_msgs::msg::Odometry` message.
 */
inline void SubscribeAndPublish::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    
    // === Step 1: T_world_base (Base link transform) ===
    tf2::Vector3 base_pos(msg->pose.pose.position.x, 
                          msg->pose.pose.position.y, 
                          msg->pose.pose.position.z);
    tf2::Quaternion base_quat(msg->pose.pose.orientation.x,
                               msg->pose.pose.orientation.y,
                               msg->pose.pose.orientation.z,
                               msg->pose.pose.orientation.w);
    tf2::Transform T_world_base(base_quat, base_pos);

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = msg->header.stamp;
    tf_msg.header.frame_id = "world";
    tf_msg.child_frame_id = "base_link";
    tf_msg.transform.translation.x = base_pos.x();
    tf_msg.transform.translation.y = base_pos.y();
    tf_msg.transform.translation.z = base_pos.z();
    tf_msg.transform.rotation.x = base_quat.x();
    tf_msg.transform.rotation.y = base_quat.y();
    tf_msg.transform.rotation.z = base_quat.z();
    tf_msg.transform.rotation.w = base_quat.w();
    tf_broadcaster_->sendTransform(tf_msg);

    // === Step 2: T_world_lidar ===
    tf2::Vector3 lidar_pos(0.0, 0.0, 0.27);
    tf2::Quaternion lidar_rot;
    lidar_rot.setRPY(0, 0, 0);
    tf2::Transform T_base_lidar(lidar_rot, lidar_pos);
    tf2::Transform T_world_lidar = T_world_base * T_base_lidar;

    nav_msgs::msg::Odometry lidar_pose_msg;
    lidar_pose_msg.header.stamp = msg->header.stamp;
    lidar_pose_msg.header.frame_id = "world";
    lidar_pose_msg.child_frame_id = "lidar_link";
    lidar_pose_msg.pose.pose.position.x = T_world_lidar.getOrigin().x();
    lidar_pose_msg.pose.pose.position.y = T_world_lidar.getOrigin().y();
    lidar_pose_msg.pose.pose.position.z = T_world_lidar.getOrigin().z();
    lidar_pose_msg.pose.pose.orientation.x = T_world_lidar.getRotation().x();
    lidar_pose_msg.pose.pose.orientation.y = T_world_lidar.getRotation().y();
    lidar_pose_msg.pose.pose.orientation.z = T_world_lidar.getRotation().z();
    lidar_pose_msg.pose.pose.orientation.w = T_world_lidar.getRotation().w();
    lidarOdometryPub_->publish(lidar_pose_msg);

    // === Step 3: T_world_camera ===
    tf2::Vector3 camera_pos(0.275, 0.0, 0.095);
    tf2::Quaternion camera_rot;
    camera_rot.setRPY(0, 0, 0);
    tf2::Transform T_base_camera(camera_rot, camera_pos);
    tf2::Transform T_world_camera = T_world_base * T_base_camera;

    nav_msgs::msg::Odometry camera_pose_msg;
    camera_pose_msg.header.stamp = msg->header.stamp;
    camera_pose_msg.header.frame_id = "world";
    camera_pose_msg.child_frame_id = "front_realsense_optical_link";
    camera_pose_msg.pose.pose.position.x = T_world_camera.getOrigin().x();
    camera_pose_msg.pose.pose.position.y = T_world_camera.getOrigin().y();
    camera_pose_msg.pose.pose.position.z = T_world_camera.getOrigin().z();
    camera_pose_msg.pose.pose.orientation.x = T_world_camera.getRotation().x();
    camera_pose_msg.pose.pose.orientation.y = T_world_camera.getRotation().y();
    camera_pose_msg.pose.pose.orientation.z = T_world_camera.getRotation().z();
    camera_pose_msg.pose.pose.orientation.w = T_world_camera.getRotation().w();
    cameraOdometryPub_->publish(camera_pose_msg);
}
