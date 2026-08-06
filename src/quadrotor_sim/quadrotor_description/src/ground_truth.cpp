/**
 * @file ground_truth.cpp
 * @brief Implementation of the GroundTruthPublisher class.
 */

#include "ground_truth.hpp"

/**
 * @brief Constructor for the GroundTruthPublisher class.
 *
 * Initializes a subscriber to the "/quadrotor/ground_truth" topic and sets up
 * publishers for the lidar and camera ground truth transforms on the
 * "/quadrotor/velodyne/ground_truth" and "/quadrotor/realsense/ground_truth" topics, respectively.
 */
GroundTruthPublisher::GroundTruthPublisher() {
    GTSub_ = nh_.subscribe("/quadrotor/ground_truth", 1, &GroundTruthPublisher::GroundTruthCallback, this);
    
    lidarGTPub_ = nh_.advertise<nav_msgs::Odometry>("/quadrotor/velodyne/ground_truth", 1);
    cameraGTPub_ = nh_.advertise<nav_msgs::Odometry>("/quadrotor/realsense/ground_truth", 1);
}

/**
 * @brief Callback function to process incoming odometry data.
 *
 * This function is called whenever a new message is received on the "/quadrotor/ground_truth" topic.
 * It calculates and publishes the transformations from the world frame to the base_link, lidar_link,
 * and front_realsense_optical_link frames.
 *
 * @param msg A constant pointer to the received nav_msgs::Odometry message.
 */
void GroundTruthPublisher::GroundTruthCallback(const nav_msgs::Odometry::ConstPtr &msg) {

    // Base pose in world (already known)
    tf::Vector3 base_pos(msg->pose.pose.position.x, 
                         msg->pose.pose.position.y, 
                         msg->pose.pose.position.z);
    tf::Quaternion base_quat(msg->pose.pose.orientation.x, 
                             msg->pose.pose.orientation.y, 
                             msg->pose.pose.orientation.z, 
                             msg->pose.pose.orientation.w);  // x, y, z, w
    tf::Transform T_world_base(base_quat, base_pos);    

    geometry_msgs::TransformStamped base_pose_msg;
    base_pose_msg.header.stamp = msg->header.stamp;
    base_pose_msg.header.frame_id = "world";
    base_pose_msg.child_frame_id = "base_link";
    base_pose_msg.transform.translation.x = msg->pose.pose.position.x;
    base_pose_msg.transform.translation.y = msg->pose.pose.position.y;
    base_pose_msg.transform.translation.z = msg->pose.pose.position.z;
    base_pose_msg.transform.rotation.x = msg->pose.pose.orientation.x;
    base_pose_msg.transform.rotation.y = msg->pose.pose.orientation.y;
    base_pose_msg.transform.rotation.z = msg->pose.pose.orientation.z;
    base_pose_msg.transform.rotation.w = msg->pose.pose.orientation.w;
    GTbroadcaster_.sendTransform(base_pose_msg);

    // T_base_lidar
    tf::Vector3 lidar_pos(0.0, 0.0, 0.2);
    tf::Quaternion lidar_rot;
    lidar_rot.setRPY(0, 0, 0);
    tf::Transform T_base_lidar(lidar_rot, lidar_pos);

    tf::Transform T_world_lidar = T_world_base * T_base_lidar;
    
    nav_msgs::Odometry lidar_pose_msg;
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
    lidarGTPub_.publish(lidar_pose_msg);

    /// base to front_realsense_chassis
    tf::Vector3 FRC_pos(0.1, 0.0, 0.095);
    tf::Quaternion FRC_rot;
    FRC_rot.setRPY(0, 0, 0);

    // front_realsense_chassis to front_realsense_lens
    tf::Vector3 FRL_pos(0.025, 0.0, 0.0);
    tf::Quaternion FRL_rot;
    FRL_rot.setRPY(0, 0, 0);

    // front_realsense_lens to front_realsense_optical_link
    tf::Vector3 FROL_pos(0.0, 0.0, 0.0);
    tf::Quaternion FROL_rot;
    FROL_rot.setRPY(-M_PI/2, 0, -M_PI/2);

    tf::Transform T_base_FRC(FRC_rot, FRC_pos);
    tf::Transform T_FRC_FRL(FRL_rot, FRL_pos);
    tf::Transform T_FRL_FROL(FROL_rot, FROL_pos);

    tf::Transform T_world_FROL = T_world_base * T_base_FRC * T_FRC_FRL * T_FRL_FROL;

    nav_msgs::Odometry front_realsense_optical_link_msg;
    front_realsense_optical_link_msg.header.stamp = msg->header.stamp;
    front_realsense_optical_link_msg.header.frame_id = "world";
    front_realsense_optical_link_msg.child_frame_id = "front_realsense_optical_link";
    front_realsense_optical_link_msg.pose.pose.position.x = T_world_FROL.getOrigin().x();
    front_realsense_optical_link_msg.pose.pose.position.y = T_world_FROL.getOrigin().y();
    front_realsense_optical_link_msg.pose.pose.position.z = T_world_FROL.getOrigin().z();
    front_realsense_optical_link_msg.pose.pose.orientation.x = T_world_FROL.getRotation().x();
    front_realsense_optical_link_msg.pose.pose.orientation.y = T_world_FROL.getRotation().y();
    front_realsense_optical_link_msg.pose.pose.orientation.z = T_world_FROL.getRotation().z();
    front_realsense_optical_link_msg.pose.pose.orientation.w = T_world_FROL.getRotation().w();
    cameraGTPub_.publish(front_realsense_optical_link_msg);
}