#include "tf_broadcaster.hpp"

TfBroadcaster::TfBroadcaster(ros::NodeHandle &nh) {
    pubLidarTransform_ = nh.advertise<nav_msgs::Odometry>("/jackal/velodyne/ground_truth", 1);
    pubDepthCameraTransform_ = nh.advertise<nav_msgs::Odometry>("/jackal/realsense/ground_truth", 1);
    sub_ = nh.subscribe("/jackal/ground_truth", 1, &TfBroadcaster::odometryCallback, this);
}

void TfBroadcaster::odometryCallback(const nav_msgs::Odometry::ConstPtr &msg) {

    // === Step 1: T_world_base (Base link transform) ===
    tf::Vector3 base_pos(msg->pose.pose.position.x, 
                         msg->pose.pose.position.y, 
                         msg->pose.pose.position.z);
    tf::Quaternion base_quat(msg->pose.pose.orientation.x,
                             msg->pose.pose.orientation.y,
                             msg->pose.pose.orientation.z,
                             msg->pose.pose.orientation.w);
    tf::Transform T_world_base(base_quat, base_pos);

    geometry_msgs::TransformStamped transform_stamped;
    transform_stamped.header.stamp = msg->header.stamp;
    transform_stamped.header.frame_id = "world";
    transform_stamped.child_frame_id = "base_link";
    transform_stamped.transform.translation.x = msg->pose.pose.position.x;
    transform_stamped.transform.translation.y = msg->pose.pose.position.y;
    transform_stamped.transform.translation.z = msg->pose.pose.position.z;

    transform_stamped.transform.rotation.x = msg->pose.pose.orientation.x;
    transform_stamped.transform.rotation.y = msg->pose.pose.orientation.y;
    transform_stamped.transform.rotation.z = msg->pose.pose.orientation.z;
    transform_stamped.transform.rotation.w = msg->pose.pose.orientation.w;
    tfMap2OdomBroadcaster_.sendTransform(transform_stamped);


    tf::Vector3 lidar_pos(0.0, 0.0, 0.3027);
    tf::Quaternion lidar_quat;
    lidar_quat.setRPY(0, 0, 0);
    tf::Transform T_base_lidar(lidar_quat, lidar_pos);
    tf::Transform T_world_lidar = T_world_base * T_base_lidar;

    // === Step 2: Publish the lidar's world-frame pose as nav_msgs/Odometry ===
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
    pubLidarTransform_.publish(lidar_pose_msg);


    // === Step 3: T_world_depth_camera (Depth camera transform) ===
    tf::Vector3 depth_camera_pos(0.198, 0.0, 0.206);
    tf::Quaternion depth_camera_quat;
    depth_camera_quat.setRPY(-M_PI/2, 0, -M_PI/2);
    tf::Transform T_base_depth_camera(depth_camera_quat, depth_camera_pos);
    tf::Transform T_world_depth_camera = T_world_base * T_base_depth_camera;

    // === Step 4: Publish the depth camera's world-frame pose as nav_msgs/Odometry ===
    nav_msgs::Odometry depth_camera_pose_msg;
    depth_camera_pose_msg.header.stamp = msg->header.stamp;
    depth_camera_pose_msg.header.frame_id = "world";
    depth_camera_pose_msg.child_frame_id = "depth_camera_link";
    depth_camera_pose_msg.pose.pose.position.x = T_world_depth_camera.getOrigin().x();
    depth_camera_pose_msg.pose.pose.position.y = T_world_depth_camera.getOrigin().y();
    depth_camera_pose_msg.pose.pose.position.z = T_world_depth_camera.getOrigin().z();
    depth_camera_pose_msg.pose.pose.orientation.x = T_world_depth_camera.getRotation().x();
    depth_camera_pose_msg.pose.pose.orientation.y = T_world_depth_camera.getRotation().y();
    depth_camera_pose_msg.pose.pose.orientation.z = T_world_depth_camera.getRotation().z();
    depth_camera_pose_msg.pose.pose.orientation.w = T_world_depth_camera.getRotation().w();
    pubDepthCameraTransform_.publish(depth_camera_pose_msg);

}