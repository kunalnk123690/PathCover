#ifndef TF_BROADCASTER_HPP
#define TF_BROADCASTER_HPP

#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>


class TfBroadcaster {
    public:
        TfBroadcaster(ros::NodeHandle &nh);
        ~TfBroadcaster(){};
        void odometryCallback(const nav_msgs::Odometry::ConstPtr &msg);
    
    private:
        ros::Publisher pubLidarTransform_;
        ros::Publisher pubDepthCameraTransform_;
        ros::Subscriber sub_;
        tf::TransformBroadcaster tfMap2OdomBroadcaster_;

};


#endif // TF_BROADCASTER_HPP