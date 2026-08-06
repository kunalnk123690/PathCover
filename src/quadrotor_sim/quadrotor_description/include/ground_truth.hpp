#ifndef GROUND_TRUTH_HPP
#define GROUND_TRUTH_HPP

#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>

using namespace std;

/**
 * @file ground_truth.hpp
 * @brief This file defines the GroundTruthPublisher class.
 */


/**
 * @class GroundTruthPublisher
 * @brief A class to subscribe to ground truth odometry, calculate transformations for sensors, and publish them.
 */
class GroundTruthPublisher {
    public:
        /**
         * @brief Constructor for the GroundTruthPublisher class.
         * Initializes the ROS node handle, subscribers, and publishers.
         */
        GroundTruthPublisher();
  
        /**
         * @brief Callback function for the ground truth subscriber.
         * @param msg A constant pointer to the received nav_msgs::Odometry message.
         * This function takes the ground truth odometry of the quadrotor, calculates the
         * transformations for the base link, lidar, and camera, and publishes them.
         */
        void GroundTruthCallback(const nav_msgs::Odometry::ConstPtr &msg);
        
    private:
        /**
         * @brief ROS NodeHandle for this publisher.
         */
        ros::NodeHandle nh_;
        
        /**
         * @brief Subscriber for the ground truth odometry topic.
         */
        ros::Subscriber GTSub_;

        /**
         * @brief Publisher for the lidar's ground truth transform.
         */
        ros::Publisher lidarGTPub_;

        /**
         * @brief Publisher for the camera's ground truth transform.
         */
        ros::Publisher cameraGTPub_;

        /**
         * @brief Transform broadcaster for publishing the base_link to world transform.
         */
        tf::TransformBroadcaster GTbroadcaster_;
};

#endif