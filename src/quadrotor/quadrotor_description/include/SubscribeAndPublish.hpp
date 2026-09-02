#ifndef SUBSCRIBEANDPUBLISH_HPP
#define SUBSCRIBEANDPUBLISH_HPP

#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>

/**
 * @class SubscribeAndPublish
 * @brief A ROS 2 node that subscribes to odometry messages, processes them, and publishes TF transforms.
 *
 * This class subscribes to `nav_msgs::msg::Odometry` messages, processes the incoming data,
 * and publishes the corresponding transforms using a `tf2_ros::TransformBroadcaster` for visualization in RViz.
 * It also publishes the same transforms as `geometry_msgs::msg::TransformStamped` messages for
 * both LiDAR and camera odometry topics.
 */
class SubscribeAndPublish : public rclcpp::Node {
public:
    /**
     * @brief Constructor for SubscribeAndPublish.
     *
     * Initializes the ROS 2 node, sets up subscriptions, publishers, and the TF broadcaster.
     */
    SubscribeAndPublish();

    /**
     * @brief Destructor for SubscribeAndPublish.
     */
    ~SubscribeAndPublish() {};

    /**
     * @brief Generic callback function for processing events.
     *
     * This function can be used to execute periodic or event-driven logic
     * unrelated to a specific subscriber.
     */
    inline void Callback();

    /**
     * @brief Callback for receiving odometry messages.
     *
     * This method processes incoming odometry messages and publishes
     * the equivalent transforms for both LiDAR and camera frames.
     *
     * @param msg Shared pointer to the received `nav_msgs::msg::Odometry` message.
     */
    inline void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

private:
    /**
     * @brief Subscription to the odometry topic.
     */
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;

    /**
     * @brief Transform broadcaster used to publish TF data to RViz.
     */
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    /**
     * @brief Publisher for LiDAR odometry transform messages.
     */
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr lidarOdometryPub_;

    /**
     * @brief Publisher for camera odometry transform messages.
     */
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr cameraOdometryPub_;
};

#endif // SUBSCRIBEANDPUBLISH_HPP
