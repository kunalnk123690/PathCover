#include <iostream>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "SubscribeAndPublish.hpp"

using namespace std;


int main(int argc, char **argv) {
    ros::init(argc, argv, "corridor_planning_node");

    // General handle: serves the cloud + target callbacks on the global queue.
    // The cloud callback is now trivial (just stows the latest message), so it
    // can never block anything.
    ros::NodeHandle nh_general;

    // Transform/odom handle: gets its OWN callback queue so high-rate pose
    // updates are processed on a dedicated thread and are never stuck behind
    // anything else. (The original code claimed separate queues but both
    // handles actually shared the single global queue.)
    ros::NodeHandle nh_transform;
    ros::CallbackQueue transform_queue;
    nh_transform.setCallbackQueue(&transform_queue);

    // Pass the handles into the main class. All heavy planning runs on an
    // internal worker thread created by SubscribeAndPublish, NOT on these
    // spinner threads.
    SubscribeAndPublish SAPObject(Config(ros::NodeHandle("~")), nh_general, nh_transform);

    // One spinner thread for the global queue (cloud + target callbacks)...
    ros::AsyncSpinner general_spinner(1);
    general_spinner.start();

    // ...and one dedicated to the odom/transform queue.
    ros::AsyncSpinner transform_spinner(1, &transform_queue);
    transform_spinner.start();

    ros::waitForShutdown();

    return 0;
}