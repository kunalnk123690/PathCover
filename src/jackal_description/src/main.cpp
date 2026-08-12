#include <iostream>
#include <ros/ros.h>
#include "tf_broadcaster.hpp"

using namespace std;


int main(int argc, char **argv) {
    ros::init(argc, argv, "jackal_tf_broadcaster");
    ros::NodeHandle nh("~");

    TfBroadcaster TfBroadcasterObject(nh);

    while(ros::ok()) {
        ros::spinOnce();
        ros::Rate(1000).sleep();
    }


    return 0;

}