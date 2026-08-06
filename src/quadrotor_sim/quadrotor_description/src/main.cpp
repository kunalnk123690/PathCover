#include <iostream>
#include "ground_truth.hpp"


int main(int argc, char** argv) {
    ros::init(argc, argv, "quadrotor_ground_truth_node");
        
    GroundTruthPublisher GroundTruthObject;
    
    while(ros::ok()) {
        ros::Rate(500).sleep();
        ros::spinOnce();
    }


}