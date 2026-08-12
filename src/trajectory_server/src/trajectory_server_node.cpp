#include "trajectory_server.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "trajectory_server_node");
    ros::NodeHandle nh("~");
    trajectory_server::TrajOptNode node(nh);
    ros::spin();
    return 0;
}
