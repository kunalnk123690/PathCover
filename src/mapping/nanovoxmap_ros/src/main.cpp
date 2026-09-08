/**
 * @file main.cpp
 * @brief Entry point for the nanovoxmap_node executable.
 */
#include <ros/ros.h>
#include <thread>
#include "NanoVoxMapNode.hpp"

/**
 * @brief Initialize ROS, construct NanoVoxMapNode, and spin with a multi-threaded spinner.
 *
 * Parameter validation errors (NanoVoxMap::Parameters) and any other
 * exception thrown during setup are caught here, logged via `ROS_FATAL`,
 * and turned into a non-zero exit code rather than an unhandled-exception
 * crash.
 * @return 0 on normal shutdown, 1 if node construction threw.
 */
int main(int argc, char **argv) {
    ros::init(argc, argv, "nanovoxmap_node");

    try {
        NanoVoxMap::NanoVoxMapNode node(NanoVoxMap::Parameters(ros::NodeHandle("~")));

        const unsigned int num_threads = std::max(1u, std::thread::hardware_concurrency());
        ros::MultiThreadedSpinner spinner(num_threads);
        spinner.spin(); // blocks, but uses multiple threads
    } catch (const std::exception& e) {
        ROS_FATAL("nanovoxmap_node: %s", e.what());
        return 1;
    }
    return 0;
}