/**
 * @file main.cpp
 * @brief Entry point for the nanovoxmap_node executable.
 */
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include "NanoVoxMapNode.hpp"

/**
 * @brief Initialize ROS, construct NanoVoxMapNode, and spin with a multi-threaded executor.
 *
 * Parameter validation errors (NanoVoxMap::Parameters) and any other
 * exception thrown during setup are caught here, logged via `RCLCPP_FATAL`,
 * and turned into a non-zero exit code rather than an unhandled-exception
 * crash.
 * @return 0 on normal shutdown, 1 if node construction threw.
 */
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<NanoVoxMap::NanoVoxMapNode>();

        const unsigned int num_threads = std::max(1u, std::thread::hardware_concurrency());
        rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), num_threads);
        executor.add_node(node);
        executor.spin(); // blocks, but uses multiple threads
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("nanovoxmap_node"), "%s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
