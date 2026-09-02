#include <rclcpp/rclcpp.hpp>
#include <thread>
#include "SubscribeAndPublish.hpp"


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    try {
        // All heavy planning runs on an internal worker thread created by
        // SubscribeAndPublish, NOT on the executor's threads. The executor
        // here only needs enough threads to keep the cloud callback and the
        // odom/target callbacks (separate callback groups) from blocking
        // each other.
        auto node = std::make_shared<SubscribeAndPublish>();

        rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
        executor.add_node(node);
        executor.spin();
    } catch (const std::exception &e) {
        RCLCPP_FATAL(rclcpp::get_logger("corridor_planning_node"), "%s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
