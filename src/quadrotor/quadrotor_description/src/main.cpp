#include "SubscribeAndPublish.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SubscribeAndPublish>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
