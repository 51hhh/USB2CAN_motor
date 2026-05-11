#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "positioning_bridge_ros2/positioning_bridge_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<positioning_bridge_ros2::PositioningBridgeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
