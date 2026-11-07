#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "zemo_navigation/collision_detector/collision_detector.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<zemo_navigation::CollisionDetector>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}