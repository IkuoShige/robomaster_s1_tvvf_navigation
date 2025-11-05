#include "robomaster_s1_tvvf_navigation/waypoint_follower_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<robomaster_s1_tvvf_navigation::WaypointFollowerNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
