#include "rclcpp/rclcpp.hpp"
#include "swarm_intelligence_ros2/mrfo/mrfo_component.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<MrfoComponent>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}