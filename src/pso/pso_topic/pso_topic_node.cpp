#include "rclcpp/rclcpp.hpp"
#include "swarm_intelligence_ros2/pso/pso_topic/pso_topic_component.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<PsoComponent>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}