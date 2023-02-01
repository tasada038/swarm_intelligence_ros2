#include "rclcpp/rclcpp.hpp"
#include "cpp_srvcli_srv/srv/function.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;
using Function = cpp_srvcli_srv::srv::Function;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("test_client");
  rclcpp::Client<Function>::SharedPtr client = node->create_client<Function>("service_test");

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto request = std::make_shared<Function::Request>();
  request->a = atoll(argv[1]);
  request->b = atoll(argv[2]);
  request->c = atoll(argv[3]);
  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service returned success %f", result.get()->d);
  }
  rclcpp::shutdown();
  return 0;
}