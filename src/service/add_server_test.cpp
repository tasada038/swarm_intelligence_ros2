#include "rclcpp/rclcpp.hpp"
#include "cpp_srvcli_srv/srv/function.hpp"
#include "std_msgs/msg/float32.hpp"
#include <memory>

using Function = cpp_srvcli_srv::srv::Function;
using std::placeholders::_1;
using std::placeholders::_2;

class ServerNode : public rclcpp::Node {
public:
  ServerNode() : Node("test_server") {
    srv_ = create_service<Function>(
        "service_test", std::bind(&ServerNode::srv_callback, this, _1, _2));
    publisher_ =
        this->create_publisher<std_msgs::msg::Float32>("/output", 10);
  }

private:
  rclcpp::Service<Function>::SharedPtr srv_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;

  void srv_callback(const std::shared_ptr<Function::Request> request,
                    const std::shared_ptr<Function::Response> response) {

    std_msgs::msg::Float32 data;

    response->d = request->a + request->b + request->c;
    data.data = response->d;
    publisher_->publish(data);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request  a: %f, b: %f, c: %f",
                request->a, request->b, request->c);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%f]", response->d);
  }
};


int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServerNode>());
  rclcpp::shutdown();
  return 0;
}