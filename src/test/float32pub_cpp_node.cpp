#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;


class Float32Pub : public rclcpp::Node
{
private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pubInt_;
    size_t count_;
    size_t cnt_loop_;
    void TimerCallback();

public:
    Float32Pub()
      : Node("float32pub_node"), count_(0)
    {
      pubInt_ = this->create_publisher<std_msgs::msg::Float32>("/input",10);
      timer_ = this->create_wall_timer(
      1500ms, std::bind(&Float32Pub::TimerCallback, this));
    }

};

void Float32Pub::TimerCallback()
{
      auto msg = std_msgs::msg::Float32();

      msg.data = count_++;

      RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", msg.data);
      pubInt_->publish(msg);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Float32Pub>());
  rclcpp::shutdown();
  return 0;
}
