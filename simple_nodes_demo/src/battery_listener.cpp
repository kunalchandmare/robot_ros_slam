#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

using std::placeholders::_1;

class BatteryListener : public rclcpp::Node
{
public:
  BatteryListener()
  : Node("battery_listener")
  {
    subscription_ = this->create_subscription<std_msgs::msg::Int32>(
      "battery_level", 10, std::bind(&BatteryListener::topic_callback, this, _1));
    RCLCPP_INFO(this->get_logger(), "C++ Battery listener started. Listening on /battery_level");
  }

private:
  void topic_callback(const std_msgs::msg::Int32::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "[C++] Received battery level: %d%%", msg->data);
  }

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BatteryListener>());
  rclcpp::shutdown();
  return 0;
}
