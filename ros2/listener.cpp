#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

void cb(const std_msgs::msg::String::SharedPtr msg)
{
  std::cout << "I heard: " << msg->data << std::endl;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("my_subscriber");
  auto sub = node->create_subscription<std_msgs::msg::String>("chatter", 10, cb);
  rclcpp::spin(node);
  return 0;
}
