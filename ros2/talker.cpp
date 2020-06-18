#include <iostream>
#include <unistd.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("my_publisher");
  auto pub = node->create_publisher<std_msgs::msg::String>("chatter", 10);
  std_msgs::msg::String message;
  auto i = 0;
  while (rclcpp::ok()) {
    message.data = "Hello world: " + std::to_string(i++);
    std::cout << "Publishing: " << message.data << std::endl;
    pub->publish(message);
    usleep(500000);
  }
  return 0;
}
