#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "../include/detection_node.h"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DetectionNode>());
  rclcpp::shutdown();
  return 0;
}