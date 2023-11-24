#include "ros2-nurirobot-driver/nurirobot.hpp"
#include <chrono>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Nurirobot>(); // What is auto??
  
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}