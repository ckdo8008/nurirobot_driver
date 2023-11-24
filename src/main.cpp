#include "ros2-nurirobot-driver/nurirobot.hpp"
#include <chrono>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // Create node after ros init
  auto node = std::make_shared<Nurirobot>(); // What is auto??
  
  // auto timer_callback = [&node]() -> void { 
  //   node->cbFeedback();
  //   usleep(2000);
  //   node->read();
  //  };
  // auto timer = node->create_wall_timer(std::chrono::milliseconds(200), timer_callback);

  // while (rclcpp::ok())
  // {
  //   // node->read();
  //   rclcpp::spin_some(node);
  //   std::this_thread::sleep_for(std::chrono::microseconds(10));
  // }  
  // rclcpp::shutdown();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}