#include "rclcpp/rclcpp.hpp"
#include "ros2_plugin.hpp"

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(spectacularai::ros2::Ros2Wrapper::build());
  rclcpp::shutdown();
  return 0;
}
