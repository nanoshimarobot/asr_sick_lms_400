#include <lms400/lms400_component.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LMS400>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}