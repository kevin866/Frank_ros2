#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("mecanum_forward_cmd");

  // Publisher to the MecanumDriveController
  auto pub = node->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/mecanum_controller/reference", 10);

  // Message setup
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.frame_id = "base_link";
  cmd.twist.linear.x = 2;   // forward speed (m/s)
  cmd.twist.linear.y = 0.0;
  cmd.twist.angular.z = 0.0;

  rclcpp::Rate rate(10);  // 10 Hz publishing rate
  RCLCPP_INFO(node->get_logger(), "Publishing forward command...");

  while (rclcpp::ok()) {
    cmd.header.stamp = node->get_clock()->now();
    pub->publish(cmd);
    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
