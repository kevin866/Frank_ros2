#pragma once

#include <rclcpp/rclcpp.hpp>

namespace ombot_base_kinematics
{

class MecanumKinOdom : public rclcpp::Node
{
public:
  explicit MecanumKinOdom(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  rclcpp::Time now_or_steady() const;
};

}  // namespace ombot_base_kinematics
