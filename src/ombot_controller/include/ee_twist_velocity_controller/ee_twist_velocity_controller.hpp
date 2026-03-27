#pragma once

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/subscription.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "std_msgs/msg/float64.hpp"

#include "kdl/chain.hpp"
#include "kdl/jntarray.hpp"
#include "kdl/jacobian.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "kdl/chainjnttojacsolver.hpp"
#include <mutex>
#include "geometry_msgs/msg/vector3_stamped.hpp"

#include <Eigen/Dense>

namespace ombot_controller
{

class EeTwistVelocityController : public controller_interface::ControllerInterface
{
public:
  EeTwistVelocityController() = default;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

  controller_interface::return_type update(const rclcpp::Time & time,
                                           const rclcpp::Duration & period) override;

private:
  // ----- params -----
  std::vector<std::string> joint_names_;
  std::string base_link_;
  std::string tip_link_;
  std::string robot_description_;

  std::string twist_topic_;
  double cmd_timeout_{0.2};          // seconds
  double max_qdot_{1.5};             // rad/s clamp
  double max_task_lin_{0.3};         // m/s clamp
  double max_task_ang_{1.0};         // rad/s clamp
  double lambda_{0.02};              // DLS damping
  double cmd_alpha_{1.0};            // 1.0 = no smoothing, smaller = smoother

  bool use_posture_nullspace_{false};
  double null_kp_{0.0};
  std::vector<double> q_nom_;

  // soft joint limits (optional)
  bool use_soft_limits_{false};
  std::vector<double> q_min_, q_max_;
  double q_soft_margin_{0.2};
  double k_limit_{2.0};              // rad/s per rad of penetration-ish
  double d_limit_{0.0};              // rad/s per rad/s

  // ----- state -----
  bool active_{false};

  // KDL
  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver_;
  KDL::JntArray q_kdl_;
  KDL::JntArray dq_kdl_;
  KDL::Jacobian J_kdl_;

  // EE cmd buffer
  using TwistMsg = geometry_msgs::msg::TwistStamped;
  realtime_tools::RealtimeBuffer<std::shared_ptr<TwistMsg>> rt_twist_;
  rclcpp::Subscription<TwistMsg>::SharedPtr twist_sub_;
  rclcpp::Time last_cmd_time_;

  // last dq for smoothing
  std::vector<double> dq_cmd_prev_;

  // helpers
  void twist_cb_(const TwistMsg::SharedPtr msg);
  void write_velocity_cmd_(const std::vector<double> & dq_cmd);
  void clamp_task_(Eigen::Matrix<double,6,1> & v) const;
  void apply_soft_limits_(const std::vector<double>& q,
                          const std::vector<double>& dq,
                          std::vector<double>& dq_cmd) const;
  

  using Vector3Msg = geometry_msgs::msg::Vector3Stamped;

  rclcpp::Subscription<Vector3Msg>::SharedPtr e1_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr manip_pub_;

  mutable std::mutex e1_mtx_;
  double e1_norm_{0.0};
  rclcpp::Time e1_stamp_{0, 0, RCL_ROS_TIME};
  bool have_e1_{false};

  std::string e1_topic_{"/debug/e1"};
  double e1_scale_{1.0};   // normalization divisor
};

}  // namespace ombot_controller