#pragma once

#include <memory>
#include <string>
#include <vector>
#include <array>

#include "controller_interface/chainable_controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"

#include "kdl/chain.hpp"
#include "kdl/jntarray.hpp"
#include "kdl/jacobian.hpp"
#include "kdl/frames.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/chainjnttojacsolver.hpp"
#include "kdl/chaindynparam.hpp"

namespace ombot_controller
{

class CartesianImpedanceController : public controller_interface::ChainableControllerInterface
{
public:
  CartesianImpedanceController();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

protected:
  controller_interface::return_type update_reference_from_subscribers() override;
  controller_interface::return_type update_and_write_commands(
  const rclcpp::Time &, const rclcpp::Duration &) override;

  // For now, don’t chain this controller (keep it simple)
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;
  bool on_set_chained_mode(bool chained) override;



private:
  // ---------------- Params ----------------
  std::vector<std::string> joint_names_;
  std::vector<double> effort_limits_;
  bool use_gravity_{true};
  KDL::Vector gravity_vec_{0.0, 0.0, -9.81};

  std::string base_link_;
  std::string tip_link_;
  std::string robot_description_;

  // Cartesian gains (diagonal)
  std::array<double,3> kp_xyz_{200.0, 200.0, 200.0};   // N/m
  std::array<double,3> kd_xyz_{ 30.0,  30.0,  30.0};   // N/(m/s)
  std::array<double,3> kr_xyz_{ 20.0,  20.0,  20.0};   // Nm/rad
  std::array<double,3> dr_xyz_{  2.0,   2.0,   2.0};   // Nm/(rad/s)

  double wrench_force_limit_{80.0};   // N
  double wrench_torque_limit_{10.0};  // Nm

  // Optional: measured dq LPF (reuse your joint controller pattern if you want)
  double vel_lpf_alpha_{1.0};

  // Publish EE pose (optional)
  bool publish_ee_pose_{true};
  std::string ee_pose_topic_{"/ee_pose"};
  std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>> ee_pub_;

  // ---------------- KDL ----------------
  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver_;
  std::unique_ptr<KDL::ChainDynParam> dyn_;

  KDL::JntArray q_kdl_, g_kdl_;
  KDL::Jacobian J_kdl_;

  // ---------------- Runtime buffers ----------------
  std::vector<double> q_, dq_, dq_filt_, tau_cmd_;

  // ---------------- Desired (RT) ----------------
  struct CartDesired
  {
    // desired pose in base_link
    KDL::Vector pd{0,0,0};
    KDL::Rotation Rd = KDL::Rotation::Identity();

    // desired twist in base_link
    KDL::Vector vd{0,0,0};
    KDL::Vector wd{0,0,0};

    // optional feedforward wrench in base_link
    KDL::Vector f_ff{0,0,0};
    KDL::Vector tau_ff{0,0,0};

    bool has_pose{false};
    bool has_twist{false};
    bool has_wrench_ff{false};
  };

  realtime_tools::RealtimeBuffer<CartDesired> des_rt_;
  CartDesired des_shadow_;

  // ---------------- ROS subs ----------------
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_twist_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_wrench_ff_;

  void pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void twist_cb(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void wrench_ff_cb(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);

  // ---------------- Helpers ----------------
  void clamp_effort(std::vector<double>& tau) const;
  void compute_gravity(const std::vector<double>& q, std::vector<double>& tau_g);
  void maybe_filter_velocity();

  // std::vector<double> ref_storage_;  // size 12


};

} // namespace ombot_controller
