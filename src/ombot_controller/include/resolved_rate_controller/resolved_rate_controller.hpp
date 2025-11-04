#pragma once
#include <memory>
#include <vector>
#include <string>
#include <algorithm>

#include "hardware_interface/handle.hpp"
#include "controller_interface/chainable_controller_interface.hpp"
#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "kdl/chain.hpp"
#include "kdl/chainjnttojacsolver.hpp"
#include "Eigen/Dense"

namespace ombot_controller {

class ResolvedRateController : public controller_interface::ChainableControllerInterface
{
public:
  ResolvedRateController();

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;


  // *** Chainable hooks ***
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;
  bool on_set_chained_mode(bool chained) override;  // exact match

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;


protected:
  controller_interface::return_type update_reference_from_subscribers() override;

  controller_interface::return_type update_and_write_commands(
      const rclcpp::Time &, const rclcpp::Duration &) override;


private:
  // params
  std::vector<std::string> joint_names_;
  std::string base_link_, tip_link_;
  double lambda_{0.02};        // DLS damping
  double qdot_limit_{2.5};     // rad/s clamp
  double integ_limit_{1e9};    // safety clamp on integrated q_ref
  double step_limit_{0.05};    // new: max per-timestep change
  double dt_ceiling_{0.1};   // max dt for integration
  double null_scale_{0.5};   // max dt for integration
  double err_db_{0.01};   // rad
  double v_min_  = 0.03;   // rad/s
  double tau_rebase_{0.5};

  

  // double null_kp_{0.2};        // posture bias (0 disables)
  std::vector<double> q_home_; // desired home posture for nullspace
  std::string inner_ctrl_name_;
  // Map from joint index -> command_interfaces_ index for each type (or -1 if not present)
  std::vector<int> pos_cmd_index_;
  std::vector<int> vel_cmd_index_;
  std::vector<double> null_kp_, null_kd_;



  // KDL / kinematics
  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver_;
  KDL::JntArray q_kdl_;
  KDL::JntArray dq_kdl_;

  // State interfaces (read from hardware)
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> pos_states_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> vel_states_;

  // Reference interfaces (we own & export, other controllers can claim *to read*)
  // But we ourselves will be the writer.
  struct RefSlot {
    std::string name;
    double value{0.0};
  };
  std::vector<RefSlot> pos_ref_slots_;
  std::vector<RefSlot> vel_ref_slots_;

  // Simple RT command buffer for desired EE twist
  struct Cmd { double vx=0, vy=0, vz=0, wx=0, wy=0, wz=0; bool valid=false; };
  realtime_tools::RealtimeBuffer<Cmd> cmd_rt_;
  Cmd cmd_cached_;

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_twist_;

  // integrator state
  std::vector<double> q_ref_;  // integrated q
  std::vector<double> qdot_ref_;

  void twist_cb(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void write_refs_to_slots();

  rclcpp::Time last_cmd_time_;
  double cmd_timeout_{0.25};   // seconds; tune 0.1–0.5
};

} // namespace ombot_controller
