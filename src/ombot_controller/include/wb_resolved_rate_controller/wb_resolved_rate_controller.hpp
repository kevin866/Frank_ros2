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
#include <kdl/chainfksolverpos_recursive.hpp>


namespace ombot_controller {

class WholeBodyResolvedRateController : public controller_interface::ChainableControllerInterface
{
public:
  WholeBodyResolvedRateController();

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
  // double lambda_{0.02};        // DLS damping
  // double qdot_limit_{0.5};     // rad/s clamp
  // double integ_limit_{0.4};    // safety clamp on integrated q_ref
  // double step_limit_{0.021};    // new: max per-timestep change
  // double dt_ceiling_{0.05};   // max dt for integration
  // double null_scale_{0.5};   // max dt for integration
  // double err_db_{0.01};   // rad
  // double v_min_  = 0.03;   // rad/s
  // double tau_rebase_{0.5};
  // === Resolved-rate controller tuning (100 Hz) ===
  double lambda_{0.01};        // DLS damping; 0.01–0.05 is typical
  double qdot_limit_{0.4};     // rad/s per joint speed limit (bump to 1.0 if safe)
  double integ_limit_{0.20};   // rad clamp for integrated posture/error (anti-windup)
  double dt_ceiling_{0.03};    // s; cap dt used for integration (100 Hz -> 0.01, allow spikes to 0.03)

  double base_vx_limit_{0.3}; // m/s
  double base_vy_limit_{0.3}; // m/s
  double base_wz_limit_{0.8}; // rad/s


  // Max per-timestep joint change (derived for consistency)
  double step_limit_{ qdot_limit_ * dt_ceiling_ };  // = 0.7 * 0.03 = 0.021 rad/step @ worst allowed dt

  double null_scale_{0.8};     // dimensionless weight for posture bias (static)
  double err_db_{0.01};        // rad deadband to ignore tiny errors/noise
  double v_min_{0.01};         // rad/s floor to overcome stiction (apply only when |cmd| < v_min_)
  double tau_rebase_{0.5};     // s; smoothing/slew time constant for reference rebasing

  

  // double null_kp_{0.2};        // posture bias (0 disables)
  std::vector<double> q_home_; // desired home posture for nullspace
  std::string inner_ctrl_name_;
  // Map from joint index -> command_interfaces_ index for each type (or -1 if not present)
  std::vector<int> pos_cmd_index_;
  std::vector<int> vel_cmd_index_;
  std::vector<double> null_kp_, null_kd_;

  double task_mag_ema_ = 0.0;   // smoothed task magnitude
  const double ema_alpha_ = 0.8; // 0..1, higher = quicker response


  // In wb_resolved_rate_controller.hpp
  double base_weight_{3.0};  // cost weight for base DOFs
  double arm_weight_{1.0};   // cost weight for arm joints
  double base_cmd_scale_ = 0.2;  // or 3.0, tune this



  // KDL / kinematics
  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver_;
  KDL::JntArray q_kdl_;
  KDL::JntArray dq_kdl_;
  // For FK to get tip position in base frame
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  KDL::Frame tip_frame_;   // base_link -> tip_link pose

  // Base command publisher (to your mecanum controller topic)
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr base_cmd_pub_;
  std::string base_cmd_topic_{"cmd_vel"};   // or whatever you actually use


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
