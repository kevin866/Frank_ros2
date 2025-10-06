#pragma once

#include <memory>
#include <string>
#include <vector>
#include <optional>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "realtime_tools/realtime_buffer.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

// KDL / URDF (optional, for gravity feed-forward)
#include "kdl/chain.hpp"
#include "kdl/jntarray.hpp"
#include "kdl/chaindynparam.hpp"
#include "kdl/frames.hpp"

namespace ombot_controller
{

/**
 * @brief Joint-space impedance controller.
 *
 * Command law (per joint i):
 *   tau_i = Kp_i*(q_d_i - q_i) + Kd_i*(dq_d_i - dq_i) + tau_ff_i [+ scale_g * tau_g_i]
 *
 * Params (declare in on_init):
 *   - joints (string[])            : ordered joint names (required)
 *   - Kp (double[])                : per-joint stiffness [Nm/rad] (required)
 *   - Kd (double[])                : per-joint damping   [Nms/rad] (required)
 *   - effort_limits (double[])     : |tau| clamp per joint; empty -> no clamp
 *   - use_gravity (bool)           : add gravity feed-forward using KDL (default: true)
 *   - gravity_xyz (double[3])      : base-frame gravity vector (default [0,0,-9.81])
 *   - base_link (string)           : for KDL (required if use_gravity)
 *   - tip_link  (string)           : for KDL (required if use_gravity)
 *   - robot_description (string)   : URDF XML; if empty, read local param of same name
 *   - vel_lpf_alpha (double)       : 1st-order LPF for measured dq if needed; 1.0 = off
 *
 * IO:
 *   State:   <joint>/position  (required), <joint>/velocity (preferred)
 *   Command: <joint>/effort
 *
 * Commands in:
 *   - topic: "~command" (sensor_msgs/JointState)
 *       name[] must match 'joints' order (or be empty -> assumes order),
 *       position[], velocity[], effort[] used as q_d, dq_d, tau_ff respectively (each optional).
 */
class JointImpedanceController : public controller_interface::ControllerInterface
{
public:
  JointImpedanceController();

  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

  controller_interface::return_type
  update(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  // ----- parameters -----
  std::vector<std::string> joint_names_;
  std::vector<double> kp_, kd_;
  std::vector<double> effort_limits_;  // optional
  bool use_gravity_{true};
  double vel_lpf_alpha_{1.0};          // 1.0 => no filtering

  // KDL (optional gravity FF)
  std::string base_link_;
  std::string tip_link_;
  KDL::Vector gravity_vec_{0.0, 0.0, -9.81};
  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainDynParam> dyn_;
  KDL::JntArray q_kdl_, g_kdl_; // current q, gravity torques

  // ----- runtime buffers (state & cmd) -----
  // State handles are acquired via controller_interface APIs; here we keep caches.
  std::vector<double> q_;               // measured positions
  std::vector<double> dq_;              // measured velocities (or filtered numerical diff)
  std::vector<double> dq_filt_;         // for LPF if needed
  std::vector<double> last_q_;          // for numerical velocity if velocity state is absent
  bool have_velocity_state_{false};

  // Desired command buffer (real-time safe)
  struct Desired
  {
    std::vector<double> qd;     // desired positions (size = dof)  [optional]
    std::vector<double> dqd;    // desired velocities              [optional]
    std::vector<double> tau_ff; // extra feed-forward efforts      [optional]
    bool has_qd{false};
    bool has_dqd{false};
    bool has_tau_ff{false};
  };
  realtime_tools::RealtimeBuffer<Desired> desired_rt_;
  Desired desired_shadow_; // non-RT staging for callbacks

  // Output buffer
  std::vector<double> tau_cmd_;

  // ROS comms
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_cmd_;

  // ----- helpers -----
  void command_cb(const sensor_msgs::msg::JointState::SharedPtr msg);
  void compute_gravity(const std::vector<double>& q, std::vector<double>& tau_g);
  void clamp_effort(std::vector<double>& tau) const;
  void maybe_filter_velocity(double dt);
};

}  // namespace ombot_controller
