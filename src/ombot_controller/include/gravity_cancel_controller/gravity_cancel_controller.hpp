#pragma once

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

// KDL / URDF
#include "kdl/chain.hpp"
#include "kdl/jntarray.hpp"
#include "kdl/chaindynparam.hpp"
#include "kdl/frames.hpp"

namespace ombot_controller
{

/**
 * @brief Feedforward gravity-cancellation controller.
 *
 * Computes tau_g with KDL::ChainDynParam from URDF-provided inertias:
 *   JntToGravity(q, g)  -> command effort = scale * g  (with optional LPF/limits)
 *
 * Params (declare in on_init):
 *   - joints (string[])          : ordered joint names (required)
 *   - base_link (string)         : URDF base link (required)
 *   - tip_link  (string)         : URDF tip link  (required)
 *   - robot_description (string) : URDF XML; if empty, read local param of same name
 *   - gravity_xyz (double[3])    : gravity vector in base frame, default [0,0,-9.81]
 *   - scale (double)             : scalar multiplier, default 1.0
 *   - effort_limits (double[])   : |tau| clamp per joint; empty -> no clamp
 *   - alpha (double)             : 1st-order LPF factor in [0,1], 1=no filter
 *
 * Interfaces:
 *   - State:   <joint>/position
 *   - Command: <joint>/effort
 */
class GravityCancelController : public controller_interface::ControllerInterface
{
public:
  GravityCancelController();

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
  std::string base_link_;
  std::string tip_link_;
  double scale_{1.0};
  double alpha_{1.0};
  std::vector<double> effort_limits_;  // optional

  // ----- KDL data -----
  KDL::Vector gravity_vec_{0.0, 0.0, -9.81};
  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainDynParam> dyn_;
  KDL::JntArray q_, g_;

  // ----- runtime buffers -----
  std::vector<double> last_cmd_;  // for LPF
};

}  // namespace ombot_controller
