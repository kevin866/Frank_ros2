#include "joint_excitation_controller/joint_excitation_controller.hpp"

#include <algorithm>
#include <cmath>

#include "pluginlib/class_list_macros.hpp"

namespace {
  // 16-bit LFSR with taps for maximal length: x^16 + x^14 + x^13 + x^11 + 1
  inline uint32_t step_lfsr(uint32_t lfsr)
  {
    unsigned lsb = lfsr & 1u;     // get LSB (output bit)
    lfsr >>= 1;                   // shift
    if (lsb) {
      lfsr ^= 0xB400u;            // polynomial taps
    }
    return lfsr ? lfsr : 0xACE1u; // avoid zero lockup
  }
}


namespace ombot_controller
{

controller_interface::InterfaceConfiguration
JointExcitationController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & j : joint_names_) {
    conf.names.push_back(j + "/effort");
  }
  return conf;
}

controller_interface::InterfaceConfiguration
JointExcitationController::state_interface_configuration() const
{
  // We don't *need* state for the excitation itself, but it is useful for logging.
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & j : joint_names_) {
    conf.names.push_back(j + "/position");
    conf.names.push_back(j + "/velocity");
  }
  return conf;
}

controller_interface::CallbackReturn
JointExcitationController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("joints", {});
    auto_declare<std::vector<double>>("effort_limits", {});
    auto_declare<std::vector<double>>("amplitudes", {});

    auto_declare<std::string>("mode", "prbs");      // "prbs", "sine", "chirp"
    auto_declare<double>("prbs_dwell", 0.05);       // s
    auto_declare<double>("sine_freq", 0.5);         // Hz
    auto_declare<double>("chirp_f_start", 0.1);     // Hz
    auto_declare<double>("chirp_f_end", 5.0);       // Hz
    auto_declare<double>("test_duration", 0.0);     // s, 0 => run forever
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "JointExcitationController on_init exception: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
JointExcitationController::on_configure(const rclcpp_lifecycle::State &)
{
  joint_names_   = get_node()->get_parameter("joints").as_string_array();
  effort_limits_ = get_node()->get_parameter("effort_limits").as_double_array();
  amplitudes_    = get_node()->get_parameter("amplitudes").as_double_array();

  mode_          = get_node()->get_parameter("mode").as_string();
  prbs_dwell_    = get_node()->get_parameter("prbs_dwell").as_double();
  sine_freq_     = get_node()->get_parameter("sine_freq").as_double();
  chirp_f_start_ = get_node()->get_parameter("chirp_f_start").as_double();
  chirp_f_end_   = get_node()->get_parameter("chirp_f_end").as_double();
  test_duration_ = get_node()->get_parameter("test_duration").as_double();

  if (joint_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "JointExcitationController: 'joints' param must be non-empty.");
    return controller_interface::CallbackReturn::ERROR;
  }

  const size_t N = joint_names_.size();

  if (!effort_limits_.empty() && effort_limits_.size() != N) {
    RCLCPP_WARN(get_node()->get_logger(),
                "effort_limits size (%zu) != joints (%zu). Ignoring limits.",
                effort_limits_.size(), N);
    effort_limits_.clear();
  }

  if (amplitudes_.empty()) {
    // default: same amplitude for all joints, e.g. 0.1 (rad-equivalent torque/current)
    amplitudes_.assign(N, 0.1);
  } else if (amplitudes_.size() != N) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "amplitudes size (%zu) != joints (%zu).", amplitudes_.size(), N);
    return controller_interface::CallbackReturn::ERROR;
  }

  tau_cmd_.assign(N, 0.0);
  prbs_state_.assign(N, +1);

  if (prbs_dwell_ <= 0.0) prbs_dwell_ = 0.05;

  RCLCPP_INFO(
    get_node()->get_logger(),
    "JointExcitationController configured: N=%zu, mode=%s, dwell=%.3f, sine_f=%.3f, "
    "chirp_f=[%.3f, %.3f], duration=%.3f",
    N, mode_.c_str(), prbs_dwell_, sine_freq_, chirp_f_start_, chirp_f_end_, test_duration_);
  
  tau_pub_ = std::make_shared<
    realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(
    get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
    "~/tau_cmd", rclcpp::QoS(10)));
    init_prbs();   

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
JointExcitationController::on_activate(const rclcpp_lifecycle::State &)
{
  if (command_interfaces_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "JointExcitationController: no command interfaces on activate.");
    return controller_interface::CallbackReturn::ERROR;
  }

  // zero initial commands
  for (auto & ci : command_interfaces_) {
    ci.set_value(0.0);
  }

  start_time_ = get_node()->get_clock()->now();
  next_switch_time_ = start_time_ + rclcpp::Duration::from_seconds(prbs_dwell_);
  std::fill(prbs_state_.begin(), prbs_state_.end(), +1);
  std::fill(tau_cmd_.begin(), tau_cmd_.end(), 0.0);
  active_ = true;

  RCLCPP_INFO(get_node()->get_logger(), "JointExcitationController activated.");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
JointExcitationController::on_deactivate(const rclcpp_lifecycle::State &)
{
  active_ = false;
  for (auto & ci : command_interfaces_) {
    ci.set_value(0.0);
  }
  RCLCPP_INFO(get_node()->get_logger(), "JointExcitationController deactivated.");
  return controller_interface::CallbackReturn::SUCCESS;
}

void JointExcitationController::clamp_effort(std::vector<double> & tau) const
{
  if (effort_limits_.empty()) return;
  for (size_t i = 0; i < tau.size(); ++i) {
    const double lim = std::abs(effort_limits_[i]);
    tau[i] = std::clamp(tau[i], -lim, lim);
  }
}

void JointExcitationController::init_prbs()
  {
    const size_t N = joint_names_.size();
    lfsr_state_.resize(N);
    prbs_state_.resize(N);

    for (size_t i = 0; i < N; ++i) {
      // simple nonzero seeds, different per joint
      lfsr_state_[i] = 0xACE1u ^ static_cast<uint32_t>(i + 1);
      prbs_state_[i] = 1;  // start at +1
    }

    next_switch_time_ = get_node()->get_clock()->now() +
      rclcpp::Duration::from_seconds(prbs_dwell_);
  }

// void JointExcitationController::update_prbs(const rclcpp::Time & now)
// {
//   const double t_now = now.seconds();
//   const double t_next = next_switch_time_.seconds();

//   if (t_now >= t_next) {
//     // flip sign for all joints each dwell interval
//     for (auto & s : prbs_state_) {
//       s = -s;
//     }
//     next_switch_time_ = now + rclcpp::Duration::from_seconds(prbs_dwell_);
//   }

//   const size_t N = joint_names_.size();
//   for (size_t i = 0; i < N; ++i) {
//     tau_cmd_[i] = static_cast<double>(prbs_state_[i]) * amplitudes_[i];
//   }
// }


void JointExcitationController::update_prbs(const rclcpp::Time & now)
{
  const double t_now  = now.seconds();
  const double t_next = next_switch_time_.seconds();

  if (t_now >= t_next) {
    // advance each joint's LFSR and map bit → ±1
    for (size_t i = 0; i < lfsr_state_.size(); ++i) {
      lfsr_state_[i] = step_lfsr(lfsr_state_[i]);
      int bit = static_cast<int>(lfsr_state_[i] & 1u);  // 0 or 1
      prbs_state_[i] = (bit == 0) ? -1 : 1;
    }
    next_switch_time_ = now + rclcpp::Duration::from_seconds(prbs_dwell_);
  }

  const size_t N = joint_names_.size();
  for (size_t i = 0; i < N; ++i) {
    tau_cmd_[i] = static_cast<double>(prbs_state_[i]) * amplitudes_[i];
  }
}


void JointExcitationController::compute_sine(double t)
{
  const double w = 2.0 * M_PI * sine_freq_;  // rad/s
  const size_t N = joint_names_.size();
  const double s = std::sin(w * t);
  for (size_t i = 0; i < N; ++i) {
    tau_cmd_[i] = amplitudes_[i] * s;
  }
}

void JointExcitationController::compute_chirp(double t)
{
  // Simple linear chirp: f(t) = f0 + k t over test_duration_
  if (test_duration_ <= 0.0) {
    // fall back to sine style if duration not set
    compute_sine(t);
    return;
  }

  const double f0 = chirp_f_start_;
  const double f1 = chirp_f_end_;
  const double T  = test_duration_;

  const double k = (f1 - f0) / T;  // Hz/s
//   const double f_t = f0 + k * t;

  // phase(t) for linear chirp: 2π ( f0 t + 0.5 k t^2 )
  const double phase = 2.0 * M_PI * (f0 * t + 0.5 * k * t * t);
  const double s = std::sin(phase);

  const size_t N = joint_names_.size();
  for (size_t i = 0; i < N; ++i) {
    tau_cmd_[i] = amplitudes_[i] * s;
  }
}

controller_interface::return_type
JointExcitationController::update(const rclcpp::Time & time,
                                  const rclcpp::Duration & /*period*/)
{
  if (!active_) {
    return controller_interface::return_type::OK;
  }

  const size_t N = joint_names_.size();
  if (command_interfaces_.size() != N) {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 2000,
      "JointExcitationController: cmd interfaces (%zu) != joints (%zu)",
      command_interfaces_.size(), N);
    return controller_interface::return_type::ERROR;
  }

  const double t = (time - start_time_).seconds();

  // Stop after test_duration_, if set
  if (test_duration_ > 0.0 && t >= test_duration_) {
    std::fill(tau_cmd_.begin(), tau_cmd_.end(), 0.0);
    for (size_t i = 0; i < N; ++i) {
      command_interfaces_[i].set_value(0.0);
    }
    return controller_interface::return_type::OK;
  }

  // --- Generate excitation ---
  if (mode_ == "prbs") {
    update_prbs(time);
  } else if (mode_ == "sine") {
    compute_sine(t);
  } else if (mode_ == "chirp") {
    compute_chirp(t);
  } else {
    // unknown mode: just zero
    std::fill(tau_cmd_.begin(), tau_cmd_.end(), 0.0);
  }

  clamp_effort(tau_cmd_);

  // Write to hardware
  for (size_t i = 0; i < N; ++i) {
    command_interfaces_[i].set_value(tau_cmd_[i]);
  }

  if (tau_pub_ && tau_pub_->trylock()) {
    auto & msg = tau_pub_->msg_;
    msg.data = tau_cmd_;   // copy vector<double> -> data
    tau_pub_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

}  // namespace ombot_controller

PLUGINLIB_EXPORT_CLASS(
  ombot_controller::JointExcitationController,
  controller_interface::ControllerInterface)


