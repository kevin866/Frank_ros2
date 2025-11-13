#include "resolved_rate_controller/resolved_rate_controller.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <sstream> 

using controller_interface::InterfaceConfiguration;
using controller_interface::interface_configuration_type;

namespace ombot_controller {

ResolvedRateController::ResolvedRateController() = default;

controller_interface::CallbackReturn ResolvedRateController::on_init() {
  try {
    auto_declare<std::vector<std::string>>("joints", {});
    auto_declare<std::string>("base_link", "");
    auto_declare<std::string>("tip_link", "");
    auto_declare<std::string>("robot_description", "");
    auto_declare<double>("lambda", 0.02);
    auto_declare<double>("qdot_limit", 2.0);
    // auto_declare<double>("null_kp", 0.0);
    auto_declare<std::vector<double>>("null_kp", {});
    auto_declare<std::vector<double>>("null_kd", {});
    
    auto_declare<std::vector<double>>("q_home", {});
    auto_declare<double>("integrator_limit", 1.0);
    auto_declare<std::string>("inner_controller", "");  // downstream controller name
    auto_declare<double>("cmd_timeout", 0.25);
    auto_declare<double>("v_min", 0.03);
    auto_declare<std::string>("twist_frame", "link_1");
    auto_declare<double>("tau_rebase", tau_rebase_);

  } catch (...) { return CallbackReturn::ERROR; }
  return CallbackReturn::SUCCESS;
}



controller_interface::CallbackReturn
ResolvedRateController::on_configure(const rclcpp_lifecycle::State &) {
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  if (joint_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'joints' is required.");
    return CallbackReturn::ERROR;
  }
  const size_t N = joint_names_.size();

  integ_limit_ = get_node()->get_parameter("integrator_limit").as_double();
  base_link_ = get_node()->get_parameter("base_link").as_string();
  tip_link_  = get_node()->get_parameter("tip_link").as_string();
  lambda_    = get_node()->get_parameter("lambda").as_double();
  qdot_limit_= get_node()->get_parameter("qdot_limit").as_double();
  // null_kp_   = get_node()->get_parameter("null_kp").as_double();
  // null_kp_ = get_node()->get_parameter("null_kp").as_double_array();
  // null_kd_ = get_node()->get_parameter("null_kd").as_double_array();
  q_home_    = get_node()->get_parameter("q_home").as_double_array();
  inner_ctrl_name_ = get_node()->get_parameter("inner_controller").as_string();
  cmd_timeout_   = get_node()->get_parameter("cmd_timeout").as_double();
  v_min_        = get_node()->get_parameter("v_min").as_double();
  tau_rebase_ = get_node()->get_parameter("tau_rebase").as_double();

  // expected_frame_= get_node()->get_parameter("twist_frame").as_string();
  if (q_home_.size() != joint_names_.size()) q_home_.assign(joint_names_.size(), 0.0);


  // null_kp
  if (get_node()->has_parameter("null_kp")) {
    auto p = get_node()->get_parameter("null_kp");
    if (p.get_type() == rclcpp::PARAMETER_DOUBLE_ARRAY) {
      auto v = p.as_double_array();
      if (v.size() == N) null_kp_ = v;
      else {
        RCLCPP_WARN(get_node()->get_logger(),
          "null_kp vector length (%zu) != joints (%zu); truncating/repeating.", v.size(), N);
        for (size_t i = 0; i < N; ++i) null_kp_[i] = v[i % v.size()];
      }
    } else if (p.get_type() == rclcpp::PARAMETER_DOUBLE) {
      double s = p.as_double();
      std::fill(null_kp_.begin(), null_kp_.end(), s);
    }
  }

  // null_kd
  if (get_node()->has_parameter("null_kd")) {
    auto p = get_node()->get_parameter("null_kd");
    if (p.get_type() == rclcpp::PARAMETER_DOUBLE_ARRAY) {
      auto v = p.as_double_array();
      if (v.size() == N) null_kd_ = v;
      else {
        RCLCPP_WARN(get_node()->get_logger(),
          "null_kd vector length (%zu) != joints (%zu); truncating/repeating.", v.size(), N);
        for (size_t i = 0; i < N; ++i) null_kd_[i] = v[i % v.size()];
      }
    } else if (p.get_type() == rclcpp::PARAMETER_DOUBLE) {
      double s = p.as_double();
      std::fill(null_kd_.begin(), null_kd_.end(), s);
    }
  }

  // KDL chain
  std::string urdf_xml = get_node()->get_parameter("robot_description").as_string();
  if (urdf_xml.empty()) {
    rclcpp::Parameter p; if (get_node()->get_parameter("robot_description", p)) urdf_xml = p.as_string();
  }
  KDL::Tree tree;
  if (!kdl_parser::treeFromString(urdf_xml, tree)) { RCLCPP_ERROR(get_node()->get_logger(),"URDF->KDL failed"); return CallbackReturn::ERROR; }
  if (!tree.getChain(base_link_, tip_link_, chain_)) { RCLCPP_ERROR(get_node()->get_logger(),"KDL chain extract failed"); return CallbackReturn::ERROR; }
  if (chain_.getNrOfJoints() != joint_names_.size()) { RCLCPP_ERROR(get_node()->get_logger(),"Chain DOF mismatch"); return CallbackReturn::ERROR; }
  jac_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(chain_);
  q_kdl_ = KDL::JntArray(chain_.getNrOfJoints());
  dq_kdl_ = KDL::JntArray(chain_.getNrOfJoints());

  // subscribe desired EE twist
  sub_twist_ = get_node()->create_subscription<geometry_msgs::msg::TwistStamped>(
      "~/ee_twist", rclcpp::SystemDefaultsQoS(),
      std::bind(&ResolvedRateController::twist_cb, this, std::placeholders::_1));

  // pre-size integrators
  q_ref_.assign(joint_names_.size(), 0.0);
  qdot_ref_.assign(joint_names_.size(), 0.0);

  return CallbackReturn::SUCCESS;
}




InterfaceConfiguration ResolvedRateController::state_interface_configuration() const {
  // We read joint positions & velocities from hardware
  InterfaceConfiguration conf;
  conf.type = interface_configuration_type::INDIVIDUAL;
  for (auto &j : joint_names_) { conf.names.push_back(j + "/" + hardware_interface::HW_IF_POSITION); }
  for (auto &j : joint_names_) { conf.names.push_back(j + "/" + hardware_interface::HW_IF_VELOCITY); }
  return conf;
}

bool ResolvedRateController::on_set_chained_mode(bool /* chained */) {
  // chained_mode_ = chained;          // optional member
  return true;
}

// InterfaceConfiguration ResolvedRateController::command_interface_configuration() const {
//   return { interface_configuration_type::NONE, {} };
// }

controller_interface::InterfaceConfiguration
ResolvedRateController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  // Full names: "<downstream_controller>/<joint>/<signal>"
  for (const auto& j : joint_names_) {
    conf.names.push_back(inner_ctrl_name_ + "/" + j + "/position");
    conf.names.push_back(inner_ctrl_name_ + "/" + j + "/velocity");
  }
  return conf;
}


std::vector<hardware_interface::CommandInterface>
ResolvedRateController::on_export_reference_interfaces()
{
  const size_t N = joint_names_.size();
  reference_interfaces_.assign(2 * N, 0.0);

  std::vector<hardware_interface::CommandInterface> refs;
  refs.reserve(2 * N);

  const std::string ctrl = get_node()->get_name();  // important!

  for (size_t i = 0; i < N; ++i) {
    refs.emplace_back(ctrl, joint_names_[i] + "/position", &reference_interfaces_[i]);
    refs.emplace_back(ctrl, joint_names_[i] + "/velocity", &reference_interfaces_[N + i]);
  }
  return refs;
}

controller_interface::CallbackReturn
ResolvedRateController::on_activate(const rclcpp_lifecycle::State &) {
  // Bind state interfaces (order: all pos then all vel)
  const size_t N = joint_names_.size();
  if (state_interfaces_.size() != 2*N) {
    RCLCPP_ERROR(get_node()->get_logger(),"state interfaces size mismatch");
    return CallbackReturn::ERROR;
  }
  for (const auto &ci : command_interfaces_) {
    RCLCPP_INFO(get_node()->get_logger(), "RR claimed -> %s/%s",
                ci.get_name().c_str(), ci.get_interface_name().c_str());
  }

  pos_states_.clear(); vel_states_.clear();
  for (size_t i=0;i<N;i++) pos_states_.push_back(state_interfaces_[i]);
  for (size_t i=0;i<N;i++) vel_states_.push_back(state_interfaces_[N+i]);

  // Initialize ref slots to current state
  for (size_t i=0;i<N;i++) {
    q_ref_[i] = pos_states_[i].get().get_value();
    qdot_ref_[i] = 0.0;
  }

  // Build mapping from returned command interfaces to per-joint position/velocity
  pos_cmd_index_.assign(N, -1);
  vel_cmd_index_.assign(N, -1);
  // for (size_t k = 0; k < command_interfaces_.size(); ++k) {
  //   const std::string iface = command_interfaces_[k].get_interface_name(); // e.g. "joint_1/position"
  //   for (size_t i = 0; i < N; ++i) {
  //     const std::string pos_name = joint_names_[i] + "/position";
  //     const std::string vel_name = joint_names_[i] + "/velocity";
  //     if (iface == pos_name) { pos_cmd_index_[i] = static_cast<int>(k); break; }
  //     if (iface == vel_name) { vel_cmd_index_[i] = static_cast<int>(k); break; }
  //   }
  // }
  // size_t pos_ct=0, vel_ct=0;
  // for (size_t i=0;i<N;i++){ pos_ct += pos_cmd_index_[i]>=0; vel_ct += vel_cmd_index_[i]>=0; }
  // RCLCPP_INFO(get_node()->get_logger(), "RR mapped %zu pos and %zu vel refs", pos_ct, vel_ct);
  for (size_t k = 0; k < command_interfaces_.size(); ++k) {
    const std::string name = command_interfaces_[k].get_name(); // e.g. "joint_impedance_controller/joint_1/position"
    for (size_t i = 0; i < N; ++i) {
      const std::string pos_full = inner_ctrl_name_ + "/" + joint_names_[i] + "/position";
      const std::string vel_full = inner_ctrl_name_ + "/" + joint_names_[i] + "/velocity";
      if (name == pos_full) { pos_cmd_index_[i] = static_cast<int>(k); break; }
      if (name == vel_full) { vel_cmd_index_[i] = static_cast<int>(k); break; }
    }
  }
  size_t pos_ct=0, vel_ct=0;
  for (size_t i=0;i<N;i++){ pos_ct += pos_cmd_index_[i]>=0; vel_ct += vel_cmd_index_[i]>=0; }
  RCLCPP_INFO(get_node()->get_logger(), "RR mapped %zu pos and %zu vel refs", pos_ct, vel_ct);

  write_refs_to_slots();
  return CallbackReturn::SUCCESS;
}



controller_interface::CallbackReturn
ResolvedRateController::on_deactivate(const rclcpp_lifecycle::State &) {
  const size_t N = joint_names_.size();

  // Zero downstream velocity refs
  // if (command_interfaces_.size() >= 2 * N) {
  //   for (size_t i = 0; i < N; ++i) {
  //     command_interfaces_[N + i].set_value(0.0);
  //   }
  // }
  if (vel_cmd_index_.size() == N) {
    for (size_t i = 0; i < N; ++i) {
      if (vel_cmd_index_[i] >= 0) { command_interfaces_[vel_cmd_index_[i]].set_value(0.0); }
    }
  }
  return CallbackReturn::SUCCESS;
}


// void ResolvedRateController::twist_cb(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
//   Cmd c; c.vx=msg->twist.linear.x; c.vy=msg->twist.linear.y; c.vz=msg->twist.linear.z;
//   c.wx=msg->twist.angular.x; c.wy=msg->twist.angular.y; c.wz=msg->twist.angular.z; c.valid=true;
//   cmd_rt_.writeFromNonRT(c);
// }
void ResolvedRateController::twist_cb(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  Cmd c;
  c.vx = msg->twist.linear.x;  c.vy = msg->twist.linear.y;  c.vz = msg->twist.linear.z;
  c.wx = msg->twist.angular.x; c.wy = msg->twist.angular.y; c.wz = msg->twist.angular.z;

  // optional: ignore bad frame
  // if (!expected_frame_.empty() && !msg->header.frame_id.empty()
  //     && msg->header.frame_id != expected_frame_) {
  //   // Don’t accept; leave c.valid=false so watchdog will idle
  //   cmd_rt_.writeFromNonRT(Cmd{});
  //   return;
  // }

  c.valid = true;
  cmd_rt_.writeFromNonRT(c);
  last_cmd_time_ = get_node()->now();
}






void ResolvedRateController::write_refs_to_slots() {
  const size_t N = joint_names_.size();

  for (size_t i = 0; i < N; ++i) {
    if (pos_cmd_index_.size() == N && pos_cmd_index_[i] >= 0) {
      command_interfaces_[pos_cmd_index_[i]].set_value(q_ref_[i]);
    }
    if (vel_cmd_index_.size() == N && vel_cmd_index_[i] >= 0) {
      command_interfaces_[vel_cmd_index_[i]].set_value(qdot_ref_[i]);
    }
  }

  // if (command_interfaces_.size() == N) {
  //   // assumed: vel-only claim
  //   for (size_t i = 0; i < N; ++i)
  //     command_interfaces_[i].set_value(qdot_ref_[i]);
  // } else if (command_interfaces_.size() == 2*N) {
  //   for (size_t i = 0; i < N; ++i) {
  //     command_interfaces_[i].set_value(q_ref_[i]);
  //     command_interfaces_[N+i].set_value(qdot_ref_[i]);
  //   }
  // }

  // Keep exporting our own reference_interfaces_ if you need them for chaining
  // if (reference_interfaces_.size() >= 2 * N) {
  //   for (size_t i = 0; i < N; ++i) {
  //     reference_interfaces_[i]     = q_ref_[i];
  //     reference_interfaces_[N + i] = qdot_ref_[i];
  //   }
  // }
}




controller_interface::return_type
ResolvedRateController::update_reference_from_subscribers()
{
  // This is called ONLY when NOT in chained mode.
  // Pull the latest Twist command from the realtime buffer and cache it.
  if (auto cptr = cmd_rt_.readFromRT(); cptr) {
    cmd_cached_ = *cptr;  // copy into cache
  } else {
    cmd_cached_ = Cmd{};  // invalid by default
  }
  return controller_interface::return_type::OK;
}

controller_interface::return_type
ResolvedRateController::update_and_write_commands(
    const rclcpp::Time &, const rclcpp::Duration &period)
{
  const size_t N = joint_names_.size();
  const double dt = period.seconds();
  const auto now = get_node()->now();
  bool timed_out = (!last_cmd_time_.nanoseconds()) || ((now - last_cmd_time_).seconds() > cmd_timeout_);


  if (auto cptr = cmd_rt_.readFromRT(); cptr) {
    cmd_cached_ = *cptr;
  }

  if (timed_out) { cmd_cached_ = Cmd{}; }   // valid=false


  // 1) Read robot state (always safe to do here)
  for (size_t i = 0; i < N; ++i) {
    q_kdl_(i)  = pos_states_[i].get().get_value();
    dq_kdl_(i) = vel_states_[i].get().get_value();
  }
  bool posture_active = std::any_of(null_kp_.begin(), null_kp_.end(),
                                    [](double k){ return k > 0.0; });
  // 2) NO TASK COMMAND → posture bias or hold
  if (!cmd_cached_.valid) {
    
    if (posture_active) {
      double err_norm = 0.0;
      double qdot_norm = 0.0;

      const double dt_used = std::min(dt, dt_ceiling_);
      const double alpha = dt_used / (0.12 + dt_used); // ~80 ms LPF
      const double beta_rb = dt_used / (tau_rebase_ + dt_used);     // rebase leak

      // const double dt_used = std::min(dt, dt_ceiling_);
      size_t sat_step = 0;
      for (size_t i = 0; i < N; ++i) {
        // 0) rebase leak so refs don't "charge up"
        q_ref_[i] = (1.0 - beta_rb) * q_ref_[i] + beta_rb * q_kdl_(i);

        // 1) posture PD in velocity space
        const double e  = q_home_[i] - q_kdl_(i);   // rad
        const double ed = -dq_kdl_(i);              // rad/s (want dq -> 0 at home)
        double qdot_cmd = null_kp_[i] * e + null_kd_[i] * ed;

        // 2) deadband + minimum-speed nudge to beat stiction
        if (std::abs(e) > err_db_ && std::abs(qdot_cmd) < v_min_) {
          qdot_cmd = std::copysign(v_min_, qdot_cmd);
        }

        // 3) clamp commanded joint speed
        qdot_cmd = std::clamp(qdot_cmd, -qdot_limit_, qdot_limit_);

        // 4) smooth the velocity reference
        qdot_ref_[i] += alpha * (qdot_cmd - qdot_ref_[i]);

        // 5) integrate with step clamp
        const double dq_raw = qdot_ref_[i] * dt_used;
        const double dq_clamped = std::clamp(dq_raw, -step_limit_, step_limit_);
        sat_step += (dq_clamped != dq_raw);
        q_ref_[i] += dq_clamped;
        err_norm  += e * e;
        qdot_norm += qdot_ref_[i] * qdot_ref_[i]; 
        // RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
        //   "dq_raw=%.4f  dq_clamped=%.4f", dq_raw, dq_clamped);

        // RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
        //   "dq_raw=%.4f  dq_clamped=%.4f", dq_raw, dq_clamped);
      }
      // RCLCPP_INFO_THROTTLE(
      //   get_node()->get_logger(),                // logger
      //   *get_node()->get_clock(),                // clock
      //   1000,                                   // ms between prints
      //   "Nullspace active: ||err||=%.4f  ||qdot_ref||=%.4f  null_kp=%.3f",
      // //   err_norm, qdot_norm, null_kp_);
      // RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
      //   "NS: dt=%.4f dt_used=%.4f ||qdot_ref||=%.4f step_limit=%.4f sat_step=%zu/%zu",
      //   dt, dt_used, Eigen::Map<Eigen::VectorXd>(qdot_ref_.data(), N).norm(),
      //   step_limit_, sat_step, N);


    } else {
      for (size_t i = 0; i < N; ++i) {
        qdot_ref_[i] = 0.0;                            // hold
        q_ref_[i]    = q_kdl_(i);                      // rebase to measured pose
        // RCLCPP_INFO_THROTTLE(
        //   get_node()->get_logger(), *get_node()->get_clock(), 1000,
        //   "Holding position (q[0]=%.3f)", q_kdl_(0));
      }
    }
    write_refs_to_slots();
    return controller_interface::return_type::OK;
  }

  // 3) Jacobian at current q
  KDL::Jacobian J(N);
  if (jac_solver_->JntToJac(q_kdl_, J) < 0) {
    // Solver failed; keep last refs
    return controller_interface::return_type::OK;
  }

  // 4) Map KDL::Jacobian -> Eigen
  Eigen::Matrix<double, 6, Eigen::Dynamic> Je(6, N);
  for (unsigned r = 0; r < 6; ++r)
    for (unsigned c = 0; c < N; ++c)
      Je(r, c) = J(r, c);

  Eigen::Matrix<double, 6, 1> v;
  v << cmd_cached_.vx, cmd_cached_.vy, cmd_cached_.vz,
       cmd_cached_.wx, cmd_cached_.wy, cmd_cached_.wz;
    
  // RCLCPP_INFO_THROTTLE(get_node()->get_logger(),            
  //     *get_node()->get_clock(),           
  //     1000,
  //   "age=%.3fs timed_out=%d valid=%d vnorm=%.4f",
  //   (now - last_cmd_time_).seconds(), timed_out, int(cmd_cached_.valid), v.norm());


  // 5) Damped resolved-rate: qdot = J^T (J J^T + λ^2 I)^-1 v
  const double lam2 = lambda_ * lambda_;
  Eigen::Matrix<double, 6, 6> JJt = Je * Je.transpose();
  Eigen::Matrix<double, 6, 6> A   = JJt + lam2 * Eigen::Matrix<double, 6, 6>::Identity();
  auto solver = A.ldlt();
  Eigen::VectorXd qdot = Je.transpose() * solver.solve(v);  // robust solve

  double task_mag = qdot.norm();

  task_mag = std::clamp(task_mag, 0.0, 1.0);

  double null_scale_adapt = 20.0 * std::pow(1.0 - task_mag, 3.0);

  RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
    "||qdot||=%.3f  qdot[0]=%.3f  dt=%.3f  step_limit=%.3f", qdot.norm(), qdot(0), dt, step_limit_);




  // 6) Nullspace posture bias: qdot += N * (K * e)
  // if (posture_active) {
  //   Eigen::MatrixXd I   = Eigen::MatrixXd::Identity(N, N);
  //   Eigen::MatrixXd Nproj = I - Je.transpose() * solver.solve(Je);  // null projector
  //   Eigen::VectorXd e(N);
  //   for (size_t i = 0; i < N; ++i) e(i) = q_home_[i] - q_kdl_(i);
  //   qdot += null_kp_ * (Nproj * e);
  // }

  // 6) Nullspace posture bias: qdot += Nproj * u_posture
  if (posture_active) {
    Eigen::MatrixXd I     = Eigen::MatrixXd::Identity(N, N);
    Eigen::MatrixXd Nproj = I - Je.transpose() * solver.solve(Je);  // null projector

    Eigen::VectorXd e(N), ed(N), u_posture(N);
    for (size_t i = 0; i < N; ++i) {
      e(i)  = q_home_[i] - q_kdl_(i);   // position error to home
      ed(i) = -dq_kdl_(i);              // want joint speed → 0 at home

      // per-joint PD in velocity space
      double ui = null_kp_[i] * e(i) + null_kd_[i] * ed(i);

      // (optional) keep null bias gentle vs. task
      // ui *= null_scale_;  // e.g., null_scale_ = 0.5
      ui *= null_scale_adapt;


      // clamp per-joint null velocity contribution
      ui = std::clamp(ui, -qdot_limit_, qdot_limit_);
      u_posture(i) = ui;
    }

    // Eigen::IOFormat fmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n", "[", "]");

    // RCLCPP_INFO_STREAM_THROTTLE(
    //     get_node()->get_logger(),
    //     *get_node()->get_clock(),
    //     100,  // ms
    //     "Nproj =\n" << Nproj.format(fmt));


    // add projected nullspace motion
    // qdot += Nproj * u_posture;
    // qdot += u_posture;
  }


  // 7) Clamp & integrate → write to exported reference slots
  for (size_t i = 0; i < N; ++i) {
    qdot_ref_[i] = std::clamp(qdot(i), -qdot_limit_, qdot_limit_);
    const double dt_used = std::min(dt, dt_ceiling_); // e.g., 0.02s
    const double dq = std::clamp(qdot_ref_[i] * dt_used, -step_limit_, step_limit_);  // add a new param step_limit_
    q_ref_[i] += dq;
  }

  // RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
  //   "State q[0]=%.3f dq[0]=%.3f", q_kdl_(0), dq_kdl_(0));

  // RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
  //   "||v||=%.3f  rank<=6, lambda=%.3f", v.norm(), lambda_);



    // RCLCPP_INFO_THROTTLE(
    //     get_node()->get_logger(),
    //     *get_node()->get_clock(),
    //     100,  // milliseconds
    //     "null_scale_adapt = %.3f", null_scale_adapt);

    write_refs_to_slots();  // fills pos_ref_slots_[i].value / vel_ref_slots_[i].value
    return controller_interface::return_type::OK;
  }


} // namespace ombot_controller

PLUGINLIB_EXPORT_CLASS(ombot_controller::ResolvedRateController, controller_interface::ChainableControllerInterface)
