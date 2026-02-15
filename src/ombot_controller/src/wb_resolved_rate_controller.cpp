#include "wb_resolved_rate_controller/wb_resolved_rate_controller.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <sstream> 

using controller_interface::InterfaceConfiguration;
using controller_interface::interface_configuration_type;

namespace ombot_controller {

WholeBodyResolvedRateController::WholeBodyResolvedRateController() = default;

controller_interface::CallbackReturn WholeBodyResolvedRateController::on_init() {
  try {
    auto_declare<std::vector<std::string>>("joints", {});
    auto_declare<std::string>("base_link", "");
    auto_declare<std::string>("tip_link", "");
    auto_declare<std::string>("robot_description", "");
    auto_declare<double>("lambda", 0.02);
    auto_declare<double>("qdot_limit", 1.0);
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
    auto_declare<double>("dt_ceiling", dt_ceiling_);
    auto_declare<double>("step_limit", step_limit_);
    auto_declare<double>("err_db", err_db_);
    auto_declare<double>("base_vx_limit", base_vx_limit_);
    auto_declare<double>("base_vy_limit", base_vy_limit_);
    auto_declare<double>("base_wz_limit", base_wz_limit_);
    // auto_declare<double>("base_weight", base_weight_);
    base_weight_ = auto_declare<double>("base_weight", base_weight_);

    // auto_declare<double>("arm_weight", arm_weight_);
    arm_weight_ = auto_declare<double>("arm_weight", arm_weight_);

    auto_declare<double>("base_cmd_scale", base_cmd_scale_);

  } catch (...) { return CallbackReturn::ERROR; }
  return CallbackReturn::SUCCESS;
}



controller_interface::CallbackReturn
WholeBodyResolvedRateController::on_configure(const rclcpp_lifecycle::State &) {
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
  qdot_limit_ = get_node()->get_parameter("qdot_limit").as_double();
  // null_kp_   = get_node()->get_parameter("null_kp").as_double();
  // null_kp_ = get_node()->get_parameter("null_kp").as_double_array();
  // null_kd_ = get_node()->get_parameter("null_kd").as_double_array();
  q_home_    = get_node()->get_parameter("q_home").as_double_array();
  qdot_post_max_ = get_node()->get_parameter("qdot_post_max").as_double_array();
  inner_ctrl_name_ = get_node()->get_parameter("inner_controller").as_string();
  cmd_timeout_   = get_node()->get_parameter("cmd_timeout").as_double();
  v_min_        = get_node()->get_parameter("v_min").as_double();
  tau_rebase_ = get_node()->get_parameter("tau_rebase").as_double();
  manip_pub_ = get_node()->create_publisher<std_msgs::msg::Float64>(
  "manipulability", 10);


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
  fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);

  base_cmd_topic_ = "/mecanum_controller/reference";  // or get_node()->declare/get_parameter(...)
  base_cmd_pub_ = get_node()->create_publisher<geometry_msgs::msg::TwistStamped>(
      base_cmd_topic_, rclcpp::SystemDefaultsQoS());


  // subscribe desired EE twist
  // sub_twist_ = get_node()->create_subscription<geometry_msgs::msg::TwistStamped>(
  //     "~/wb_cmd", rclcpp::SystemDefaultsQoS(),
  //     std::bind(&WholeBodyResolvedRateController::twist_cb, this, std::placeholders::_1));

  wb_cmd_sub_ = get_node()->create_subscription<ombot_msgs::msg::WholeBodyCmd>(
    "/wb_cmd", rclcpp::SystemDefaultsQoS(),
    std::bind(&WholeBodyResolvedRateController::wbCmdCb, this, std::placeholders::_1)
  );


  // pre-size integrators
  q_ref_.assign(joint_names_.size(), 0.0);
  qdot_ref_.assign(joint_names_.size(), 0.0);

  return CallbackReturn::SUCCESS;
}




InterfaceConfiguration WholeBodyResolvedRateController::state_interface_configuration() const {
  // We read joint positions & velocities from hardware
  InterfaceConfiguration conf;
  conf.type = interface_configuration_type::INDIVIDUAL;
  for (auto &j : joint_names_) { conf.names.push_back(j + "/" + hardware_interface::HW_IF_POSITION); }
  for (auto &j : joint_names_) { conf.names.push_back(j + "/" + hardware_interface::HW_IF_VELOCITY); }
  return conf;
}

bool WholeBodyResolvedRateController::on_set_chained_mode(bool /* chained */) {
  // chained_mode_ = chained;          // optional member
  return true;
}


controller_interface::InterfaceConfiguration
WholeBodyResolvedRateController::command_interface_configuration() const
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
WholeBodyResolvedRateController::on_export_reference_interfaces()
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
WholeBodyResolvedRateController::on_activate(const rclcpp_lifecycle::State &) {
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
WholeBodyResolvedRateController::on_deactivate(const rclcpp_lifecycle::State &) {
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


void WholeBodyResolvedRateController::wbCmdCb(const ombot_msgs::msg::WholeBodyCmd::SharedPtr msg)
{
  Cmd c;
  c.valid = msg->valid;

  // EE twist (base_link)
  c.vx = msg->ee.linear.x;
  c.vy = msg->ee.linear.y;
  c.vz = msg->ee.linear.z;

  c.wx = msg->ee.angular.x;
  c.wy = msg->ee.angular.y;
  c.wz = msg->ee.angular.z;

  // Base desired twist (base_link): [vx, vy, wz]
  c.bvx = msg->bvx;
  c.bvy = msg->bvy;
  c.bwz = msg->bwz;
  
  last_cmd_time_ = msg->header.stamp;

  // Optional: reject wrong frame
  // if (msg->header.frame_id != "base_link") { c.valid = false; }
  cmd_rt_.writeFromNonRT(c);
  // {
  //   std::lock_guard<std::mutex> lock(cmd_mtx_);
  //   cmd_ = c;
  // }
}


void WholeBodyResolvedRateController::write_refs_to_slots() {
  const size_t N = joint_names_.size();

  for (size_t i = 0; i < N; ++i) {
    if (pos_cmd_index_.size() == N && pos_cmd_index_[i] >= 0) {
      command_interfaces_[pos_cmd_index_[i]].set_value(q_ref_[i]);
    }
    if (vel_cmd_index_.size() == N && vel_cmd_index_[i] >= 0) {
      command_interfaces_[vel_cmd_index_[i]].set_value(qdot_ref_[i]);
    }
  }
}




controller_interface::return_type
WholeBodyResolvedRateController::update_reference_from_subscribers()
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
WholeBodyResolvedRateController::update_and_write_commands(
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

    } else {
      for (size_t i = 0; i < N; ++i) {
        qdot_ref_[i] = 0.0;                            // hold
        q_ref_[i]    = q_kdl_(i);                      // rebase to measured pose
      }
    }

    geometry_msgs::msg::TwistStamped base_cmd;
    base_cmd.header.stamp = now;
    base_cmd.header.frame_id = base_link_;
    // base_cmd_pub_->publish(base_cmd);  // all zeros by default

    write_refs_to_slots();
    return controller_interface::return_type::OK;
  }


  if (fk_solver_->JntToCart(q_kdl_, tip_frame_) < 0) {
    geometry_msgs::msg::TwistStamped base_cmd;
    base_cmd.header.stamp = now;
    base_cmd.header.frame_id = base_link_;
    // base_cmd_pub_->publish(base_cmd);
    // FK failed; just bail out gracefully
    write_refs_to_slots();
    return controller_interface::return_type::OK;
  }

  Eigen::Vector3d r_be;
  r_be << tip_frame_.p.x(), tip_frame_.p.y(), tip_frame_.p.z();  // z is often small but fine

  // 3) Jacobian at current q
  KDL::Jacobian J(N);
  if (jac_solver_->JntToJac(q_kdl_, J) < 0) {
    // Solver failed; keep last refs
    geometry_msgs::msg::TwistStamped base_cmd;
    base_cmd.header.stamp = now;
    base_cmd.header.frame_id = base_link_;
    // base_cmd_pub_->publish(base_cmd);
    return controller_interface::return_type::OK;
  }

  // 4) Map KDL::Jacobian -> Eigen
  Eigen::Matrix<double, 6, Eigen::Dynamic> Je(6, N);
  for (unsigned r = 0; r < 6; ++r)
    for (unsigned c = 0; c < N; ++c)
      Je(r, c) = J(r, c);

  // Base Jacobian (6x3) for [v_x, v_y, ω_z] in base_link frame
  Eigen::Matrix<double, 6, 3> Jb;
  Jb.setZero();
  Jb(0,0) = 1.0;            // v_x → xdot
  Jb(1,1) = 1.0;            // v_y → ydot
  Jb(0,2) = -r_be.y();      // ω_z × r
  Jb(1,2) =  r_be.x();
  Jb(5,2) = 1.0;            // yaw rate at EE


  // After filling Jb normally:
  double k_base = base_cmd_scale_;  // same factor as you multiply the command with
  // Jb *= k_base;

  const int M = static_cast<int>(3 + N);  // 3 base + N joints


  // // Whole-body J: [ J_base | J_arm ]  (6 x (3+N))
  Eigen::Matrix<double, 6, Eigen::Dynamic> Jwhole(6, 3 + N);
  Jwhole.block<6,3>(0,0)       = Jb;
  Jwhole.block(0,3,6,N)        = Je; 

  Eigen::Matrix<double,6,1> v;
  v << cmd_cached_.vx, cmd_cached_.vy, cmd_cached_.vz,
      cmd_cached_.wx, cmd_cached_.wy, cmd_cached_.wz;

  // optional: scale linear vs angular
  v.head<3>() *= v_lin_scale_;
  v.tail<3>() *= v_ang_scale_;

  const double lam2 = lambda_ * lambda_;

  // weights on DOFs (bigger = more expensive motion)
  Eigen::VectorXd w(M);
  w.segment<3>(0).setConstant(base_weight_);                 // base vx, vy, wz
  w.segment(3, static_cast<int>(N)).setConstant(arm_weight_);// joints

  Eigen::VectorXd inv_w = w.cwiseInverse();
  // std::stringstream ss;
  // ss << w.transpose();  // or << inv_w.transpose().format(fmt);

  // RCLCPP_INFO(get_node()->get_logger(), "w = %s", ss.str().c_str());
    // Column-scale: J_scaled = Jwhole * W^{-1}
    Eigen::MatrixXd J_scaled = Jwhole;
    for (int j = 0; j < M; ++j) J_scaled.col(j) *= inv_w(j);

  // A = J_scaled J_scaled^T + λ^2 I  (6x6)
  Eigen::Matrix<double,6,6> A =
      (J_scaled * J_scaled.transpose())
    + lam2 * Eigen::Matrix<double,6,6>::Identity();
  Eigen::LDLT<Eigen::Matrix<double,6,6>> ldlt(A);

  Eigen::VectorXd u_prime = J_scaled.transpose() * ldlt.solve(v);
  Eigen::VectorXd u_task = inv_w.asDiagonal() * u_prime;


  Eigen::Matrix<double,6,6> Ainv = ldlt.solve(Eigen::Matrix<double,6,6>::Identity());

  Eigen::MatrixXd J_pinv = inv_w.asDiagonal() * (J_scaled.transpose() * Ainv);  // (Mx6)

  Eigen::MatrixXd Nproj = Eigen::MatrixXd::Identity(M, M) - J_pinv * Jwhole;

  Eigen::VectorXd u_post = Eigen::VectorXd::Zero(M);

  // simple posture hold toward q_home_ (joint space)
  // for (size_t i = 0; i < N; ++i) {
  //   const double e = q_home_[i] - q_kdl_(i);
  //   u_post(3 + i) = null_kp_[i] * e - null_kd_[i] * dq_kdl_(i);
  // }
  double task_norm = u_task.norm();

  Eigen::VectorXd e_post(N);

  for (size_t i = 0; i < N; ++i) {
    e_post(i) = q_home_[i] - q_kdl_(i);
    u_post(3 + i) = null_kp_[i] * e_post(i)
                  - null_kd_[i] * dq_kdl_(i);
  }

  double e_post_norm = e_post.norm();

  RCLCPP_INFO_THROTTLE(
    get_node()->get_logger(),
    *get_node()->get_clock(),
    100,
    "posture_error_norm = %.4f rad, task_command_norm = %.4f",
    e_post_norm,
    task_norm
  );

  double alpha = 1.0;

  const double task_thresh = 0.2;   // tune this
  const double task_exit  = 0.4;   // hysteresis

  // if (task_norm < task_enter)
  //     alpha = 1.0;
  // else if (task_norm > task_exit)
  //     alpha = 0.0;
  // else
  //     alpha = (task_exit - task_norm) / (task_exit - task_enter);
  // alpha = std::clamp(
  //     (task_thresh - task_norm) / task_thresh,
  //     0.0,
  //     1.0
  // );

  // // Combine
  // for (size_t i = 0; i < N; ++i) {
  //   u_post(3+i) = std::clamp(u_post(3+i), -qdot_post_max_[i], qdot_post_max_[i]);
  // }
  
  Eigen::VectorXd u_total = u_task + alpha * (Nproj * u_post);


  // split
  Eigen::Vector3d v_base = u_total.head<3>();
  Eigen::VectorXd qdot   = u_total.tail(N);

  Eigen::MatrixXd leak = Jwhole * (Nproj * u_post);
  // RCLCPP_INFO(get_node()->get_logger(), "leak_norm=%g", leak.norm());



  double margin = 5.0 * M_PI / 180.0;  // 5 deg safety margin

  for (size_t i = 0; i < N; ++i) {
    if (q_kdl_(i) <= q_min_[i] + margin && qdot(i) < 0.0) {
      qdot(i) = 0.0;
    }
    if (q_kdl_(i) >= q_max_[i] - margin && qdot(i) > 0.0) {
      qdot(i) = 0.0;
    }
  }

  // double task_mag = qdot.norm();

  // task_mag = std::clamp(task_mag, 0.0, 1.0);
  size_t sat_step = 0;

  for (size_t i = 0; i < N; ++i) {
    const double dt_used = std::min(dt, dt_ceiling_);

    double qdot_cmd = std::clamp(qdot(i), -qdot_limit_, qdot_limit_);

    // 2) angle limit as a velocity bound: q_min <= q + dt*qdot <= q_max
    const double q      = q_kdl_(i);  // use measured joint angle, not q_ref_
    const double qd_min = (q_min_[i] - q) / dt_used;
    const double qd_max = (q_max_[i] - q) / dt_used;

    // combine with your velocity limit
    const double lo = std::max(-qdot_limit_, qd_min);
    const double hi = std::min( qdot_limit_, qd_max);

    // if lo>hi, you're already out of bounds or dt_used weird; force safest behavior
    if (lo <= hi) qdot_cmd = std::clamp(qdot_cmd, lo, hi);
    else          qdot_cmd = 0.0;

    qdot_ref_[i] = qdot_cmd;

    // 3) integrate with your step clamp
    const double dq_raw     = qdot_ref_[i] * dt_used;
    const double dq_clamped = std::clamp(dq_raw, -step_limit_, step_limit_);
    sat_step += (dq_clamped != dq_raw);
    q_ref_[i] += dq_clamped;

    // 4) (optional) final safety: keep q_ref_ inside limits too
    q_ref_[i] = std::clamp(q_ref_[i], q_min_[i], q_max_[i]);
  }


    // RCLCPP_INFO_THROTTLE(
    //   get_node()->get_logger(), *get_node()->get_clock(), 1000,
    //   "WB: sat_step = %zu / %zu", sat_step, N);

    
    // RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
    //   "State q[0]=%.3f dq[0]=%.3f", q_kdl_(0), dq_kdl_(0));

    // RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
    //   "||v||=%.3f  rank<=6, lambda=%.3f", v.norm(), lambda_);

    // RCLCPP_INFO_THROTTLE(
    //     get_node()->get_logger(),
    //     *get_node()->get_clock(),
    //     100,  // milliseconds
    //     "null_scale_adapt = %.3f", null_scale_adapt);

    // Clamp base velocities if you want
    v_base[0] = std::clamp(v_base[0], -base_vx_limit_, base_vx_limit_);
    v_base[1] = std::clamp(v_base[1], -base_vy_limit_, base_vy_limit_);
    v_base[2] = std::clamp(v_base[2], -base_wz_limit_, base_wz_limit_);


    // Publish base twist command
    geometry_msgs::msg::TwistStamped base_cmd;
    base_cmd.header.stamp = now;
    base_cmd.header.frame_id = base_link_;  // e.g. "base_link"
    base_cmd.twist.linear.x  = -k_base * v_base[0];
    base_cmd.twist.linear.y  = -k_base * v_base[1];
    base_cmd.twist.linear.z  = 0.0;
    base_cmd.twist.angular.x = 0.0;
    base_cmd.twist.angular.y = 0.0;
    base_cmd.twist.angular.z = k_base * v_base[2];
    base_cmd_pub_->publish(base_cmd);

    // RCLCPP_INFO_THROTTLE(
    //     get_node()->get_logger(), *get_node()->get_clock(), 1000,
    //     "WB: v_base=(%.3f %.3f %.3f)  qdot=(%.3f %.3f %.3f %.3f %.3f %.3f)  r_be=(%.3f %.3f)",
    //     v_base.x(), v_base.y(), v_base.z(),
    //     qdot[0], qdot[1], qdot[2], qdot[3], qdot[4], qdot[5],
    //     r_be.x(), r_be.y()
    // );

    // v_ee contribution from arm joints
    // Eigen::Matrix<double, 6, 1> v_ee_arm = Je * qdot;

    // J: 6 x n Jacobian (Eigen::MatrixXd)
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J.data, Eigen::ComputeThinU | Eigen::ComputeThinV);

    // manipulability = product of singular values
    double manipulability = 1.0;
    for (int i = 0; i < svd.singularValues().size(); ++i) {
      manipulability *= svd.singularValues()(i);
    }
    // std_msgs::msg::Float64 msg;
    // msg.data = manipulability;
    // manip_pub_->publish(msg);
    // RCLCPP_INFO_THROTTLE(
    //   get_node()->get_logger(),
    //   *get_node()->get_clock(),
    //   100,   // ms
    //   "Manipulability = %.4e", manipulability
    // );

    // v_ee contribution from base
    // Eigen::Matrix<double, 6, 1> v_ee_base = Jb * v_base;

    // // total
    // Eigen::Matrix<double, 6, 1> v_ee_tot = v_ee_arm + v_ee_base;

    // RCLCPP_INFO_THROTTLE(
    //     get_node()->get_logger(), *get_node()->get_clock(), 1000,
    //     "EE: v_arm=(%.3f %.3f %.3f | %.3f %.3f %.3f) ",
    //     v_ee_arm(0), v_ee_arm(1), v_ee_arm(2), v_ee_arm(3), v_ee_arm(4), v_ee_arm(5)
    // );

    write_refs_to_slots();  
    // static double dt_min = 1e9;
    // static double dt_max = 0.0;
    // static double dt_sum = 0.0;
    // static size_t dt_count = 0;

    // // double dt = period.seconds();
    // dt_min = std::min(dt_min, dt);
    // dt_max = std::max(dt_max, dt);
    // dt_sum += dt;
    // dt_count++;

    // RCLCPP_INFO_THROTTLE(
    //     get_node()->get_logger(), *get_node()->get_clock(), 1000,
    //     "WB: dt=%.6f s  dt_min=%.6f  dt_max=%.6f  dt_avg=%.6f (N=%zu)",
    //     dt, dt_min, dt_max, dt_sum / dt_count, dt_count);

    return controller_interface::return_type::OK;
  }


} // namespace ombot_controller

PLUGINLIB_EXPORT_CLASS(ombot_controller::WholeBodyResolvedRateController, controller_interface::ChainableControllerInterface)




