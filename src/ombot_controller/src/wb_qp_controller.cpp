
#include "wb_qp_controller/wb_qp_controller.hpp"
#include <algorithm>
#include <cmath>
#include <pluginlib/class_list_macros.hpp>
#include "controller_interface/controller_interface.hpp"

using controller_interface::InterfaceConfiguration;
using controller_interface::interface_configuration_type;

namespace ombot_controller {

WholeBodyQPController::WholeBodyQPController() = default;

controller_interface::CallbackReturn WholeBodyQPController::on_init() 
{
  try {
    auto_declare<std::vector<std::string>>("joints", {});
    auto_declare<std::string>("base_link", "");
    auto_declare<std::string>("tip_link", "");
    auto_declare<std::string>("robot_description", "");

    auto_declare<std::string>("inner_controller", "");
    auto_declare<double>("cmd_timeout", 0.25);
    auto_declare<std::string>("obstacles_topic", "/obstacles");
    auto_declare<double>("obstacles_timeout", 0.25);

    // tracking regularization
    auto_declare<double>("lambda", 0.02);

    auto_declare<int>("cbf_max_obstacles", 20);
    auto_declare<double>("cbf_alpha", 3.0);
    auto_declare<double>("ee_radius", 0.02);
    auto_declare<double>("cbf_margin", 0.03);
    auto_declare<double>("cbf_activate_h", 0.30);

    // Position gains
    auto_declare<double>("ee.kp_pos", 0.0);
    auto_declare<double>("ee.kd_pos", 0.0);

    // Rotation gains
    auto_declare<double>("ee.kp_rot", 0.0);
    auto_declare<double>("ee.kd_rot", 0.0);

    ee_kp_pos_ = get_node()->get_parameter("ee.kp_pos").as_double();
    ee_kd_pos_ = get_node()->get_parameter("ee.kd_pos").as_double();
    ee_kp_rot_ = get_node()->get_parameter("ee.kp_rot").as_double();
    ee_kd_rot_ = get_node()->get_parameter("ee.kd_rot").as_double();

    RCLCPP_INFO(
      get_node()->get_logger(),
      "EE gains: kp_pos=%.3f kd_pos=%.3f kp_rot=%.3f kd_rot=%.3f",
      ee_kp_pos_, ee_kd_pos_, ee_kp_rot_, ee_kd_rot_);


    // limits
    auto_declare<double>("qdot_limit", 1.0);
    auto_declare<double>("dt_ceiling", 0.03);
    auto_declare<double>("step_limit", 0.02);

    auto_declare<double>("base_vx_limit", 0.3);
    auto_declare<double>("base_vy_limit", 0.3);
    auto_declare<double>("base_wz_limit", 0.5);

    auto_declare<double>("base_cmd_scale", 1.0);

    // weights (DOF effort weights)
    auto_declare<double>("base_weight", 1.0);
    auto_declare<double>("arm_weight", 1.0);

    // task weights (soft objectives)
    auto_declare<double>("base_task_weight", 0.2); // usually low
    auto_declare<double>("ee_task_weight",  1.0);  // usually higher

    // posture objective
    auto_declare<std::vector<double>>("null_kp", {});
    auto_declare<std::vector<double>>("null_kd", {});
    auto_declare<std::vector<double>>("q_home", {});
    auto_declare<double>("posture_weight", 0.1);

    // row-axis weights (optional knobs)
    auto_declare<double>("w_pos", 1.0);
    auto_declare<double>("w_rot", 1.0);
    auto_declare<double>("w_z",  1.0); // extra scale on z if you want

    // safety: joint limits (optional params; if missing we just won't apply position bounds)
    auto_declare<std::vector<double>>("q_min", {});
    auto_declare<std::vector<double>>("q_max", {});
  } catch (...) {
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn 
WholeBodyQPController::on_configure(const rclcpp_lifecycle::State&) 
{
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  if (joint_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'joints' is required.");
    return controller_interface::CallbackReturn::ERROR;
  }
  const size_t N = joint_names_.size();

  Kmax_ = get_node()->get_parameter("cbf_max_obstacles").as_int();
  CBF_alpha_ = get_node()->get_parameter("cbf_alpha").as_double();
  ee_radius_ = get_node()->get_parameter("ee_radius").as_double();    
  cbf_margin_ = get_node()->get_parameter("cbf_margin").as_double();
  cbf_activate_h_ = get_node()->get_parameter("cbf_activate_h").as_double();  

  obstacles_topic_   = get_node()->get_parameter("obstacles_topic").as_string();
  obstacles_timeout_ = get_node()->get_parameter("obstacles_timeout").as_double();

  base_link_       = get_node()->get_parameter("base_link").as_string();
  tip_link_        = get_node()->get_parameter("tip_link").as_string();
  inner_ctrl_name_ = get_node()->get_parameter("inner_controller").as_string();

  cmd_timeout_  = get_node()->get_parameter("cmd_timeout").as_double();
  lambda_       = get_node()->get_parameter("lambda").as_double();

  qdot_limit_   = get_node()->get_parameter("qdot_limit").as_double();
  dt_ceiling_   = get_node()->get_parameter("dt_ceiling").as_double();
  step_limit_   = get_node()->get_parameter("step_limit").as_double();

  base_vx_limit_ = get_node()->get_parameter("base_vx_limit").as_double();
  base_vy_limit_ = get_node()->get_parameter("base_vy_limit").as_double();
  base_wz_limit_ = get_node()->get_parameter("base_wz_limit").as_double();

  base_cmd_scale_ = get_node()->get_parameter("base_cmd_scale").as_double();

  base_weight_ = get_node()->get_parameter("base_weight").as_double();
  arm_weight_  = get_node()->get_parameter("arm_weight").as_double();

  posture_weight_   = get_node()->get_parameter("posture_weight").as_double();

  w_pos_ = get_node()->get_parameter("w_pos").as_double();
  w_rot_ = get_node()->get_parameter("w_rot").as_double();
  w_z_   = get_node()->get_parameter("w_z").as_double();
  obstacles_sub_ = get_node()->create_subscription<ombot_msgs::msg::ObstacleArray>(
      obstacles_topic_, rclcpp::SystemDefaultsQoS(),
      std::bind(&WholeBodyQPController::obstaclesCb, this, std::placeholders::_1)
      );


  // posture vectors
  null_kp_.assign(N, 0.0);
  null_kd_.assign(N, 0.0);
  q_home_ = get_node()->get_parameter("q_home").as_double_array();
  if (q_home_.size() != N) q_home_.assign(N, 0.0);

  // null_kp vector/scalar flexibility
  if (get_node()->has_parameter("null_kp")) {
    auto p = get_node()->get_parameter("null_kp");
    if (p.get_type() == rclcpp::PARAMETER_DOUBLE_ARRAY) {
      auto v = p.as_double_array();
      if (v.empty()) {
        // leave zeros
      } else if (v.size() == N) {
        null_kp_ = v;
      } else {
        for (size_t i=0;i<N;++i) null_kp_[i] = v[i % v.size()];
      }
    } else if (p.get_type() == rclcpp::PARAMETER_DOUBLE) {
      std::fill(null_kp_.begin(), null_kp_.end(), p.as_double());
    }
  }
  if (get_node()->has_parameter("null_kd")) {
    auto p = get_node()->get_parameter("null_kd");
    if (p.get_type() == rclcpp::PARAMETER_DOUBLE_ARRAY) {
      auto v = p.as_double_array();
      if (v.empty()) {
        // leave zeros
      } else if (v.size() == N) {
        null_kd_ = v;
      } else {
        for (size_t i=0;i<N;++i) null_kd_[i] = v[i % v.size()];
      }
    } else if (p.get_type() == rclcpp::PARAMETER_DOUBLE) {
      std::fill(null_kd_.begin(), null_kd_.end(), p.as_double());
    }
  }

  // joint limits (optional)
  q_min_ = get_node()->get_parameter("q_min").as_double_array();
  q_max_ = get_node()->get_parameter("q_max").as_double_array();
  if (q_min_.size() != N) q_min_.assign(N, -1e9);
  if (q_max_.size() != N) q_max_.assign(N, +1e9);

  // Build KDL chain
  std::string urdf_xml = get_node()->get_parameter("robot_description").as_string();
  if (urdf_xml.empty()) {
    rclcpp::Parameter p;
    if (get_node()->get_parameter("robot_description", p)) urdf_xml = p.as_string();
  }
  KDL::Tree tree;
  if (!kdl_parser::treeFromString(urdf_xml, tree)) {
    RCLCPP_ERROR(get_node()->get_logger(), "URDF->KDL failed");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!tree.getChain(base_link_, tip_link_, chain_)) {
    RCLCPP_ERROR(get_node()->get_logger(), "KDL chain extract failed");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (chain_.getNrOfJoints() != joint_names_.size()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Chain DOF mismatch");
    return controller_interface::CallbackReturn::ERROR;
  }

  jac_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(chain_);
  fk_solver_  = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);

  q_kdl_  = KDL::JntArray(chain_.getNrOfJoints());
  dq_kdl_ = KDL::JntArray(chain_.getNrOfJoints());

  // Publisher for base twist
  base_cmd_pub_ = get_node()->create_publisher<geometry_msgs::msg::TwistStamped>(
    "/mecanum_controller/reference", rclcpp::SystemDefaultsQoS());

  // Subscription for whole-body cmd
  wb_cmd_sub_ = get_node()->create_subscription<ombot_msgs::msg::WholeBodyCmd>(
    "/wb_cmd", rclcpp::SystemDefaultsQoS(),
    std::bind(&WholeBodyQPController::wbCmdCb, this, std::placeholders::_1));

  // refs
  q_ref_.assign(N, 0.0);
  qdot_ref_.assign(N, 0.0);



  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_node()->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Initialize cache to identity so RT always has something
  T_wb_rt_.writeFromNonRT(Eigen::Isometry3d::Identity());

  // Timer to refresh TF in non-RT
  tf_timer_ = get_node()->create_wall_timer(
    std::chrono::milliseconds(20),  // 50 Hz
    [this]()
    {
      try {
        // world <- base
        auto tf = tf_buffer_->lookupTransform(
          world_frame_, base_frame_, tf2::TimePointZero);

        Eigen::Quaterniond q(
          tf.transform.rotation.w,
          tf.transform.rotation.x,
          tf.transform.rotation.y,
          tf.transform.rotation.z);

        Eigen::Vector3d t(
          tf.transform.translation.x,
          tf.transform.translation.y,
          tf.transform.translation.z);

        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        T.linear() = q.normalized().toRotationMatrix();
        T.translation() = t;

        T_wb_rt_.writeFromNonRT(T);
      }
      catch (const tf2::TransformException & e) {
        // Non-RT: OK to warn occasionally
        RCLCPP_WARN_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 2000,
          "TF lookupTransform %s<- %s failed: %s",
          world_frame_.c_str(), base_frame_.c_str(), e.what());
      }
    });


  return controller_interface::CallbackReturn::SUCCESS;
}

void WholeBodyQPController::obstaclesCb(
const ombot_msgs::msg::ObstacleArray::SharedPtr msg)
{
  ObstaclesCache c;
  c.valid   = true;
  c.stamp   = msg->header.stamp;
  c.frame_id = msg->header.frame_id;

  c.obs.clear();
  c.obs.reserve(msg->obstacles.size());

  for (const auto& o : msg->obstacles) {
    ObstacleLite ol;
    ol.cx = static_cast<float>(o.center.x);
    ol.cy = static_cast<float>(o.center.y);
    ol.cz = static_cast<float>(o.center.z);
    ol.r  = static_cast<float>(o.radius);

    // if you included velocity in your msg:
    ol.vx = static_cast<float>(o.velocity.x);
    ol.vy = static_cast<float>(o.velocity.y);
    ol.vz = static_cast<float>(o.velocity.z);

    // ignore junk obstacles
    if (!(std::isfinite(ol.cx) && std::isfinite(ol.cy) && std::isfinite(ol.r))) continue;
    if (ol.r <= 0.f) continue;

    c.obs.push_back(ol);
  }

  obstacles_rt_.writeFromNonRT(c);
}

void WholeBodyQPController::init_osqp_structures(int N)
{
  M_ = 3 + N;

  const int n_vars = M_;
  const int n_cons = M_ + Kmax_;

  // Allocate vectors
  q_osqp_.setZero(n_vars);
  l_osqp_.setZero(n_cons);
  u_osqp_.setZero(n_cons);
  x_prev_.setZero(n_vars);

  // ---------- Build A sparsity pattern ----------
  // A = [ I_M
  //       A_cbf (Kmax rows, dense 1xM each) ]
  A_triplets_.clear();
  A_triplets_.reserve(M_ + Kmax_ * M_);

  // Identity block
  for (int i = 0; i < M_; ++i) {
    A_triplets_.emplace_back(i, i, 1.0);
  }

  // CBF rows: each row may have up to M nonzeros (dense)
  // Row index in A: M_ + k
  for (int k = 0; k < Kmax_; ++k) {
    int row = M_ + k;
    for (int j = 0; j < M_; ++j) {
      A_triplets_.emplace_back(row, j, 0.0);  // value will be updated each cycle
    }
  }

  Asp_.resize(n_cons, n_vars);
  Asp_.setFromTriplets(A_triplets_.begin(), A_triplets_.end());
  Asp_.makeCompressed();

  // ---------- Build P sparsity pattern ----------
  // P is symmetric. OSQP expects upper triangular. We will rebuild values each cycle,
  // but keep same nonzero pattern by using a dense upper triangle (safe for small M).
  std::vector<Eigen::Triplet<double>> P_triplets;
  P_triplets.reserve((M_ * (M_ + 1)) / 2);
  for (int i = 0; i < M_; ++i) {
    for (int j = i; j < M_; ++j) {
      P_triplets.emplace_back(i, j, 0.0);
    }
  }
  Psp_.resize(M_, M_);
  Psp_.setFromTriplets(P_triplets.begin(), P_triplets.end());
  Psp_.makeCompressed();

  // ---------- OSQP settings ----------
  osqp_.settings()->setWarmStart(true);
  osqp_.settings()->setVerbosity(false);
  osqp_.settings()->setAlpha(1.6);

  osqp_.data()->setNumberOfVariables(n_vars);
  osqp_.data()->setNumberOfConstraints(n_cons);

  // Set matrices/vectors once; values updated each loop
  osqp_.data()->setHessianMatrix(Psp_);
  osqp_.data()->setGradient(q_osqp_);
  osqp_.data()->setLinearConstraintsMatrix(Asp_);
  osqp_.data()->setLowerBound(l_osqp_);
  osqp_.data()->setUpperBound(u_osqp_);

  osqp_ready_ = osqp_.initSolver();
  if (!osqp_ready_) {
    RCLCPP_ERROR(get_node()->get_logger(), "OSQP initSolver() failed");
  }
}



InterfaceConfiguration WholeBodyQPController::state_interface_configuration() const 
{
  InterfaceConfiguration conf;
  conf.type = interface_configuration_type::INDIVIDUAL;
  for (auto &j : joint_names_) conf.names.push_back(j + "/" + hardware_interface::HW_IF_POSITION);
  for (auto &j : joint_names_) conf.names.push_back(j + "/" + hardware_interface::HW_IF_VELOCITY);
  return conf;
}

InterfaceConfiguration WholeBodyQPController::command_interface_configuration() const 
{
  InterfaceConfiguration conf;
  conf.type = interface_configuration_type::INDIVIDUAL;
  for (const auto& j : joint_names_) {
    conf.names.push_back(inner_ctrl_name_ + "/" + j + "/position");
    conf.names.push_back(inner_ctrl_name_ + "/" + j + "/velocity");
  }
  return conf;
}

std::vector<hardware_interface::CommandInterface> 
WholeBodyQPController::on_export_reference_interfaces() 
{
  if (joint_names_.empty()) {
    joint_names_ = get_node()->get_parameter("joints").as_string_array();
  }
  const size_t N = joint_names_.size();
  reference_interfaces_.assign(2 * N, 0.0);
  RCLCPP_INFO(get_node()->get_logger(),
              "EXPORT: ref storage size=%zu, addr=%p",
              reference_interfaces_.size(),
              (void*)reference_interfaces_.data());



  std::vector<hardware_interface::CommandInterface> refs;
  refs.reserve(2 * N);

  const std::string ctrl = get_node()->get_name();
  for (size_t i = 0; i < N; ++i) {
    refs.emplace_back(ctrl, joint_names_[i] + "/position", &reference_interfaces_[i]);
    refs.emplace_back(ctrl, joint_names_[i] + "/velocity", &reference_interfaces_[N + i]);
  }
  return refs;
}

bool WholeBodyQPController::on_set_chained_mode(bool /*chained*/) { return true; }

controller_interface::CallbackReturn WholeBodyQPController::on_activate(const rclcpp_lifecycle::State&) 
{
  const size_t N = joint_names_.size();

  init_osqp_structures((int)joint_names_.size());


  if (state_interfaces_.size() != 2 * N) {
    RCLCPP_ERROR(get_node()->get_logger(), "state interfaces size mismatch");
    return controller_interface::CallbackReturn::ERROR;
  }

  pos_states_.clear();
  vel_states_.clear();
  for (size_t i = 0; i < N; ++i) pos_states_.push_back(state_interfaces_[i]);
  for (size_t i = 0; i < N; ++i) vel_states_.push_back(state_interfaces_[N + i]);

  // init refs to current state
  for (size_t i = 0; i < N; ++i) {
    q_ref_[i]    = pos_states_[i].get().get_value();
    qdot_ref_[i] = 0.0;
  }

  // map command interfaces (full name matching)
  pos_cmd_index_.assign(N, -1);
  vel_cmd_index_.assign(N, -1);
  for (size_t k = 0; k < command_interfaces_.size(); ++k) {
    const std::string name = command_interfaces_[k].get_name();
    for (size_t i = 0; i < N; ++i) {
      const std::string pos_full = inner_ctrl_name_ + "/" + joint_names_[i] + "/position";
      const std::string vel_full = inner_ctrl_name_ + "/" + joint_names_[i] + "/velocity";
      if (name == pos_full) { pos_cmd_index_[i] = static_cast<int>(k); break; }
      if (name == vel_full) { vel_cmd_index_[i] = static_cast<int>(k); break; }
    }
  }

  write_refs_to_slots();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn WholeBodyQPController::on_deactivate(const rclcpp_lifecycle::State&) 
{
  const size_t N = joint_names_.size();
  
  if (vel_cmd_index_.size() == N) {
    for (size_t i = 0; i < N; ++i) {
      if (vel_cmd_index_[i] >= 0) command_interfaces_[vel_cmd_index_[i]].set_value(0.0);
    }
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type WholeBodyQPController::update_reference_from_subscribers() 
{
  if (auto cptr = cmd_rt_.readFromRT(); cptr) cmd_cached_ = *cptr;
  else cmd_cached_ = Cmd{};
  return controller_interface::return_type::OK;
}

controller_interface::return_type WholeBodyQPController::update_and_write_commands(
    const rclcpp::Time&, const rclcpp::Duration& period) 
{
  const size_t N = joint_names_.size();
  const double dt = period.seconds();
  const double dt_used = std::max(1e-4, std::min(dt, dt_ceiling_));
  const auto now = get_node()->now();
  // auto t0 = std::chrono::steady_clock::now();



  // Pull latest obstacle snapshot into RT cache
  if (auto optr = obstacles_rt_.readFromRT(); optr) {
    obstacles_cached_ = *optr;
  }

  // Validate obstacle cache
  bool obstacles_ok = obstacles_cached_.valid;
  if (obstacles_ok) {
    // Option A: obstacles must already be in base_link
    if (!obstacles_cached_.frame_id.empty() && obstacles_cached_.frame_id != base_link_) {
      obstacles_ok = false;
    }

    // timeout
    if ((now - obstacles_cached_.stamp).seconds() > obstacles_timeout_) {
        obstacles_ok = false;
    }
  }

  // RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
  //   "Obstacles: ok=%d count=%zu frame='%s' age=%.3fs",
  //   obstacles_ok ? 1 : 0,
  //   obstacles_cached_.obs.size(),
  //   obstacles_cached_.frame_id.c_str(),
  //   obstacles_cached_.valid ? (now - obstacles_cached_.stamp).seconds() : 999.0
  //   );


  // --- cmd cache / timeout ---
  bool timed_out = (!last_cmd_time_.nanoseconds()) ||
                    ((now - last_cmd_time_).seconds() > cmd_timeout_);

  if (auto cptr = cmd_rt_.readFromRT(); cptr) cmd_cached_ = *cptr;
  if (timed_out) cmd_cached_ = Cmd{};

  // --- read state ---
  for (size_t i = 0; i < N; ++i) {
    q_kdl_(i)  = pos_states_[i].get().get_value();
    dq_kdl_(i) = vel_states_[i].get().get_value();
  }

  bool posture_active = std::any_of(null_kp_.begin(), null_kp_.end(),
                                    [](double k){ return k > 0.0; });

  // --- if no task cmd: hold (simple) ---
  if (!cmd_cached_.valid) {
    ee_ref_init_ = false;
    for (size_t i = 0; i < N; ++i) {
      qdot_ref_[i] = 0.0;
      q_ref_[i]    = q_kdl_(i);
    }
    write_refs_to_slots();
    return controller_interface::return_type::OK;
  }

  // --- build FK and Jacobians ---
  if (fk_solver_->JntToCart(q_kdl_, tip_frame_) < 0) {
    write_refs_to_slots();
    return controller_interface::return_type::OK;
  }
  Eigen::Vector3d r_be(tip_frame_.p.x(), tip_frame_.p.y(), tip_frame_.p.z());

  KDL::Jacobian J(N);
  if (jac_solver_->JntToJac(q_kdl_, J) < 0) {
    write_refs_to_slots();
    return controller_interface::return_type::OK;
  }

  Eigen::Matrix<double, 6, Eigen::Dynamic> Je(6, N);
  for (unsigned r = 0; r < 6; ++r)
    for (unsigned c = 0; c < N; ++c)
      Je(r, c) = J(r, c);

  Eigen::Matrix<double, 6, 3> Jb;
  Jb.setZero();
  Jb(0,0) = 1.0;
  Jb(1,1) = 1.0;
  Jb(0,2) = -r_be.y();
  Jb(1,2) =  r_be.x();
  Jb(5,2) =  1.0;

  const int M = static_cast<int>(3 + N);


  // x = [base; joints], M = 3 + N
  Eigen::MatrixXd Jtask(6, M);
  Jtask.setZero();
  Jtask.block<6,3>(0,0) = Jb;
  Jtask.block(0,3,6,(int)N) = Je;
  // Read cached world<-base transform (RT-safe)
  Eigen::Isometry3d T_wb = *T_wb_rt_.readFromRT();

  // ---- feedforward EE twist command (base_link frame) ----
  Eigen::Vector3d vff_lin(cmd_cached_.vx, cmd_cached_.vy, cmd_cached_.vz);
  Eigen::Vector3d vff_ang(cmd_cached_.wx, cmd_cached_.wy, cmd_cached_.wz);

  // world <- base rotation
  Eigen::Matrix3d R_wb = T_wb.linear();

  // convert commanded vel into world before integrating world position reference
  vff_lin = R_wb * vff_lin;
  vff_ang = R_wb * vff_ang;
  // Current EE pose (in base_link)
  const KDL::Vector p_cur = tip_frame_.p;
  const KDL::Rotation R_cur = tip_frame_.M;




  // p_cur_base from KDL
  Eigen::Vector3d p_cur_base(p_cur.x(), p_cur.y(), p_cur.z());

  // Convert to world
  Eigen::Vector3d p_cur_world = T_wb.linear() * p_cur_base + T_wb.translation();

  // Initialize reference the first time (or after invalid cmd)
  if (!ee_ref_init_) {
    p_ref_ = p_cur_world;
    R_ref_ = R_cur;
    ee_ref_init_ = true;
  }

  // ---- integrate the reference forward using feedforward velocity ----
  p_ref_.x() += vff_lin.x() * dt_used;
  p_ref_.y() += vff_lin.y() * dt_used;
  p_ref_.z() += vff_lin.z() * dt_used;
  // RCLCPP_INFO_THROTTLE(
  //   get_node()->get_logger(), *get_node()->get_clock(), 500,
  //   "EE REF p_ref_world=(%.3f %.3f %.3f)",
  //   p_ref_.x(), p_ref_.y(), p_ref_.z()
  // );

  // Now perr in world (assuming p_ref_ is world)
  Eigen::Vector3d p_ref_world(p_ref_.x(), p_ref_.y(), p_ref_.z());
  Eigen::Vector3d p_err_world = p_ref_world - p_cur_world;

  // base <- world rotation is transpose if R_wb is orthonormal
  Eigen::Vector3d p_err = R_wb.transpose() * p_err_world;

  RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 500,
                        "perr=(%.3f %.3f %.3f)",
                        p_err.x(), p_err.y(), p_err.z()
                      );

  RCLCPP_INFO_THROTTLE(
    get_node()->get_logger(), *get_node()->get_clock(), 500,
    "EE ACT p_cur_world=(%.3f %.3f %.3f)",
    p_cur_world.x(), p_cur_world.y(), p_cur_world.z()
  );

  // rotation reference integrate: R_ref <- Exp(w*dt) * R_ref
  const double wx = vff_ang.x(), wy = vff_ang.y(), wz = vff_ang.z();
  const double wnorm = std::sqrt(wx*wx + wy*wy + wz*wz);
  if (wnorm > 1e-9) {
    const double ang = wnorm * dt_used;
    const double ax = wx / wnorm, ay = wy / wnorm, az = wz / wnorm;
    KDL::Rotation dR = KDL::Rotation::Rot(KDL::Vector(ax, ay, az), ang);
    R_ref_ = dR * R_ref_;
  }

  // double roll_ref, pitch_ref, yaw_ref;
  // R_ref_.GetRPY(roll_ref, pitch_ref, yaw_ref);

  // RCLCPP_INFO_THROTTLE(
  //   get_node()->get_logger(), *get_node()->get_clock(), 500,
  //   "EE R_ref RPY = (%.3f, %.3f, %.3f)",
  //   roll_ref, pitch_ref, yaw_ref
  // );

  // Optional anti-windup: leak reference slowly toward current pose
  if (ee_ref_leak_ > 0.0) {
    const double a = std::clamp(ee_ref_leak_ * dt_used, 0.0, 1.0);
    p_ref_ = p_ref_ * (1.0 - a) + p_cur_world * a;
    // (Rotation leak can be added later; position leak alone helps a lot.)
  }

  // ---- compute pose error ----
  // KDL::Vector p_err = p_cur - p_ref_;

  // clamp position error magnitude (prevents runaway)
  const double perr_norm = std::sqrt(p_err.x()*p_err.x() + p_err.y()*p_err.y() + p_err.z()*p_err.z());
  if (perr_norm > ee_err_max_) {
    const double s = ee_err_max_ / (perr_norm + 1e-9);
    p_err = p_err * s;
  }


  if (perr_norm < ee_err_deadband_) {
    p_err.setZero();
  } else if (perr_norm > ee_err_max_) {
    const double s = ee_err_max_ / (perr_norm + 1e-9);
    p_err *= s;
  }


  // orientation error: R_err = R_ref * R_cur^{-1}
  // R_err = R_ref_ * R_cur.Inverse();

  KDL::Rotation R_err = R_ref_ * R_cur.Inverse(); 

  // angle from trace
  double tr = R_err(0,0) + R_err(1,1) + R_err(2,2);
  double c = std::max(-1.0, std::min(1.0, 0.5*(tr - 1.0)));
  double angle = std::acos(c);

  // axis from skew
  Eigen::Vector3d axis;
  axis << (R_err(2,1) - R_err(1,2)),
          (R_err(0,2) - R_err(2,0)),
          (R_err(1,0) - R_err(0,1));
  axis *= 0.5;

  double s = axis.norm();
  if (s > 1e-9) axis /= s;
  else axis.setZero();

  // rotation vector
  Eigen::Vector3d rotvec = angle * axis;
  if (rotvec.norm() < ee_rot_deadband_) rotvec.setZero();
  Eigen::Vector3d vcorr_ang_p = ee_kp_rot_ * rotvec;



  // RCLCPP_INFO_THROTTLE(
  //   get_node()->get_logger(),
  //   *get_node()->get_clock(),
  //   500,
  //   "ROTERR(trace): angle=%.6f axis=(%.3f %.3f %.3f) rotvec=(%.3f %.3f %.3f)",
  //   angle, axis.x(), axis.y(), axis.z(), rotvec.x(), rotvec.y(), rotvec.z()
  // );


 
  Eigen::VectorXd dq_eig(N);
  for (size_t i=0;i<N;++i) dq_eig((int)i) = dq_kdl_(i);

  // approximate EE twist from joints only (good enough to damp)
  Eigen::Matrix<double,6,1> vee_j = Je * dq_eig;


  // ---- feedback correction twist ----
  Eigen::Vector3d vcorr_lin_p(ee_kp_pos_ * p_err.x(),
                            ee_kp_pos_ * p_err.y(),
                            ee_kp_pos_ * p_err.z());

  // Eigen::Vector3d vcorr_ang_p(ee_kp_rot_ * axis.x() * angle,
                            // ee_kp_rot_ * axis.y() * angle,
                            // ee_kp_rot_ * axis.z() * angle);


  // D term (damp measured EE motion)
  Eigen::Vector3d vcorr_lin = vcorr_lin_p - ee_kd_pos_ * vee_j.head<3>();
  Eigen::Vector3d vcorr_ang = vcorr_ang_p - ee_kd_rot_ * vee_j.tail<3>();


  // ---- corrected desired EE twist ----
  Eigen::Vector3d vdes_lin = vff_lin + vcorr_lin;
  Eigen::Vector3d vdes_ang = vff_ang + vcorr_ang;

  // clamp final desired twist (safety)
  const double vlin = vdes_lin.norm();
  if (vlin > ee_vmax_lin_) vdes_lin *= (ee_vmax_lin_ / (vlin + 1e-9));
  const double vang = vdes_ang.norm();
  if (vang > ee_vmax_ang_) vdes_ang *= (ee_vmax_ang_ / (vang + 1e-9));

  // v (6): ee_des only (the thing fed into QP)
  Eigen::VectorXd v(6);
  v << vdes_lin.x(), vdes_lin.y(), vdes_lin.z(),
      vdes_ang.x(), vdes_ang.y(), vdes_ang.z();

  // v (6): ee_des only
  // Eigen::VectorXd v(6);
  // v << cmd_cached_.vx, cmd_cached_.vy, cmd_cached_.vz,
  //     cmd_cached_.wx, cmd_cached_.wy, cmd_cached_.wz;

  // RCLCPP_INFO_THROTTLE(
  //   get_node()->get_logger(), *get_node()->get_clock(), 500,
  //   "EE ERR perr=(%.3f %.3f %.3f)",
  //   p_ref_.x() - p_cur_world.x(),
  //   p_ref_.y() - p_cur_world.y(),
  //   p_ref_.z() - p_cur_world.z()
  // );


  // Base preferred velocity from navigation / trajectory generator
  Eigen::Vector3d vb_ref(
    cmd_cached_.bvx,
    cmd_cached_.bvy,
    cmd_cached_.bwz
  );

  RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 500,
                        "EE vff=(%.3f %.3f %.3f | %.3f %.3f %.3f)  vdes=(%.3f %.3f %.3f | %.3f %.3f %.3f)  perr=(%.3f %.3f %.3f)",
                        vff_lin.x(), vff_lin.y(), vff_lin.z(), vff_ang.x(), vff_ang.y(), vff_ang.z(),
                        v(0), v(1), v(2), v(3), v(4), v(5),
                        p_err.x(), p_err.y(), p_err.z()
                      );



  Eigen::VectorXd wrow(6);
  wrow.setConstant(1.0);   // you said ignore weights; keep 1.0 everywhere

  Eigen::MatrixXd Jw = Jtask;
  Eigen::VectorXd vw = v;
  for (int r = 0; r < 6; ++r) {
    Jw.row(r) *= wrow(r);
    vw(r)     *= wrow(r);
  }


  // DOF effort weights Wu (diagonal)
  Eigen::VectorXd wu(M);
  wu.segment<3>(0).setConstant(std::max(1e-6, base_weight_));
  wu.segment(3,(int)N).setConstant(std::max(1e-6, arm_weight_));

  // posture target qdot_post (N)
  Eigen::VectorXd qdot_post(N);
  qdot_post.setZero();
  if (posture_active) {
    for (size_t i=0;i<N;++i) {
      const double e  = q_home_[i] - q_kdl_(i);
      const double ed = -dq_kdl_(i);
      double ui = null_kp_[i] * e + null_kd_[i] * ed;
      qdot_post((int)i) = std::clamp(ui, -qdot_limit_, qdot_limit_);
    }
  }

  // Build quadratic form:
  // P = 2*(Jw^T Jw + lambda^2 * diag(wu^2) + posture_weight * Sq^T Sq)
  // q = -2*(Jw^T vw + posture_weight * Sq^T qdot_post)
  Eigen::MatrixXd P = Eigen::MatrixXd::Zero(M, M);
  Eigen::VectorXd q = Eigen::VectorXd::Zero(M);

  P.noalias() += 2.0 * (Jw.transpose() * Jw);
  q.noalias() += -2.0 * (Jw.transpose() * vw);

  // Base preference: || v_base - vb_ref ||^2  (soft)
  const double alpha_qp = 1.0;   // "how strongly to follow base cmd"; start with 1.0

  Eigen::MatrixXd Sb = Eigen::MatrixXd::Zero(3, M);
  Sb.block<3,3>(0,0).setIdentity();   // selects base part of x

  P.noalias() += 2.0 * alpha_qp * (Sb.transpose() * Sb);
  q.noalias() += -2.0 * alpha_qp * (Sb.transpose() * vb_ref);

  // regularization on all DOFs
  const double lam2 = lambda_ * lambda_;
  P.diagonal().array() += 2.0 * lam2 * wu.array().square();

  // posture term acts only on joint DOFs
  const double pw = (posture_active ? std::max(0.0, posture_weight_) : 0.0);
  for (int i=0;i<(int)N;++i) {
    const int idx = 3 + i;
    P(idx, idx) += 2.0 * pw;
    q(idx)      += -2.0 * pw * qdot_post(i);
  }


  // -----------------------------
  // CBF constraints (EE point vs circle obstacles)
  // Constraint per obstacle:
  //   a^T x >= b
  // where a^T = grad^T * Jp,  b = -alpha * h
  // -----------------------------
  struct CbfRow {
    Eigen::RowVectorXd a;  // 1xM
    double b;
  };

  std::vector<CbfRow> cbf_rows;

  const double cbf_alpha   = 3.0;   // tune 1..10
  const double ee_radius   = 0.02;  // meters (treat EE as circle)
  const double cbf_margin  = 0.03;  // meters extra safety
  const double activate_h  = 0.30;  // only constrain if within 30cm
  const double eps         = 1e-6;

  if (obstacles_ok) {
    Eigen::Vector3d p_ee = r_be;

    // Jp: translational Jacobian mapping x=[vbase; qdot] -> EE linear vel
    Eigen::MatrixXd Jp(3, M);
    Jp.setZero();
    Jp.block<3,3>(0,0) = Jb.topRows<3>();
    Jp.block(0,3,3,(int)N) = Je.topRows<3>();

    for (const auto& o : obstacles_cached_.obs) {
      Eigen::Vector3d c(o.cx, o.cy, o.cz);
      const double r_obs = (double)o.r;

      Eigen::Vector3d d = p_ee - c;
      const double dist = d.norm();
      if (dist < eps) continue;

      const double h = dist - (r_obs + ee_radius + cbf_margin);
      if (h > activate_h) continue;  // too far, skip

      const Eigen::Vector3d grad = d / dist;        // ∇h
      Eigen::RowVectorXd a = grad.transpose() * Jp; // 1xM
      const double b = -cbf_alpha * h;              // scalar

      cbf_rows.push_back({a, b});
    }
  }

  // // Solve unconstrained
  // // minimize 0.5 x^T P x + q^T x  -> P x = -q
  // Eigen::VectorXd x = P.ldlt().solve(-q);

  // // Build bounds umin/umax
  Eigen::VectorXd umin(M), umax(M);
  umin(0) = -base_vx_limit_; umax(0) = base_vx_limit_;
  umin(1) = -base_vy_limit_; umax(1) = base_vy_limit_;
  umin(2) = -base_wz_limit_; umax(2) = base_wz_limit_;

  for (size_t i=0;i<N;++i) {
    const double qcur = q_kdl_(i);

    // position bounds -> velocity bounds
    const double qd_min_pos = (q_min_[i] - qcur) / dt_used;
    const double qd_max_pos = (q_max_[i] - qcur) / dt_used;

    const double lo = std::max(-qdot_limit_, qd_min_pos);
    const double hi = std::min( qdot_limit_, qd_max_pos);

    umin(3+(int)i) = lo;
    umax(3+(int)i) = hi;
  }

  // Fill q
  q_osqp_ = q;

  // Fill P upper triangle into sparse
  for (int i = 0; i < M_; ++i) {
    for (int j = i; j < M_; ++j) {
      Psp_.coeffRef(i, j) = P(i, j);
    }
  }
  // Psp_.makeCompressed();

  for (int i = 0; i < M_; ++i) {
    l_osqp_(i) = umin(i);
    u_osqp_(i) = umax(i);
  }

  const double alpha  = CBF_alpha_;
  const double ee_r   = ee_radius_;
  const double margin = cbf_margin_;
  const double h_act  = cbf_activate_h_;


  int k = 0;

  if (obstacles_ok) {
    Eigen::Vector3d p_ee = r_be;

    Eigen::MatrixXd Jp(3, M_);
    Jp.setZero();
    Jp.block<3,3>(0,0) = Jb.topRows<3>();
    Jp.block(0,3,3,(int)N) = Je.topRows<3>();

    for (const auto& o : obstacles_cached_.obs) {
      if (k >= Kmax_) break;

      Eigen::Vector3d c(o.cx, o.cy, o.cz);
      const double r_obs = (double)o.r;

      Eigen::Vector3d d = p_ee - c;
      const double dist = d.norm();
      if (dist < eps) continue;

      const double h = dist - (r_obs + ee_r + margin);
      if (h > h_act) continue;

      const Eigen::Vector3d grad = d / dist;
      Eigen::RowVectorXd a = grad.transpose() * Jp;  // 1xM

      const double b = -alpha * h;

      // Write this row into Asp_ at row = M_ + k
      const int row = M_ + k;
      for (int j = 0; j < M_; ++j) {
        Asp_.coeffRef(row, j) = a(j);
      }

      l_osqp_(row) = b;
      u_osqp_(row) = osqp_inf_;

      ++k;
    }
  }

  // Disable remaining CBF rows (no constraint)
  for (; k < Kmax_; ++k) {
    const int row = M_ + k;
    for (int j = 0; j < M_; ++j) {
      Asp_.coeffRef(row, j) = 0.0;
    }
    l_osqp_(row) = -osqp_inf_;
    u_osqp_(row) =  osqp_inf_;
  }

  // Asp_.makeCompressed();

  if (!osqp_ready_) {
    write_refs_to_slots();
    return controller_interface::return_type::OK;
  }

  // Update matrices/bounds
  osqp_.updateHessianMatrix(Psp_);
  osqp_.updateGradient(q_osqp_);
  osqp_.updateLinearConstraintsMatrix(Asp_);
  osqp_.updateLowerBound(l_osqp_);
  osqp_.updateUpperBound(u_osqp_);

  // Warm start
  if (x_prev_.size() == M_) osqp_.setPrimalVariable(x_prev_);

  const bool ok = osqp_.solveProblem() == OsqpEigen::ErrorExitFlag::NoError;

  // Safe fallback: zero velocities
  Eigen::VectorXd x = Eigen::VectorXd::Zero(M_);
  if (ok) {
    x = osqp_.getSolution();
    x_prev_ = x;
  } else {
    x_prev_.setZero();  // optional
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                    "OSQP solve failed (keeping safe hold)");
  }
  // RCLCPP_INFO_THROTTLE(
  //   get_node()->get_logger(), *get_node()->get_clock(), 500,
  //   "QP SOL x = [ vb=(%.3f %.3f %.3f) | qdot_norm=%.4f ]",
  //   x(0), x(1), x(2),
  //   x.tail((int)N).norm()
  // );

  // -----------------------------
  // Task tracking error (DEBUG)
  // -----------------------------
  // Eigen::VectorXd task_err = Jtask * x - v;

  // double base_err = task_err.segment<3>(0).norm();
  // double ee_err   = task_err.segment<6>(3).norm();

  // RCLCPP_INFO_THROTTLE(
  //   get_node()->get_logger(), *get_node()->get_clock(), 500,
  //   "QP ERR base=%.4f ee=%.4f | ||x||=%.4f",
  //   base_err, ee_err, x.norm()
  // );


  // Split results
  Eigen::Vector3d v_base = x.head<3>();
  Eigen::VectorXd qdot   = x.tail((int)N);

  Eigen::VectorXd qdot_exec = qdot;  // what you'll really execute
  // for (size_t i = 0; i < N; ++i) {
  //   double dq = qdot(i)*dt_used;
  //   dq = std::clamp(dq, -step_limit_, step_limit_);
  //   qdot_exec(i) = dq / dt_used;
  // }
  // Eigen::Matrix<double,6,1> vee_res = Jb * v_base + Je * qdot_exec;
  Eigen::Matrix<double,6,1> vee_res = Je * qdot;

  RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 500, "vee_res=(%.3f %.3f %.3f | %.3f %.3f %.3f)",
    vee_res(0),vee_res(1),vee_res(2),vee_res(3),vee_res(4),vee_res(5));



  // Integrate joint refs (your downstream controller expects pos+vel)
  for (size_t i=0;i<N;++i) {
    qdot_ref_[i] = qdot((int)i);
    const double dq_raw     = qdot_ref_[i] * dt_used;
    const double dq_clamped = std::clamp(dq_raw, -step_limit_, step_limit_);
    q_ref_[i] += dq_clamped;
    q_ref_[i] = std::clamp(q_ref_[i], q_min_[i], q_max_[i]);
  }

  // Publish base twist
  geometry_msgs::msg::TwistStamped base_cmd;
  base_cmd.header.stamp = now;
  base_cmd.header.frame_id = base_link_;
  base_cmd.twist.linear.x  = -base_cmd_scale_ * v_base[0];
  base_cmd.twist.linear.y  = -base_cmd_scale_ * v_base[1];
  base_cmd.twist.angular.z =  base_cmd_scale_ * v_base[2];
  base_cmd_pub_->publish(base_cmd);

  // RCLCPP_INFO_THROTTLE(
  //   get_node()->get_logger(), *get_node()->get_clock(), 1000,
  //   "QP: v_base=(%.3f %.3f %.3f) qdot_norm=%.3f r_be=(%.3f %.3f %.3f) posture=%d",
  //   v_base.x(), v_base.y(), v_base.z(),
  //   qdot.norm(),
  //   r_be.x(), r_be.y(), r_be.z(),
  //   posture_active ? 1 : 0
  // );
  // command to arm joints
  write_refs_to_slots();
  return controller_interface::return_type::OK;
}

void WholeBodyQPController::wbCmdCb(const ombot_msgs::msg::WholeBodyCmd::SharedPtr msg)
{
  Cmd c;
  c.valid = msg->valid;

  c.vx = msg->ee.linear.x;
  c.vy = msg->ee.linear.y;
  c.vz = msg->ee.linear.z;

  c.wx = msg->ee.angular.x;
  c.wy = msg->ee.angular.y;
  c.wz = msg->ee.angular.z;

  c.bvx = msg->bvx;
  c.bvy = msg->bvy;
  c.bwz = msg->bwz;

  last_cmd_time_ = msg->header.stamp;
  cmd_rt_.writeFromNonRT(c);
}

void WholeBodyQPController::write_refs_to_slots()
{
  const size_t N = joint_names_.size();
  for (size_t i = 0; i < N; ++i) {
    if (pos_cmd_index_.size() == N && pos_cmd_index_[i] >= 0)
      command_interfaces_[pos_cmd_index_[i]].set_value(q_ref_[i]);
    if (vel_cmd_index_.size() == N && vel_cmd_index_[i] >= 0)
      command_interfaces_[vel_cmd_index_[i]].set_value(qdot_ref_[i]);
  }
}
}// namespace ombot_controller



PLUGINLIB_EXPORT_CLASS(ombot_controller::WholeBodyQPController,
                       controller_interface::ChainableControllerInterface)
