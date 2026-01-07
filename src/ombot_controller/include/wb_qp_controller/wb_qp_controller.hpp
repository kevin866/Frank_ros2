#pragma once

#include "controller_interface/chainable_controller_interface.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "ombot_msgs/msg/whole_body_cmd.hpp"

#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include "ombot_msgs/msg/obstacle_array.hpp"
#include "ombot_msgs/msg/obstacle.hpp"
#include <OsqpEigen/OsqpEigen.h>

#include <Eigen/Dense>
#include "pluginlib/class_list_macros.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "controller_interface/chainable_controller_interface.hpp"
#include "controller_interface/controller_interface.hpp"

#include "realtime_tools/realtime_buffer.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"


#include <algorithm>
#include <cmath>
#include <string>
#include <vector>
#include <stdexcept>

namespace ombot_controller {

class WholeBodyQPController : public controller_interface::ChainableControllerInterface
{
public:
  WholeBodyQPController();

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;
  
  bool on_set_chained_mode(bool) override;

  controller_interface::return_type update_reference_from_subscribers() override;
  controller_interface::return_type update_and_write_commands(
      const rclcpp::Time&, const rclcpp::Duration& period) override;

private:
  struct Cmd {
    bool valid{false};
    double vx{0}, vy{0}, vz{0};
    double wx{0}, wy{0}, wz{0};
    double bvx{0}, bvy{0}, bwz{0};
  };


  struct ObstacleLite {
    float cx{0.f}, cy{0.f}, cz{0.f};
    float r{0.f};
    float vx{0.f}, vy{0.f}, vz{0.f};
    };

    struct ObstaclesCache {
    bool valid{false};
    rclcpp::Time stamp;
    std::string frame_id;
    std::vector<ObstacleLite> obs;   // small, numeric-only
    };

    

    realtime_tools::RealtimeBuffer<ObstaclesCache> obstacles_rt_;
    ObstaclesCache obstacles_cached_;

    rclcpp::Subscription<ombot_msgs::msg::ObstacleArray>::SharedPtr obstacles_sub_;

    std::string obstacles_topic_{"/obstacles"};
    double obstacles_timeout_{0.25};   // seconds


  // --- ROS ---
  rclcpp::Subscription<ombot_msgs::msg::WholeBodyCmd>::SharedPtr wb_cmd_sub_;
  realtime_tools::RealtimeBuffer<Cmd> cmd_rt_;
  Cmd cmd_cached_;
  rclcpp::Time last_cmd_time_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr base_cmd_pub_;
  std::string base_cmd_topic_{"/mecanum_controller/reference"};

  void wbCmdCb(const ombot_msgs::msg::WholeBodyCmd::SharedPtr msg);
  void obstaclesCb(const ombot_msgs::msg::ObstacleArray::SharedPtr msg);


  // --- Params ---
  std::vector<std::string> joint_names_;
  std::string base_link_, tip_link_, inner_ctrl_name_;
  double lambda_{0.02};
  double qdot_limit_{1.0};
  double cmd_timeout_{0.25};

  double base_vx_limit_{0.3}, base_vy_limit_{0.3}, base_wz_limit_{0.5};
  double base_weight_{1.0}, arm_weight_{1.0};
  double base_cmd_scale_{1.0};
  double posture_weight_{0.1};


  // NEW QP params
  double rho_posture_{0.05};     // posture strength in QP
  double qp_reg_{1e-4};          // add small diag to H for numerical stability
  double w_pos_{1.0}, w_rot_{1.0}, w_z_{1.0};  // EE tracking weights (simple knobs)

  // posture gains (existing)
  std::vector<double> null_kp_, null_kd_, q_home_;

  // Integrator/limits (existing)
  double dt_ceiling_{0.03};
  double step_limit_{0.02};
  std::vector<double> q_min_, q_max_;  // ensure you fill these somewhere


  // --- KDL ---
  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  KDL::JntArray q_kdl_, dq_kdl_;
  KDL::Frame tip_frame_;

  // --- State/command interfaces ---
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> pos_states_, vel_states_;
  std::vector<int> pos_cmd_index_, vel_cmd_index_;
  std::vector<double> q_ref_, qdot_ref_;

  double CBF_alpha_{3.0};
  double ee_radius_{0.02};
  double cbf_margin_{0.03};
  double cbf_activate_h_{0.30};


  void write_refs_to_slots();
  void init_osqp_structures(int N);

  // --- Core helpers ---
  void build_Jb_Je(Eigen::Matrix<double,6,3>& Jb,
                   Eigen::Matrix<double,6,Eigen::Dynamic>& Je,
                   Eigen::Vector3d& r_be);

  // Stage A: nominal tracking solve
  Eigen::VectorXd solve_nominal_dls(const Eigen::MatrixXd& Jee,
                                   const Eigen::VectorXd& vee,
                                   const Eigen::VectorXd& wu,
                                   double lambda);

  // Stage B: QP filter (currently: box-only solve; later swap in OSQP)
  Eigen::VectorXd solve_qp_filter_box_only(
      const Eigen::VectorXd& u0,
      const Eigen::VectorXd& umin,
      const Eigen::VectorXd& umax,
      const Eigen::VectorXd& qdot_post,
      double rho_post,
      const Eigen::VectorXd& wu);

    // --- OSQP ---
    OsqpEigen::Solver osqp_;
    bool osqp_ready_{false};

    int M_{0};                 // num vars = 3 + N
    int Kmax_{20};             // max obstacle constraints
    double osqp_inf_{1e10};    // OSQP "infinity"

    // Matrices/vectors kept as members to avoid realloc every loop
    Eigen::SparseMatrix<double> Psp_;   // Hessian (MxM)
    Eigen::SparseMatrix<double> Asp_;   // Constraints ((M+Kmax) x M)
    Eigen::VectorXd q_osqp_;            // gradient (M)
    Eigen::VectorXd l_osqp_, u_osqp_;   // bounds (M+Kmax)

    // Precomputed sparsity pattern triplets for A (fixed)
    std::vector<Eigen::Triplet<double>> A_triplets_;

    // Optional: store last solution for warm start
    Eigen::VectorXd x_prev_;


};

} // namespace ombot_controller
