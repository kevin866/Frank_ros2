#pragma once

#include <memory>
#include <string>
#include <vector>
#include <cstdint>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

// KDL
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl/chaindynparam.hpp>

namespace ombot_controller
{

class JointFrictionCalibController : public controller_interface::ControllerInterface
{
public:
  JointFrictionCalibController() = default;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(const rclcpp::Time & time,
                                           const rclcpp::Duration & period) override;

            

private:
  // --------------------------
  // Params / config
  // --------------------------
  std::vector<std::string> joint_names_;
  std::string base_link_;
  std::string tip_link_;
  KDL::Vector gravity_vec_{0.0, 0.0, -9.81};

  // Gravity scaling/filter
  double g_scale_{1.0};
  double alpha_{1.0};                       // gravity output filter
  std::vector<double> effort_limits_;       // optional clamp

  // Calibration settings
  std::string active_joint_name_{""};       // prefer name; fallback to index
  int active_joint_index_{-1};

  double v_amp_{0.2};                       // rad/s excitation amplitude
  double v_freq_{0.2};                      // Hz
  double settle_time_{2.0};                 // seconds before collecting samples
  double run_time_{20.0};                   // seconds to collect samples (per joint)

  // Velocity servo (effort output)
  double Kv_{2.0};                          // Nm / (rad/s)
  double Kd_{0.0};                          // Nm / (rad/s) damping on dq (optional)
  double max_servo_tau_{100.0};               // clamp extra torque (safety)

  // Fitting gates
  double fit_min_speed_{0.03};              // ignore |dq| below this
  double v_sgn_eps_{0.01};                  // smooth sign eps for tanh(dq/eps)

  // Behavior
  bool auto_advance_{true};                 // auto run joints one by one if active_joint not set
  bool print_each_joint_{true};             // print per-joint result when done
  bool apply_friction_ff_after_fit_{false}; // optional: apply fitted friction compensation in same run

  // --------------------------
  // KDL
  // --------------------------
  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainDynParam> dyn_;
  KDL::JntArray q_;
  KDL::JntArray g_;

  std::vector<double> last_g_cmd_;          // filtered gravity command

  // --------------------------
  // Runtime state
  // --------------------------
  rclcpp::Time t0_;
  bool active_{false};

  enum class Phase { SETTLE, RUN, DONE };
  Phase phase_{Phase::SETTLE};

  // Least squares accumulator for y = Fc*s + B*v + bias
  // Using normal equations: A = Σ φφ^T (3x3), b = Σ φ y (3x1)
  struct LS3 {
    double A[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
    double b[3]    = {0,0,0};
    int    n       = 0;

    void reset() {
      for (int i=0;i<3;i++){ b[i]=0; for(int j=0;j<3;j++) A[i][j]=0; }
      n = 0;
    }


    void add(double s, double v, double y) {
      const double phi[3] = {s, v, 1.0};
      for (int i=0;i<3;i++){
        b[i] += phi[i]*y;
        for (int j=0;j<3;j++) A[i][j] += phi[i]*phi[j];
      }
      n++;
    }

    bool solve(double x_out[3]) const; // Fc, B, bias
  };

  std::vector<LS3> fit_;                    // one fitter per joint
  std::vector<double> Fc_, B_, tau_bias_;   // results per joint
  std::vector<bool> joint_done_;

  // Helpers
  int find_joint_index_(const std::string & name) const;
  void recompute_active_joint_();
  void clamp_effort_(std::vector<double> & tau) const;


  double smooth_sign_(double v) const;  // tanh(v/eps)

  // Compute and write effort commands
  void compute_gravity_(const std::vector<double> & q, std::vector<double> & tau_g);
  void write_effort_cmd_(const std::vector<double> & tau_cmd);
  void write_velocity_cmd_(const std::vector<double> & dq_cmd);

  // Phase handling
  void start_joint_(int j, const rclcpp::Time & now);
  void finish_joint_(int j);
  void maybe_advance_joint_(const rclcpp::Time & now);

  // Printing
  void print_yaml_results_() const;
  std::vector<double> q_min_;
  std::vector<double> q_max_;
  double q_soft_margin_{0.2};
  double k_limit_{15.0};
  double d_limit_{1.0};
  std::vector<double> dq_prev_;
  std::vector<double> Kv_joints_;
  std::vector<double> Kd_joints_;

  KDL::JntArray dq_kdl_;
  KDL::JntArray c_kdl_;   // Coriolis torques
};

} // namespace ombot_controller
