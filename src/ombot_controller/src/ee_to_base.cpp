#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <algorithm>
#include <cmath>

class EeToBase : public rclcpp::Node {
public:
  EeToBase() : Node("ee_to_base") {
    // Params
    use_stamped_out_  = declare_parameter("use_stamped_out", true);  // true -> TwistStamped
    kp_lin_           = declare_parameter("kp_lin", 0.4);            // m/s per meter
    kp_yaw_           = declare_parameter("kp_yaw", 1.0);            // rad/s per rad
    vmax_             = declare_parameter("vmax", 0.5);              // m/s
    wmax_             = declare_parameter("wmax", 1.0);              // rad/s
    alpha_            = declare_parameter("lpf_alpha", 0.4);         // 0..1
    deadband_m_       = declare_parameter("deadband_m", 0.01);       // meters
    deadband_yaw_     = declare_parameter("deadband_yaw", 0.02);     // rad
    watchdog_ms_      = declare_parameter("watchdog_ms", 150);       // zero cmd if ee pose stale
    zero_on_startup_  = declare_parameter("zero_on_startup", true);  // latch first /ee_pose as zero
    enable_           = declare_parameter("enable", true);
    rate_hz_          = declare_parameter("publish_rate_hz", 50.0);  // cmd publish rate

    // I/O
    sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/ee_pose", rclcpp::SensorDataQoS(),
      std::bind(&EeToBase::eeCb, this, std::placeholders::_1));

    if (use_stamped_out_) {
      pub_stamped_ = create_publisher<geometry_msgs::msg::TwistStamped>("/mecanum_controller/reference", 10);
    } else {
      pub_ = create_publisher<geometry_msgs::msg::Twist>("/mecanum_controller/reference", 10);
    }

    // Reset zero service
    reset_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/reset_zero", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                             std::shared_ptr<std_srvs::srv::Trigger::Response> res){
        have_zero_ = false;
        res->success = true;
        res->message = "Zero will be re-latched on next /ee_pose.";
      });

    // Timer for publishing commands
    timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / std::max(1.0, rate_hz_))),
      std::bind(&EeToBase::publishCmd, this));
  }

private:
  void eeCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    last_stamp_ = msg->header.stamp;

    // Latch zero pose if needed
    if (zero_on_startup_ && !have_zero_) {
      zero_pose_ = *msg;  // store the whole pose (position + orientation)
      have_zero_ = true;
      RCLCPP_INFO(get_logger(), "Latched /ee_pose as zero (frame_id=%s).", msg->header.frame_id.c_str());
    }

    // Compute deflection relative to zero (or absolute if no zero latched)
    double ex = msg->pose.position.x - (have_zero_ ? zero_pose_.pose.position.x : 0.0);
    double ey = msg->pose.position.y - (have_zero_ ? zero_pose_.pose.position.y : 0.0);

    // Yaw error (orientation)
    auto yaw_of = [](const geometry_msgs::msg::Quaternion &q)->double{
      tf2::Quaternion tq; tf2::fromMsg(q, tq);
      double r,p,y; tf2::Matrix3x3(tq).getRPY(r,p,y); return y;
    };
    double yaw = yaw_of(msg->pose.orientation);
    double yaw0 = have_zero_ ? yaw_of(zero_pose_.pose.orientation) : 0.0;
    double e_yaw = normalizeAngle(yaw - yaw0);

    // Deadbands
    if (std::hypot(ex, ey) < deadband_m_) { ex = 0.0; ey = 0.0; }
    if (std::fabs(e_yaw) < deadband_yaw_) e_yaw = 0.0;

    // Proportional mapping in the *pose frame provided*.
    // If /ee_pose isn’t in base frame, consider transforming before mapping.
    double vx = kp_lin_ * ex;
    double vy = kp_lin_ * ey;
    double wz = kp_yaw_ * e_yaw;

    // Saturate
    vx = std::clamp(vx, -vmax_,  vmax_);
    vy = std::clamp(vy, -vmax_,  vmax_);
    wz = std::clamp(wz, -wmax_,  wmax_);

    // Low-pass
    vx_f_ = alpha_ * vx + (1.0 - alpha_) * vx_f_;
    vy_f_ = alpha_ * vy + (1.0 - alpha_) * vy_f_;
    wz_f_ = alpha_ * wz + (1.0 - alpha_) * wz_f_;
  }

  void publishCmd() {
    if (!enable_) return;

    // Watchdog: zero if ee pose stale
    const auto now = this->get_clock()->now();
    const bool stale = (watchdog_ms_ > 0) &&
                       (!last_stamp_.nanoseconds() ||
                        (now - last_stamp_) > rclcpp::Duration::from_seconds(watchdog_ms_ / 1000.0));

    if (stale) { vx_f_ = 0.0; vy_f_ = 0.0; wz_f_ = 0.0; }

    if (use_stamped_out_) {
      geometry_msgs::msg::TwistStamped ts;
      ts.header.stamp = now;
      ts.header.frame_id = "base_link"; // optional: set to whatever your base controller expects
      ts.twist.linear.x  = vx_f_;
      ts.twist.linear.y  = vy_f_;
      ts.twist.angular.z = wz_f_;
      pub_stamped_->publish(ts);
    } else {
      geometry_msgs::msg::Twist t;
      t.linear.x  = vx_f_;
      t.linear.y  = vy_f_;
      t.angular.z = wz_f_;
      pub_->publish(t);
    }
  }

  static double normalizeAngle(double a) {
    while (a >  M_PI) a -= 2*M_PI;
    while (a < -M_PI) a += 2*M_PI;
    return a;
  }

  // ROS I/O
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_stamped_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Params / state
  bool use_stamped_out_, zero_on_startup_, enable_, have_zero_{false};
  double kp_lin_, kp_yaw_, vmax_, wmax_, alpha_, deadband_m_, deadband_yaw_, rate_hz_;
  int watchdog_ms_;
  geometry_msgs::msg::PoseStamped zero_pose_;
  rclcpp::Time last_stamp_;
  double vx_f_{0}, vy_f_{0}, wz_f_{0};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EeToBase>());
  rclcpp::shutdown();
  return 0;
}
