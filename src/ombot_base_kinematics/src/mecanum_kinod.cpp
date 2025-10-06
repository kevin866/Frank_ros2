#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <array>
#include <cmath>
#include "ombot_base_kinematics/mecanum_kinod.hpp"

class MecanumKinOdom : public rclcpp::Node
{
public:
  MecanumKinOdom() : Node("mecanum_kinod")
  {
    // Params
    wheel_radius_ = declare_parameter("wheel_radius", 0.0762);
    Lx_ = declare_parameter("Lx", 0.20); // half-length
    Ly_ = declare_parameter("Ly", 0.17); // half-width
    cmd_topic_ = declare_parameter<std::string>("cmd_topic", "/wheel_velocity_controller/commands");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    publish_tf_ = declare_parameter("publish_tf", true);

    // I/O
    sub_twist_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&MecanumKinOdom::twistCb, this, std::placeholders::_1));

    pub_cmd_ = create_publisher<std_msgs::msg::Float64MultiArray>(cmd_topic_, 10);
    pub_odom_ = create_publisher<nav_msgs::msg::Odometry>("odom", 50);

    if (publish_tf_) tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Timer for odom integration @ 100 Hz
    timer_ = create_wall_timer(std::chrono::milliseconds(10), std::bind(&MecanumKinOdom::onTimer, this));

    RCLCPP_INFO(get_logger(), "MecanumKinOdom started (r=%.3f, Lx=%.3f, Ly=%.3f)", wheel_radius_, Lx_, Ly_);
  }

private:
  // Params
  double wheel_radius_, Lx_, Ly_;
  std::string cmd_topic_, odom_frame_, base_frame_;
  bool publish_tf_;

  // State
  std::array<double,4> last_w_rad_s_{0,0,0,0}; // if you later read encoder speeds, set these from a sub
  double x_{0}, y_{0}, yaw_{0};

  // ROS
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_cmd_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_t_;

  void twistCb(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    const double vx = msg->linear.x;
    const double vy = msg->linear.y;
    const double wz = msg->angular.z;

    // ω = (1/r) * M * [vx vy wz]^T
    const double sumL = (Lx_ + Ly_);
    const double s = 1.0 / wheel_radius_;

    // FL, FR, RL, RR (make sure your joint order matches controllers.yaml)
    std::array<double,4> w{
      s*(+vx - vy - sumL*wz),
      s*(+vx + vy + sumL*wz),
      s*(+vx + vy - sumL*wz),
      s*(+vx - vy + sumL*wz)
    };

    std_msgs::msg::Float64MultiArray cmd;
    cmd.data.assign(w.begin(), w.end());
    pub_cmd_->publish(cmd);

    last_w_rad_s_ = w; // used for dead-reckon odom if you don’t yet read encoder speeds
  }

  void onTimer()
  {
    // Integrate odom from wheel speeds (dead-reckon). Replace with encoder-derived speeds when available.
    const rclcpp::Time now = now_or_steady();
    if (last_t_.nanoseconds() == 0) { last_t_ = now; return; }
    const double dt = (now - last_t_).seconds();
    last_t_ = now;

    const double sumL = (Lx_ + Ly_);
    // Inverse kinematics: [vx vy wz]^T = (r/4) * M^{-1} * ω
    const double r = wheel_radius_;
    const double vx = (r/4.0) * ( last_w_rad_s_[0] + last_w_rad_s_[1] + last_w_rad_s_[2] + last_w_rad_s_[3] );
    const double vy = (r/4.0) * ( -last_w_rad_s_[0] + last_w_rad_s_[1] + last_w_rad_s_[2] - last_w_rad_s_[3] );
    const double wz = (r/(4.0*sumL)) * ( -last_w_rad_s_[0] + last_w_rad_s_[1] - last_w_rad_s_[2] + last_w_rad_s_[3] );

    // Integrate in world
    const double c = std::cos(yaw_), s = std::sin(yaw_);
    x_   += ( c*vx - s*vy) * dt;
    y_   += ( s*vx + c*vy) * dt;
    yaw_ += wz * dt;

    publishOdom(now, vx, vy, wz);
  }

  rclcpp::Time now_or_steady() const { return this->now(); }

  void publishOdom(const rclcpp::Time& stamp, double vx, double vy, double wz)
  {
    nav_msgs::msg::Odometry od;
    od.header.stamp = stamp;
    od.header.frame_id = odom_frame_;
    od.child_frame_id = base_frame_;
    od.pose.pose.position.x = x_;
    od.pose.pose.position.y = y_;
    tf2::Quaternion q; q.setRPY(0,0,yaw_);
    od.pose.pose.orientation.x = q.x();
    od.pose.pose.orientation.y = q.y();
    od.pose.pose.orientation.z = q.z();
    od.pose.pose.orientation.w = q.w();
    od.twist.twist.linear.x = vx;
    od.twist.twist.linear.y = vy;
    od.twist.twist.angular.z = wz;

    pub_odom_->publish(od);

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = stamp;
      tf.header.frame_id = odom_frame_;
      tf.child_frame_id = base_frame_;
      tf.transform.translation.x = x_;
      tf.transform.translation.y = y_;
      tf.transform.translation.z = 0.0;
      tf.transform.rotation.x = q.x();
      tf.transform.rotation.y = q.y();
      tf.transform.rotation.z = q.z();
      tf.transform.rotation.w = q.w();
      tf_broadcaster_->sendTransform(tf);
    }
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MecanumKinOdom>());
  rclcpp::shutdown();
  return 0;
}
