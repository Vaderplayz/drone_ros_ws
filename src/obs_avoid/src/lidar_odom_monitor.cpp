#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace
{
double wrap_pi(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

bool finite_pose(const nav_msgs::msg::Odometry & msg)
{
  const auto & p = msg.pose.pose.position;
  const auto & q = msg.pose.pose.orientation;
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
         std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w);
}

diagnostic_msgs::msg::KeyValue key_value(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  return item;
}
}  // namespace

class LidarOdomMonitor : public rclcpp::Node
{
public:
  LidarOdomMonitor()
  : Node("lidar_odom_monitor")
  {
    raw_topic_ = declare_parameter<std::string>("raw_odom_topic", "/lidar/odom_raw");
    output_topic_ = declare_parameter<std::string>("output_odom_topic", "/lidar/odom");
    diagnostics_topic_ = declare_parameter<std::string>(
      "diagnostics_topic", "/lidar_odom/diagnostics");
    expected_parent_frame_ = declare_parameter<std::string>("expected_parent_frame", "lidar_odom");
    expected_child_frame_ = declare_parameter<std::string>("expected_child_frame", "base_footprint");
    odom_timeout_sec_ = declare_parameter<double>("odom_timeout_sec", 0.50);
    max_position_jump_m_ = declare_parameter<double>("max_position_jump_m", 0.50);
    max_yaw_jump_rad_ = declare_parameter<double>("max_yaw_jump_rad", 0.50);
    max_abs_z_m_ = declare_parameter<double>("max_abs_z_m", 0.05);
    max_roll_pitch_rad_ = declare_parameter<double>("max_roll_pitch_rad", 0.10);

    output_pub_ = create_publisher<nav_msgs::msg::Odometry>(output_topic_, rclcpp::QoS(10));
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      diagnostics_topic_, rclcpp::QoS(10));
    raw_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      raw_topic_, rclcpp::QoS(10).best_effort(),
      std::bind(&LidarOdomMonitor::odom_callback, this, std::placeholders::_1));
    diagnostics_timer_ = create_wall_timer(
      std::chrono::milliseconds(200), std::bind(&LidarOdomMonitor::publish_diagnostics, this));

    set_status(false, "waiting_for_rf2o");
    RCLCPP_INFO(
      get_logger(), "Monitoring %s -> %s (%s -> %s)", raw_topic_.c_str(), output_topic_.c_str(),
      expected_parent_frame_.c_str(), expected_child_frame_.c_str());
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const auto receipt_time = now();
    last_receipt_time_ = receipt_time;

    if (msg->header.frame_id != expected_parent_frame_ ||
      msg->child_frame_id != expected_child_frame_)
    {
      set_status(false, "unexpected_frame");
      return;
    }

    if (!finite_pose(*msg)) {
      set_status(false, "nonfinite_pose");
      return;
    }

    const rclcpp::Time stamp(msg->header.stamp);
    if (stamp.nanoseconds() <= 0) {
      set_status(false, "invalid_timestamp");
      return;
    }
    if (have_stamp_ && stamp <= last_stamp_) {
      reset_history("nonmonotonic_timestamp");
      return;
    }

    const auto & orientation = msg->pose.pose.orientation;
    tf2::Quaternion quaternion(
      orientation.x, orientation.y, orientation.z, orientation.w);
    if (quaternion.length2() < std::numeric_limits<double>::epsilon()) {
      set_status(false, "invalid_quaternion");
      return;
    }
    quaternion.normalize();

    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
    if (std::abs(roll) > max_roll_pitch_rad_ || std::abs(pitch) > max_roll_pitch_rad_ ||
      std::abs(msg->pose.pose.position.z) > max_abs_z_m_)
    {
      set_status(false, "nonplanar_rf2o_pose");
      return;
    }

    const double x = msg->pose.pose.position.x;
    const double y = msg->pose.pose.position.y;
    if (have_pose_) {
      const double position_jump = std::hypot(x - last_x_, y - last_y_);
      const double yaw_jump = std::abs(wrap_pi(yaw - last_yaw_));
      if (position_jump > max_position_jump_m_ || yaw_jump > max_yaw_jump_rad_) {
        reset_history(position_jump > max_position_jump_m_ ? "position_jump" : "yaw_jump");
        return;
      }
    }

    nav_msgs::msg::Odometry output = *msg;
    tf2::Quaternion yaw_only;
    yaw_only.setRPY(0.0, 0.0, yaw);
    yaw_only.normalize();
    output.pose.pose.position.z = 0.0;
    output.pose.pose.orientation.x = 0.0;
    output.pose.pose.orientation.y = 0.0;
    output.pose.pose.orientation.z = yaw_only.z();
    output.pose.pose.orientation.w = yaw_only.w();
    output_pub_->publish(output);

    last_stamp_ = stamp;
    have_stamp_ = true;
    last_x_ = x;
    last_y_ = y;
    last_yaw_ = yaw;
    have_pose_ = true;
    valid_ = true;
    reason_ = "OK";
  }

  void reset_history(const std::string & reason)
  {
    have_pose_ = false;
    have_stamp_ = false;
    set_status(false, reason);
    RCLCPP_ERROR(get_logger(), "RF2O rejected: %s; monitor history reset", reason.c_str());
  }

  void set_status(bool valid, const std::string & reason)
  {
    if (reason != reason_) {
      RCLCPP_WARN(get_logger(), "RF2O monitor state: %s", reason.c_str());
    }
    valid_ = valid;
    reason_ = reason;
  }

  void publish_diagnostics()
  {
    double age = std::numeric_limits<double>::infinity();
    if (last_receipt_time_.nanoseconds() > 0) {
      age = (now() - last_receipt_time_).seconds();
    }
    if (age > odom_timeout_sec_) {
      valid_ = false;
      reason_ = "rf2o_stale";
    }

    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.level = valid_ ? diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.name = "lidar_odom_monitor";
    status.hardware_id = "rf2o_laser_odometry";
    status.message = valid_ ? "OK" : reason_;
    status.values.push_back(key_value("valid", valid_ ? "true" : "false"));
    status.values.push_back(key_value("reason", reason_));
    status.values.push_back(key_value("age_sec", std::to_string(age)));
    status.values.push_back(key_value("parent_frame", expected_parent_frame_));
    status.values.push_back(key_value("child_frame", expected_child_frame_));
    array.status.push_back(status);
    diagnostics_pub_->publish(array);
  }

  std::string raw_topic_;
  std::string output_topic_;
  std::string diagnostics_topic_;
  std::string expected_parent_frame_;
  std::string expected_child_frame_;
  double odom_timeout_sec_{0.5};
  double max_position_jump_m_{0.5};
  double max_yaw_jump_rad_{0.5};
  double max_abs_z_m_{0.05};
  double max_roll_pitch_rad_{0.1};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr raw_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr output_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;

  rclcpp::Time last_receipt_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
  bool valid_{false};
  bool have_stamp_{false};
  bool have_pose_{false};
  double last_x_{0.0};
  double last_y_{0.0};
  double last_yaw_{0.0};
  std::string reason_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarOdomMonitor>());
  rclcpp::shutdown();
  return 0;
}
