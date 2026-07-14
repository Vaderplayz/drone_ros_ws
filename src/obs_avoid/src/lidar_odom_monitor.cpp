#include <chrono>
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
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
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
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan_rf2o");
    raw_topic_ = declare_parameter<std::string>("raw_odom_topic", "/lidar/odom_raw");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/mavros/imu/data");
    output_topic_ = declare_parameter<std::string>("output_odom_topic", "/lidar/odom");
    diagnostics_topic_ = declare_parameter<std::string>(
      "diagnostics_topic", "/lidar_odom/diagnostics");
    expected_parent_frame_ = declare_parameter<std::string>("expected_parent_frame", "lidar_odom");
    expected_child_frame_ = declare_parameter<std::string>("expected_child_frame", "base_footprint");
    expected_scan_bins_ = declare_parameter<int>("expected_scan_bins", 720);
    min_valid_beam_ratio_ = declare_parameter<double>("min_valid_beam_ratio", 0.05);
    min_scan_rate_hz_ = declare_parameter<double>("min_scan_rate_hz", 1.0);
    scan_timeout_sec_ = declare_parameter<double>("scan_timeout_sec", 0.75);
    odom_timeout_sec_ = declare_parameter<double>("odom_timeout_sec", 0.75);
    imu_timeout_sec_ = declare_parameter<double>("imu_timeout_sec", 0.75);
    max_position_jump_m_ = declare_parameter<double>("max_position_jump_m", 0.50);
    max_yaw_jump_rad_ = declare_parameter<double>("max_yaw_jump_rad", 0.50);
    max_abs_z_m_ = declare_parameter<double>("max_abs_z_m", 0.05);
    max_rf2o_roll_pitch_rad_ = declare_parameter<double>("max_rf2o_roll_pitch_rad", 0.10);
    max_vehicle_roll_pitch_rad_ = declare_parameter<double>("max_vehicle_roll_pitch_rad", 0.35);

    output_pub_ = create_publisher<nav_msgs::msg::Odometry>(output_topic_, rclcpp::QoS(10));
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      diagnostics_topic_, rclcpp::QoS(10));
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LidarOdomMonitor::scan_callback, this, std::placeholders::_1));
    raw_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      raw_topic_, rclcpp::QoS(10).best_effort(),
      std::bind(&LidarOdomMonitor::odom_callback, this, std::placeholders::_1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LidarOdomMonitor::imu_callback, this, std::placeholders::_1));
    diagnostics_timer_ = create_wall_timer(
      std::chrono::milliseconds(200), std::bind(&LidarOdomMonitor::publish_diagnostics, this));

    transition("WAITING_FOR_SCAN");
    RCLCPP_INFO(
      get_logger(), "Monitoring canonical scan %s and RF2O %s -> %s",
      scan_topic_.c_str(), raw_topic_.c_str(), output_topic_.c_str());
  }

private:
  double age(const rclcpp::Time & receipt) const
  {
    return receipt.nanoseconds() > 0 ? (now() - receipt).seconds() :
      std::numeric_limits<double>::infinity();
  }

  void update_rate(const rclcpp::Time & receipt, rclcpp::Time & previous, double & rate)
  {
    if (previous.nanoseconds() > 0) {
      const double period = (receipt - previous).seconds();
      if (period > 0.0 && std::isfinite(period)) {
        const double instant = 1.0 / period;
        rate = rate > 0.0 ? 0.8 * rate + 0.2 * instant : instant;
      }
    }
    previous = receipt;
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    const auto receipt = now();
    update_rate(receipt, previous_scan_receipt_, scan_rate_hz_);
    scan_receipt_ = receipt;
    scan_length_ = msg->ranges.size();
    const rclcpp::Time stamp(msg->header.stamp);
    scan_timestamp_monotonic_ = stamp.nanoseconds() > 0 &&
      (!have_scan_stamp_ || stamp > last_scan_stamp_);
    if (scan_timestamp_monotonic_) {
      last_scan_stamp_ = stamp;
      have_scan_stamp_ = true;
    }

    std::size_t valid_beams = 0;
    for (const float range : msg->ranges) {
      if (std::isfinite(range) && range >= msg->range_min && range <= msg->range_max) {
        ++valid_beams;
      }
    }
    valid_beam_ratio_ = msg->ranges.empty() ? 0.0 :
      static_cast<double>(valid_beams) / static_cast<double>(msg->ranges.size());
    scan_valid_ = scan_length_ == static_cast<std::size_t>(expected_scan_bins_) &&
      scan_timestamp_monotonic_ && valid_beam_ratio_ >= min_valid_beam_ratio_;
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    const auto & q = msg->orientation;
    tf2::Quaternion quaternion(q.x, q.y, q.z, q.w);
    if (!std::isfinite(quaternion.length2()) ||
      quaternion.length2() < std::numeric_limits<double>::epsilon())
    {
      imu_valid_ = false;
      return;
    }
    quaternion.normalize();
    double yaw = 0.0;
    tf2::Matrix3x3(quaternion).getRPY(imu_roll_, imu_pitch_, yaw);
    imu_valid_ = std::isfinite(imu_roll_) && std::isfinite(imu_pitch_) &&
      std::abs(imu_roll_) <= max_vehicle_roll_pitch_rad_ &&
      std::abs(imu_pitch_) <= max_vehicle_roll_pitch_rad_;
    imu_receipt_ = now();
  }

  bool scan_health_ok() const
  {
    return scan_valid_ && age(scan_receipt_) <= scan_timeout_sec_ &&
      scan_rate_hz_ >= min_scan_rate_hz_;
  }

  bool imu_health_ok() const
  {
    return imu_valid_ && age(imu_receipt_) <= imu_timeout_sec_;
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    odom_receipt_ = now();
    if (msg->header.frame_id != expected_parent_frame_ ||
      msg->child_frame_id != expected_child_frame_)
    {
      odom_valid_ = false;
      odom_reason_ = "UNEXPECTED_ODOM_FRAME";
      return;
    }
    if (!finite_pose(*msg)) {
      odom_valid_ = false;
      odom_reason_ = "NONFINITE_ODOM";
      return;
    }

    const rclcpp::Time stamp(msg->header.stamp);
    if (stamp.nanoseconds() <= 0 || (have_odom_stamp_ && stamp <= last_odom_stamp_)) {
      reset_odom_history("NONMONOTONIC_ODOM_TIMESTAMP");
      return;
    }

    const auto & orientation = msg->pose.pose.orientation;
    tf2::Quaternion quaternion(orientation.x, orientation.y, orientation.z, orientation.w);
    if (quaternion.length2() < std::numeric_limits<double>::epsilon()) {
      odom_valid_ = false;
      odom_reason_ = "INVALID_ODOM_QUATERNION";
      return;
    }
    quaternion.normalize();
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
    if (std::abs(roll) > max_rf2o_roll_pitch_rad_ ||
      std::abs(pitch) > max_rf2o_roll_pitch_rad_ ||
      std::abs(msg->pose.pose.position.z) > max_abs_z_m_)
    {
      odom_valid_ = false;
      odom_reason_ = "NONPLANAR_RF2O_ODOM";
      return;
    }

    const double x = msg->pose.pose.position.x;
    const double y = msg->pose.pose.position.y;
    if (have_pose_) {
      const double position_jump = std::hypot(x - last_x_, y - last_y_);
      const double yaw_jump = std::abs(wrap_pi(yaw - last_yaw_));
      if (position_jump > max_position_jump_m_ || yaw_jump > max_yaw_jump_rad_) {
        reset_odom_history(position_jump > max_position_jump_m_ ?
          "RF2O_POSITION_JUMP" : "RF2O_YAW_JUMP");
        return;
      }
    }

    last_odom_stamp_ = stamp;
    have_odom_stamp_ = true;
    last_x_ = x;
    last_y_ = y;
    last_yaw_ = yaw;
    have_pose_ = true;
    odom_valid_ = true;
    odom_reason_ = "OK";

    if (!scan_health_ok() || !imu_health_ok()) {
      return;
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
  }

  void reset_odom_history(const std::string & reason)
  {
    have_pose_ = false;
    have_odom_stamp_ = false;
    odom_valid_ = false;
    odom_reason_ = reason;
    RCLCPP_ERROR(get_logger(), "RF2O rejected: %s; monitor history reset", reason.c_str());
  }

  std::string current_state() const
  {
    if (scan_receipt_.nanoseconds() <= 0) {
      return "WAITING_FOR_SCAN";
    }
    if (age(scan_receipt_) > scan_timeout_sec_) {
      return "STALE_CANONICAL_SCAN";
    }
    if (!scan_timestamp_monotonic_) {
      return "NONMONOTONIC_SCAN_TIMESTAMP";
    }
    if (scan_length_ != static_cast<std::size_t>(expected_scan_bins_)) {
      return "INVALID_CANONICAL_SCAN_WIDTH";
    }
    if (valid_beam_ratio_ < min_valid_beam_ratio_) {
      return "LOW_VALID_BEAM_RATIO";
    }
    if (scan_rate_hz_ < min_scan_rate_hz_) {
      return "LOW_CANONICAL_SCAN_RATE";
    }
    if (imu_receipt_.nanoseconds() <= 0) {
      return "WAITING_FOR_IMU";
    }
    if (age(imu_receipt_) > imu_timeout_sec_) {
      return "STALE_IMU";
    }
    if (!imu_valid_) {
      return "UNSAFE_ROLL_PITCH";
    }
    if (odom_receipt_.nanoseconds() <= 0) {
      return "WAITING_FOR_ODOM";
    }
    if (age(odom_receipt_) > odom_timeout_sec_) {
      return "STALE_RF2O_ODOM";
    }
    return odom_valid_ ? "OK" : odom_reason_;
  }

  void transition(const std::string & state)
  {
    if (state == state_) {
      return;
    }
    if (state == "OK") {
      RCLCPP_INFO(get_logger(), "RF2O monitor state: OK");
    } else {
      RCLCPP_WARN(get_logger(), "RF2O monitor state: %s", state.c_str());
    }
    state_ = state;
  }

  void publish_diagnostics()
  {
    transition(current_state());
    const bool healthy = state_ == "OK";
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.level = healthy ? diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.name = "lidar_odom_monitor";
    status.hardware_id = "rf2o_laser_odometry";
    status.message = state_;
    status.values.push_back(key_value("valid", healthy ? "true" : "false"));
    status.values.push_back(key_value("state", state_));
    status.values.push_back(key_value("scan_topic", scan_topic_));
    status.values.push_back(key_value("scan_age_sec", std::to_string(age(scan_receipt_))));
    status.values.push_back(key_value("scan_rate_hz", std::to_string(scan_rate_hz_)));
    status.values.push_back(key_value("scan_length", std::to_string(scan_length_)));
    status.values.push_back(key_value("valid_beam_ratio", std::to_string(valid_beam_ratio_)));
    status.values.push_back(key_value(
      "scan_timestamp_monotonic", scan_timestamp_monotonic_ ? "true" : "false"));
    status.values.push_back(key_value("odom_age_sec", std::to_string(age(odom_receipt_))));
    status.values.push_back(key_value("odom_valid", odom_valid_ ? "true" : "false"));
    status.values.push_back(key_value("imu_age_sec", std::to_string(age(imu_receipt_))));
    status.values.push_back(key_value("imu_roll_rad", std::to_string(imu_roll_)));
    status.values.push_back(key_value("imu_pitch_rad", std::to_string(imu_pitch_)));
    status.values.push_back(key_value("parent_frame", expected_parent_frame_));
    status.values.push_back(key_value("child_frame", expected_child_frame_));
    array.status.push_back(status);
    diagnostics_pub_->publish(array);
  }

  std::string scan_topic_;
  std::string raw_topic_;
  std::string imu_topic_;
  std::string output_topic_;
  std::string diagnostics_topic_;
  std::string expected_parent_frame_;
  std::string expected_child_frame_;
  int expected_scan_bins_{720};
  double min_valid_beam_ratio_{0.05};
  double min_scan_rate_hz_{1.0};
  double scan_timeout_sec_{0.75};
  double odom_timeout_sec_{0.75};
  double imu_timeout_sec_{0.75};
  double max_position_jump_m_{0.5};
  double max_yaw_jump_rad_{0.5};
  double max_abs_z_m_{0.05};
  double max_rf2o_roll_pitch_rad_{0.1};
  double max_vehicle_roll_pitch_rad_{0.35};

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr raw_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr output_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;

  rclcpp::Time scan_receipt_{0, 0, RCL_ROS_TIME};
  rclcpp::Time previous_scan_receipt_{0, 0, RCL_ROS_TIME};
  rclcpp::Time odom_receipt_{0, 0, RCL_ROS_TIME};
  rclcpp::Time imu_receipt_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_scan_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_odom_stamp_{0, 0, RCL_ROS_TIME};
  bool have_scan_stamp_{false};
  bool have_odom_stamp_{false};
  bool have_pose_{false};
  bool scan_valid_{false};
  bool scan_timestamp_monotonic_{true};
  bool odom_valid_{false};
  bool imu_valid_{false};
  std::size_t scan_length_{0};
  double scan_rate_hz_{0.0};
  double valid_beam_ratio_{0.0};
  double imu_roll_{0.0};
  double imu_pitch_{0.0};
  double last_x_{0.0};
  double last_y_{0.0};
  double last_yaw_{0.0};
  std::string odom_reason_{"WAITING_FOR_ODOM"};
  std::string state_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarOdomMonitor>());
  rclcpp::shutdown();
  return 0;
}
