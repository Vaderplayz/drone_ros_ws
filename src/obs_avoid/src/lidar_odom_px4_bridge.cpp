#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace
{
constexpr double kRadToDeg = 180.0 / M_PI;

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

bool extract_rpy(const nav_msgs::msg::Odometry & msg, double & roll, double & pitch, double & yaw)
{
  const auto & q = msg.pose.pose.orientation;
  tf2::Quaternion quaternion(q.x, q.y, q.z, q.w);
  if (!std::isfinite(quaternion.length2()) ||
    quaternion.length2() < std::numeric_limits<double>::epsilon())
  {
    return false;
  }
  quaternion.normalize();
  tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
  return std::isfinite(roll) && std::isfinite(pitch) && std::isfinite(yaw);
}

diagnostic_msgs::msg::KeyValue key_value(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  return item;
}

std::string bool_string(bool value)
{
  return value ? "true" : "false";
}
}  // namespace

class LidarOdomPx4Bridge : public rclcpp::Node
{
public:
  LidarOdomPx4Bridge()
  : Node("lidar_odom_px4_bridge")
  {
    lidar_odom_topic_ = declare_parameter<std::string>("lidar_odom_topic", "/lidar/odom");
    monitor_diagnostics_topic_ = declare_parameter<std::string>(
      "monitor_diagnostics_topic", "/lidar_odom/diagnostics");
    px4_odom_topic_ = declare_parameter<std::string>(
      "px4_odom_topic", "/mavros/local_position/odom");
    mavros_state_topic_ = declare_parameter<std::string>("mavros_state_topic", "/mavros/state");
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    output_topic_ = declare_parameter<std::string>("output_topic", "/mavros/odometry/out");
    diagnostics_topic_ = declare_parameter<std::string>(
      "diagnostics_topic", "/lidar_odom_px4_bridge/diagnostics");
    expected_lidar_frame_ = declare_parameter<std::string>("expected_lidar_frame", "lidar_odom");
    output_parent_frame_ = declare_parameter<std::string>("output_parent_frame", "odom");
    output_child_frame_ = declare_parameter<std::string>("output_child_frame", "base_link");
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 10.0);
    odom_timeout_sec_ = declare_parameter<double>("odom_timeout_sec", 0.50);
    scan_timeout_sec_ = declare_parameter<double>("scan_timeout_sec", 0.50);
    diagnostic_timeout_sec_ = declare_parameter<double>("diagnostic_timeout_sec", 1.0);
    px4_odom_timeout_sec_ = declare_parameter<double>("px4_odom_timeout_sec", 0.50);
    require_disarmed_for_alignment_ = declare_parameter<bool>(
      "require_disarmed_for_alignment", true);
    horizontal_position_stddev_m_ = declare_parameter<double>(
      "horizontal_position_stddev_m", 0.30);
    yaw_stddev_rad_ = declare_parameter<double>("yaw_stddev_rad", 0.20);
    unused_axis_variance_ = declare_parameter<double>("unused_axis_variance", 1000000.0);
    max_roll_pitch_rad_ = declare_parameter<double>("max_roll_pitch_rad", 0.35);
    max_position_jump_m_ = declare_parameter<double>("max_position_jump_m", 0.50);
    max_yaw_jump_rad_ = declare_parameter<double>("max_yaw_jump_rad", 0.50);
    health_csv_path_ = declare_parameter<std::string>("health_csv_path", "");

    validate_parameters();

    output_pub_ = create_publisher<nav_msgs::msg::Odometry>(output_topic_, rclcpp::QoS(10));
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      diagnostics_topic_, rclcpp::QoS(10));
    lidar_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      lidar_odom_topic_, rclcpp::QoS(10).best_effort(),
      std::bind(&LidarOdomPx4Bridge::lidar_odom_callback, this, std::placeholders::_1));
    monitor_diagnostics_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      monitor_diagnostics_topic_, rclcpp::QoS(10),
      std::bind(&LidarOdomPx4Bridge::monitor_diagnostics_callback, this, std::placeholders::_1));
    px4_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      px4_odom_topic_, rclcpp::QoS(10).best_effort(),
      std::bind(&LidarOdomPx4Bridge::px4_odom_callback, this, std::placeholders::_1));
    mavros_state_sub_ = create_subscription<mavros_msgs::msg::State>(
      mavros_state_topic_, rclcpp::QoS(10),
      std::bind(&LidarOdomPx4Bridge::mavros_state_callback, this, std::placeholders::_1));
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LidarOdomPx4Bridge::scan_callback, this, std::placeholders::_1));

    const auto publish_period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    publish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(publish_period),
      std::bind(&LidarOdomPx4Bridge::publish_tick, this));
    diagnostics_timer_ = create_wall_timer(
      std::chrono::milliseconds(200), std::bind(&LidarOdomPx4Bridge::publish_diagnostics, this));
    health_timer_ = create_wall_timer(
      std::chrono::seconds(1), std::bind(&LidarOdomPx4Bridge::write_health_row, this));

    open_health_csv();
    set_reason("waiting_for_inputs");
    RCLCPP_INFO(
      get_logger(), "LiDAR external odometry bridge: %s -> %s (%s -> %s)",
      lidar_odom_topic_.c_str(), output_topic_.c_str(), output_parent_frame_.c_str(),
      output_child_frame_.c_str());
    RCLCPP_INFO(
      get_logger(), "Covariance: xy_stddev=%.3f m yaw_stddev=%.3f rad unused_variance=%.1f",
      horizontal_position_stddev_m_, yaw_stddev_rad_, unused_axis_variance_);
  }

private:
  void validate_parameters()
  {
    if (publish_rate_hz_ <= 0.0 || odom_timeout_sec_ <= 0.0 || scan_timeout_sec_ <= 0.0 ||
      diagnostic_timeout_sec_ <= 0.0 || px4_odom_timeout_sec_ <= 0.0 ||
      horizontal_position_stddev_m_ <= 0.0 || yaw_stddev_rad_ <= 0.0 ||
      unused_axis_variance_ <= 0.0 || max_roll_pitch_rad_ <= 0.0 ||
      max_position_jump_m_ <= 0.0 || max_yaw_jump_rad_ <= 0.0)
    {
      throw std::invalid_argument("LiDAR odometry bridge parameters must be positive");
    }
    if (output_parent_frame_ != "odom" || output_child_frame_ != "base_link") {
      throw std::invalid_argument("Output frames must be exactly odom -> base_link");
    }
  }

  void open_health_csv()
  {
    if (health_csv_path_.empty()) {
      RCLCPP_WARN(get_logger(), "health_csv_path is empty; CSV health logging is disabled");
      return;
    }
    health_csv_.open(health_csv_path_, std::ios::out | std::ios::app);
    if (!health_csv_) {
      RCLCPP_ERROR(get_logger(), "Cannot open health CSV: %s", health_csv_path_.c_str());
      return;
    }
    health_csv_.seekp(0, std::ios::end);
    if (health_csv_.tellp() == 0) {
      health_csv_ << "timestamp,mavros_connected,px4_armed,px4_mode,alignment_complete,"
        "rf2o_valid,rf2o_age_sec,aligned_x,aligned_y,aligned_yaw_deg,px4_x,px4_y,"
        "px4_yaw_deg,position_difference_m,yaw_difference_deg,publishing_to_px4,"
        "rejection_reason\n";
      health_csv_.flush();
    }
  }

  void lidar_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const rclcpp::Time stamp(msg->header.stamp);
    if (msg->header.frame_id != expected_lidar_frame_ || !finite_pose(*msg) ||
      stamp.nanoseconds() <= 0)
    {
      invalidate_alignment("invalid_lidar_odometry");
      return;
    }
    if (have_lidar_stamp_ && stamp <= last_lidar_stamp_) {
      invalidate_alignment("nonmonotonic_lidar_timestamp");
      return;
    }

    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    if (!extract_rpy(*msg, roll, pitch, yaw)) {
      invalidate_alignment("invalid_lidar_quaternion");
      return;
    }

    if (have_lidar_pose_) {
      const double jump = std::hypot(
        msg->pose.pose.position.x - last_lidar_x_, msg->pose.pose.position.y - last_lidar_y_);
      const double yaw_jump = std::abs(wrap_pi(yaw - last_lidar_yaw_));
      if (jump > max_position_jump_m_ || yaw_jump > max_yaw_jump_rad_) {
        invalidate_alignment(jump > max_position_jump_m_ ? "lidar_position_jump" : "lidar_yaw_jump");
        have_lidar_pose_ = false;
        have_lidar_stamp_ = false;
        return;
      }
    }

    latest_lidar_odom_ = *msg;
    lidar_receipt_time_ = now();
    last_lidar_stamp_ = stamp;
    have_lidar_stamp_ = true;
    last_lidar_x_ = msg->pose.pose.position.x;
    last_lidar_y_ = msg->pose.pose.position.y;
    last_lidar_yaw_ = yaw;
    have_lidar_pose_ = true;
    have_lidar_odom_ = true;
    ++lidar_sequence_;
  }

  void monitor_diagnostics_callback(
    const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg)
  {
    bool found = false;
    bool ok = false;
    std::string monitor_message = "monitor_status_missing";
    for (const auto & status : msg->status) {
      if (status.name == "lidar_odom_monitor") {
        found = true;
        ok = status.level == diagnostic_msgs::msg::DiagnosticStatus::OK;
        monitor_message = status.message;
        break;
      }
    }
    monitor_ok_ = found && ok;
    monitor_reason_ = monitor_message;
    monitor_diagnostics_receipt_time_ = now();
    have_monitor_diagnostics_ = found;
    if (found && !ok && alignment_complete_) {
      invalidate_alignment("rf2o_monitor_" + monitor_message);
    }
  }

  void px4_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!finite_pose(*msg)) {
      have_px4_odom_ = false;
      return;
    }
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    if (!extract_rpy(*msg, roll, pitch, yaw)) {
      have_px4_odom_ = false;
      return;
    }
    latest_px4_odom_ = *msg;
    px4_roll_ = roll;
    px4_pitch_ = pitch;
    px4_yaw_ = yaw;
    px4_odom_receipt_time_ = now();
    have_px4_odom_ = true;
  }

  void mavros_state_callback(const mavros_msgs::msg::State::SharedPtr msg)
  {
    mavros_connected_ = msg->connected;
    px4_armed_ = msg->armed;
    px4_mode_ = msg->mode;
    have_mavros_state_ = true;
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr)
  {
    scan_receipt_time_ = now();
    have_scan_ = true;
  }

  double age_sec(const rclcpp::Time & receipt_time) const
  {
    if (receipt_time.nanoseconds() <= 0) {
      return std::numeric_limits<double>::infinity();
    }
    return (now() - receipt_time).seconds();
  }

  bool inputs_ready(std::string & reason) const
  {
    if (!have_mavros_state_ || !mavros_connected_) {
      reason = "mavros_disconnected";
      return false;
    }
    if (!have_lidar_odom_ || age_sec(lidar_receipt_time_) > odom_timeout_sec_) {
      reason = "lidar_odom_stale";
      return false;
    }
    if (!have_scan_ || age_sec(scan_receipt_time_) > scan_timeout_sec_) {
      reason = "scan_stale";
      return false;
    }
    if (!have_monitor_diagnostics_ ||
      age_sec(monitor_diagnostics_receipt_time_) > diagnostic_timeout_sec_)
    {
      reason = "monitor_diagnostics_stale";
      return false;
    }
    if (!monitor_ok_) {
      reason = "rf2o_monitor_" + monitor_reason_;
      return false;
    }
    if (!have_px4_odom_ || age_sec(px4_odom_receipt_time_) > px4_odom_timeout_sec_) {
      reason = "px4_odom_stale";
      return false;
    }
    if (std::abs(px4_roll_) > max_roll_pitch_rad_ ||
      std::abs(px4_pitch_) > max_roll_pitch_rad_)
    {
      reason = "roll_pitch_limit";
      return false;
    }
    reason = "none";
    return true;
  }

  bool try_alignment(std::string & reason)
  {
    if (!inputs_ready(reason)) {
      return false;
    }
    if (require_disarmed_for_alignment_ && px4_armed_) {
      reason = "vehicle_armed_alignment_required";
      return false;
    }

    double lidar_roll = 0.0;
    double lidar_pitch = 0.0;
    double lidar_yaw = 0.0;
    if (!extract_rpy(latest_lidar_odom_, lidar_roll, lidar_pitch, lidar_yaw)) {
      reason = "invalid_lidar_quaternion";
      return false;
    }

    alignment_yaw_ = wrap_pi(px4_yaw_ - lidar_yaw);
    const double c = std::cos(alignment_yaw_);
    const double s = std::sin(alignment_yaw_);
    const double lidar_x = latest_lidar_odom_.pose.pose.position.x;
    const double lidar_y = latest_lidar_odom_.pose.pose.position.y;
    alignment_x_ = latest_px4_odom_.pose.pose.position.x - (c * lidar_x - s * lidar_y);
    alignment_y_ = latest_px4_odom_.pose.pose.position.y - (s * lidar_x + c * lidar_y);
    alignment_complete_ = true;
    alignment_time_ = now();
    last_published_sequence_ = 0;
    reason = "none";
    RCLCPP_INFO(
      get_logger(), "Alignment complete while disarmed: x=%.4f y=%.4f yaw=%.3f deg",
      alignment_x_, alignment_y_, alignment_yaw_ * kRadToDeg);
    return true;
  }

  void aligned_pose(double & x, double & y, double & yaw) const
  {
    double roll = 0.0;
    double pitch = 0.0;
    double lidar_yaw = 0.0;
    extract_rpy(latest_lidar_odom_, roll, pitch, lidar_yaw);
    const double c = std::cos(alignment_yaw_);
    const double s = std::sin(alignment_yaw_);
    const double lidar_x = latest_lidar_odom_.pose.pose.position.x;
    const double lidar_y = latest_lidar_odom_.pose.pose.position.y;
    x = alignment_x_ + c * lidar_x - s * lidar_y;
    y = alignment_y_ + s * lidar_x + c * lidar_y;
    yaw = wrap_pi(alignment_yaw_ + lidar_yaw);
  }

  void publish_tick()
  {
    std::string reason;
    if (!alignment_complete_ && !try_alignment(reason)) {
      publishing_to_px4_ = false;
      set_reason(reason);
      return;
    }
    if (!inputs_ready(reason)) {
      publishing_to_px4_ = false;
      set_reason(reason);
      return;
    }
    if (lidar_sequence_ == last_published_sequence_) {
      publishing_to_px4_ = age_sec(last_output_time_) <= (2.0 / publish_rate_hz_);
      set_reason("none");
      return;
    }

    const rclcpp::Time source_stamp(latest_lidar_odom_.header.stamp);
    if (last_output_stamp_.nanoseconds() > 0 && source_stamp <= last_output_stamp_) {
      invalidate_alignment("nonmonotonic_output_timestamp");
      return;
    }

    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    aligned_pose(x, y, yaw);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw)) {
      invalidate_alignment("nonfinite_aligned_pose");
      return;
    }

    nav_msgs::msg::Odometry output;
    output.header.stamp = latest_lidar_odom_.header.stamp;
    output.header.frame_id = output_parent_frame_;
    output.child_frame_id = output_child_frame_;
    output.pose.pose.position.x = x;
    output.pose.pose.position.y = y;
    output.pose.pose.position.z = 0.0;
    tf2::Quaternion yaw_only;
    yaw_only.setRPY(0.0, 0.0, yaw);
    yaw_only.normalize();
    output.pose.pose.orientation.x = 0.0;
    output.pose.pose.orientation.y = 0.0;
    output.pose.pose.orientation.z = yaw_only.z();
    output.pose.pose.orientation.w = yaw_only.w();

    output.pose.covariance.fill(0.0);
    output.pose.covariance[0] = horizontal_position_stddev_m_ * horizontal_position_stddev_m_;
    output.pose.covariance[7] = horizontal_position_stddev_m_ * horizontal_position_stddev_m_;
    output.pose.covariance[14] = unused_axis_variance_;
    output.pose.covariance[21] = unused_axis_variance_;
    output.pose.covariance[28] = unused_axis_variance_;
    output.pose.covariance[35] = yaw_stddev_rad_ * yaw_stddev_rad_;
    output.twist.covariance.fill(0.0);
    output.twist.covariance[0] = unused_axis_variance_;
    output.twist.covariance[7] = unused_axis_variance_;
    output.twist.covariance[14] = unused_axis_variance_;
    output.twist.covariance[21] = unused_axis_variance_;
    output.twist.covariance[28] = unused_axis_variance_;
    output.twist.covariance[35] = unused_axis_variance_;

    output_pub_->publish(output);
    last_output_stamp_ = source_stamp;
    last_output_time_ = now();
    last_published_sequence_ = lidar_sequence_;
    publishing_to_px4_ = true;
    set_reason("none");
  }

  void invalidate_alignment(const std::string & reason)
  {
    if (alignment_complete_) {
      RCLCPP_ERROR(
        get_logger(), "External odometry stopped; disarmed re-alignment required: %s",
        reason.c_str());
    }
    alignment_complete_ = false;
    publishing_to_px4_ = false;
    last_output_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    set_reason(reason);
  }

  void set_reason(const std::string & reason)
  {
    if (reason != rejection_reason_) {
      RCLCPP_WARN(get_logger(), "Bridge state: %s", reason.c_str());
      rejection_reason_ = reason;
    }
  }

  void publish_diagnostics()
  {
    std::string gate_reason;
    const bool inputs_valid = inputs_ready(gate_reason);
    const bool healthy = alignment_complete_ && inputs_valid;

    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.level = healthy ? diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.name = "lidar_odom_px4_bridge";
    status.hardware_id = "rplidar_rf2o_px4";
    status.message = healthy ? "OK" : rejection_reason_;
    status.values.push_back(key_value("mavros_connected", bool_string(mavros_connected_)));
    status.values.push_back(key_value("px4_armed", bool_string(px4_armed_)));
    status.values.push_back(key_value("px4_mode", px4_mode_));
    status.values.push_back(key_value("alignment_complete", bool_string(alignment_complete_)));
    status.values.push_back(key_value("rf2o_valid", bool_string(monitor_ok_)));
    status.values.push_back(key_value("rf2o_age_sec", std::to_string(age_sec(lidar_receipt_time_))));
    status.values.push_back(key_value("scan_age_sec", std::to_string(age_sec(scan_receipt_time_))));
    status.values.push_back(key_value("publishing_to_px4", bool_string(publishing_to_px4_)));
    status.values.push_back(key_value("rejection_reason", rejection_reason_));
    status.values.push_back(key_value("output_frame", output_parent_frame_));
    status.values.push_back(key_value("output_child_frame", output_child_frame_));
    status.values.push_back(key_value(
      "horizontal_position_stddev_m", std::to_string(horizontal_position_stddev_m_)));
    status.values.push_back(key_value("yaw_stddev_rad", std::to_string(yaw_stddev_rad_)));
    status.values.push_back(key_value("unused_axis_variance", std::to_string(unused_axis_variance_)));
    if (alignment_complete_) {
      status.values.push_back(key_value("alignment_x", std::to_string(alignment_x_)));
      status.values.push_back(key_value("alignment_y", std::to_string(alignment_y_)));
      status.values.push_back(key_value("alignment_yaw_rad", std::to_string(alignment_yaw_)));
      status.values.push_back(key_value("alignment_time", std::to_string(alignment_time_.seconds())));
    }
    array.status.push_back(status);
    diagnostics_pub_->publish(array);
  }

  std::string csv_escape(const std::string & value) const
  {
    std::string escaped = value;
    std::replace(escaped.begin(), escaped.end(), ',', ';');
    return escaped;
  }

  void write_health_row()
  {
    if (!health_csv_) {
      return;
    }

    double aligned_x = std::numeric_limits<double>::quiet_NaN();
    double aligned_y = std::numeric_limits<double>::quiet_NaN();
    double aligned_yaw = std::numeric_limits<double>::quiet_NaN();
    if (alignment_complete_ && have_lidar_odom_) {
      aligned_pose(aligned_x, aligned_y, aligned_yaw);
    }
    const double px4_x = have_px4_odom_ ? latest_px4_odom_.pose.pose.position.x :
      std::numeric_limits<double>::quiet_NaN();
    const double px4_y = have_px4_odom_ ? latest_px4_odom_.pose.pose.position.y :
      std::numeric_limits<double>::quiet_NaN();
    const double position_difference = alignment_complete_ && have_px4_odom_ ?
      std::hypot(aligned_x - px4_x, aligned_y - px4_y) :
      std::numeric_limits<double>::quiet_NaN();
    const double yaw_difference = alignment_complete_ && have_px4_odom_ ?
      wrap_pi(aligned_yaw - px4_yaw_) * kRadToDeg :
      std::numeric_limits<double>::quiet_NaN();

    health_csv_ << std::fixed << std::setprecision(6) << now().seconds() << ','
                << bool_string(mavros_connected_) << ',' << bool_string(px4_armed_) << ','
                << csv_escape(px4_mode_) << ',' << bool_string(alignment_complete_) << ','
                << bool_string(monitor_ok_) << ',' << age_sec(lidar_receipt_time_) << ','
                << aligned_x << ',' << aligned_y << ',' << aligned_yaw * kRadToDeg << ','
                << px4_x << ',' << px4_y << ',' << px4_yaw_ * kRadToDeg << ','
                << position_difference << ',' << yaw_difference << ','
                << bool_string(publishing_to_px4_) << ',' << csv_escape(rejection_reason_) << '\n';
    health_csv_.flush();
  }

  std::string lidar_odom_topic_;
  std::string monitor_diagnostics_topic_;
  std::string px4_odom_topic_;
  std::string mavros_state_topic_;
  std::string scan_topic_;
  std::string output_topic_;
  std::string diagnostics_topic_;
  std::string expected_lidar_frame_;
  std::string output_parent_frame_;
  std::string output_child_frame_;
  std::string health_csv_path_;
  double publish_rate_hz_{10.0};
  double odom_timeout_sec_{0.5};
  double scan_timeout_sec_{0.5};
  double diagnostic_timeout_sec_{1.0};
  double px4_odom_timeout_sec_{0.5};
  bool require_disarmed_for_alignment_{true};
  double horizontal_position_stddev_m_{0.3};
  double yaw_stddev_rad_{0.2};
  double unused_axis_variance_{1000000.0};
  double max_roll_pitch_rad_{0.35};
  double max_position_jump_m_{0.5};
  double max_yaw_jump_rad_{0.5};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lidar_odom_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    monitor_diagnostics_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr px4_odom_sub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr mavros_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr output_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  rclcpp::TimerBase::SharedPtr health_timer_;

  nav_msgs::msg::Odometry latest_lidar_odom_;
  nav_msgs::msg::Odometry latest_px4_odom_;
  rclcpp::Time lidar_receipt_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time monitor_diagnostics_receipt_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time px4_odom_receipt_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time scan_receipt_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_lidar_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_output_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_output_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time alignment_time_{0, 0, RCL_ROS_TIME};
  bool have_lidar_odom_{false};
  bool have_monitor_diagnostics_{false};
  bool have_px4_odom_{false};
  bool have_mavros_state_{false};
  bool have_scan_{false};
  bool have_lidar_stamp_{false};
  bool have_lidar_pose_{false};
  bool monitor_ok_{false};
  bool mavros_connected_{false};
  bool px4_armed_{false};
  bool alignment_complete_{false};
  bool publishing_to_px4_{false};
  double px4_roll_{0.0};
  double px4_pitch_{0.0};
  double px4_yaw_{0.0};
  double last_lidar_x_{0.0};
  double last_lidar_y_{0.0};
  double last_lidar_yaw_{0.0};
  double alignment_x_{0.0};
  double alignment_y_{0.0};
  double alignment_yaw_{0.0};
  uint64_t lidar_sequence_{0};
  uint64_t last_published_sequence_{0};
  std::string px4_mode_{"UNKNOWN"};
  std::string monitor_reason_{"missing"};
  std::string rejection_reason_;
  std::ofstream health_csv_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarOdomPx4Bridge>());
  rclcpp::shutdown();
  return 0;
}
