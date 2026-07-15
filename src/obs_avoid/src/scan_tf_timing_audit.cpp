#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/exceptions.hpp>
#include <tf2/time.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace
{
constexpr double kRadToDeg = 180.0 / M_PI;

double wrap_pi(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
{
  tf2::Quaternion quaternion(q.x, q.y, q.z, q.w);
  if (!std::isfinite(quaternion.length2()) || quaternion.length2() < 1e-12) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  quaternion.normalize();
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
  return yaw;
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

std::string csv_text(std::string value)
{
  std::replace(value.begin(), value.end(), ',', ';');
  std::replace(value.begin(), value.end(), '\n', ' ');
  return value;
}
}  // namespace

class ScanTfTimingAudit : public rclcpp::Node
{
public:
  ScanTfTimingAudit()
  : Node("scan_tf_timing_audit")
  {
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan_rf2o");
    odom_topic_ = declare_parameter<std::string>(
      "odom_topic", "/mavros/local_position/odom");
    diagnostics_topic_ = declare_parameter<std::string>(
      "diagnostics_topic", "/scan_tf_timing_audit/diagnostics");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    laser_frame_ = declare_parameter<std::string>("laser_frame", "laser_frame");
    timing_csv_path_ = declare_parameter<std::string>("timing_csv_path", "");
    motion_csv_path_ = declare_parameter<std::string>("motion_csv_path", "");
    scan_timeout_sec_ = declare_parameter<double>("scan_timeout_sec", 0.5);
    odom_timeout_sec_ = declare_parameter<double>("odom_timeout_sec", 0.5);
    tf_lookup_timeout_sec_ = declare_parameter<double>("tf_lookup_timeout_sec", 0.3);
    maximum_allowed_scan_tf_offset_sec_ = declare_parameter<double>(
      "maximum_allowed_scan_tf_offset_sec", 0.03);
    deskew_yaw_threshold_deg_ = declare_parameter<double>(
      "deskew_yaw_threshold_deg", 2.0);
    deskew_translation_threshold_m_ = declare_parameter<double>(
      "deskew_translation_threshold_m", 0.02);
    diagnostics_rate_hz_ = declare_parameter<double>("diagnostics_rate_hz", 1.0);
    if (scan_timeout_sec_ <= 0.0 || odom_timeout_sec_ <= 0.0 ||
      tf_lookup_timeout_sec_ < 0.0 || maximum_allowed_scan_tf_offset_sec_ <= 0.0 ||
      deskew_yaw_threshold_deg_ <= 0.0 || deskew_translation_threshold_m_ <= 0.0 ||
      diagnostics_rate_hz_ <= 0.0)
    {
      throw std::invalid_argument("Invalid scan-to-TF timing audit parameters");
    }

    open_csv_files();
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      diagnostics_topic_, rclcpp::QoS(10));
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ScanTfTimingAudit::scan_callback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ScanTfTimingAudit::odom_callback, this, std::placeholders::_1));
    diagnostics_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / diagnostics_rate_hz_)),
      std::bind(&ScanTfTimingAudit::publish_diagnostics, this));

    RCLCPP_INFO(
      get_logger(), "Auditing exact TF timing for %s in %s -> %s",
      scan_topic_.c_str(), odom_frame_.c_str(), laser_frame_.c_str());
  }

private:
  void open_csv_files()
  {
    if (!timing_csv_path_.empty()) {
      timing_csv_.open(timing_csv_path_, std::ios::out | std::ios::app);
      if (!timing_csv_) {
        throw std::runtime_error("Cannot open scan TF timing CSV: " + timing_csv_path_);
      }
      timing_csv_.seekp(0, std::ios::end);
      if (timing_csv_.tellp() == 0) {
        timing_csv_ << "wall_time,scan_stamp,scan_age_sec,scan_time_sec,time_increment_sec,"
          "scan_size,scan_rate_hz,latest_odom_stamp,latest_odom_age_sec,odom_rate_hz,"
          "tf_lookup_at_scan_stamp_success,base_tf_lookup_at_scan_stamp_success,"
          "tf_lookup_latest_success,tf_at_scan_stamp,"
          "tf_latest_stamp,tf_at_scan_age_sec,tf_latest_age_sec,scan_minus_latest_odom_sec,"
          "scan_minus_tf_sec,lookup_latency_ms,lookup_exception,odom_x_at_scan,odom_y_at_scan,"
          "odom_yaw_deg_at_scan,latest_odom_x,latest_odom_y,latest_odom_yaw_deg,"
          "yaw_difference_scan_vs_latest_deg,status\n";
      }
    }
    if (!motion_csv_path_.empty()) {
      motion_csv_.open(motion_csv_path_, std::ios::out | std::ios::app);
      if (!motion_csv_) {
        throw std::runtime_error("Cannot open scan motion CSV: " + motion_csv_path_);
      }
      motion_csv_.seekp(0, std::ios::end);
      if (motion_csv_.tellp() == 0) {
        motion_csv_ << "timestamp,scan_duration_sec,scan_start_tf_available,"
          "scan_end_tf_available,translation_during_scan_m,yaw_change_during_scan_deg,"
          "average_yaw_rate_degps,maximum_expected_edge_error_m,deskew_recommended,status\n";
      }
    }
  }

  void update_rate(const rclcpp::Time & receipt, rclcpp::Time & previous, double & rate)
  {
    if (previous.nanoseconds() > 0) {
      const double period = (receipt - previous).seconds();
      if (period > 0.0 && std::isfinite(period)) {
        const double instant_rate = 1.0 / period;
        rate = rate > 0.0 ? 0.8 * rate + 0.2 * instant_rate : instant_rate;
      }
    }
    previous = receipt;
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const auto receipt = now();
    update_rate(receipt, previous_odom_receipt_, odom_rate_hz_);
    latest_odom_receipt_ = receipt;
    const rclcpp::Time stamp(msg->header.stamp);
    odom_timestamp_monotonic_ = stamp.nanoseconds() > 0 &&
      (!have_odom_stamp_ || stamp > latest_odom_stamp_);
    if (odom_timestamp_monotonic_) {
      latest_odom_stamp_ = stamp;
      have_odom_stamp_ = true;
    }
    latest_odom_x_ = msg->pose.pose.position.x;
    latest_odom_y_ = msg->pose.pose.position.y;
    latest_odom_yaw_ = yaw_from_quaternion(msg->pose.pose.orientation);
  }

  std::string classify_lookup_exception(const std::string & exception) const
  {
    if (exception.find("future") != std::string::npos ||
      exception.find("latest data is at time") != std::string::npos)
    {
      return "EXTRAPOLATION_FUTURE";
    }
    if (exception.find("past") != std::string::npos ||
      exception.find("earliest data is at time") != std::string::npos)
    {
      return "EXTRAPOLATION_PAST";
    }
    if (exception.find("timeout") != std::string::npos ||
      exception.find("Timeout") != std::string::npos)
    {
      return "TF_TIMEOUT";
    }
    return "TF_UNAVAILABLE_AT_SCAN_TIME";
  }

  double scan_duration(const sensor_msgs::msg::LaserScan & scan) const
  {
    if (scan.ranges.size() > 1 && std::isfinite(scan.time_increment) &&
      scan.time_increment > 0.0F)
    {
      return static_cast<double>(scan.time_increment) *
             static_cast<double>(scan.ranges.size() - 1);
    }
    return std::isfinite(scan.scan_time) && scan.scan_time > 0.0F ?
      static_cast<double>(scan.scan_time) : 0.0;
  }

  void calculate_motion(
    const geometry_msgs::msg::TransformStamped & start_tf,
    const geometry_msgs::msg::TransformStamped & end_tf,
    const sensor_msgs::msg::LaserScan & scan)
  {
    translation_during_scan_m_ = std::hypot(
      end_tf.transform.translation.x - start_tf.transform.translation.x,
      end_tf.transform.translation.y - start_tf.transform.translation.y);
    const double start_yaw = yaw_from_quaternion(start_tf.transform.rotation);
    const double end_yaw = yaw_from_quaternion(end_tf.transform.rotation);
    yaw_change_during_scan_deg_ = std::abs(wrap_pi(end_yaw - start_yaw)) * kRadToDeg;
    average_yaw_rate_degps_ = scan_duration_sec_ > 0.0 ?
      yaw_change_during_scan_deg_ / scan_duration_sec_ : 0.0;
    double maximum_valid_range = 0.0;
    for (const float range : scan.ranges) {
      if (std::isfinite(range) && range >= scan.range_min && range <= scan.range_max) {
        maximum_valid_range = std::max(maximum_valid_range, static_cast<double>(range));
      }
    }
    const double yaw_change_rad = yaw_change_during_scan_deg_ / kRadToDeg;
    maximum_expected_edge_error_m_ = translation_during_scan_m_ +
      2.0 * maximum_valid_range * std::sin(0.5 * yaw_change_rad);
    deskew_recommended_ = yaw_change_during_scan_deg_ >= deskew_yaw_threshold_deg_ ||
      translation_during_scan_m_ >= deskew_translation_threshold_m_;
    const bool required = yaw_change_during_scan_deg_ >= 2.0 * deskew_yaw_threshold_deg_ ||
      translation_during_scan_m_ >= 2.0 * deskew_translation_threshold_m_;
    motion_status_ = required ? "DESKEW_REQUIRED" :
      (deskew_recommended_ ? "DESKEW_RECOMMENDED" :
      ((yaw_change_during_scan_deg_ >= 0.5 * deskew_yaw_threshold_deg_ ||
      translation_during_scan_m_ >= 0.5 * deskew_translation_threshold_m_) ?
      "MODERATE_MOTION" : "LOW_MOTION"));
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    const auto receipt = now();
    update_rate(receipt, previous_scan_receipt_, scan_rate_hz_);
    latest_scan_receipt_ = receipt;
    ++scan_count_;
    scan_size_ = scan->ranges.size();
    scan_time_sec_ = scan->scan_time;
    time_increment_sec_ = scan->time_increment;
    scan_duration_sec_ = scan_duration(*scan);
    const rclcpp::Time scan_stamp(scan->header.stamp);
    latest_scan_stamp_ = scan_stamp;
    scan_age_sec_ = (receipt - scan_stamp).seconds();
    scan_timestamp_monotonic_ = scan_stamp.nanoseconds() > 0 &&
      (!have_scan_stamp_ || scan_stamp > previous_scan_stamp_);
    if (scan_timestamp_monotonic_) {
      previous_scan_stamp_ = scan_stamp;
      have_scan_stamp_ = true;
    }
    scan_minus_latest_odom_sec_ = have_odom_stamp_ ?
      (scan_stamp - latest_odom_stamp_).seconds() :
      std::numeric_limits<double>::quiet_NaN();

    exact_tf_success_ = false;
    base_tf_success_ = false;
    latest_tf_success_ = false;
    end_tf_success_ = false;
    lookup_exception_ = "none";
    lookup_latency_ms_ = 0.0;
    const auto lookup_start = std::chrono::steady_clock::now();
    geometry_msgs::msg::TransformStamped exact_tf;
    geometry_msgs::msg::TransformStamped base_tf;
    geometry_msgs::msg::TransformStamped latest_tf;
    geometry_msgs::msg::TransformStamped end_tf;
    try {
      base_tf = tf_buffer_->lookupTransform(
        odom_frame_, base_frame_, scan_stamp,
        rclcpp::Duration::from_seconds(tf_lookup_timeout_sec_));
      base_tf_success_ = true;
    } catch (const tf2::TransformException & exception) {
      lookup_exception_ = exception.what();
    }
    try {
      exact_tf = tf_buffer_->lookupTransform(
        odom_frame_, laser_frame_, scan_stamp,
        rclcpp::Duration::from_seconds(tf_lookup_timeout_sec_));
      exact_tf_success_ = true;
    } catch (const tf2::TransformException & exception) {
      lookup_exception_ = exception.what();
    }
    try {
      latest_tf = tf_buffer_->lookupTransform(
        odom_frame_, laser_frame_, rclcpp::Time(0, 0, get_clock()->get_clock_type()),
        rclcpp::Duration::from_seconds(tf_lookup_timeout_sec_));
      latest_tf_success_ = true;
    } catch (const tf2::TransformException & exception) {
      if (lookup_exception_ == "none") {
        lookup_exception_ = exception.what();
      }
    }
    const rclcpp::Time end_stamp = scan_stamp +
      rclcpp::Duration::from_seconds(scan_duration_sec_);
    try {
      end_tf = tf_buffer_->lookupTransform(
        odom_frame_, laser_frame_, end_stamp,
        rclcpp::Duration::from_seconds(tf_lookup_timeout_sec_));
      end_tf_success_ = true;
    } catch (const tf2::TransformException & exception) {
      if (lookup_exception_ == "none") {
        lookup_exception_ = exception.what();
      }
    }
    lookup_latency_ms_ = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - lookup_start).count();

    if (exact_tf_success_) {
      tf_at_scan_stamp_ = rclcpp::Time(exact_tf.header.stamp);
      tf_at_scan_age_sec_ = (receipt - tf_at_scan_stamp_).seconds();
      scan_minus_tf_sec_ = (scan_stamp - tf_at_scan_stamp_).seconds();
      odom_x_at_scan_ = exact_tf.transform.translation.x;
      odom_y_at_scan_ = exact_tf.transform.translation.y;
      odom_yaw_at_scan_ = yaw_from_quaternion(exact_tf.transform.rotation);
    }
    if (latest_tf_success_) {
      tf_latest_stamp_ = rclcpp::Time(latest_tf.header.stamp);
      tf_latest_age_sec_ = (receipt - tf_latest_stamp_).seconds();
      latest_tf_yaw_ = yaw_from_quaternion(latest_tf.transform.rotation);
      yaw_difference_scan_vs_latest_deg_ = exact_tf_success_ ?
        std::abs(wrap_pi(latest_tf_yaw_ - odom_yaw_at_scan_)) * kRadToDeg :
        std::numeric_limits<double>::quiet_NaN();
    }

    if (!scan_timestamp_monotonic_) {
      timing_status_ = "NON_MONOTONIC_SCAN_TIMESTAMP";
    } else if (!have_odom_stamp_) {
      timing_status_ = "WAITING_FOR_ODOM";
    } else if (!odom_timestamp_monotonic_) {
      timing_status_ = "NON_MONOTONIC_ODOM_TIMESTAMP";
    } else if (scan_age_sec_ > scan_timeout_sec_) {
      timing_status_ = "SCAN_STALE";
    } else if ((receipt - latest_odom_receipt_).seconds() > odom_timeout_sec_) {
      timing_status_ = "ODOM_STALE";
    } else if (!base_tf_success_ || !exact_tf_success_) {
      timing_status_ = classify_lookup_exception(lookup_exception_);
    } else if (std::abs(scan_minus_tf_sec_) > maximum_allowed_scan_tf_offset_sec_) {
      timing_status_ = "CLOCK_DOMAIN_MISMATCH";
    } else {
      timing_status_ = "OK";
    }
    timing_valid_ = timing_status_ == "OK";

    if (exact_tf_success_ && end_tf_success_) {
      calculate_motion(exact_tf, end_tf, *scan);
    } else {
      translation_during_scan_m_ = std::numeric_limits<double>::quiet_NaN();
      yaw_change_during_scan_deg_ = std::numeric_limits<double>::quiet_NaN();
      average_yaw_rate_degps_ = std::numeric_limits<double>::quiet_NaN();
      maximum_expected_edge_error_m_ = std::numeric_limits<double>::quiet_NaN();
      deskew_recommended_ = false;
      motion_status_ = "TF_UNAVAILABLE";
    }
    write_csv(receipt);
  }

  void write_csv(const rclcpp::Time & wall_time)
  {
    if (timing_csv_) {
      timing_csv_ << std::fixed << std::setprecision(9)
                  << wall_time.seconds() << ',' << latest_scan_stamp_.seconds() << ','
                  << scan_age_sec_ << ',' << scan_time_sec_ << ',' << time_increment_sec_ << ','
                  << scan_size_ << ',' << scan_rate_hz_ << ',' << latest_odom_stamp_.seconds() << ','
                  << (wall_time - latest_odom_stamp_).seconds() << ',' << odom_rate_hz_ << ','
                  << bool_string(exact_tf_success_) << ',' << bool_string(base_tf_success_) << ','
                  << bool_string(latest_tf_success_) << ','
                  << tf_at_scan_stamp_.seconds() << ',' << tf_latest_stamp_.seconds() << ','
                  << tf_at_scan_age_sec_ << ',' << tf_latest_age_sec_ << ','
                  << scan_minus_latest_odom_sec_ << ',' << scan_minus_tf_sec_ << ','
                  << lookup_latency_ms_ << ',' << csv_text(lookup_exception_) << ','
                  << odom_x_at_scan_ << ',' << odom_y_at_scan_ << ','
                  << odom_yaw_at_scan_ * kRadToDeg << ',' << latest_odom_x_ << ','
                  << latest_odom_y_ << ',' << latest_odom_yaw_ * kRadToDeg << ','
                  << yaw_difference_scan_vs_latest_deg_ << ',' << timing_status_ << '\n';
      timing_csv_.flush();
    }
    if (motion_csv_) {
      motion_csv_ << std::fixed << std::setprecision(9)
                  << latest_scan_stamp_.seconds() << ',' << scan_duration_sec_ << ','
                  << bool_string(exact_tf_success_) << ',' << bool_string(end_tf_success_) << ','
                  << translation_during_scan_m_ << ',' << yaw_change_during_scan_deg_ << ','
                  << average_yaw_rate_degps_ << ',' << maximum_expected_edge_error_m_ << ','
                  << bool_string(deskew_recommended_) << ',' << motion_status_ << '\n';
      motion_csv_.flush();
    }
  }

  void publish_diagnostics()
  {
    std::string state = timing_status_;
    if (scan_count_ == 0) {
      state = "WAITING_FOR_SCAN";
    } else if ((now() - latest_scan_receipt_).seconds() > scan_timeout_sec_) {
      state = "SCAN_STALE";
    } else if (!have_odom_stamp_) {
      state = "WAITING_FOR_ODOM";
    } else if ((now() - latest_odom_receipt_).seconds() > odom_timeout_sec_) {
      state = "ODOM_STALE";
    }
    const bool healthy = state == "OK";
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.level = healthy ? diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.name = "scan_tf_timing_audit";
    status.hardware_id = "scan_tf_chain";
    status.message = state;
    status.values.push_back(key_value("state", state));
    status.values.push_back(key_value("timing_valid", bool_string(healthy)));
    status.values.push_back(key_value(
      "tf_lookup_at_scan_stamp_success", bool_string(exact_tf_success_)));
    status.values.push_back(key_value(
      "base_tf_lookup_at_scan_stamp_success", bool_string(base_tf_success_)));
    status.values.push_back(key_value("tf_lookup_latest_success", bool_string(latest_tf_success_)));
    status.values.push_back(key_value("scan_stamp", std::to_string(latest_scan_stamp_.seconds())));
    status.values.push_back(key_value("scan_age_sec", std::to_string(scan_age_sec_)));
    status.values.push_back(key_value("scan_time_sec", std::to_string(scan_time_sec_)));
    status.values.push_back(key_value("time_increment_sec", std::to_string(time_increment_sec_)));
    status.values.push_back(key_value("scan_size", std::to_string(scan_size_)));
    status.values.push_back(key_value("scan_rate_hz", std::to_string(scan_rate_hz_)));
    status.values.push_back(key_value(
      "latest_odom_stamp", std::to_string(latest_odom_stamp_.seconds())));
    status.values.push_back(key_value(
      "latest_odom_age_sec", std::to_string((now() - latest_odom_stamp_).seconds())));
    status.values.push_back(key_value("odom_rate_hz", std::to_string(odom_rate_hz_)));
    status.values.push_back(key_value(
      "scan_minus_latest_odom_sec", std::to_string(scan_minus_latest_odom_sec_)));
    status.values.push_back(key_value("scan_minus_tf_sec", std::to_string(scan_minus_tf_sec_)));
    status.values.push_back(key_value("lookup_latency_ms", std::to_string(lookup_latency_ms_)));
    status.values.push_back(key_value("lookup_exception", lookup_exception_));
    status.values.push_back(key_value(
      "translation_during_scan_m", std::to_string(translation_during_scan_m_)));
    status.values.push_back(key_value(
      "yaw_change_during_scan_deg", std::to_string(yaw_change_during_scan_deg_)));
    status.values.push_back(key_value(
      "average_yaw_rate_degps", std::to_string(average_yaw_rate_degps_)));
    status.values.push_back(key_value(
      "maximum_expected_edge_error_m", std::to_string(maximum_expected_edge_error_m_)));
    status.values.push_back(key_value(
      "deskew_recommended", bool_string(deskew_recommended_)));
    status.values.push_back(key_value("motion_status", motion_status_));
    status.values.push_back(key_value(
      "scan_timestamp_monotonic", bool_string(scan_timestamp_monotonic_)));
    status.values.push_back(key_value(
      "odom_timestamp_monotonic", bool_string(odom_timestamp_monotonic_)));
    array.status.push_back(status);
    diagnostics_pub_->publish(array);
  }

  std::string scan_topic_;
  std::string odom_topic_;
  std::string diagnostics_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string laser_frame_;
  std::string timing_csv_path_;
  std::string motion_csv_path_;
  double scan_timeout_sec_{0.5};
  double odom_timeout_sec_{0.5};
  double tf_lookup_timeout_sec_{0.3};
  double maximum_allowed_scan_tf_offset_sec_{0.03};
  double deskew_yaw_threshold_deg_{2.0};
  double deskew_translation_threshold_m_{0.02};
  double diagnostics_rate_hz_{1.0};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  std::ofstream timing_csv_;
  std::ofstream motion_csv_;

  rclcpp::Time previous_scan_receipt_{0, 0, RCL_ROS_TIME};
  rclcpp::Time latest_scan_receipt_{0, 0, RCL_ROS_TIME};
  rclcpp::Time previous_odom_receipt_{0, 0, RCL_ROS_TIME};
  rclcpp::Time latest_odom_receipt_{0, 0, RCL_ROS_TIME};
  rclcpp::Time previous_scan_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time latest_scan_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time latest_odom_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time tf_at_scan_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time tf_latest_stamp_{0, 0, RCL_ROS_TIME};
  bool have_scan_stamp_{false};
  bool have_odom_stamp_{false};
  bool scan_timestamp_monotonic_{true};
  bool odom_timestamp_monotonic_{true};
  bool exact_tf_success_{false};
  bool base_tf_success_{false};
  bool latest_tf_success_{false};
  bool end_tf_success_{false};
  bool timing_valid_{false};
  bool deskew_recommended_{false};
  std::size_t scan_count_{0};
  std::size_t scan_size_{0};
  double scan_rate_hz_{0.0};
  double odom_rate_hz_{0.0};
  double scan_age_sec_{std::numeric_limits<double>::infinity()};
  double scan_time_sec_{0.0};
  double time_increment_sec_{0.0};
  double scan_duration_sec_{0.0};
  double tf_at_scan_age_sec_{std::numeric_limits<double>::infinity()};
  double tf_latest_age_sec_{std::numeric_limits<double>::infinity()};
  double scan_minus_latest_odom_sec_{std::numeric_limits<double>::quiet_NaN()};
  double scan_minus_tf_sec_{std::numeric_limits<double>::quiet_NaN()};
  double lookup_latency_ms_{0.0};
  double odom_x_at_scan_{0.0};
  double odom_y_at_scan_{0.0};
  double odom_yaw_at_scan_{0.0};
  double latest_odom_x_{0.0};
  double latest_odom_y_{0.0};
  double latest_odom_yaw_{0.0};
  double latest_tf_yaw_{0.0};
  double yaw_difference_scan_vs_latest_deg_{0.0};
  double translation_during_scan_m_{0.0};
  double yaw_change_during_scan_deg_{0.0};
  double average_yaw_rate_degps_{0.0};
  double maximum_expected_edge_error_m_{0.0};
  std::string timing_status_{"WAITING_FOR_SCAN"};
  std::string motion_status_{"TF_UNAVAILABLE"};
  std::string lookup_exception_{"none"};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanTfTimingAudit>());
  rclcpp::shutdown();
  return 0;
}
