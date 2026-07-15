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
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/LinearMath/Vector3.hpp>
#include <tf2/exceptions.hpp>
#include <tf2/time.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace
{
constexpr double kRadToDeg = 180.0 / M_PI;

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

double yaw_from_transform(const geometry_msgs::msg::TransformStamped & transform)
{
  tf2::Quaternion q;
  tf2::fromMsg(transform.transform.rotation, q);
  if (!std::isfinite(q.length2()) || q.length2() < 1e-12) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  q.normalize();
  return tf2::getYaw(q);
}
}  // namespace

class LaserScanDeskewer : public rclcpp::Node
{
public:
  LaserScanDeskewer()
  : Node("laser_scan_deskewer")
  {
    enabled_ = declare_parameter<bool>("enabled", false);
    input_topic_ = declare_parameter<std::string>("input_topic", "/scan_rf2o");
    output_topic_ = declare_parameter<std::string>("output_topic", "/scan_deskewed");
    output_frame_ = declare_parameter<std::string>("output_frame", "laser_frame");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    reference_time_ = declare_parameter<std::string>("reference_time", "midpoint");
    diagnostics_topic_ = declare_parameter<std::string>(
      "diagnostics_topic", "/scan_deskewed/diagnostics");
    health_csv_path_ = declare_parameter<std::string>("health_csv_path", "");
    minimum_valid_tf_ratio_ = declare_parameter<double>("minimum_valid_tf_ratio", 0.95);
    maximum_tf_lookup_timeout_sec_ = declare_parameter<double>(
      "maximum_tf_lookup_timeout_sec", 0.02);
    output_bins_ = declare_parameter<int>("output_bins", 720);
    output_angle_min_ = declare_parameter<double>("output_angle_min", -M_PI);
    output_angle_max_ = declare_parameter<double>("output_angle_max", M_PI);
    diagnostics_rate_hz_ = declare_parameter<double>("diagnostics_rate_hz", 1.0);
    if (reference_time_ != "midpoint" || minimum_valid_tf_ratio_ <= 0.0 ||
      minimum_valid_tf_ratio_ > 1.0 || maximum_tf_lookup_timeout_sec_ < 0.0 ||
      output_bins_ < 2 || output_angle_max_ <= output_angle_min_ ||
      diagnostics_rate_hz_ <= 0.0)
    {
      throw std::invalid_argument("Invalid laser scan deskewer parameters");
    }
    output_angle_increment_ =
      (output_angle_max_ - output_angle_min_) / static_cast<double>(output_bins_);

    open_csv();
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    output_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(
      output_topic_, rclcpp::SensorDataQoS());
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      diagnostics_topic_, rclcpp::QoS(10));
    input_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LaserScanDeskewer::scan_callback, this, std::placeholders::_1));
    diagnostics_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / diagnostics_rate_hz_)),
      std::bind(&LaserScanDeskewer::publish_diagnostics, this));

    RCLCPP_INFO(
      get_logger(), "Laser scan deskewer enabled=%s input=%s output=%s",
      bool_string(enabled_).c_str(), input_topic_.c_str(), output_topic_.c_str());
  }

private:
  void open_csv()
  {
    if (health_csv_path_.empty()) {
      return;
    }
    health_csv_.open(health_csv_path_, std::ios::out | std::ios::app);
    if (!health_csv_) {
      throw std::runtime_error("Cannot open deskew health CSV: " + health_csv_path_);
    }
    health_csv_.seekp(0, std::ios::end);
    if (health_csv_.tellp() == 0) {
      health_csv_ << "timestamp,input_scan_size,output_scan_size,valid_input_rays,"
        "valid_tf_rays,valid_tf_ratio,reference_time,processing_time_ms,input_yaw_span_deg,"
        "output_finite_ratio,published,rejection_reason\n";
    }
  }

  double normalize_angle(double angle) const
  {
    const double span = output_angle_max_ - output_angle_min_;
    double normalized = std::fmod(angle - output_angle_min_, span);
    if (normalized < 0.0) {
      normalized += span;
    }
    return output_angle_min_ + normalized;
  }

  std::size_t angle_to_bin(double angle) const
  {
    const double normalized = normalize_angle(angle);
    const auto bin = static_cast<std::size_t>(
      std::floor((normalized - output_angle_min_) / output_angle_increment_));
    return std::min(bin, static_cast<std::size_t>(output_bins_ - 1));
  }

  double input_duration(const sensor_msgs::msg::LaserScan & scan) const
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

  bool lookup_transform(
    const std::string & source_frame, const rclcpp::Time & stamp, double timeout_sec,
    geometry_msgs::msg::TransformStamped & result, std::string & reason)
  {
    try {
      result = tf_buffer_->lookupTransform(
        odom_frame_, source_frame, stamp, rclcpp::Duration::from_seconds(timeout_sec));
      return true;
    } catch (const tf2::TransformException & exception) {
      reason = exception.what();
      return false;
    }
  }

  void reject(const std::string & reason, const std::chrono::steady_clock::time_point & start)
  {
    published_ = false;
    rejection_reason_ = reason;
    processing_time_ms_ = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count();
    state_ = "REJECTED";
    ++rejected_scans_;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", reason.c_str());
    write_csv();
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    const auto processing_start = std::chrono::steady_clock::now();
    ++input_scans_;
    input_scan_size_ = scan->ranges.size();
    output_scan_size_ = 0;
    valid_input_rays_ = 0;
    valid_tf_rays_ = 0;
    valid_tf_ratio_ = 0.0;
    output_finite_ratio_ = 0.0;
    input_yaw_span_deg_ = 0.0;
    latest_input_stamp_ = rclcpp::Time(scan->header.stamp);
    if (!enabled_) {
      state_ = "DISABLED";
      rejection_reason_ = "deskewer disabled";
      processing_time_ms_ = 0.0;
      published_ = false;
      write_csv();
      return;
    }
    if (scan->ranges.size() != static_cast<std::size_t>(output_bins_) ||
      latest_input_stamp_.nanoseconds() <= 0 || scan->header.frame_id.empty() ||
      !std::isfinite(scan->angle_min) || !std::isfinite(scan->angle_increment) ||
      scan->angle_increment <= 0.0F)
    {
      reject("invalid input scan geometry or timestamp", processing_start);
      return;
    }
    if (have_input_stamp_ && latest_input_stamp_ <= previous_input_stamp_) {
      reject("non-monotonic input scan timestamp", processing_start);
      return;
    }
    previous_input_stamp_ = latest_input_stamp_;
    have_input_stamp_ = true;

    const double duration = input_duration(*scan);
    if (duration <= 0.0 || !std::isfinite(scan->time_increment) ||
      scan->time_increment <= 0.0F)
    {
      reject("input scan has no usable per-ray acquisition timing", processing_start);
      return;
    }
    if (scan->header.frame_id != output_frame_) {
      reject("input frame does not match configured output frame", processing_start);
      return;
    }
    const rclcpp::Time reference_stamp = latest_input_stamp_ +
      rclcpp::Duration::from_seconds(0.5 * duration);
    const rclcpp::Time end_stamp = latest_input_stamp_ +
      rclcpp::Duration::from_seconds(duration);
    geometry_msgs::msg::TransformStamped reference_msg;
    geometry_msgs::msg::TransformStamped start_msg;
    geometry_msgs::msg::TransformStamped end_msg;
    std::string tf_reason = "none";
    if (!lookup_transform(
        scan->header.frame_id, reference_stamp, maximum_tf_lookup_timeout_sec_,
        reference_msg, tf_reason) ||
      !lookup_transform(
        scan->header.frame_id, latest_input_stamp_, maximum_tf_lookup_timeout_sec_,
        start_msg, tf_reason) ||
      !lookup_transform(
        scan->header.frame_id, end_stamp, maximum_tf_lookup_timeout_sec_, end_msg, tf_reason))
    {
      reject("reference/start/end TF unavailable: " + tf_reason, processing_start);
      return;
    }

    const double start_yaw = yaw_from_transform(start_msg);
    const double end_yaw = yaw_from_transform(end_msg);
    input_yaw_span_deg_ = std::abs(std::atan2(
      std::sin(end_yaw - start_yaw), std::cos(end_yaw - start_yaw))) * kRadToDeg;
    tf2::Transform reference_tf;
    tf2::fromMsg(reference_msg.transform, reference_tf);
    const tf2::Transform odom_to_reference_inverse = reference_tf.inverse();
    std::vector<float> output_ranges(
      static_cast<std::size_t>(output_bins_), std::numeric_limits<float>::infinity());

    for (std::size_t index = 0; index < scan->ranges.size(); ++index) {
      const float range = scan->ranges[index];
      const bool valid_range = std::isfinite(range) &&
        range >= scan->range_min && range <= scan->range_max;
      if (valid_range) {
        ++valid_input_rays_;
      }
      const rclcpp::Time ray_stamp = latest_input_stamp_ + rclcpp::Duration::from_seconds(
        static_cast<double>(index) * static_cast<double>(scan->time_increment));
      geometry_msgs::msg::TransformStamped ray_msg;
      std::string ray_reason;
      if (!lookup_transform(scan->header.frame_id, ray_stamp, 0.0, ray_msg, ray_reason)) {
        continue;
      }
      ++valid_tf_rays_;
      if (!valid_range) {
        continue;
      }
      const double input_angle = static_cast<double>(scan->angle_min) +
        static_cast<double>(index) * static_cast<double>(scan->angle_increment);
      const tf2::Vector3 point_at_ray(
        static_cast<double>(range) * std::cos(input_angle),
        static_cast<double>(range) * std::sin(input_angle), 0.0);
      tf2::Transform ray_tf;
      tf2::fromMsg(ray_msg.transform, ray_tf);
      const tf2::Vector3 point_at_reference =
        odom_to_reference_inverse * (ray_tf * point_at_ray);
      const double output_angle = std::atan2(
        point_at_reference.y(), point_at_reference.x());
      const float output_range = static_cast<float>(std::hypot(
        point_at_reference.x(), point_at_reference.y()));
      if (std::isfinite(output_range) && output_range >= scan->range_min &&
        output_range <= scan->range_max)
      {
        const std::size_t bin = angle_to_bin(output_angle);
        output_ranges[bin] = std::min(output_ranges[bin], output_range);
      }
    }

    valid_tf_ratio_ = static_cast<double>(valid_tf_rays_) /
      static_cast<double>(scan->ranges.size());
    if (valid_tf_ratio_ < minimum_valid_tf_ratio_) {
      reject("per-ray TF availability below minimum_valid_tf_ratio", processing_start);
      return;
    }

    const std::size_t finite_output = static_cast<std::size_t>(std::count_if(
      output_ranges.begin(), output_ranges.end(),
      [](float range) {return std::isfinite(range);}));
    output_finite_ratio_ = static_cast<double>(finite_output) /
      static_cast<double>(output_bins_);
    sensor_msgs::msg::LaserScan output;
    output.header.stamp = reference_stamp;
    output.header.frame_id = output_frame_;
    output.angle_min = static_cast<float>(output_angle_min_);
    output.angle_increment = static_cast<float>(output_angle_increment_);
    output.angle_max = static_cast<float>(
      output_angle_min_ + static_cast<double>(output_bins_ - 1) * output_angle_increment_);
    output.time_increment = 0.0F;
    output.scan_time = 0.0F;
    output.range_min = scan->range_min;
    output.range_max = scan->range_max;
    output.ranges = std::move(output_ranges);
    output.intensities.clear();
    output_pub_->publish(output);

    output_scan_size_ = output_bins_;
    published_ = true;
    rejection_reason_ = "none";
    state_ = "OK";
    ++output_scans_;
    processing_time_ms_ = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - processing_start).count();
    write_csv();
  }

  void write_csv()
  {
    if (!health_csv_) {
      return;
    }
    health_csv_ << std::fixed << std::setprecision(9)
                << latest_input_stamp_.seconds() << ',' << input_scan_size_ << ','
                << output_scan_size_ << ',' << valid_input_rays_ << ',' << valid_tf_rays_ << ','
                << valid_tf_ratio_ << ',' << reference_time_ << ',' << processing_time_ms_ << ','
                << input_yaw_span_deg_ << ',' << output_finite_ratio_ << ','
                << bool_string(published_) << ',' << csv_text(rejection_reason_) << '\n';
    health_csv_.flush();
  }

  void publish_diagnostics()
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.level = state_ == "OK" ? diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.name = "laser_scan_deskewer";
    status.hardware_id = "scan_tf_interpolation";
    status.message = state_;
    status.values.push_back(key_value("state", state_));
    status.values.push_back(key_value("enabled", bool_string(enabled_)));
    status.values.push_back(key_value("published", bool_string(published_)));
    status.values.push_back(key_value("input_scans", std::to_string(input_scans_)));
    status.values.push_back(key_value("output_scans", std::to_string(output_scans_)));
    status.values.push_back(key_value("rejected_scans", std::to_string(rejected_scans_)));
    status.values.push_back(key_value("input_scan_size", std::to_string(input_scan_size_)));
    status.values.push_back(key_value("output_scan_size", std::to_string(output_scan_size_)));
    status.values.push_back(key_value("valid_input_rays", std::to_string(valid_input_rays_)));
    status.values.push_back(key_value("valid_tf_rays", std::to_string(valid_tf_rays_)));
    status.values.push_back(key_value("valid_tf_ratio", std::to_string(valid_tf_ratio_)));
    status.values.push_back(key_value("reference_time", reference_time_));
    status.values.push_back(key_value("processing_time_ms", std::to_string(processing_time_ms_)));
    status.values.push_back(key_value("input_yaw_span_deg", std::to_string(input_yaw_span_deg_)));
    status.values.push_back(key_value(
      "output_finite_ratio", std::to_string(output_finite_ratio_)));
    status.values.push_back(key_value("rejection_reason", rejection_reason_));
    array.status.push_back(status);
    diagnostics_pub_->publish(array);
  }

  bool enabled_{false};
  std::string input_topic_;
  std::string output_topic_;
  std::string output_frame_;
  std::string odom_frame_;
  std::string reference_time_;
  std::string diagnostics_topic_;
  std::string health_csv_path_;
  double minimum_valid_tf_ratio_{0.95};
  double maximum_tf_lookup_timeout_sec_{0.02};
  int output_bins_{720};
  double output_angle_min_{-M_PI};
  double output_angle_max_{M_PI};
  double output_angle_increment_{0.0};
  double diagnostics_rate_hz_{1.0};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr input_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr output_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  std::ofstream health_csv_;

  rclcpp::Time previous_input_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time latest_input_stamp_{0, 0, RCL_ROS_TIME};
  bool have_input_stamp_{false};
  bool published_{false};
  std::size_t input_scans_{0};
  std::size_t output_scans_{0};
  std::size_t rejected_scans_{0};
  std::size_t input_scan_size_{0};
  std::size_t output_scan_size_{0};
  std::size_t valid_input_rays_{0};
  std::size_t valid_tf_rays_{0};
  double valid_tf_ratio_{0.0};
  double processing_time_ms_{0.0};
  double input_yaw_span_deg_{0.0};
  double output_finite_ratio_{0.0};
  std::string state_{"WAITING_FOR_SCAN"};
  std::string rejection_reason_{"none"};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaserScanDeskewer>());
  rclcpp::shutdown();
  return 0;
}
