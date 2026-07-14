#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace
{
diagnostic_msgs::msg::KeyValue key_value(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  return item;
}
}  // namespace

class LaserScanCanonicalizer : public rclcpp::Node
{
public:
  LaserScanCanonicalizer()
  : Node("laser_scan_canonicalizer")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/scan");
    output_topic_ = declare_parameter<std::string>("output_topic", "/scan_rf2o");
    output_frame_ = declare_parameter<std::string>("output_frame", "laser_frame");
    output_bins_ = declare_parameter<int>("output_bins", 720);
    output_angle_min_ = declare_parameter<double>("output_angle_min", -M_PI);
    output_angle_max_ = declare_parameter<double>("output_angle_max", M_PI);
    min_valid_input_ratio_ = declare_parameter<double>("min_valid_input_ratio", 0.05);
    diagnostics_topic_ = declare_parameter<std::string>(
      "diagnostics_topic", "/scan_rf2o/diagnostics");

    if (output_bins_ < 2 || !std::isfinite(output_angle_min_) ||
      !std::isfinite(output_angle_max_) || output_angle_max_ <= output_angle_min_ ||
      min_valid_input_ratio_ < 0.0 || min_valid_input_ratio_ > 1.0)
    {
      throw std::invalid_argument("Invalid laser scan canonicalizer parameters");
    }

    output_angle_increment_ =
      (output_angle_max_ - output_angle_min_) / static_cast<double>(output_bins_);

    output_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(
      output_topic_, rclcpp::SensorDataQoS());
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      diagnostics_topic_, rclcpp::QoS(10));
    input_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LaserScanCanonicalizer::scan_callback, this, std::placeholders::_1));
    diagnostics_timer_ = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&LaserScanCanonicalizer::publish_diagnostics, this));

    set_state("WAITING_FOR_SCAN");
    RCLCPP_INFO(
      get_logger(), "Canonicalizing %s to %s with exactly %d bins",
      input_topic_.c_str(), output_topic_.c_str(), output_bins_);
  }

private:
  double normalize_angle(double angle) const
  {
    const double span = output_angle_max_ - output_angle_min_;
    double normalized = std::fmod(angle - output_angle_min_, span);
    if (normalized < 0.0) {
      normalized += span;
    }
    return output_angle_min_ + normalized;
  }

  void update_rate(const rclcpp::Time & now_time, rclcpp::Time & previous_time, double & rate)
  {
    if (previous_time.nanoseconds() > 0) {
      const double period = (now_time - previous_time).seconds();
      if (period > 0.0 && std::isfinite(period)) {
        const double instantaneous_rate = 1.0 / period;
        rate = rate > 0.0 ? 0.8 * rate + 0.2 * instantaneous_rate : instantaneous_rate;
      }
    }
    previous_time = now_time;
  }

  void reject(const std::string & state, const std::string & warning)
  {
    timestamp_monotonic_ = state != "NONMONOTONIC_TIMESTAMP";
    set_state(state);
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", warning.c_str());
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr input)
  {
    const auto receipt_time = now();
    update_rate(receipt_time, last_input_receipt_, input_rate_hz_);
    ++input_count_;
    input_length_ = input->ranges.size();
    input_length_min_ = std::min(input_length_min_, input_length_);
    input_length_max_ = std::max(input_length_max_, input_length_);

    if (input->ranges.empty()) {
      reject("EMPTY_SCAN", "Rejecting empty raw LaserScan");
      return;
    }
    if (!std::isfinite(input->angle_increment) || input->angle_increment == 0.0F ||
      !std::isfinite(input->angle_min) || !std::isfinite(input->angle_max) ||
      !std::isfinite(input->range_min) || !std::isfinite(input->range_max) ||
      input->range_max < input->range_min)
    {
      reject("INVALID_METADATA", "Rejecting raw LaserScan with invalid metadata");
      return;
    }

    const rclcpp::Time stamp(input->header.stamp);
    if (stamp.nanoseconds() <= 0 || (have_stamp_ && stamp <= last_stamp_)) {
      reject("NONMONOTONIC_TIMESTAMP", "Rejecting non-monotonic raw LaserScan timestamp");
      return;
    }
    timestamp_monotonic_ = true;

    sensor_msgs::msg::LaserScan output;
    output.header = input->header;
    if (!output_frame_.empty()) {
      output.header.frame_id = output_frame_;
    }
    output.angle_min = static_cast<float>(output_angle_min_);
    output.angle_increment = static_cast<float>(output_angle_increment_);
    output.angle_max = static_cast<float>(
      output_angle_min_ + static_cast<double>(output_bins_ - 1) * output_angle_increment_);
    output.range_min = input->range_min;
    output.range_max = input->range_max;
    output.scan_time = input->scan_time;
    output.time_increment = input->scan_time > 0.0F ?
      input->scan_time / static_cast<float>(output_bins_) : 0.0F;
    output.ranges.assign(static_cast<std::size_t>(output_bins_),
      std::numeric_limits<float>::infinity());

    std::size_t valid_input_count = 0;
    for (std::size_t i = 0; i < input->ranges.size(); ++i) {
      const float range = input->ranges[i];
      if (!std::isfinite(range) || range < input->range_min || range > input->range_max) {
        continue;
      }
      ++valid_input_count;
      const double input_angle = static_cast<double>(input->angle_min) +
        static_cast<double>(i) * static_cast<double>(input->angle_increment);
      const double output_angle = normalize_angle(input_angle);
      const auto bin = static_cast<std::size_t>(std::floor(
        (output_angle - output_angle_min_) / output_angle_increment_));
      if (bin < output.ranges.size()) {
        output.ranges[bin] = std::min(output.ranges[bin], range);
      }
    }

    valid_input_count_ = valid_input_count;
    valid_output_count_ = static_cast<std::size_t>(std::count_if(
      output.ranges.begin(), output.ranges.end(),
      [](float range) {return std::isfinite(range);}));
    valid_output_ratio_ = static_cast<double>(valid_output_count_) /
      static_cast<double>(output_bins_);
    const double valid_input_ratio = static_cast<double>(valid_input_count_) /
      static_cast<double>(input->ranges.size());

    last_stamp_ = stamp;
    have_stamp_ = true;
    if (valid_input_ratio < min_valid_input_ratio_) {
      reject("LOW_VALID_INPUT_RATIO", "Rejecting raw LaserScan with too few valid ranges");
      return;
    }

    output_pub_->publish(output);
    ++output_count_;
    update_rate(receipt_time, last_output_receipt_, output_rate_hz_);
    set_state("OK");
  }

  void set_state(const std::string & state)
  {
    if (state != state_) {
      RCLCPP_INFO(get_logger(), "Canonical scan state: %s", state.c_str());
      state_ = state;
    }
  }

  void publish_diagnostics()
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    const bool output_fresh = last_output_receipt_.nanoseconds() > 0 &&
      (now() - last_output_receipt_).seconds() < 1.0;
    const bool healthy = state_ == "OK" && output_fresh;
    const std::string diagnostic_state = state_ == "OK" && !output_fresh ?
      "STALE_INPUT" : state_;
    status.level = healthy ? diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.name = "laser_scan_canonicalizer";
    status.hardware_id = "rplidar_a1m8";
    status.message = healthy ? "OK" : diagnostic_state;
    status.values.push_back(key_value("state", diagnostic_state));
    status.values.push_back(key_value("input_array_length", std::to_string(input_length_)));
    status.values.push_back(key_value("output_array_length", std::to_string(output_bins_)));
    status.values.push_back(key_value("valid_input_count", std::to_string(valid_input_count_)));
    status.values.push_back(key_value("valid_output_bin_count", std::to_string(valid_output_count_)));
    status.values.push_back(key_value("valid_output_ratio", std::to_string(valid_output_ratio_)));
    status.values.push_back(key_value("input_rate_hz", std::to_string(input_rate_hz_)));
    status.values.push_back(key_value("output_rate_hz", std::to_string(output_rate_hz_)));
    status.values.push_back(key_value("timestamp_monotonic", timestamp_monotonic_ ? "true" : "false"));
    status.values.push_back(key_value("raw_scan_length_min",
      input_length_min_ == std::numeric_limits<std::size_t>::max() ? "0" :
      std::to_string(input_length_min_)));
    status.values.push_back(key_value("raw_scan_length_max", std::to_string(input_length_max_)));
    status.values.push_back(key_value("input_messages", std::to_string(input_count_)));
    status.values.push_back(key_value("output_messages", std::to_string(output_count_)));
    array.status.push_back(status);
    diagnostics_pub_->publish(array);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string output_frame_;
  std::string diagnostics_topic_;
  int output_bins_{720};
  double output_angle_min_{-M_PI};
  double output_angle_max_{M_PI};
  double output_angle_increment_{0.0};
  double min_valid_input_ratio_{0.05};

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr input_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr output_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;

  rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_input_receipt_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_output_receipt_{0, 0, RCL_ROS_TIME};
  bool have_stamp_{false};
  bool timestamp_monotonic_{true};
  std::size_t input_length_{0};
  std::size_t input_length_min_{std::numeric_limits<std::size_t>::max()};
  std::size_t input_length_max_{0};
  std::size_t valid_input_count_{0};
  std::size_t valid_output_count_{0};
  std::size_t input_count_{0};
  std::size_t output_count_{0};
  double valid_output_ratio_{0.0};
  double input_rate_hz_{0.0};
  double output_rate_hz_{0.0};
  std::string state_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaserScanCanonicalizer>());
  rclcpp::shutdown();
  return 0;
}
