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
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

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
}  // namespace

class LaserScanStreamAudit : public rclcpp::Node
{
public:
  LaserScanStreamAudit()
  : Node("laser_scan_stream_audit")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/scan");
    diagnostics_topic_ = declare_parameter<std::string>(
      "diagnostics_topic", "/scan_stream_audit/diagnostics");
    csv_path_ = declare_parameter<std::string>("csv_path", "");
    classification_window_messages_ = declare_parameter<int>(
      "classification_window_messages", 20);
    full_revolution_min_span_rad_ = declare_parameter<double>(
      "full_revolution_min_span_rad", 5.8);
    diagnostics_rate_hz_ = declare_parameter<double>("diagnostics_rate_hz", 1.0);

    if (classification_window_messages_ < 20 || full_revolution_min_span_rad_ <= 0.0 ||
      diagnostics_rate_hz_ <= 0.0)
    {
      throw std::invalid_argument("Invalid scan stream audit parameters");
    }

    open_csv();
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      diagnostics_topic_, rclcpp::QoS(10));
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LaserScanStreamAudit::scan_callback, this, std::placeholders::_1));
    diagnostics_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / diagnostics_rate_hz_)),
      std::bind(&LaserScanStreamAudit::publish_diagnostics, this));

    transition("CLASSIFYING_INPUT");
    RCLCPP_INFO(
      get_logger(), "Auditing raw scan geometry on %s for at least %d messages",
      input_topic_.c_str(), classification_window_messages_);
  }

private:
  void open_csv()
  {
    if (csv_path_.empty()) {
      return;
    }
    csv_.open(csv_path_, std::ios::out | std::ios::app);
    if (!csv_) {
      throw std::runtime_error("Cannot open raw scan audit CSV: " + csv_path_);
    }
    csv_.seekp(0, std::ios::end);
    if (csv_.tellp() == 0) {
      csv_ << "timestamp,message_index,ranges_size,intensities_size,angle_min,angle_max,"
        "angle_increment,declared_span_rad,indexed_span_rad,declared_span_deg,"
        "indexed_span_deg,scan_time,time_increment,range_min,range_max,finite_count,"
        "finite_ratio,frame_id,timestamp_monotonic,input_rate_hz,publisher_count,"
        "classification\n";
      csv_.flush();
    }
  }

  void update_rate(const rclcpp::Time & receipt)
  {
    if (last_receipt_.nanoseconds() > 0) {
      const double period = (receipt - last_receipt_).seconds();
      if (period > 0.0 && std::isfinite(period)) {
        const double instantaneous = 1.0 / period;
        input_rate_hz_ = input_rate_hz_ > 0.0 ?
          0.8 * input_rate_hz_ + 0.2 * instantaneous : instantaneous;
      }
    }
    last_receipt_ = receipt;
  }

  bool metadata_valid(const sensor_msgs::msg::LaserScan & scan) const
  {
    return !scan.ranges.empty() && std::isfinite(scan.angle_min) &&
      std::isfinite(scan.angle_max) && std::isfinite(scan.angle_increment) &&
      scan.angle_increment != 0.0F && std::isfinite(scan.range_min) &&
      std::isfinite(scan.range_max) && scan.range_max >= scan.range_min;
  }

  std::string classify(
    const sensor_msgs::msg::LaserScan & scan, bool timestamp_monotonic,
    double declared_span, double indexed_span) const
  {
    if (!metadata_valid(scan) || !timestamp_monotonic) {
      return "MALFORMED";
    }
    if (!std::isfinite(declared_span) || !std::isfinite(indexed_span) || indexed_span <= 0.0) {
      return "UNKNOWN";
    }
    const bool indexed_full = indexed_span >= full_revolution_min_span_rad_;
    const bool metadata_consistent_full = declared_span >= full_revolution_min_span_rad_ &&
      indexed_span >= 0.9 * declared_span;
    return indexed_full || metadata_consistent_full ?
      "FULL_REVOLUTION" : "PARTIAL_SECTOR";
  }

  void update_stream_classification(const std::string & classification)
  {
    if (classification_locked_) {
      return;
    }
    ++classification_samples_;
    if (classification == "FULL_REVOLUTION") {
      ++full_samples_;
    } else if (classification == "PARTIAL_SECTOR") {
      ++partial_samples_;
    } else if (classification == "MALFORMED") {
      ++malformed_samples_;
    } else {
      ++unknown_samples_;
    }

    if (classification_samples_ < static_cast<std::size_t>(classification_window_messages_)) {
      return;
    }

    const std::size_t classified = full_samples_ + partial_samples_;
    if (classified > 0 && 10 * full_samples_ >= 9 * classified && malformed_samples_ == 0) {
      stream_classification_ = "FULL_REVOLUTION";
    } else if (classified > 0 && 10 * partial_samples_ >= 9 * classified && malformed_samples_ == 0) {
      stream_classification_ = "PARTIAL_SECTOR";
    } else {
      stream_classification_ = "UNKNOWN";
    }
    classification_locked_ = true;
    transition(stream_classification_);
    RCLCPP_INFO(
      get_logger(),
      "Raw scan classification locked: %s (full=%zu partial=%zu malformed=%zu unknown=%zu)",
      stream_classification_.c_str(), full_samples_, partial_samples_, malformed_samples_,
      unknown_samples_);
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    const auto receipt = now();
    update_rate(receipt);
    ++message_index_;
    publisher_count_ = get_publishers_info_by_topic(input_topic_).size();
    last_ranges_size_ = scan->ranges.size();

    const rclcpp::Time stamp(scan->header.stamp);
    timestamp_monotonic_ = stamp.nanoseconds() > 0 &&
      (!have_stamp_ || stamp > last_stamp_);
    if (timestamp_monotonic_) {
      last_stamp_ = stamp;
      have_stamp_ = true;
    }

    declared_span_rad_ = std::abs(
      static_cast<double>(scan->angle_max) - static_cast<double>(scan->angle_min));
    indexed_span_rad_ = scan->ranges.empty() ? 0.0 :
      std::abs(static_cast<double>(scan->angle_increment)) *
      static_cast<double>(scan->ranges.size() - 1);
    finite_count_ = 0;
    for (const float range : scan->ranges) {
      if (std::isfinite(range) && range >= scan->range_min && range <= scan->range_max) {
        ++finite_count_;
      }
    }
    finite_ratio_ = scan->ranges.empty() ? 0.0 :
      static_cast<double>(finite_count_) / static_cast<double>(scan->ranges.size());
    message_classification_ = classify(
      *scan, timestamp_monotonic_, declared_span_rad_, indexed_span_rad_);
    update_stream_classification(message_classification_);

    if (publisher_count_ != 1) {
      transition("DUPLICATE_PUBLISHER");
    } else if (!classification_locked_) {
      transition("CLASSIFYING_INPUT");
    }

    if (csv_) {
      csv_ << std::fixed << std::setprecision(9)
           << stamp.seconds() << ',' << message_index_ << ',' << scan->ranges.size() << ','
           << scan->intensities.size() << ',' << scan->angle_min << ',' << scan->angle_max << ','
           << scan->angle_increment << ',' << declared_span_rad_ << ',' << indexed_span_rad_ << ','
           << declared_span_rad_ * kRadToDeg << ',' << indexed_span_rad_ * kRadToDeg << ','
           << scan->scan_time << ',' << scan->time_increment << ',' << scan->range_min << ','
           << scan->range_max << ',' << finite_count_ << ',' << finite_ratio_ << ','
           << '"' << scan->header.frame_id << '"' << ',' << bool_string(timestamp_monotonic_) << ','
           << input_rate_hz_ << ',' << publisher_count_ << ',' << message_classification_ << '\n';
      csv_.flush();
    }
  }

  void transition(const std::string & state)
  {
    if (state == state_) {
      return;
    }
    RCLCPP_INFO(get_logger(), "Raw scan audit state: %s", state.c_str());
    state_ = state;
  }

  void publish_diagnostics()
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    const bool ready = classification_locked_ && stream_classification_ != "UNKNOWN" &&
      publisher_count_ == 1;
    status.level = ready ? diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.name = "laser_scan_stream_audit";
    status.hardware_id = "rplidar_a1m8";
    status.message = state_;
    status.values.push_back(key_value("state", state_));
    status.values.push_back(key_value("classification_ready", bool_string(classification_locked_)));
    status.values.push_back(key_value("classification", stream_classification_));
    status.values.push_back(key_value("message_classification", message_classification_));
    status.values.push_back(key_value("messages", std::to_string(message_index_)));
    status.values.push_back(key_value("full_samples", std::to_string(full_samples_)));
    status.values.push_back(key_value("partial_samples", std::to_string(partial_samples_)));
    status.values.push_back(key_value("malformed_samples", std::to_string(malformed_samples_)));
    status.values.push_back(key_value("ranges_size", std::to_string(last_ranges_size_)));
    status.values.push_back(key_value("declared_span_rad", std::to_string(declared_span_rad_)));
    status.values.push_back(key_value("indexed_span_rad", std::to_string(indexed_span_rad_)));
    status.values.push_back(key_value("finite_count", std::to_string(finite_count_)));
    status.values.push_back(key_value("finite_ratio", std::to_string(finite_ratio_)));
    status.values.push_back(key_value("timestamp_monotonic", bool_string(timestamp_monotonic_)));
    status.values.push_back(key_value("input_rate_hz", std::to_string(input_rate_hz_)));
    status.values.push_back(key_value("publisher_count", std::to_string(publisher_count_)));
    array.status.push_back(status);
    diagnostics_pub_->publish(array);
  }

  std::string input_topic_;
  std::string diagnostics_topic_;
  std::string csv_path_;
  int classification_window_messages_{20};
  double full_revolution_min_span_rad_{5.8};
  double diagnostics_rate_hz_{1.0};

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  std::ofstream csv_;

  rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_receipt_{0, 0, RCL_ROS_TIME};
  bool have_stamp_{false};
  bool timestamp_monotonic_{true};
  bool classification_locked_{false};
  std::size_t message_index_{0};
  std::size_t classification_samples_{0};
  std::size_t full_samples_{0};
  std::size_t partial_samples_{0};
  std::size_t malformed_samples_{0};
  std::size_t unknown_samples_{0};
  std::size_t finite_count_{0};
  std::size_t publisher_count_{0};
  std::size_t last_ranges_size_{0};
  double declared_span_rad_{0.0};
  double indexed_span_rad_{0.0};
  double finite_ratio_{0.0};
  double input_rate_hz_{0.0};
  std::string message_classification_{"UNKNOWN"};
  std::string stream_classification_{"UNKNOWN"};
  std::string state_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaserScanStreamAudit>());
  rclcpp::shutdown();
  return 0;
}
