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
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace
{
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

class LaserScanCanonicalizer : public rclcpp::Node
{
public:
  LaserScanCanonicalizer()
  : Node("laser_scan_canonicalizer")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/scan");
    output_topic_ = declare_parameter<std::string>("output_topic", "/scan_rf2o");
    output_frame_ = declare_parameter<std::string>("output_frame", "laser_frame");
    requested_mode_ = declare_parameter<std::string>("processing_mode", "auto");
    output_bins_ = declare_parameter<int>("output_bins", 720);
    output_angle_min_ = declare_parameter<double>("output_angle_min", -M_PI);
    output_angle_max_ = declare_parameter<double>("output_angle_max", M_PI);
    classification_window_messages_ = declare_parameter<int>(
      "classification_window_messages", 20);
    full_revolution_min_span_rad_ = declare_parameter<double>(
      "full_revolution_min_span_rad", 5.8);
    minimum_angular_coverage_ratio_ = declare_parameter<double>(
      "minimum_angular_coverage_ratio", 0.70);
    minimum_finite_return_ratio_ = declare_parameter<double>(
      "minimum_finite_return_ratio", 0.05);
    maximum_revolution_duration_sec_ = declare_parameter<double>(
      "maximum_revolution_duration_sec", 0.50);
    maximum_segment_gap_sec_ = declare_parameter<double>("maximum_segment_gap_sec", 0.20);
    maximum_input_messages_per_revolution_ = declare_parameter<int>(
      "maximum_input_messages_per_revolution", 20);
    publish_rate_limit_hz_ = declare_parameter<double>("publish_rate_limit_hz", 12.0);
    diagnostics_rate_hz_ = declare_parameter<double>("diagnostics_rate_hz", 1.0);
    diagnostics_topic_ = declare_parameter<std::string>(
      "diagnostics_topic", "/scan_rf2o/diagnostics");
    health_csv_path_ = declare_parameter<std::string>("health_csv_path", "");
    enable_deskew_ = declare_parameter<bool>("enable_deskew", false);
    deskew_fixed_frame_ = declare_parameter<std::string>("deskew_fixed_frame", "odom");
    deskew_stamp_policy_ = declare_parameter<std::string>("deskew_stamp_policy", "start");
    deskew_timeout_sec_ = declare_parameter<double>("deskew_timeout_sec", 0.02);

    validate_parameters();
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
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
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / diagnostics_rate_hz_)),
      std::bind(&LaserScanCanonicalizer::publish_diagnostics, this));
    open_health_csv();

    if (requested_mode_ == "per_message_full_scan") {
      selected_mode_ = requested_mode_;
      mode_locked_ = true;
      transition("INITIALIZING");
    } else if (requested_mode_ == "assemble_partial_segments") {
      selected_mode_ = requested_mode_;
      mode_locked_ = true;
      transition("INITIALIZING");
    } else {
      selected_mode_ = "unselected";
      transition("CLASSIFYING_INPUT");
    }
    RCLCPP_INFO(
      get_logger(), "Canonicalizing %s to %s with %d bins in %s mode",
      input_topic_.c_str(), output_topic_.c_str(), output_bins_, requested_mode_.c_str());
  }

private:
  enum class GeometryClass {Full, Partial, Malformed};

  void validate_parameters() const
  {
    const bool valid_mode = requested_mode_ == "auto" ||
      requested_mode_ == "per_message_full_scan" ||
      requested_mode_ == "assemble_partial_segments";
    const bool valid_deskew_policy = deskew_stamp_policy_ == "start" ||
      deskew_stamp_policy_ == "end";
    if (!valid_mode || output_bins_ < 2 || classification_window_messages_ < 20 ||
      !std::isfinite(output_angle_min_) || !std::isfinite(output_angle_max_) ||
      output_angle_max_ <= output_angle_min_ || full_revolution_min_span_rad_ <= 0.0 ||
      minimum_angular_coverage_ratio_ <= 0.0 || minimum_angular_coverage_ratio_ > 1.0 ||
      minimum_finite_return_ratio_ < 0.0 || minimum_finite_return_ratio_ > 1.0 ||
      maximum_revolution_duration_sec_ <= 0.0 || maximum_segment_gap_sec_ <= 0.0 ||
      maximum_input_messages_per_revolution_ < 1 || publish_rate_limit_hz_ <= 0.0 ||
      diagnostics_rate_hz_ <= 0.0 || deskew_timeout_sec_ < 0.0 || !valid_deskew_policy)
    {
      throw std::invalid_argument("Invalid laser scan canonicalizer parameters");
    }
  }

  void open_health_csv()
  {
    if (health_csv_path_.empty()) {
      return;
    }
    health_csv_.open(health_csv_path_, std::ios::out | std::ios::app);
    if (!health_csv_) {
      throw std::runtime_error("Cannot open canonical scan health CSV: " + health_csv_path_);
    }
    health_csv_.seekp(0, std::ios::end);
    if (health_csv_.tellp() == 0) {
      health_csv_ << "timestamp,mode,input_size,output_size,declared_span_rad,indexed_span_rad,"
        "angular_coverage_ratio,finite_return_ratio,accumulated_segments,"
        "revolution_duration_sec,input_rate_hz,output_rate_hz,valid,rejection_reason\n";
      health_csv_.flush();
    }
  }

  bool metadata_valid(const sensor_msgs::msg::LaserScan & scan) const
  {
    return !scan.ranges.empty() && std::isfinite(scan.angle_increment) &&
      scan.angle_increment != 0.0F && std::isfinite(scan.angle_min) &&
      std::isfinite(scan.angle_max) && std::isfinite(scan.range_min) &&
      std::isfinite(scan.range_max) && scan.range_max >= scan.range_min;
  }

  GeometryClass classify_geometry(
    const sensor_msgs::msg::LaserScan & scan, double declared_span, double indexed_span) const
  {
    if (!metadata_valid(scan)) {
      return GeometryClass::Malformed;
    }
    const bool indexed_full = indexed_span >= full_revolution_min_span_rad_;
    const bool metadata_consistent_full = declared_span >= full_revolution_min_span_rad_ &&
      indexed_span >= 0.9 * declared_span;
    return indexed_full || metadata_consistent_full ?
      GeometryClass::Full : GeometryClass::Partial;
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
    const double index = (normalized - output_angle_min_) / output_angle_increment_;
    return std::min(
      static_cast<std::size_t>(std::floor(index)),
      static_cast<std::size_t>(output_bins_ - 1));
  }

  void update_rate(const rclcpp::Time & receipt, rclcpp::Time & previous, double & rate)
  {
    if (previous.nanoseconds() > 0) {
      const double period = (receipt - previous).seconds();
      if (period > 0.0 && std::isfinite(period)) {
        const double instantaneous = 1.0 / period;
        rate = rate > 0.0 ? 0.8 * rate + 0.2 * instantaneous : instantaneous;
      }
    }
    previous = receipt;
  }

  void transition(const std::string & state)
  {
    if (state == state_) {
      return;
    }
    if (state == "OK_FULL_SCAN" || state == "OK_ASSEMBLED_REVOLUTION") {
      RCLCPP_INFO(get_logger(), "Canonical scan state: %s", state.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "Canonical scan state: %s", state.c_str());
    }
    state_ = state;
  }

  void reject(const std::string & state, const std::string & reason, bool count_message = true)
  {
    if (count_message) {
      ++discarded_messages_;
    }
    last_rejection_reason_ = reason;
    transition(state);
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", reason.c_str());
  }

  void update_auto_classification(GeometryClass geometry)
  {
    ++classification_samples_;
    if (geometry == GeometryClass::Full) {
      ++full_classification_samples_;
    } else if (geometry == GeometryClass::Partial) {
      ++partial_classification_samples_;
    } else {
      ++malformed_classification_samples_;
    }
    if (classification_samples_ < static_cast<std::size_t>(classification_window_messages_)) {
      transition("CLASSIFYING_INPUT");
      return;
    }

    const std::size_t classified = full_classification_samples_ + partial_classification_samples_;
    if (classified > 0 && 10 * full_classification_samples_ >= 9 * classified &&
      malformed_classification_samples_ == 0)
    {
      selected_mode_ = "per_message_full_scan";
      mode_locked_ = true;
      transition("INITIALIZING");
    } else if (classified > 0 && 10 * partial_classification_samples_ >= 9 * classified &&
      malformed_classification_samples_ == 0)
    {
      selected_mode_ = "assemble_partial_segments";
      mode_locked_ = true;
      transition("INITIALIZING");
    } else {
      selected_mode_ = "unstable";
      mode_locked_ = true;
      geometry_unstable_ = true;
      reject(
        "INPUT_GEOMETRY_UNSTABLE",
        "Raw scan classification window contains inconsistent geometry", false);
    }
    RCLCPP_INFO(
      get_logger(), "Canonicalizer mode locked: %s (full=%zu partial=%zu malformed=%zu)",
      selected_mode_.c_str(), full_classification_samples_, partial_classification_samples_,
      malformed_classification_samples_);
  }

  void rebin_scan(
    const sensor_msgs::msg::LaserScan & scan, std::vector<float> & ranges,
    std::vector<bool> & observed)
  {
    current_valid_input_count_ = 0;
    last_deskew_applied_ = false;
    for (std::size_t index = 0; index < scan.ranges.size(); ++index) {
      const double angle = static_cast<double>(scan.angle_min) +
        static_cast<double>(index) * static_cast<double>(scan.angle_increment);
      const std::size_t bin = angle_to_bin(angle);
      observed[bin] = true;
      const float range = scan.ranges[index];
      if (std::isfinite(range) && range >= scan.range_min && range <= scan.range_max) {
        ++current_valid_input_count_;
        ranges[bin] = std::min(ranges[bin], range);
      }
    }
  }

  void rebin_scan_deskewed(
    const sensor_msgs::msg::LaserScan & scan, const rclcpp::Time & stamp,
    std::vector<float> & ranges, std::vector<bool> & observed)
  {
    current_valid_input_count_ = 0;
    last_deskew_applied_ = false;
    if (!enable_deskew_) {
      rebin_scan(scan, ranges, observed);
      return;
    }
    if (scan.header.frame_id.empty() || deskew_fixed_frame_.empty()) {
      ++deskew_fallbacks_;
      last_deskew_status_ = "missing_frame";
      rebin_scan(scan, ranges, observed);
      return;
    }

    try {
      const auto timeout = tf2::durationFromSec(deskew_timeout_sec_);
      const auto ref_msg = tf_buffer_->lookupTransform(
        deskew_fixed_frame_, scan.header.frame_id, stamp, timeout);
      tf2::Transform fixed_from_ref;
      tf2::fromMsg(ref_msg.transform, fixed_from_ref);
      const tf2::Transform ref_from_fixed = fixed_from_ref.inverse();

      for (std::size_t index = 0; index < scan.ranges.size(); ++index) {
        const float range = scan.ranges[index];
        if (!std::isfinite(range) || range < scan.range_min || range > scan.range_max) {
          continue;
        }

        double beam_time_offset = static_cast<double>(index) *
          static_cast<double>(scan.time_increment);
        if (deskew_stamp_policy_ == "end") {
          const double scan_time = scan.scan_time > 0.0F ?
            static_cast<double>(scan.scan_time) :
            static_cast<double>(scan.ranges.size() - 1) *
            static_cast<double>(scan.time_increment);
          beam_time_offset -= scan_time;
        }
        const rclcpp::Time beam_stamp = std::abs(beam_time_offset) > 0.0 ? stamp +
          rclcpp::Duration::from_seconds(beam_time_offset) : stamp;
        const auto beam_msg = tf_buffer_->lookupTransform(
          deskew_fixed_frame_, scan.header.frame_id, beam_stamp, timeout);
        tf2::Transform fixed_from_beam;
        tf2::fromMsg(beam_msg.transform, fixed_from_beam);

        const double angle = static_cast<double>(scan.angle_min) +
          static_cast<double>(index) * static_cast<double>(scan.angle_increment);
        const tf2::Vector3 point_beam(
          static_cast<double>(range) * std::cos(angle),
          static_cast<double>(range) * std::sin(angle),
          0.0);
        const tf2::Vector3 point_ref = ref_from_fixed * (fixed_from_beam * point_beam);
        const double corrected_range = std::hypot(point_ref.x(), point_ref.y());
        if (!std::isfinite(corrected_range) || corrected_range < scan.range_min ||
          corrected_range > scan.range_max)
        {
          continue;
        }
        const std::size_t bin = angle_to_bin(std::atan2(point_ref.y(), point_ref.x()));
        observed[bin] = true;
        ++current_valid_input_count_;
        ranges[bin] = std::min(ranges[bin], static_cast<float>(corrected_range));
      }
      last_deskew_applied_ = true;
      ++deskewed_scans_;
      last_deskew_status_ = "applied";
    } catch (const tf2::TransformException & ex) {
      ++deskew_fallbacks_;
      last_deskew_status_ = ex.what();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Deskew fallback to raw scan: %s", ex.what());
      rebin_scan(scan, ranges, observed);
    }
  }

  void update_coverage(
    const std::vector<float> & ranges, const std::vector<bool> & observed)
  {
    const std::size_t observed_count = static_cast<std::size_t>(std::count(
      observed.begin(), observed.end(), true));
    const std::size_t finite_count = static_cast<std::size_t>(std::count_if(
      ranges.begin(), ranges.end(), [](float range) {return std::isfinite(range);}));
    angular_coverage_ratio_ = static_cast<double>(observed_count) /
      static_cast<double>(output_bins_);
    finite_return_ratio_ = static_cast<double>(finite_count) /
      static_cast<double>(output_bins_);
  }

  bool publish_grid(
    const rclcpp::Time & stamp, float scan_time, float range_min, float range_max,
    const std::vector<float> & ranges, const std::vector<bool> & observed,
    const std::string & success_state)
  {
    update_coverage(ranges, observed);
    if (angular_coverage_ratio_ < minimum_angular_coverage_ratio_) {
      reject(
        "INCOMPLETE_REVOLUTION", "Angular observation coverage is below the configured minimum");
      return false;
    }
    if (finite_return_ratio_ < minimum_finite_return_ratio_) {
      reject(
        "INCOMPLETE_REVOLUTION", "Finite obstacle-return ratio is below the configured minimum");
      return false;
    }
    const auto receipt = now();
    if (last_output_receipt_.nanoseconds() > 0 &&
      (receipt - last_output_receipt_).seconds() < 1.0 / publish_rate_limit_hz_)
    {
      reject("INCOMPLETE_REVOLUTION", "Canonical output rate limit suppressed this revolution");
      return false;
    }

    sensor_msgs::msg::LaserScan output;
    output.header.stamp = stamp;
    output.header.frame_id = output_frame_.empty() ? last_input_frame_ : output_frame_;
    output.angle_min = static_cast<float>(output_angle_min_);
    output.angle_increment = static_cast<float>(output_angle_increment_);
    output.angle_max = static_cast<float>(
      output_angle_min_ + static_cast<double>(output_bins_ - 1) * output_angle_increment_);
    output.range_min = range_min;
    output.range_max = range_max;
    output.scan_time = scan_time;
    output.time_increment = scan_time > 0.0F ?
      scan_time / static_cast<float>(output_bins_) : 0.0F;
    output.ranges = ranges;
    output.intensities.clear();
    output_pub_->publish(output);
    ++output_messages_;
    last_output_stamp_ = stamp;
    last_output_angular_coverage_ratio_ = angular_coverage_ratio_;
    last_output_finite_return_ratio_ = finite_return_ratio_;
    last_success_state_ = success_state;
    update_rate(receipt, last_output_receipt_, output_rate_hz_);
    last_rejection_reason_ = "none";
    transition(success_state);
    return true;
  }

  void process_full_scan(const sensor_msgs::msg::LaserScan & scan, const rclcpp::Time & stamp)
  {
    std::vector<float> ranges(
      static_cast<std::size_t>(output_bins_), std::numeric_limits<float>::infinity());
    std::vector<bool> observed(static_cast<std::size_t>(output_bins_), false);
    rebin_scan_deskewed(scan, stamp, ranges, observed);
    update_coverage(ranges, observed);
    last_segment_count_ = 1;
    last_revolution_duration_sec_ = std::max(0.0, static_cast<double>(scan.scan_time));
    ++completed_revolutions_;
    if (!publish_grid(
        stamp, scan.scan_time, scan.range_min, scan.range_max, ranges, observed,
        "OK_FULL_SCAN"))
    {
      ++discarded_revolutions_;
    }
  }

  void reset_assembly()
  {
    assembly_active_ = false;
    assembly_ranges_.clear();
    assembly_observed_.clear();
    assembly_segment_count_ = 0;
    assembly_accumulated_span_rad_ = 0.0;
    assembly_range_min_ = 0.0F;
    assembly_range_max_ = 0.0F;
    have_previous_segment_end_ = false;
  }

  void begin_assembly(const sensor_msgs::msg::LaserScan & scan, const rclcpp::Time & stamp)
  {
    assembly_active_ = true;
    assembly_ranges_.assign(
      static_cast<std::size_t>(output_bins_), std::numeric_limits<float>::infinity());
    assembly_observed_.assign(static_cast<std::size_t>(output_bins_), false);
    assembly_first_stamp_ = stamp;
    assembly_last_stamp_ = stamp;
    assembly_segment_count_ = 0;
    assembly_accumulated_span_rad_ = 0.0;
    assembly_range_min_ = scan.range_min;
    assembly_range_max_ = scan.range_max;
    assembly_direction_ = scan.angle_increment > 0.0F ? 1 : -1;
    have_previous_segment_end_ = false;
  }

  bool segment_wrap_detected(const sensor_msgs::msg::LaserScan & scan) const
  {
    if (!assembly_active_ || !have_previous_segment_end_) {
      return false;
    }
    const double span = output_angle_max_ - output_angle_min_;
    const double start = normalize_angle(scan.angle_min) - output_angle_min_;
    if (assembly_direction_ > 0) {
      return previous_segment_end_normalized_ - start > 0.5 * span;
    }
    return start - previous_segment_end_normalized_ > 0.5 * span;
  }

  bool finish_assembly()
  {
    if (!assembly_active_) {
      return false;
    }
    update_coverage(assembly_ranges_, assembly_observed_);
    last_segment_count_ = assembly_segment_count_;
    last_revolution_duration_sec_ = std::max(
      0.0, (assembly_last_stamp_ - assembly_first_stamp_).seconds());
    const bool coherent = angular_coverage_ratio_ >= minimum_angular_coverage_ratio_ &&
      finite_return_ratio_ >= minimum_finite_return_ratio_ &&
      last_revolution_duration_sec_ <= maximum_revolution_duration_sec_;
    bool published = false;
    if (coherent) {
      ++completed_revolutions_;
      published = publish_grid(
        assembly_first_stamp_, static_cast<float>(last_revolution_duration_sec_),
        assembly_range_min_, assembly_range_max_, assembly_ranges_, assembly_observed_,
        "OK_ASSEMBLED_REVOLUTION");
      if (!published) {
        ++discarded_revolutions_;
      }
    } else {
      ++discarded_revolutions_;
      reject(
        "INCOMPLETE_REVOLUTION", "Discarding incomplete or over-duration assembled revolution",
        false);
    }
    reset_assembly();
    return published;
  }

  void process_partial_segment(
    const sensor_msgs::msg::LaserScan & scan, const rclcpp::Time & stamp)
  {
    if (assembly_active_) {
      const double gap = (stamp - assembly_last_stamp_).seconds();
      const double duration = (stamp - assembly_first_stamp_).seconds();
      if (gap < 0.0 || gap > maximum_segment_gap_sec_ ||
        duration > maximum_revolution_duration_sec_ ||
        assembly_segment_count_ >=
        static_cast<std::size_t>(maximum_input_messages_per_revolution_))
      {
        ++discarded_revolutions_;
        reject(
          "INCOMPLETE_REVOLUTION", "Partial-scan assembly exceeded gap, duration, or segment limit",
          false);
        reset_assembly();
      } else if (segment_wrap_detected(scan)) {
        finish_assembly();
      }
    }
    if (!assembly_active_) {
      begin_assembly(scan, stamp);
    }

    rebin_scan_deskewed(scan, stamp, assembly_ranges_, assembly_observed_);
    ++assembly_segment_count_;
    assembly_last_stamp_ = stamp;
    assembly_range_min_ = std::min(assembly_range_min_, scan.range_min);
    assembly_range_max_ = std::max(assembly_range_max_, scan.range_max);
    assembly_accumulated_span_rad_ += std::min(
      indexed_input_span_rad_, output_angle_max_ - output_angle_min_);
    const double end_angle = static_cast<double>(scan.angle_min) +
      static_cast<double>(scan.ranges.size() - 1) * static_cast<double>(scan.angle_increment);
    previous_segment_end_normalized_ = normalize_angle(end_angle) - output_angle_min_;
    have_previous_segment_end_ = true;
    update_coverage(assembly_ranges_, assembly_observed_);
    last_segment_count_ = assembly_segment_count_;
    last_revolution_duration_sec_ = std::max(
      0.0, (assembly_last_stamp_ - assembly_first_stamp_).seconds());

    const bool sufficient_span =
      assembly_accumulated_span_rad_ >= full_revolution_min_span_rad_;
    const bool nearly_complete = angular_coverage_ratio_ >= 0.995;
    if ((sufficient_span && angular_coverage_ratio_ >= minimum_angular_coverage_ratio_) ||
      nearly_complete)
    {
      finish_assembly();
    } else {
      transition("WAITING_FOR_MORE_ANGULAR_COVERAGE");
    }
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    const auto receipt = now();
    update_rate(receipt, last_input_receipt_, input_rate_hz_);
    ++input_messages_;
    input_array_length_ = scan->ranges.size();
    raw_length_min_ = std::min(raw_length_min_, input_array_length_);
    raw_length_max_ = std::max(raw_length_max_, input_array_length_);
    last_input_frame_ = scan->header.frame_id;
    publisher_count_ = get_publishers_info_by_topic(input_topic_).size();
    declared_input_span_rad_ = std::abs(
      static_cast<double>(scan->angle_max) - static_cast<double>(scan->angle_min));
    indexed_input_span_rad_ = scan->ranges.empty() ? 0.0 :
      std::abs(static_cast<double>(scan->angle_increment)) *
      static_cast<double>(scan->ranges.size() - 1);

    if (publisher_count_ != 1) {
      reset_assembly();
      reject("DUPLICATE_PUBLISHER", "Raw scan topic does not have exactly one publisher");
      return;
    }
    if (!metadata_valid(*scan)) {
      ++malformed_messages_;
      reset_assembly();
      reject("MALFORMED_SCAN", "Rejecting raw scan with malformed geometry or range metadata");
      return;
    }

    const rclcpp::Time stamp(scan->header.stamp);
    timestamp_monotonic_ = stamp.nanoseconds() > 0 &&
      (!have_input_stamp_ || stamp > last_input_stamp_);
    if (!timestamp_monotonic_) {
      ++malformed_messages_;
      reset_assembly();
      reject("NON_MONOTONIC_TIMESTAMP", "Rejecting non-monotonic raw scan timestamp");
      return;
    }
    last_input_stamp_ = stamp;
    have_input_stamp_ = true;

    const GeometryClass geometry = classify_geometry(
      *scan, declared_input_span_rad_, indexed_input_span_rad_);
    if (!mode_locked_) {
      update_auto_classification(geometry);
      return;
    }
    if (geometry_unstable_) {
      reject("INPUT_GEOMETRY_UNSTABLE", "Canonicalizer is locked out by unstable input geometry");
      return;
    }
    const bool expected_full = selected_mode_ == "per_message_full_scan";
    if ((expected_full && geometry != GeometryClass::Full) ||
      (!expected_full && geometry != GeometryClass::Partial))
    {
      geometry_unstable_ = true;
      reset_assembly();
      reject("INPUT_GEOMETRY_UNSTABLE", "Raw scan geometry changed after mode selection");
      return;
    }

    if (expected_full) {
      process_full_scan(*scan, stamp);
    } else {
      process_partial_segment(*scan, stamp);
    }
  }

  void write_health_csv(
    bool valid, const std::string & diagnostic_state, double diagnostic_coverage,
    double diagnostic_finite_ratio)
  {
    if (!health_csv_) {
      return;
    }
    health_csv_ << std::fixed << std::setprecision(9) << now().seconds() << ','
                << selected_mode_ << ',' << input_array_length_ << ',' << output_bins_ << ','
                << declared_input_span_rad_ << ',' << indexed_input_span_rad_ << ','
                << diagnostic_coverage << ',' << diagnostic_finite_ratio << ','
                << last_segment_count_ << ',' << last_revolution_duration_sec_ << ','
                << input_rate_hz_ << ',' << output_rate_hz_ << ',' << bool_string(valid) << ','
                << (valid ? "none" : diagnostic_state + ":" + last_rejection_reason_) << '\n';
    health_csv_.flush();
  }

  void publish_diagnostics()
  {
    std::string diagnostic_state = state_;
    const bool input_fresh = last_input_receipt_.nanoseconds() > 0 &&
      (now() - last_input_receipt_).seconds() <= maximum_segment_gap_sec_;
    const double output_fresh_limit = std::max(
      0.5, maximum_revolution_duration_sec_ + maximum_segment_gap_sec_);
    const bool output_fresh = last_output_receipt_.nanoseconds() > 0 &&
      (now() - last_output_receipt_).seconds() <= output_fresh_limit;
    if (!input_fresh && input_messages_ > 0) {
      diagnostic_state = "STALE_INPUT";
    }
    const double diagnostic_coverage = output_messages_ > 0 ?
      last_output_angular_coverage_ratio_ : angular_coverage_ratio_;
    const double diagnostic_finite_ratio = output_messages_ > 0 ?
      last_output_finite_return_ratio_ : finite_return_ratio_;
    const bool healthy = input_fresh && output_fresh && mode_locked_ &&
      !geometry_unstable_ && timestamp_monotonic_ &&
      diagnostic_coverage >= minimum_angular_coverage_ratio_ &&
      diagnostic_finite_ratio >= minimum_finite_return_ratio_;

    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.level = healthy ? diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.name = "laser_scan_canonicalizer";
    status.hardware_id = "rplidar_a1m8";
    status.message = diagnostic_state;
    status.values.push_back(key_value("state", diagnostic_state));
    status.values.push_back(key_value("last_success_state", last_success_state_));
    status.values.push_back(key_value("processing_mode", selected_mode_));
    status.values.push_back(key_value("input_messages", std::to_string(input_messages_)));
    status.values.push_back(key_value("output_messages", std::to_string(output_messages_)));
    status.values.push_back(key_value("discarded_messages", std::to_string(discarded_messages_)));
    status.values.push_back(key_value(
      "completed_revolutions", std::to_string(completed_revolutions_)));
    status.values.push_back(key_value(
      "discarded_revolutions", std::to_string(discarded_revolutions_)));
    status.values.push_back(key_value("input_array_length", std::to_string(input_array_length_)));
    status.values.push_back(key_value(
      "raw_length_min", raw_length_min_ == std::numeric_limits<std::size_t>::max() ?
      "0" : std::to_string(raw_length_min_)));
    status.values.push_back(key_value("raw_length_max", std::to_string(raw_length_max_)));
    status.values.push_back(key_value("output_array_length", std::to_string(output_bins_)));
    status.values.push_back(key_value(
      "declared_input_span_rad", std::to_string(declared_input_span_rad_)));
    status.values.push_back(key_value(
      "indexed_input_span_rad", std::to_string(indexed_input_span_rad_)));
    status.values.push_back(key_value(
      "angular_observation_coverage_ratio", std::to_string(diagnostic_coverage)));
    status.values.push_back(key_value(
      "finite_return_ratio", std::to_string(diagnostic_finite_ratio)));
    status.values.push_back(key_value(
      "malformed_sample_ratio", input_messages_ > 0 ?
      std::to_string(static_cast<double>(malformed_messages_) /
      static_cast<double>(input_messages_)) : "0.0"));
    status.values.push_back(key_value(
      "accumulated_segment_count", std::to_string(last_segment_count_)));
    status.values.push_back(key_value(
      "revolution_duration_sec", std::to_string(last_revolution_duration_sec_)));
    status.values.push_back(key_value("input_rate_hz", std::to_string(input_rate_hz_)));
    status.values.push_back(key_value("output_rate_hz", std::to_string(output_rate_hz_)));
    status.values.push_back(key_value("timestamp_monotonic", bool_string(timestamp_monotonic_)));
    status.values.push_back(key_value("deskew_enabled", bool_string(enable_deskew_)));
    status.values.push_back(key_value("deskew_applied", bool_string(last_deskew_applied_)));
    status.values.push_back(key_value("deskew_stamp_policy", deskew_stamp_policy_));
    status.values.push_back(key_value("deskewed_scans", std::to_string(deskewed_scans_)));
    status.values.push_back(key_value("deskew_fallbacks", std::to_string(deskew_fallbacks_)));
    status.values.push_back(key_value("deskew_status", last_deskew_status_));
    status.values.push_back(key_value("publisher_count", std::to_string(publisher_count_)));
    status.values.push_back(key_value("last_rejection_reason", last_rejection_reason_));
    array.status.push_back(status);
    diagnostics_pub_->publish(array);
    write_health_csv(healthy, diagnostic_state, diagnostic_coverage, diagnostic_finite_ratio);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string output_frame_;
  std::string requested_mode_;
  std::string selected_mode_;
  std::string diagnostics_topic_;
  std::string health_csv_path_;
  std::string deskew_fixed_frame_;
  std::string deskew_stamp_policy_;
  int output_bins_{720};
  int classification_window_messages_{20};
  int maximum_input_messages_per_revolution_{20};
  double output_angle_min_{-M_PI};
  double output_angle_max_{M_PI};
  double output_angle_increment_{0.0};
  double full_revolution_min_span_rad_{5.8};
  double minimum_angular_coverage_ratio_{0.70};
  double minimum_finite_return_ratio_{0.05};
  double maximum_revolution_duration_sec_{0.50};
  double maximum_segment_gap_sec_{0.20};
  double publish_rate_limit_hz_{12.0};
  double diagnostics_rate_hz_{1.0};
  double deskew_timeout_sec_{0.02};
  bool enable_deskew_{false};

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr input_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr output_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::ofstream health_csv_;

  rclcpp::Time last_input_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_output_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_input_receipt_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_output_receipt_{0, 0, RCL_ROS_TIME};
  bool have_input_stamp_{false};
  bool timestamp_monotonic_{true};
  bool mode_locked_{false};
  bool geometry_unstable_{false};
  std::size_t classification_samples_{0};
  std::size_t full_classification_samples_{0};
  std::size_t partial_classification_samples_{0};
  std::size_t malformed_classification_samples_{0};
  std::size_t input_messages_{0};
  std::size_t output_messages_{0};
  std::size_t discarded_messages_{0};
  std::size_t malformed_messages_{0};
  std::size_t completed_revolutions_{0};
  std::size_t discarded_revolutions_{0};
  std::size_t input_array_length_{0};
  std::size_t raw_length_min_{std::numeric_limits<std::size_t>::max()};
  std::size_t raw_length_max_{0};
  std::size_t current_valid_input_count_{0};
  std::size_t publisher_count_{0};
  std::size_t last_segment_count_{0};
  std::size_t deskewed_scans_{0};
  std::size_t deskew_fallbacks_{0};
  double declared_input_span_rad_{0.0};
  double indexed_input_span_rad_{0.0};
  double angular_coverage_ratio_{0.0};
  double finite_return_ratio_{0.0};
  double last_output_angular_coverage_ratio_{0.0};
  double last_output_finite_return_ratio_{0.0};
  double last_revolution_duration_sec_{0.0};
  double input_rate_hz_{0.0};
  double output_rate_hz_{0.0};
  std::string last_input_frame_;
  std::string last_rejection_reason_{"none"};
  std::string last_success_state_{"none"};
  std::string last_deskew_status_{"disabled"};
  std::string state_;
  bool last_deskew_applied_{false};

  bool assembly_active_{false};
  std::vector<float> assembly_ranges_;
  std::vector<bool> assembly_observed_;
  rclcpp::Time assembly_first_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time assembly_last_stamp_{0, 0, RCL_ROS_TIME};
  std::size_t assembly_segment_count_{0};
  double assembly_accumulated_span_rad_{0.0};
  float assembly_range_min_{0.0F};
  float assembly_range_max_{0.0F};
  int assembly_direction_{1};
  bool have_previous_segment_end_{false};
  double previous_segment_end_normalized_{0.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaserScanCanonicalizer>());
  rclcpp::shutdown();
  return 0;
}
