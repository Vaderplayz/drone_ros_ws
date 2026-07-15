#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
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
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/exceptions.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace
{
struct PlanarPose
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

diagnostic_msgs::msg::KeyValue key_value(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  return item;
}

double shortest_angle(double from, double to)
{
  return std::atan2(std::sin(to - from), std::cos(to - from));
}

PlanarPose interpolate(const PlanarPose & first, const PlanarPose & second, double alpha)
{
  return {
    first.x + alpha * (second.x - first.x),
    first.y + alpha * (second.y - first.y),
    first.yaw + alpha * shortest_angle(first.yaw, second.yaw)
  };
}
}  // namespace

class LaserScanDeskewer : public rclcpp::Node
{
public:
  LaserScanDeskewer()
  : Node("laser_scan_deskewer")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/scan_rf2o");
    output_topic_ = declare_parameter<std::string>("output_topic", "/scan_deskewed");
    output_frame_ = declare_parameter<std::string>("output_frame", "laser_frame");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    diagnostics_topic_ = declare_parameter<std::string>(
      "diagnostics_topic", "/scan_deskewed/diagnostics");
    input_stamp_is_end_ = declare_parameter<bool>("input_stamp_is_end", true);
    pose_sample_stride_ = declare_parameter<int>("pose_sample_stride", 24);
    tf_lookup_timeout_sec_ = declare_parameter<double>("tf_lookup_timeout_sec", 0.03);
    output_bins_ = declare_parameter<int>("output_bins", 720);
    output_angle_min_ = declare_parameter<double>("output_angle_min", -M_PI);
    output_angle_max_ = declare_parameter<double>("output_angle_max", M_PI);
    diagnostics_rate_hz_ = declare_parameter<double>("diagnostics_rate_hz", 1.0);

    if (pose_sample_stride_ < 1 || tf_lookup_timeout_sec_ < 0.0 || output_bins_ < 2 ||
      output_angle_max_ <= output_angle_min_ || diagnostics_rate_hz_ <= 0.0)
    {
      throw std::invalid_argument("Invalid laser scan deskewer parameters");
    }

    output_angle_increment_ =
      (output_angle_max_ - output_angle_min_) / static_cast<double>(output_bins_);
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
      get_logger(),
      "Deskewing %s to %s in %s using %d-ray TF interpolation knots; input stamp is %s",
      input_topic_.c_str(), output_topic_.c_str(), odom_frame_.c_str(), pose_sample_stride_,
      input_stamp_is_end_ ? "scan end" : "scan start");
  }

private:
  bool lookup_pose(const rclcpp::Time & stamp, PlanarPose & pose, std::string & reason)
  {
    try {
      const auto transform = tf_buffer_->lookupTransform(
        odom_frame_, output_frame_, stamp,
        rclcpp::Duration::from_seconds(tf_lookup_timeout_sec_));
      tf2::Quaternion quaternion;
      tf2::fromMsg(transform.transform.rotation, quaternion);
      if (!std::isfinite(quaternion.length2()) || quaternion.length2() < 1e-12) {
        reason = "invalid TF quaternion";
        return false;
      }
      quaternion.normalize();
      pose.x = transform.transform.translation.x;
      pose.y = transform.transform.translation.y;
      pose.yaw = tf2::getYaw(quaternion);
      if (!std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.yaw)) {
        reason = "non-finite planar TF";
        return false;
      }
      return true;
    } catch (const tf2::TransformException & exception) {
      reason = exception.what();
      return false;
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

  void reject(const std::string & reason)
  {
    state_ = "REJECTED";
    last_rejection_reason_ = reason;
    ++rejected_scans_;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", reason.c_str());
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    ++input_scans_;
    const std::size_t input_size = scan->ranges.size();
    if (input_size != static_cast<std::size_t>(output_bins_) || input_size < 2 ||
      scan->header.frame_id != output_frame_ ||
      (scan->header.stamp.sec == 0 && scan->header.stamp.nanosec == 0) ||
      !std::isfinite(scan->angle_min) || !std::isfinite(scan->angle_increment) ||
      scan->angle_increment <= 0.0F || !std::isfinite(scan->scan_time) ||
      scan->scan_time <= 0.0F)
    {
      reject("invalid input scan geometry, frame, timestamp, or scan_time");
      return;
    }

    const rclcpp::Time message_stamp(scan->header.stamp);
    if (have_last_input_stamp_ && message_stamp <= last_input_stamp_) {
      reject("non-monotonic input scan timestamp");
      return;
    }
    last_input_stamp_ = message_stamp;
    have_last_input_stamp_ = true;

    const double duration = static_cast<double>(scan->scan_time);
    const rclcpp::Time start_stamp = input_stamp_is_end_ ?
      message_stamp - rclcpp::Duration::from_seconds(duration) : message_stamp;
    const rclcpp::Time reference_stamp =
      start_stamp + rclcpp::Duration::from_seconds(0.5 * duration);

    std::vector<std::size_t> knot_indices;
    for (std::size_t index = 0; index < input_size - 1;
      index += static_cast<std::size_t>(pose_sample_stride_))
    {
      knot_indices.push_back(index);
    }
    if (knot_indices.empty() || knot_indices.back() != input_size - 1) {
      knot_indices.push_back(input_size - 1);
    }

    std::vector<PlanarPose> knot_poses(knot_indices.size());
    for (std::size_t knot = 0; knot < knot_indices.size(); ++knot) {
      const double offset = duration * static_cast<double>(knot_indices[knot]) /
        static_cast<double>(input_size - 1);
      const auto stamp = start_stamp + rclcpp::Duration::from_seconds(offset);
      std::string reason;
      if (!lookup_pose(stamp, knot_poses[knot], reason)) {
        reject("TF unavailable inside scan: " + reason);
        return;
      }
    }

    PlanarPose reference_pose;
    std::string reference_reason;
    if (!lookup_pose(reference_stamp, reference_pose, reference_reason)) {
      reject("midpoint TF unavailable: " + reference_reason);
      return;
    }

    const double yaw_span = shortest_angle(knot_poses.front().yaw, knot_poses.back().yaw);
    yaw_change_deg_ = std::abs(yaw_span) * 180.0 / M_PI;
    translation_m_ = std::hypot(
      knot_poses.back().x - knot_poses.front().x,
      knot_poses.back().y - knot_poses.front().y);
    tf_knots_ = knot_poses.size();

    std::vector<float> output_ranges(
      static_cast<std::size_t>(output_bins_), std::numeric_limits<float>::infinity());
    std::size_t segment = 0;
    std::size_t valid_input = 0;
    for (std::size_t index = 0; index < input_size; ++index) {
      while (segment + 1 < knot_indices.size() - 1 && index > knot_indices[segment + 1]) {
        ++segment;
      }
      const std::size_t first_index = knot_indices[segment];
      const std::size_t second_index = knot_indices[segment + 1];
      const double alpha = second_index > first_index ?
        static_cast<double>(index - first_index) /
        static_cast<double>(second_index - first_index) : 0.0;
      const PlanarPose ray_pose = interpolate(
        knot_poses[segment], knot_poses[segment + 1], alpha);

      const float range = scan->ranges[index];
      if (!std::isfinite(range) || range < scan->range_min || range > scan->range_max) {
        continue;
      }
      ++valid_input;
      const double input_angle = static_cast<double>(scan->angle_min) +
        static_cast<double>(index) * static_cast<double>(scan->angle_increment);
      const double ray_x = static_cast<double>(range) * std::cos(input_angle);
      const double ray_y = static_cast<double>(range) * std::sin(input_angle);
      const double cos_ray = std::cos(ray_pose.yaw);
      const double sin_ray = std::sin(ray_pose.yaw);
      const double odom_x = ray_pose.x + cos_ray * ray_x - sin_ray * ray_y;
      const double odom_y = ray_pose.y + sin_ray * ray_x + cos_ray * ray_y;
      const double delta_x = odom_x - reference_pose.x;
      const double delta_y = odom_y - reference_pose.y;
      const double cos_reference = std::cos(reference_pose.yaw);
      const double sin_reference = std::sin(reference_pose.yaw);
      const double output_x = cos_reference * delta_x + sin_reference * delta_y;
      const double output_y = -sin_reference * delta_x + cos_reference * delta_y;
      const float output_range = static_cast<float>(std::hypot(output_x, output_y));
      const std::size_t output_bin = angle_to_bin(std::atan2(output_y, output_x));
      output_ranges[output_bin] = std::min(output_ranges[output_bin], output_range);
    }

    const std::size_t finite_output = static_cast<std::size_t>(std::count_if(
      output_ranges.begin(), output_ranges.end(),
        [](float range) {return std::isfinite(range);}));
    valid_input_ratio_ = static_cast<double>(valid_input) / static_cast<double>(input_size);
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
    output_pub_->publish(output);

    state_ = "OK";
    last_rejection_reason_ = "none";
    ++output_scans_;
  }

  void publish_diagnostics()
  {
    const bool healthy = state_ == "OK";
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.level = healthy ? diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.name = "laser_scan_deskewer";
    status.hardware_id = "rplidar_a1m8_planar_tf";
    status.message = state_;
    status.values.push_back(key_value("state", state_));
    status.values.push_back(key_value("published", healthy ? "true" : "false"));
    status.values.push_back(key_value("input_stamp_is_end",
      input_stamp_is_end_ ? "true" : "false"));
    status.values.push_back(key_value("input_scans", std::to_string(input_scans_)));
    status.values.push_back(key_value("output_scans", std::to_string(output_scans_)));
    status.values.push_back(key_value("rejected_scans", std::to_string(rejected_scans_)));
    status.values.push_back(key_value("tf_knots", std::to_string(tf_knots_)));
    status.values.push_back(key_value("yaw_change_during_scan_deg",
      std::to_string(yaw_change_deg_)));
    status.values.push_back(key_value("translation_during_scan_m", std::to_string(translation_m_)));
    status.values.push_back(key_value("valid_input_ratio", std::to_string(valid_input_ratio_)));
    status.values.push_back(key_value("output_finite_ratio", std::to_string(output_finite_ratio_)));
    status.values.push_back(key_value("last_rejection_reason", last_rejection_reason_));
    array.status.push_back(status);
    diagnostics_pub_->publish(array);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string output_frame_;
  std::string odom_frame_;
  std::string diagnostics_topic_;
  bool input_stamp_is_end_{true};
  int pose_sample_stride_{24};
  double tf_lookup_timeout_sec_{0.03};
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

  rclcpp::Time last_input_stamp_{0, 0, RCL_ROS_TIME};
  bool have_last_input_stamp_{false};
  std::string state_{"WAITING_FOR_SCAN"};
  std::string last_rejection_reason_{"none"};
  std::size_t input_scans_{0};
  std::size_t output_scans_{0};
  std::size_t rejected_scans_{0};
  std::size_t tf_knots_{0};
  double yaw_change_deg_{0.0};
  double translation_m_{0.0};
  double valid_input_ratio_{0.0};
  double output_finite_ratio_{0.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaserScanDeskewer>());
  rclcpp::shutdown();
  return 0;
}
