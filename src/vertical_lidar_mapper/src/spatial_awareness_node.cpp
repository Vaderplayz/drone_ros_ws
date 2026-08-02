#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/LinearMath/Vector3.hpp>
#include <tf2/time.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "vertical_lidar_mapper/spatial_awareness_geometry.hpp"

namespace vertical_lidar_mapper
{

namespace awareness = spatial_awareness;

namespace
{

diagnostic_msgs::msg::KeyValue keyValue(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  return item;
}

std::string formatDouble(double value, int precision = 3)
{
  if (!std::isfinite(value)) {
    return "unknown";
  }
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(precision) << value;
  return stream.str();
}

geometry_msgs::msg::Point toRosPoint(const awareness::Point3 & point)
{
  geometry_msgs::msg::Point output;
  output.x = point.x;
  output.y = point.y;
  output.z = point.z;
  return output;
}

std_msgs::msg::ColorRGBA stateColor(awareness::ClearanceState state, float alpha = 1.0F)
{
  std_msgs::msg::ColorRGBA color;
  color.a = alpha;
  switch (state) {
    case awareness::ClearanceState::Clear:
      color.r = 0.10F;
      color.g = 0.85F;
      color.b = 0.20F;
      break;
    case awareness::ClearanceState::Warning:
      color.r = 1.0F;
      color.g = 0.72F;
      color.b = 0.05F;
      break;
    case awareness::ClearanceState::Danger:
      color.r = 0.95F;
      color.g = 0.08F;
      color.b = 0.06F;
      break;
    case awareness::ClearanceState::Unknown:
      color.r = 0.55F;
      color.g = 0.58F;
      color.b = 0.62F;
      break;
  }
  return color;
}

}  // namespace

class SpatialAwarenessNode : public rclcpp::Node
{
public:
  SpatialAwarenessNode()
  : Node("spatial_awareness")
  {
    declareAndValidateParameters();

    horizontal_direction_mask_ = parseDirectionMask(horizontal_directions_);
    vertical_direction_mask_ = parseDirectionMask(vertical_directions_);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    local_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      local_cloud_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile());
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      marker_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile());
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      diagnostics_topic_, rclcpp::QoS(rclcpp::KeepLast(5)).reliable().durability_volatile());

    horizontal_scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      horizontal_scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&SpatialAwarenessNode::horizontalScanCallback, this, std::placeholders::_1));
    vertical_scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      vertical_scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&SpatialAwarenessNode::verticalScanCallback, this, std::placeholders::_1));
    vertical_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      vertical_cloud_topic_, rclcpp::SensorDataQoS(),
      std::bind(&SpatialAwarenessNode::verticalCloudCallback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&SpatialAwarenessNode::odomCallback, this, std::placeholders::_1));

    processing_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / input_processing_hz_)),
      std::bind(&SpatialAwarenessNode::processPendingInputs, this));
    publish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / local_cloud_publish_hz_)),
      std::bind(&SpatialAwarenessNode::publishLocalAwareness, this));
    diagnostics_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / diagnostics_publish_hz_)),
      std::bind(&SpatialAwarenessNode::publishDiagnostics, this));

    last_results_ = emptyResults();
    RCLCPP_INFO(
      get_logger(),
      "Spatial awareness: horizontal=%s vertical_scan=%s vertical_cloud=%s fixed=%s output=%s",
      horizontal_scan_topic_.c_str(), vertical_scan_topic_.c_str(),
      vertical_cloud_topic_.c_str(), fixed_frame_.c_str(), base_frame_.c_str());
    RCLCPP_INFO(
      get_logger(),
      "Collision span %.2fx%.2fx%.2fm (propeller tips included), safety margins h=%.2fm v=%.2fm",
      effectiveLength(), effectiveWidth(), geometry_template_.body_height_m,
      geometry_template_.horizontal_safety_margin_m,
      geometry_template_.vertical_safety_margin_m);
  }

private:
  enum class SensorSource
  {
    Horizontal,
    Vertical
  };

  enum class InputResult
  {
    Success,
    Retry,
    Drop
  };

  struct PendingHorizontal
  {
    sensor_msgs::msg::LaserScan::ConstSharedPtr message;
    rclcpp::Time received_at;
  };

  struct PendingVertical
  {
    sensor_msgs::msg::PointCloud2::ConstSharedPtr message;
    rclcpp::Time received_at;
  };

  struct Observation
  {
    SensorSource source{SensorSource::Horizontal};
    rclcpp::Time measurement_stamp;
    rclcpp::Time received_at;
    std::vector<awareness::Point3> points_fixed;
  };

  struct PoseSample
  {
    rclcpp::Time stamp;
    rclcpp::Time received_at;
    tf2::Transform fixed_from_body;
  };

  struct LocalPoint
  {
    awareness::Point3 point;
    SensorSource source{SensorSource::Horizontal};
  };

  struct VoxelKey
  {
    std::int32_t x{0};
    std::int32_t y{0};
    std::int32_t z{0};

    bool operator==(const VoxelKey & other) const
    {
      return x == other.x && y == other.y && z == other.z;
    }
  };

  struct VoxelKeyHash
  {
    std::size_t operator()(const VoxelKey & key) const
    {
      std::size_t seed = std::hash<std::int32_t>{}(key.x);
      seed ^= std::hash<std::int32_t>{}(key.y) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
      seed ^= std::hash<std::int32_t>{}(key.z) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
      return seed;
    }
  };

  void declareAndValidateParameters()
  {
    horizontal_scan_topic_ = declare_parameter<std::string>(
      "horizontal_scan_topic", "/scan_slam");
    vertical_scan_topic_ = declare_parameter<std::string>(
      "vertical_scan_topic", "/scan_vertical");
    vertical_cloud_topic_ = declare_parameter<std::string>(
      "vertical_cloud_topic", "/vertical_cloud");
    odom_topic_ = declare_parameter<std::string>(
      "odom_topic", "/mavros/local_position/odom");
    horizontal_scan_stamp_reference_ = declare_parameter<std::string>(
      "horizontal_scan_stamp_reference", "end");
    local_cloud_topic_ = declare_parameter<std::string>(
      "local_cloud_topic", "/mapping/local_obstacle_cloud");
    marker_topic_ = declare_parameter<std::string>(
      "marker_topic", "/mapping/spatial_awareness/markers");
    diagnostics_topic_ = declare_parameter<std::string>(
      "diagnostics_topic", "/mapping/spatial_awareness/status");
    fixed_frame_ = declare_parameter<std::string>("fixed_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");

    local_radius_m_ = declare_parameter<double>("local_radius_m", 5.0);
    rolling_window_sec_ = declare_parameter<double>("rolling_window_sec", 0.8);
    sensor_stale_timeout_sec_ = declare_parameter<double>("sensor_stale_timeout_sec", 0.6);
    odom_stale_timeout_sec_ = declare_parameter<double>("odom_stale_timeout_sec", 0.35);
    pending_message_timeout_sec_ = declare_parameter<double>("pending_message_timeout_sec", 0.5);
    odom_interpolation_max_gap_sec_ = declare_parameter<double>(
      "odom_interpolation_max_gap_sec", 0.12);
    odom_nearest_sample_tolerance_sec_ = declare_parameter<double>(
      "odom_nearest_sample_tolerance_sec", 0.06);
    input_processing_hz_ = declare_parameter<double>("input_processing_hz", 50.0);
    local_cloud_publish_hz_ = declare_parameter<double>("local_cloud_publish_hz", 10.0);
    diagnostics_publish_hz_ = declare_parameter<double>("diagnostics_publish_hz", 2.0);
    local_voxel_size_m_ = declare_parameter<double>("local_voxel_size_m", 0.05);
    min_obstacle_range_m_ = declare_parameter<double>("min_obstacle_range_m", 0.08);
    max_obstacle_range_m_ = declare_parameter<double>("max_obstacle_range_m", 12.0);
    max_pending_messages_ = declare_parameter<int>("max_pending_messages", 8);
    max_messages_per_cycle_ = declare_parameter<int>("max_messages_per_cycle", 4);
    max_observations_ = declare_parameter<int>("max_observations", 32);
    max_points_per_observation_ = declare_parameter<int>("max_points_per_observation", 5000);
    max_buffered_points_ = declare_parameter<int>("max_buffered_points", 80000);
    max_local_points_ = declare_parameter<int>("max_local_points", 30000);
    marker_lifetime_sec_ = declare_parameter<double>("marker_lifetime_sec", 0.35);

    geometry_template_.center.x = declare_parameter<double>("collision_center_x_m", 0.0);
    geometry_template_.center.y = declare_parameter<double>("collision_center_y_m", 0.0);
    geometry_template_.center.z = declare_parameter<double>("collision_center_z_offset_m", 0.0);
    geometry_template_.body_length_m = declare_parameter<double>("drone_body_length_m", 0.45);
    geometry_template_.body_width_m = declare_parameter<double>("drone_body_width_m", 0.35);
    geometry_template_.body_height_m = declare_parameter<double>("drone_body_height_m", 0.22);
    geometry_template_.propeller_tip_to_tip_length_m = declare_parameter<double>(
      "propeller_tip_to_tip_length_m", 0.75);
    geometry_template_.propeller_tip_to_tip_width_m = declare_parameter<double>(
      "propeller_tip_to_tip_width_m", 0.75);
    geometry_template_.horizontal_safety_margin_m = declare_parameter<double>(
      "horizontal_safety_margin_m", 0.40);
    geometry_template_.vertical_safety_margin_m = declare_parameter<double>(
      "vertical_safety_margin_m", 0.30);
    geometry_template_.horizontal_danger_margin_m = declare_parameter<double>(
      "horizontal_danger_margin_m", 0.15);
    geometry_template_.vertical_danger_margin_m = declare_parameter<double>(
      "vertical_danger_margin_m", 0.12);
    geometry_template_.self_filter_padding_m = declare_parameter<double>(
      "self_filter_padding_m", 0.04);

    horizontal_directions_ = declare_parameter<std::vector<std::string>>(
      "horizontal_sensor_directions", {"front", "rear", "left", "right"});
    vertical_directions_ = declare_parameter<std::vector<std::string>>(
      "vertical_sensor_directions", {"left", "right", "top", "bottom"});

    const bool invalid_geometry =
      geometry_template_.body_length_m <= 0.0 || geometry_template_.body_width_m <= 0.0 ||
      geometry_template_.body_height_m <= 0.0 ||
      geometry_template_.propeller_tip_to_tip_length_m <= 0.0 ||
      geometry_template_.propeller_tip_to_tip_width_m <= 0.0 ||
      geometry_template_.horizontal_safety_margin_m <= 0.0 ||
      geometry_template_.vertical_safety_margin_m <= 0.0 ||
      geometry_template_.horizontal_danger_margin_m < 0.0 ||
      geometry_template_.vertical_danger_margin_m < 0.0 ||
      geometry_template_.horizontal_danger_margin_m > geometry_template_.horizontal_safety_margin_m ||
      geometry_template_.vertical_danger_margin_m > geometry_template_.vertical_safety_margin_m ||
      geometry_template_.self_filter_padding_m < 0.0;
    const bool invalid_runtime =
      local_radius_m_ <= 0.0 || rolling_window_sec_ <= 0.0 ||
      sensor_stale_timeout_sec_ <= 0.0 || odom_stale_timeout_sec_ <= 0.0 ||
      pending_message_timeout_sec_ <= 0.0 || odom_interpolation_max_gap_sec_ <= 0.0 ||
      odom_nearest_sample_tolerance_sec_ < 0.0 || input_processing_hz_ <= 0.0 ||
      local_cloud_publish_hz_ <= 0.0 || diagnostics_publish_hz_ <= 0.0 ||
      local_voxel_size_m_ <= 0.0 || min_obstacle_range_m_ < 0.0 ||
      max_obstacle_range_m_ <= min_obstacle_range_m_ || max_pending_messages_ < 1 ||
      max_messages_per_cycle_ < 1 || max_observations_ < 1 ||
      max_points_per_observation_ < 1 || max_buffered_points_ < 1 ||
      max_local_points_ < 1 || marker_lifetime_sec_ <= 0.0;
    const bool invalid_stamp_reference =
      horizontal_scan_stamp_reference_ != "start" &&
      horizontal_scan_stamp_reference_ != "midpoint" &&
      horizontal_scan_stamp_reference_ != "end";
    if (invalid_geometry || invalid_runtime || invalid_stamp_reference ||
      fixed_frame_.empty() || base_frame_.empty())
    {
      throw std::invalid_argument("Invalid spatial-awareness parameters");
    }
  }

  std::uint8_t parseDirectionMask(const std::vector<std::string> & names) const
  {
    std::uint8_t mask = 0U;
    for (const auto & name : names) {
      bool found = false;
      for (const auto direction : awareness::kDirections) {
        if (name == awareness::directionName(direction)) {
          mask |= directionBit(direction);
          found = true;
          break;
        }
      }
      if (!found) {
        throw std::invalid_argument("Unknown spatial-awareness direction: " + name);
      }
    }
    return mask;
  }

  static std::uint8_t directionBit(awareness::Direction direction)
  {
    return static_cast<std::uint8_t>(1U << static_cast<std::size_t>(direction));
  }

  double effectiveLength() const
  {
    return std::max(
      geometry_template_.body_length_m,
      geometry_template_.propeller_tip_to_tip_length_m);
  }

  double effectiveWidth() const
  {
    return std::max(
      geometry_template_.body_width_m,
      geometry_template_.propeller_tip_to_tip_width_m);
  }

  void horizontalScanCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr message)
  {
    const rclcpp::Time received_at = now();
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_horizontal_scan_stamp_ = rclcpp::Time(message->header.stamp);
    last_horizontal_scan_receive_ = received_at;
    pending_horizontal_.push_back(PendingHorizontal{message, received_at});
    while (pending_horizontal_.size() > static_cast<std::size_t>(max_pending_messages_)) {
      pending_horizontal_.pop_front();
      ++pending_queue_overflows_;
    }
  }

  void verticalScanCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr message)
  {
    const bool metadata_valid = !message->ranges.empty() &&
      std::isfinite(message->angle_min) && std::isfinite(message->angle_increment) &&
      message->angle_increment != 0.0F;
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_vertical_scan_stamp_ = rclcpp::Time(message->header.stamp);
    last_vertical_scan_receive_ = now();
    if (metadata_valid) {
      last_vertical_scan_valid_receive_ = last_vertical_scan_receive_;
    } else {
      ++invalid_input_messages_;
    }
  }

  void verticalCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr message)
  {
    const rclcpp::Time received_at = now();
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_vertical_cloud_stamp_ = rclcpp::Time(message->header.stamp);
    last_vertical_cloud_receive_ = received_at;
    pending_vertical_.push_back(PendingVertical{message, received_at});
    while (pending_vertical_.size() > static_cast<std::size_t>(max_pending_messages_)) {
      pending_vertical_.pop_front();
      ++pending_queue_overflows_;
    }
  }

  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr message)
  {
    const auto & position = message->pose.pose.position;
    const auto & orientation = message->pose.pose.orientation;
    if (!std::isfinite(position.x) || !std::isfinite(position.y) || !std::isfinite(position.z) ||
      !std::isfinite(orientation.x) || !std::isfinite(orientation.y) ||
      !std::isfinite(orientation.z) || !std::isfinite(orientation.w))
    {
      ++invalid_input_messages_;
      return;
    }

    tf2::Quaternion rotation(
      orientation.x, orientation.y, orientation.z, orientation.w);
    if (rotation.length2() < 1e-12) {
      ++invalid_input_messages_;
      return;
    }
    rotation.normalize();
    tf2::Transform fixed_from_body(
      rotation, tf2::Vector3(position.x, position.y, position.z));

    if (!message->header.frame_id.empty() && message->header.frame_id != fixed_frame_) {
      try {
        const auto fixed_from_odom_msg = tf_buffer_->lookupTransform(
          fixed_frame_, message->header.frame_id, tf2::TimePointZero);
        tf2::Transform fixed_from_odom;
        tf2::fromMsg(fixed_from_odom_msg.transform, fixed_from_odom);
        fixed_from_body = fixed_from_odom * fixed_from_body;
      } catch (const tf2::TransformException &) {
        ++tf_lookup_failures_;
        ++tf_input_drops_;
        return;
      }
    }

    const PoseSample sample{
      rclcpp::Time(message->header.stamp), now(), fixed_from_body};
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!pose_buffer_.empty() && sample.stamp < pose_buffer_.back().stamp) {
      pose_buffer_.clear();
      ++time_reset_count_;
    }
    pose_buffer_.push_back(sample);
    last_odom_stamp_ = sample.stamp;
    last_odom_receive_ = sample.received_at;
    const double keep_sec = std::max(2.0, rolling_window_sec_ + 1.0);
    while (pose_buffer_.size() > 2U &&
      (sample.stamp - pose_buffer_.front().stamp).seconds() > keep_sec)
    {
      pose_buffer_.pop_front();
    }
    while (pose_buffer_.size() > 600U) {
      pose_buffer_.pop_front();
    }
  }

  void processPendingInputs()
  {
    int processed = 0;
    while (processed < max_messages_per_cycle_) {
      bool had_message = false;
      if (processOneHorizontal()) {
        had_message = true;
        ++processed;
      }
      if (processed < max_messages_per_cycle_ && processOneVertical()) {
        had_message = true;
        ++processed;
      }
      if (!had_message) {
        break;
      }
    }
  }

  bool processOneHorizontal()
  {
    PendingHorizontal pending;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (pending_horizontal_.empty()) {
        return false;
      }
      pending = pending_horizontal_.front();
      pending_horizontal_.pop_front();
    }

    const InputResult result = convertHorizontalScan(pending);
    if (result == InputResult::Retry && ageSeconds(pending.received_at) < pending_message_timeout_sec_) {
      std::lock_guard<std::mutex> lock(data_mutex_);
      pending_horizontal_.push_front(pending);
      return false;
    }
    if (result != InputResult::Success) {
      ++dropped_input_messages_;
    }
    return true;
  }

  bool processOneVertical()
  {
    PendingVertical pending;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (pending_vertical_.empty()) {
        return false;
      }
      pending = pending_vertical_.front();
      pending_vertical_.pop_front();
    }

    const InputResult result = convertVerticalCloud(pending);
    if (result == InputResult::Retry && ageSeconds(pending.received_at) < pending_message_timeout_sec_) {
      std::lock_guard<std::mutex> lock(data_mutex_);
      pending_vertical_.push_front(pending);
      return false;
    }
    if (result != InputResult::Success) {
      ++dropped_input_messages_;
    }
    return true;
  }

  InputResult convertHorizontalScan(const PendingHorizontal & pending)
  {
    const auto & scan = *pending.message;
    if (scan.header.frame_id.empty() || scan.ranges.empty() ||
      !std::isfinite(scan.angle_min) || !std::isfinite(scan.angle_increment) ||
      scan.angle_increment == 0.0F)
    {
      ++invalid_input_messages_;
      return InputResult::Drop;
    }

    rclcpp::Time pose_stamp(scan.header.stamp);
    const double duration = scan.time_increment > 0.0F ?
      static_cast<double>(scan.time_increment) * static_cast<double>(scan.ranges.size() - 1U) :
      std::max(0.0, static_cast<double>(scan.scan_time));
    if (horizontal_scan_stamp_reference_ == "start") {
      pose_stamp = pose_stamp + rclcpp::Duration::from_seconds(0.5 * duration);
    } else if (horizontal_scan_stamp_reference_ == "end") {
      pose_stamp = pose_stamp - rclcpp::Duration::from_seconds(0.5 * duration);
    }

    const auto fixed_from_body = interpolateBodyPose(pose_stamp);
    if (!fixed_from_body.has_value()) {
      ++pose_interpolation_failures_;
      return InputResult::Retry;
    }

    tf2::Transform body_from_sensor;
    if (scan.header.frame_id != base_frame_) {
      try {
        const auto transform_message = tf_buffer_->lookupTransform(
          base_frame_, scan.header.frame_id, tf2::TimePointZero);
        tf2::fromMsg(transform_message.transform, body_from_sensor);
      } catch (const tf2::TransformException &) {
        ++tf_lookup_failures_;
        ++tf_input_drops_;
        return InputResult::Retry;
      }
    } else {
      body_from_sensor.setIdentity();
    }

    std::vector<awareness::Point3> points;
    points.reserve(std::min(
      scan.ranges.size(), static_cast<std::size_t>(max_points_per_observation_)));
    const std::size_t stride = std::max<std::size_t>(
      1U, (scan.ranges.size() + static_cast<std::size_t>(max_points_per_observation_) - 1U) /
      static_cast<std::size_t>(max_points_per_observation_));
    for (std::size_t index = 0; index < scan.ranges.size(); index += stride) {
      const double range = scan.ranges[index];
      const double minimum = std::max(
        min_obstacle_range_m_, static_cast<double>(scan.range_min));
      const double maximum = std::min(
        max_obstacle_range_m_, static_cast<double>(scan.range_max));
      if (!std::isfinite(range) || range < minimum || range > maximum) {
        continue;
      }
      const double angle = static_cast<double>(scan.angle_min) +
        static_cast<double>(index) * static_cast<double>(scan.angle_increment);
      const tf2::Vector3 point_sensor(range * std::cos(angle), range * std::sin(angle), 0.0);
      const tf2::Vector3 point_fixed = fixed_from_body.value() * (body_from_sensor * point_sensor);
      points.push_back(awareness::Point3{
        point_fixed.x(), point_fixed.y(), point_fixed.z()});
    }

    addObservation(Observation{
      SensorSource::Horizontal, rclcpp::Time(scan.header.stamp), pending.received_at,
      std::move(points)});
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      last_horizontal_valid_receive_ = now();
    }
    ++horizontal_observations_integrated_;
    return InputResult::Success;
  }

  InputResult convertVerticalCloud(const PendingVertical & pending)
  {
    const auto & cloud = *pending.message;
    if (cloud.header.frame_id.empty() || cloud.width * cloud.height == 0U) {
      ++invalid_input_messages_;
      return InputResult::Drop;
    }

    tf2::Transform fixed_from_cloud;
    if (cloud.header.frame_id != fixed_frame_) {
      try {
        const auto transform_message = tf_buffer_->lookupTransform(
          fixed_frame_, cloud.header.frame_id, rclcpp::Time(cloud.header.stamp));
        tf2::fromMsg(transform_message.transform, fixed_from_cloud);
      } catch (const tf2::TransformException &) {
        ++tf_lookup_failures_;
        ++tf_input_drops_;
        return InputResult::Retry;
      }
    } else {
      fixed_from_cloud.setIdentity();
    }

    const std::size_t input_points = static_cast<std::size_t>(cloud.width) * cloud.height;
    const std::size_t stride = std::max<std::size_t>(
      1U, (input_points + static_cast<std::size_t>(max_points_per_observation_) - 1U) /
      static_cast<std::size_t>(max_points_per_observation_));
    std::vector<awareness::Point3> points;
    points.reserve(std::min(
      input_points, static_cast<std::size_t>(max_points_per_observation_)));
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x_iterator(cloud, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y_iterator(cloud, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z_iterator(cloud, "z");
      for (std::size_t index = 0; index < input_points;
        ++index, ++x_iterator, ++y_iterator, ++z_iterator)
      {
        if (index % stride != 0U) {
          continue;
        }
        const tf2::Vector3 point_cloud(*x_iterator, *y_iterator, *z_iterator);
        if (!std::isfinite(point_cloud.x()) || !std::isfinite(point_cloud.y()) ||
          !std::isfinite(point_cloud.z()))
        {
          continue;
        }
        const tf2::Vector3 point_fixed = fixed_from_cloud * point_cloud;
        points.push_back(awareness::Point3{
          point_fixed.x(), point_fixed.y(), point_fixed.z()});
      }
    } catch (const std::runtime_error &) {
      ++invalid_input_messages_;
      return InputResult::Drop;
    }

    addObservation(Observation{
      SensorSource::Vertical, rclcpp::Time(cloud.header.stamp), pending.received_at,
      std::move(points)});
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      last_vertical_cloud_valid_receive_ = now();
    }
    ++vertical_observations_integrated_;
    return InputResult::Success;
  }

  std::optional<tf2::Transform> interpolateBodyPose(const rclcpp::Time & stamp)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (pose_buffer_.empty()) {
      return std::nullopt;
    }
    if (pose_buffer_.size() == 1U) {
      if (std::fabs((stamp - pose_buffer_.front().stamp).seconds()) <=
        odom_nearest_sample_tolerance_sec_)
      {
        return pose_buffer_.front().fixed_from_body;
      }
      return std::nullopt;
    }

    if (stamp <= pose_buffer_.front().stamp) {
      if ((pose_buffer_.front().stamp - stamp).seconds() <= odom_nearest_sample_tolerance_sec_) {
        return pose_buffer_.front().fixed_from_body;
      }
      return std::nullopt;
    }
    if (stamp >= pose_buffer_.back().stamp) {
      if ((stamp - pose_buffer_.back().stamp).seconds() <= odom_nearest_sample_tolerance_sec_) {
        return pose_buffer_.back().fixed_from_body;
      }
      return std::nullopt;
    }

    auto upper = std::upper_bound(
      pose_buffer_.begin(), pose_buffer_.end(), stamp,
      [](const rclcpp::Time & value, const PoseSample & sample) {
        return value < sample.stamp;
      });
    if (upper == pose_buffer_.begin() || upper == pose_buffer_.end()) {
      return std::nullopt;
    }
    const auto lower = std::prev(upper);
    const double gap = (upper->stamp - lower->stamp).seconds();
    if (gap <= 0.0 || gap > odom_interpolation_max_gap_sec_) {
      return std::nullopt;
    }
    const double ratio = std::clamp((stamp - lower->stamp).seconds() / gap, 0.0, 1.0);
    const tf2::Vector3 translation = lower->fixed_from_body.getOrigin().lerp(
      upper->fixed_from_body.getOrigin(), ratio);
    tf2::Quaternion rotation = lower->fixed_from_body.getRotation().slerp(
      upper->fixed_from_body.getRotation(), ratio);
    rotation.normalize();
    return tf2::Transform(rotation, translation);
  }

  void addObservation(Observation observation)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    buffered_point_count_ += observation.points_fixed.size();
    observations_.push_back(std::move(observation));
    while (observations_.size() > static_cast<std::size_t>(max_observations_) ||
      buffered_point_count_ > static_cast<std::size_t>(max_buffered_points_))
    {
      buffered_point_count_ -= observations_.front().points_fixed.size();
      observations_.pop_front();
      ++observation_evictions_;
    }
  }

  void purgeExpiredObservations(const rclcpp::Time & current_time)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    while (!observations_.empty()) {
      const double age = (current_time - observations_.front().received_at).seconds();
      if (age >= 0.0 && age <= rolling_window_sec_) {
        break;
      }
      buffered_point_count_ -= observations_.front().points_fixed.size();
      observations_.pop_front();
    }
  }

  std::vector<Observation> observationSnapshot() const
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    return std::vector<Observation>(observations_.begin(), observations_.end());
  }

  void publishLocalAwareness()
  {
    const auto processing_start = std::chrono::steady_clock::now();
    const rclcpp::Time current_time = now();
    purgeExpiredObservations(current_time);

    tf2::Transform body_from_fixed;
    bool output_tf_valid = true;
    try {
      const auto transform_message = tf_buffer_->lookupTransform(
        base_frame_, fixed_frame_, tf2::TimePointZero);
      tf2::fromMsg(transform_message.transform, body_from_fixed);
    } catch (const tf2::TransformException &) {
      output_tf_valid = false;
      ++tf_lookup_failures_;
      ++output_tf_failures_;
      body_from_fixed.setIdentity();
    }

    const double horizontal_age = receiveAge(last_horizontal_valid_receive_, current_time);
    const double vertical_scan_age = receiveAge(last_vertical_scan_valid_receive_, current_time);
    const double vertical_cloud_age = receiveAge(last_vertical_cloud_valid_receive_, current_time);
    const double odom_age = receiveAge(last_odom_receive_, current_time);
    const bool odom_fresh = odom_age <= odom_stale_timeout_sec_;
    const bool horizontal_fresh = output_tf_valid && odom_fresh &&
      horizontal_age <= sensor_stale_timeout_sec_;
    const bool vertical_fresh = output_tf_valid && odom_fresh &&
      vertical_scan_age <= sensor_stale_timeout_sec_ &&
      vertical_cloud_age <= sensor_stale_timeout_sec_;

    awareness::CollisionGeometry geometry = geometry_template_;
    if (output_tf_valid) {
      const auto latest_pose = latestBodyPose();
      if (latest_pose.has_value()) {
        const tf2::Vector3 center_in_body =
          body_from_fixed * latest_pose.value().getOrigin();
        geometry.center.x += center_in_body.x();
        geometry.center.y += center_in_body.y();
        geometry.center.z += center_in_body.z();
      }
    }

    std::vector<LocalPoint> local_points;
    if (output_tf_valid) {
      local_points = buildLocalPoints(
        body_from_fixed, geometry, horizontal_fresh, vertical_fresh);
    }
    const auto results = classifyDirections(
      local_points, geometry, horizontal_fresh, vertical_fresh);

    publishLocalCloud(local_points, current_time);
    publishMarkers(results, geometry, current_time);

    const auto processing_end = std::chrono::steady_clock::now();
    const double processing_ms = std::chrono::duration<double, std::milli>(
      processing_end - processing_start).count();
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      last_results_ = results;
      last_geometry_ = geometry;
      last_horizontal_fresh_ = horizontal_fresh;
      last_vertical_fresh_ = vertical_fresh;
      last_odom_fresh_ = odom_fresh;
      last_output_tf_valid_ = output_tf_valid;
      last_local_point_count_ = local_points.size();
      last_processing_duration_ms_ = processing_ms;
      max_processing_duration_ms_ = std::max(max_processing_duration_ms_, processing_ms);
      ++local_cloud_publish_count_;
    }
  }

  std::optional<tf2::Transform> latestBodyPose() const
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (pose_buffer_.empty()) {
      return std::nullopt;
    }
    return pose_buffer_.back().fixed_from_body;
  }

  std::vector<LocalPoint> buildLocalPoints(
    const tf2::Transform & body_from_fixed,
    const awareness::CollisionGeometry & geometry,
    bool horizontal_fresh,
    bool vertical_fresh) const
  {
    const auto observations = observationSnapshot();
    std::vector<LocalPoint> output;
    output.reserve(std::min(
      buffered_point_count_, static_cast<std::size_t>(max_local_points_)));
    std::unordered_set<VoxelKey, VoxelKeyHash> occupied_voxels;
    occupied_voxels.reserve(static_cast<std::size_t>(max_local_points_));
    const double radius_squared = local_radius_m_ * local_radius_m_;

    for (auto observation = observations.rbegin(); observation != observations.rend(); ++observation) {
      const bool source_fresh = observation->source == SensorSource::Horizontal ?
        horizontal_fresh : vertical_fresh;
      if (!source_fresh) {
        continue;
      }
      for (const auto & point_fixed : observation->points_fixed) {
        const tf2::Vector3 transformed = body_from_fixed * tf2::Vector3(
          point_fixed.x, point_fixed.y, point_fixed.z);
        const awareness::Point3 point{transformed.x(), transformed.y(), transformed.z()};
        if (!awareness::isFinite(point)) {
          continue;
        }
        const double dx = point.x - geometry.center.x;
        const double dy = point.y - geometry.center.y;
        const double dz = point.z - geometry.center.z;
        if (dx * dx + dy * dy + dz * dz > radius_squared ||
          awareness::isSelfPoint(point, geometry))
        {
          continue;
        }
        const VoxelKey key{
          static_cast<std::int32_t>(std::floor(point.x / local_voxel_size_m_)),
          static_cast<std::int32_t>(std::floor(point.y / local_voxel_size_m_)),
          static_cast<std::int32_t>(std::floor(point.z / local_voxel_size_m_))};
        if (!occupied_voxels.insert(key).second) {
          continue;
        }
        output.push_back(LocalPoint{point, observation->source});
        if (output.size() >= static_cast<std::size_t>(max_local_points_)) {
          return output;
        }
      }
    }
    return output;
  }

  std::array<awareness::DirectionResult, 6> classifyDirections(
    const std::vector<LocalPoint> & points,
    const awareness::CollisionGeometry & geometry,
    bool horizontal_fresh,
    bool vertical_fresh) const
  {
    auto results = emptyResults();
    for (std::size_t index = 0; index < results.size(); ++index) {
      const auto direction = results[index].direction;
      const bool horizontal_coverage = horizontal_fresh &&
        (horizontal_direction_mask_ & directionBit(direction)) != 0U;
      const bool vertical_coverage = vertical_fresh &&
        (vertical_direction_mask_ & directionBit(direction)) != 0U;
      results[index].sensor_coverage = horizontal_coverage || vertical_coverage;

      for (const auto & point : points) {
        const bool source_covers_direction = point.source == SensorSource::Horizontal ?
          horizontal_coverage : vertical_coverage;
        if (!source_covers_direction) {
          continue;
        }
        const auto clearance = awareness::directionalClearance(
          point.point, direction, geometry);
        if (!clearance.has_value()) {
          continue;
        }
        if (!results[index].clearance_m.has_value() ||
          clearance.value() < results[index].clearance_m.value())
        {
          results[index].clearance_m = clearance;
          results[index].nearest_point = point.point;
        }
      }
      results[index].state = awareness::classifyClearance(
        results[index].clearance_m, direction, results[index].sensor_coverage, geometry);
    }
    return results;
  }

  static std::array<awareness::DirectionResult, 6> emptyResults()
  {
    std::array<awareness::DirectionResult, 6> results;
    for (std::size_t index = 0; index < awareness::kDirections.size(); ++index) {
      results[index].direction = awareness::kDirections[index];
    }
    return results;
  }

  void publishLocalCloud(
    const std::vector<LocalPoint> & points,
    const rclcpp::Time & stamp)
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = base_frame_;
    cloud.header.stamp = stamp;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());
    sensor_msgs::PointCloud2Iterator<float> x_iterator(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> y_iterator(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> z_iterator(cloud, "z");
    for (const auto & point : points) {
      *x_iterator = static_cast<float>(point.point.x);
      *y_iterator = static_cast<float>(point.point.y);
      *z_iterator = static_cast<float>(point.point.z);
      ++x_iterator;
      ++y_iterator;
      ++z_iterator;
    }
    cloud.is_dense = true;
    local_cloud_pub_->publish(cloud);
  }

  void publishMarkers(
    const std::array<awareness::DirectionResult, 6> & results,
    const awareness::CollisionGeometry & geometry,
    const rclcpp::Time & stamp)
  {
    visualization_msgs::msg::MarkerArray marker_array;
    const auto physical = awareness::physicalBounds(geometry);
    const auto safety = awareness::safetyBounds(geometry);

    visualization_msgs::msg::Marker collision;
    initializeMarker(collision, "collision_volume", 0, stamp);
    collision.type = visualization_msgs::msg::Marker::CUBE;
    collision.pose.position = toRosPoint(geometry.center);
    collision.pose.orientation.w = 1.0;
    collision.scale.x = physical.max_x - physical.min_x;
    collision.scale.y = physical.max_y - physical.min_y;
    collision.scale.z = physical.max_z - physical.min_z;
    collision.color.r = 0.15F;
    collision.color.g = 0.45F;
    collision.color.b = 1.0F;
    collision.color.a = 0.30F;
    marker_array.markers.push_back(collision);

    visualization_msgs::msg::Marker safety_marker;
    initializeMarker(safety_marker, "safety_volume", 1, stamp);
    safety_marker.type = visualization_msgs::msg::Marker::CUBE;
    safety_marker.pose.position = toRosPoint(geometry.center);
    safety_marker.pose.orientation.w = 1.0;
    safety_marker.scale.x = safety.max_x - safety.min_x;
    safety_marker.scale.y = safety.max_y - safety.min_y;
    safety_marker.scale.z = safety.max_z - safety.min_z;
    safety_marker.color.r = 0.15F;
    safety_marker.color.g = 0.65F;
    safety_marker.color.b = 1.0F;
    safety_marker.color.a = 0.10F;
    marker_array.markers.push_back(safety_marker);

    for (std::size_t index = 0; index < results.size(); ++index) {
      const auto & result = results[index];
      const auto unit = awareness::directionUnitVector(result.direction);
      const auto surface = awareness::collisionSurfacePoint(result.direction, geometry);
      const double arrow_length = awareness::isVertical(result.direction) ?
        geometry.vertical_safety_margin_m + 0.25 :
        geometry.horizontal_safety_margin_m + 0.25;
      const awareness::Point3 arrow_end{
        surface.x + unit.x * arrow_length,
        surface.y + unit.y * arrow_length,
        surface.z + unit.z * arrow_length};

      visualization_msgs::msg::Marker arrow;
      initializeMarker(arrow, "direction_arrow", 10 + static_cast<int>(index), stamp);
      arrow.type = visualization_msgs::msg::Marker::ARROW;
      arrow.points = {toRosPoint(surface), toRosPoint(arrow_end)};
      arrow.scale.x = 0.035;
      arrow.scale.y = 0.085;
      arrow.scale.z = 0.11;
      arrow.color = stateColor(result.state);
      marker_array.markers.push_back(arrow);

      visualization_msgs::msg::Marker text;
      initializeMarker(text, "direction_text", 30 + static_cast<int>(index), stamp);
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.pose.position.x = arrow_end.x + unit.x * 0.10;
      text.pose.position.y = arrow_end.y + unit.y * 0.10;
      text.pose.position.z = arrow_end.z + unit.z * 0.10 + 0.04;
      text.pose.orientation.w = 1.0;
      text.scale.z = 0.13;
      text.color = stateColor(result.state);
      text.text = awareness::directionName(result.direction) + " " +
        awareness::stateName(result.state);
      if (result.clearance_m.has_value()) {
        text.text += "\n" + formatDouble(result.clearance_m.value(), 2) + " m";
      }
      marker_array.markers.push_back(text);

      if (result.nearest_point.has_value()) {
        visualization_msgs::msg::Marker nearest;
        initializeMarker(nearest, "nearest_obstacle", 50 + static_cast<int>(index), stamp);
        nearest.type = visualization_msgs::msg::Marker::SPHERE;
        nearest.pose.position = toRosPoint(result.nearest_point.value());
        nearest.pose.orientation.w = 1.0;
        nearest.scale.x = 0.11;
        nearest.scale.y = 0.11;
        nearest.scale.z = 0.11;
        nearest.color = stateColor(result.state);
        marker_array.markers.push_back(nearest);

        visualization_msgs::msg::Marker line;
        initializeMarker(line, "nearest_line", 70 + static_cast<int>(index), stamp);
        line.type = visualization_msgs::msg::Marker::LINE_LIST;
        line.points = {toRosPoint(surface), toRosPoint(result.nearest_point.value())};
        line.scale.x = 0.018;
        line.color = stateColor(result.state, 0.75F);
        marker_array.markers.push_back(line);
      }
    }
    marker_pub_->publish(marker_array);
  }

  void initializeMarker(
    visualization_msgs::msg::Marker & marker,
    const std::string & marker_namespace,
    int id,
    const rclcpp::Time & stamp) const
  {
    marker.header.frame_id = base_frame_;
    marker.header.stamp = stamp;
    marker.ns = marker_namespace;
    marker.id = id;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.lifetime = rclcpp::Duration::from_seconds(marker_lifetime_sec_);
  }

  void publishDiagnostics()
  {
    const rclcpp::Time current_time = now();
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = current_time;
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "spatial_awareness/local_obstacles";
    status.hardware_id = "horizontal_and_vertical_lidar";

    std::array<awareness::DirectionResult, 6> results;
    bool horizontal_fresh;
    bool vertical_fresh;
    bool odom_fresh;
    bool output_tf_valid;
    std::size_t local_points;
    std::size_t observation_count;
    std::size_t buffered_points;
    std::size_t horizontal_pending;
    std::size_t vertical_pending;
    double processing_ms;
    double max_processing_ms;
    std::uint64_t publish_count;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      results = last_results_;
      horizontal_fresh = last_horizontal_fresh_;
      vertical_fresh = last_vertical_fresh_;
      odom_fresh = last_odom_fresh_;
      output_tf_valid = last_output_tf_valid_;
      local_points = last_local_point_count_;
      observation_count = observations_.size();
      buffered_points = buffered_point_count_;
      horizontal_pending = pending_horizontal_.size();
      vertical_pending = pending_vertical_.size();
      processing_ms = last_processing_duration_ms_;
      max_processing_ms = max_processing_duration_ms_;
      publish_count = local_cloud_publish_count_;
    }

    const double interval = last_diagnostics_time_.has_value() ?
      (current_time - last_diagnostics_time_.value()).seconds() : 0.0;
    double update_rate_hz = last_update_rate_hz_;
    if (interval > 0.0) {
      update_rate_hz = static_cast<double>(publish_count - last_diagnostics_publish_count_) / interval;
    }
    last_diagnostics_time_ = current_time;
    last_diagnostics_publish_count_ = publish_count;
    last_update_rate_hz_ = update_rate_hz;

    const bool degraded = !horizontal_fresh || !vertical_fresh || !odom_fresh || !output_tf_valid;
    status.level = degraded ?
      diagnostic_msgs::msg::DiagnosticStatus::WARN :
      diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = degraded ? "DEGRADED: missing data is UNKNOWN" : "OK";

    status.values.push_back(keyValue(
      "horizontal_scan_age_sec", formatDouble(headerAge(last_horizontal_scan_stamp_, current_time))));
    status.values.push_back(keyValue(
      "horizontal_receive_age_sec", formatDouble(receiveAge(last_horizontal_scan_receive_, current_time))));
    status.values.push_back(keyValue(
      "vertical_scan_age_sec", formatDouble(headerAge(last_vertical_scan_stamp_, current_time))));
    status.values.push_back(keyValue(
      "vertical_receive_age_sec", formatDouble(receiveAge(last_vertical_scan_receive_, current_time))));
    status.values.push_back(keyValue(
      "vertical_cloud_age_sec", formatDouble(headerAge(last_vertical_cloud_stamp_, current_time))));
    status.values.push_back(keyValue(
      "vertical_cloud_receive_age_sec", formatDouble(receiveAge(last_vertical_cloud_receive_, current_time))));
    status.values.push_back(keyValue(
      "odom_age_sec", formatDouble(headerAge(last_odom_stamp_, current_time))));
    status.values.push_back(keyValue("horizontal_sensor_state", horizontal_fresh ? "FRESH" : "STALE_OR_UNKNOWN"));
    status.values.push_back(keyValue("vertical_sensor_state", vertical_fresh ? "FRESH" : "STALE_OR_UNKNOWN"));
    status.values.push_back(keyValue("odom_state", odom_fresh ? "FRESH" : "STALE_OR_UNKNOWN"));
    status.values.push_back(keyValue("output_tf_state", output_tf_valid ? "AVAILABLE" : "UNAVAILABLE"));
    status.values.push_back(keyValue("local_cloud_update_rate_hz", formatDouble(update_rate_hz, 2)));
    status.values.push_back(keyValue("local_point_count", std::to_string(local_points)));
    status.values.push_back(keyValue("rolling_observation_count", std::to_string(observation_count)));
    status.values.push_back(keyValue("buffered_point_count", std::to_string(buffered_points)));
    status.values.push_back(keyValue("horizontal_pending_messages", std::to_string(horizontal_pending)));
    status.values.push_back(keyValue("vertical_pending_messages", std::to_string(vertical_pending)));

    for (const auto & result : results) {
      const std::string prefix = awareness::directionName(result.direction);
      status.values.push_back(keyValue(prefix + "_state", awareness::stateName(result.state)));
      status.values.push_back(keyValue(
        prefix + "_clearance_m",
        result.clearance_m.has_value() ? formatDouble(result.clearance_m.value()) :
        (result.sensor_coverage ? "inf" : "unknown")));
    }

    std::vector<std::string> stale_warnings;
    if (!horizontal_fresh) {stale_warnings.push_back("horizontal_lidar");}
    if (!vertical_fresh) {stale_warnings.push_back("vertical_lidar");}
    if (!odom_fresh) {stale_warnings.push_back("full_pose_odom");}
    if (!output_tf_valid) {stale_warnings.push_back("base_tf");}
    std::ostringstream warning_stream;
    for (std::size_t index = 0; index < stale_warnings.size(); ++index) {
      if (index > 0U) {warning_stream << ',';}
      warning_stream << stale_warnings[index];
    }
    status.values.push_back(keyValue(
      "stale_sensor_warnings", warning_stream.str().empty() ? "none" : warning_stream.str()));
    status.values.push_back(keyValue("tf_lookup_failures", std::to_string(tf_lookup_failures_)));
    status.values.push_back(keyValue("tf_input_drops", std::to_string(tf_input_drops_)));
    status.values.push_back(keyValue("output_tf_failures", std::to_string(output_tf_failures_)));
    status.values.push_back(keyValue(
      "pose_interpolation_failures", std::to_string(pose_interpolation_failures_)));
    status.values.push_back(keyValue("pending_queue_overflows", std::to_string(pending_queue_overflows_)));
    status.values.push_back(keyValue("observation_evictions", std::to_string(observation_evictions_)));
    status.values.push_back(keyValue("invalid_input_messages", std::to_string(invalid_input_messages_)));
    status.values.push_back(keyValue("dropped_input_messages", std::to_string(dropped_input_messages_)));
    status.values.push_back(keyValue(
      "horizontal_observations_integrated", std::to_string(horizontal_observations_integrated_)));
    status.values.push_back(keyValue(
      "vertical_observations_integrated", std::to_string(vertical_observations_integrated_)));
    status.values.push_back(keyValue("processing_duration_ms", formatDouble(processing_ms)));
    status.values.push_back(keyValue("max_processing_duration_ms", formatDouble(max_processing_ms)));
    status.values.push_back(keyValue("time_reset_count", std::to_string(time_reset_count_)));
    array.status.push_back(status);
    diagnostics_pub_->publish(array);
  }

  double ageSeconds(const rclcpp::Time & time) const
  {
    const double age = (now() - time).seconds();
    return age < 0.0 ? 0.0 : age;
  }

  static double receiveAge(
    const std::optional<rclcpp::Time> & receive_time,
    const rclcpp::Time & current_time)
  {
    if (!receive_time.has_value()) {
      return std::numeric_limits<double>::infinity();
    }
    return std::max(0.0, (current_time - receive_time.value()).seconds());
  }

  static double headerAge(
    const std::optional<rclcpp::Time> & message_time,
    const rclcpp::Time & current_time)
  {
    if (!message_time.has_value()) {
      return std::numeric_limits<double>::infinity();
    }
    return (current_time - message_time.value()).seconds();
  }

  std::string horizontal_scan_topic_;
  std::string vertical_scan_topic_;
  std::string vertical_cloud_topic_;
  std::string odom_topic_;
  std::string horizontal_scan_stamp_reference_;
  std::string local_cloud_topic_;
  std::string marker_topic_;
  std::string diagnostics_topic_;
  std::string fixed_frame_;
  std::string base_frame_;
  std::vector<std::string> horizontal_directions_;
  std::vector<std::string> vertical_directions_;

  double local_radius_m_{5.0};
  double rolling_window_sec_{0.8};
  double sensor_stale_timeout_sec_{0.6};
  double odom_stale_timeout_sec_{0.35};
  double pending_message_timeout_sec_{0.5};
  double odom_interpolation_max_gap_sec_{0.12};
  double odom_nearest_sample_tolerance_sec_{0.06};
  double input_processing_hz_{50.0};
  double local_cloud_publish_hz_{10.0};
  double diagnostics_publish_hz_{2.0};
  double local_voxel_size_m_{0.05};
  double min_obstacle_range_m_{0.08};
  double max_obstacle_range_m_{12.0};
  int max_pending_messages_{8};
  int max_messages_per_cycle_{4};
  int max_observations_{32};
  int max_points_per_observation_{5000};
  int max_buffered_points_{80000};
  int max_local_points_{30000};
  double marker_lifetime_sec_{0.35};
  awareness::CollisionGeometry geometry_template_;
  std::uint8_t horizontal_direction_mask_{0U};
  std::uint8_t vertical_direction_mask_{0U};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr horizontal_scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr vertical_scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr vertical_cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_cloud_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr processing_timer_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;

  mutable std::mutex data_mutex_;
  std::deque<PendingHorizontal> pending_horizontal_;
  std::deque<PendingVertical> pending_vertical_;
  std::deque<Observation> observations_;
  std::deque<PoseSample> pose_buffer_;
  std::size_t buffered_point_count_{0U};
  std::optional<rclcpp::Time> last_horizontal_scan_stamp_;
  std::optional<rclcpp::Time> last_horizontal_scan_receive_;
  std::optional<rclcpp::Time> last_horizontal_valid_receive_;
  std::optional<rclcpp::Time> last_vertical_scan_stamp_;
  std::optional<rclcpp::Time> last_vertical_scan_receive_;
  std::optional<rclcpp::Time> last_vertical_scan_valid_receive_;
  std::optional<rclcpp::Time> last_vertical_cloud_stamp_;
  std::optional<rclcpp::Time> last_vertical_cloud_receive_;
  std::optional<rclcpp::Time> last_vertical_cloud_valid_receive_;
  std::optional<rclcpp::Time> last_odom_stamp_;
  std::optional<rclcpp::Time> last_odom_receive_;
  std::array<awareness::DirectionResult, 6> last_results_;
  awareness::CollisionGeometry last_geometry_;
  bool last_horizontal_fresh_{false};
  bool last_vertical_fresh_{false};
  bool last_odom_fresh_{false};
  bool last_output_tf_valid_{false};
  std::size_t last_local_point_count_{0U};
  double last_processing_duration_ms_{0.0};
  double max_processing_duration_ms_{0.0};
  std::uint64_t local_cloud_publish_count_{0U};
  std::optional<rclcpp::Time> last_diagnostics_time_;
  std::uint64_t last_diagnostics_publish_count_{0U};
  double last_update_rate_hz_{0.0};

  std::uint64_t tf_lookup_failures_{0U};
  std::uint64_t tf_input_drops_{0U};
  std::uint64_t output_tf_failures_{0U};
  std::uint64_t pose_interpolation_failures_{0U};
  std::uint64_t pending_queue_overflows_{0U};
  std::uint64_t observation_evictions_{0U};
  std::uint64_t invalid_input_messages_{0U};
  std::uint64_t dropped_input_messages_{0U};
  std::uint64_t horizontal_observations_integrated_{0U};
  std::uint64_t vertical_observations_integrated_{0U};
  std::uint64_t time_reset_count_{0U};
};

}  // namespace vertical_lidar_mapper

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<vertical_lidar_mapper::SpatialAwarenessNode>());
  rclcpp::shutdown();
  return 0;
}
