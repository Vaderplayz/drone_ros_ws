#include <chrono>
#include <cmath>
#include <functional>
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
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_ros/transform_broadcaster.h>

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

class Px4OdomFlattenNode : public rclcpp::Node
{
public:
  Px4OdomFlattenNode()
  : Node("px4_odom_flatten_node")
  {
    odom_topic_ = declare_parameter<std::string>(
      "odom_topic", "/mavros/local_position/odom");
    parent_frame_ = declare_parameter<std::string>("parent_frame", "odom");
    child_frame_ = declare_parameter<std::string>("child_frame", "base_footprint");
    diagnostics_topic_ = declare_parameter<std::string>(
      "diagnostics_topic", "/odom_flatten/diagnostics");
    source_timeout_sec_ = declare_parameter<double>("source_timeout_sec", 0.5);
    diagnostics_rate_hz_ = declare_parameter<double>("diagnostics_rate_hz", 1.0);
    if (source_timeout_sec_ <= 0.0 || diagnostics_rate_hz_ <= 0.0) {
      throw std::invalid_argument("Invalid odom flatten diagnostic parameters");
    }

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      diagnostics_topic_, rclcpp::QoS(10));
    sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&Px4OdomFlattenNode::odom_callback, this, std::placeholders::_1));
    diagnostics_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / diagnostics_rate_hz_)),
      std::bind(&Px4OdomFlattenNode::publish_diagnostics, this));

    RCLCPP_INFO(
      get_logger(), "Flattening %s to timestamped TF %s -> %s",
      odom_topic_.c_str(), parent_frame_.c_str(), child_frame_.c_str());
  }

private:
  void update_rate(const rclcpp::Time & receipt)
  {
    if (last_receipt_.nanoseconds() > 0) {
      const double period = (receipt - last_receipt_).seconds();
      if (period > 0.0 && std::isfinite(period)) {
        const double instant_rate = 1.0 / period;
        source_rate_hz_ = source_rate_hz_ > 0.0 ?
          0.8 * source_rate_hz_ + 0.2 * instant_rate : instant_rate;
      }
    }
    last_receipt_ = receipt;
  }

  void reject(const std::string & reason)
  {
    ++rejected_count_;
    last_rejection_reason_ = reason;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", reason.c_str());
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const auto receipt = now();
    update_rate(receipt);
    source_frame_id_ = msg->header.frame_id;
    source_child_frame_id_ = msg->child_frame_id;
    const rclcpp::Time stamp(msg->header.stamp);
    source_stamp_ = stamp;
    source_age_sec_ = (receipt - stamp).seconds();

    if (stamp.nanoseconds() <= 0) {
      timestamp_monotonic_ = false;
      reject("zero source odometry timestamp");
      return;
    }
    timestamp_monotonic_ = !have_last_stamp_ || stamp > last_stamp_;
    if (!timestamp_monotonic_) {
      reject("non-monotonic source odometry timestamp");
      return;
    }

    const auto & position = msg->pose.pose.position;
    const auto & orientation = msg->pose.pose.orientation;
    if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
      !std::isfinite(orientation.x) || !std::isfinite(orientation.y) ||
      !std::isfinite(orientation.z) || !std::isfinite(orientation.w))
    {
      reject("non-finite source odometry pose");
      return;
    }

    tf2::Quaternion odom_q(
      orientation.x, orientation.y, orientation.z, orientation.w);
    source_quaternion_norm_ = std::sqrt(odom_q.length2());
    if (!std::isfinite(source_quaternion_norm_) || source_quaternion_norm_ < 1e-6) {
      reject("invalid source odometry quaternion norm");
      return;
    }
    odom_q.normalize();
    tf2::Matrix3x3(odom_q).getRPY(source_roll_, source_pitch_, source_yaw_);
    if (!std::isfinite(source_roll_) || !std::isfinite(source_pitch_) ||
      !std::isfinite(source_yaw_))
    {
      reject("non-finite source odometry attitude");
      return;
    }

    tf2::Quaternion yaw_q;
    yaw_q.setRPY(0.0, 0.0, source_yaw_);
    yaw_q.normalize();

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = msg->header.stamp;
    tf_msg.header.frame_id = parent_frame_;
    tf_msg.child_frame_id = child_frame_;
    tf_msg.transform.translation.x = position.x;
    tf_msg.transform.translation.y = position.y;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation.x = yaw_q.x();
    tf_msg.transform.rotation.y = yaw_q.y();
    tf_msg.transform.rotation.z = yaw_q.z();
    tf_msg.transform.rotation.w = yaw_q.w();
    tf_broadcaster_->sendTransform(tf_msg);

    output_x_ = position.x;
    output_y_ = position.y;
    output_yaw_ = source_yaw_;
    last_stamp_ = stamp;
    have_last_stamp_ = true;
    ++publish_count_;
    last_rejection_reason_ = "none";
  }

  void publish_diagnostics()
  {
    const bool waiting = !have_last_stamp_;
    const double age = last_receipt_.nanoseconds() > 0 ?
      (now() - last_receipt_).seconds() : std::numeric_limits<double>::infinity();
    const bool stale = !waiting && age > source_timeout_sec_;
    const bool healthy = !waiting && !stale && timestamp_monotonic_ &&
      last_rejection_reason_ == "none";
    const std::string state = waiting ? "WAITING_FOR_ODOM" :
      (stale ? "ODOM_STALE" : (healthy ? "OK" : "ODOM_REJECTED"));

    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.level = healthy ? diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.name = "px4_odom_flatten_node";
    status.hardware_id = "mavros_local_odometry";
    status.message = state;
    status.values.push_back(key_value("source_stamp", std::to_string(source_stamp_.seconds())));
    status.values.push_back(key_value("source_age_sec", std::to_string(source_age_sec_)));
    status.values.push_back(key_value("source_rate_hz", std::to_string(source_rate_hz_)));
    status.values.push_back(key_value("source_frame_id", source_frame_id_));
    status.values.push_back(key_value("source_child_frame_id", source_child_frame_id_));
    status.values.push_back(key_value(
      "source_quaternion_norm", std::to_string(source_quaternion_norm_)));
    status.values.push_back(key_value("source_roll_deg", std::to_string(source_roll_ * kRadToDeg)));
    status.values.push_back(key_value(
      "source_pitch_deg", std::to_string(source_pitch_ * kRadToDeg)));
    status.values.push_back(key_value("source_yaw_deg", std::to_string(source_yaw_ * kRadToDeg)));
    status.values.push_back(key_value("output_x", std::to_string(output_x_)));
    status.values.push_back(key_value("output_y", std::to_string(output_y_)));
    status.values.push_back(key_value("output_yaw_deg", std::to_string(output_yaw_ * kRadToDeg)));
    status.values.push_back(key_value("timestamp_monotonic", bool_string(timestamp_monotonic_)));
    status.values.push_back(key_value("publish_count", std::to_string(publish_count_)));
    status.values.push_back(key_value("rejected_count", std::to_string(rejected_count_)));
    status.values.push_back(key_value("last_rejection_reason", last_rejection_reason_));
    array.status.push_back(status);
    diagnostics_pub_->publish(array);
  }

  std::string odom_topic_;
  std::string parent_frame_;
  std::string child_frame_;
  std::string diagnostics_topic_;
  double source_timeout_sec_{0.5};
  double diagnostics_rate_hz_{1.0};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time source_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_receipt_{0, 0, RCL_ROS_TIME};
  bool have_last_stamp_{false};
  bool timestamp_monotonic_{true};
  std::size_t publish_count_{0};
  std::size_t rejected_count_{0};
  double source_age_sec_{std::numeric_limits<double>::infinity()};
  double source_rate_hz_{0.0};
  double source_quaternion_norm_{0.0};
  double source_roll_{0.0};
  double source_pitch_{0.0};
  double source_yaw_{0.0};
  double output_x_{0.0};
  double output_y_{0.0};
  double output_yaw_{0.0};
  std::string source_frame_id_;
  std::string source_child_frame_id_;
  std::string last_rejection_reason_{"none"};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Px4OdomFlattenNode>());
  rclcpp::shutdown();
  return 0;
}
