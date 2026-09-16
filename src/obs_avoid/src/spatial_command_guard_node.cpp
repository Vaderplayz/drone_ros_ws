#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <limits>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Matrix3x3.hpp"
#include "tf2/LinearMath/Quaternion.hpp"

#include "obs_avoid/spatial_command_guard.hpp"

using namespace std::chrono_literals;

namespace
{

double clamp(double value, double lower, double upper)
{
  return std::max(lower, std::min(upper, value));
}

double yawFromOdom(const nav_msgs::msg::Odometry & odom)
{
  const auto & orientation = odom.pose.pose.orientation;
  tf2::Quaternion quaternion(
    orientation.x, orientation.y, orientation.z, orientation.w);
  tf2::Matrix3x3 matrix(quaternion);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  matrix.getRPY(roll, pitch, yaw);
  return yaw;
}

void worldToBody(double yaw, double world_x, double world_y, double & body_x, double & body_y)
{
  const double cosine = std::cos(yaw);
  const double sine = std::sin(yaw);
  body_x = cosine * world_x + sine * world_y;
  body_y = -sine * world_x + cosine * world_y;
}

void bodyToWorld(double yaw, double body_x, double body_y, double & world_x, double & world_y)
{
  const double cosine = std::cos(yaw);
  const double sine = std::sin(yaw);
  world_x = cosine * body_x - sine * body_y;
  world_y = sine * body_x + cosine * body_y;
}

diagnostic_msgs::msg::KeyValue keyValue(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue output;
  output.key = key;
  output.value = value;
  return output;
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

}  // namespace

class SpatialCommandGuardNode : public rclcpp::Node
{
public:
  SpatialCommandGuardNode()
  : Node("spatial_command_guard")
  {
    input_command_topic_ = declare_parameter<std::string>(
      "input_command_topic", "/planner_cmd_vel_raw");
    output_command_topic_ = declare_parameter<std::string>(
      "output_command_topic", "/planner_cmd_vel");
    awareness_topic_ = declare_parameter<std::string>(
      "awareness_topic", "/mapping/spatial_awareness/status");
    odom_topic_ = declare_parameter<std::string>(
      "odom_topic", "/mavros/local_position/odom");
    diagnostics_topic_ = declare_parameter<std::string>(
      "diagnostics_topic", "/mapping/obstacle_avoidance/guard_status");
    command_xy_frame_ = declare_parameter<std::string>("command_xy_frame", "world");
    command_timeout_sec_ = declare_parameter<double>("command_timeout_sec", 0.30);
    awareness_timeout_sec_ = declare_parameter<double>("awareness_timeout_sec", 0.65);
    odom_timeout_sec_ = declare_parameter<double>("odom_timeout_sec", 0.35);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 20.0);
    diagnostics_rate_hz_ = declare_parameter<double>("diagnostics_rate_hz", 2.0);
    warning_speed_scale_ = declare_parameter<double>("warning_speed_scale", 0.25);
    unknown_blocks_motion_ = declare_parameter<bool>("unknown_blocks_motion", true);
    max_horizontal_speed_mps_ = declare_parameter<double>("max_horizontal_speed_mps", 0.30);
    max_vertical_speed_mps_ = declare_parameter<double>("max_vertical_speed_mps", 0.20);
    max_yaw_rate_rad_s_ = declare_parameter<double>("max_yaw_rate_rad_s", 0.30);

    if (command_xy_frame_ != "world" && command_xy_frame_ != "body") {
      throw std::invalid_argument("command_xy_frame must be 'world' or 'body'");
    }
    if (publish_rate_hz_ <= 0.0 || diagnostics_rate_hz_ <= 0.0) {
      throw std::invalid_argument("publish rates must be positive");
    }

    direction_states_.fill(obs_avoid::spatial_guard::DirectionState::Unknown);
    direction_clearances_.fill(std::numeric_limits<double>::quiet_NaN());
    last_guard_states_.fill(obs_avoid::spatial_guard::DirectionState::Unknown);

    const auto sensor_qos = rclcpp::SensorDataQoS();
    command_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      input_command_topic_, rclcpp::QoS(10).reliable(),
      [this](const geometry_msgs::msg::TwistStamped::SharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_command_ = *message;
        last_command_receive_ = now();
      });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, sensor_qos,
      [this](const nav_msgs::msg::Odometry::SharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_odom_ = *message;
        last_odom_receive_ = now();
      });
    awareness_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      awareness_topic_, rclcpp::QoS(10).reliable(),
      std::bind(&SpatialCommandGuardNode::awarenessCallback, this, std::placeholders::_1));

    command_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      output_command_topic_, rclcpp::QoS(10).reliable());
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      diagnostics_topic_, rclcpp::QoS(10).reliable());

    command_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / publish_rate_hz_)),
      std::bind(&SpatialCommandGuardNode::publishGuardedCommand, this));
    diagnostics_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / diagnostics_rate_hz_)),
      std::bind(&SpatialCommandGuardNode::publishDiagnostics, this));

    RCLCPP_INFO(
      get_logger(),
      "Spatial command guard ready: %s -> %s; awareness=%s; fail_closed=%s",
      input_command_topic_.c_str(), output_command_topic_.c_str(), awareness_topic_.c_str(),
      unknown_blocks_motion_ ? "true" : "false");
    RCLCPP_WARN(
      get_logger(),
      "This node publishes only an intermediate guarded topic; it does not arm, change mode, or publish to MAVROS.");
  }

private:
  using DirectionState = obs_avoid::spatial_guard::DirectionState;

  static DirectionState parseState(const std::string & value)
  {
    if (value == "CLEAR") {
      return DirectionState::Clear;
    }
    if (value == "WARNING") {
      return DirectionState::Warning;
    }
    if (value == "DANGER") {
      return DirectionState::Danger;
    }
    return DirectionState::Unknown;
  }

  static std::string stateName(DirectionState state)
  {
    switch (state) {
      case DirectionState::Clear: return "CLEAR";
      case DirectionState::Warning: return "WARNING";
      case DirectionState::Danger: return "DANGER";
      case DirectionState::Unknown: return "UNKNOWN";
    }
    return "UNKNOWN";
  }

  void awarenessCallback(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr message)
  {
    const diagnostic_msgs::msg::DiagnosticStatus * spatial_status = nullptr;
    for (const auto & status : message->status) {
      if (status.name == "spatial_awareness/local_obstacles") {
        spatial_status = &status;
        break;
      }
    }
    if (spatial_status == nullptr) {
      return;
    }

    std::unordered_map<std::string, std::string> values;
    values.reserve(spatial_status->values.size());
    for (const auto & value : spatial_status->values) {
      values[value.key] = value.value;
    }

    static const std::array<std::string, 6> names{
      "front", "rear", "left", "right", "top", "bottom"};
    std::array<DirectionState, 6> states;
    std::array<double, 6> clearances;
    states.fill(DirectionState::Unknown);
    clearances.fill(std::numeric_limits<double>::quiet_NaN());
    for (std::size_t index = 0; index < names.size(); ++index) {
      const auto state_iterator = values.find(names[index] + "_state");
      if (state_iterator != values.end()) {
        states[index] = parseState(state_iterator->second);
      }
      const auto clearance_iterator = values.find(names[index] + "_clearance_m");
      if (clearance_iterator != values.end()) {
        try {
          clearances[index] = std::stod(clearance_iterator->second);
        } catch (const std::exception &) {
          clearances[index] = std::numeric_limits<double>::quiet_NaN();
        }
      }
    }

    std::lock_guard<std::mutex> lock(mutex_);
    direction_states_ = states;
    direction_clearances_ = clearances;
    last_awareness_receive_ = now();
  }

  static double age(
    const std::optional<rclcpp::Time> & receive_time,
    const rclcpp::Time & current_time)
  {
    if (!receive_time.has_value()) {
      return std::numeric_limits<double>::infinity();
    }
    return std::max(0.0, (current_time - receive_time.value()).seconds());
  }

  void publishGuardedCommand()
  {
    std::optional<geometry_msgs::msg::TwistStamped> command;
    std::optional<nav_msgs::msg::Odometry> odom;
    std::array<DirectionState, 6> states;
    std::optional<rclcpp::Time> command_receive;
    std::optional<rclcpp::Time> odom_receive;
    std::optional<rclcpp::Time> awareness_receive;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      command = latest_command_;
      odom = latest_odom_;
      states = direction_states_;
      command_receive = last_command_receive_;
      odom_receive = last_odom_receive_;
      awareness_receive = last_awareness_receive_;
    }

    if (!command.has_value()) {
      updateGuardState("WAITING_FOR_COMMAND", false, {}, {}, states, 0.0);
      return;
    }

    const rclcpp::Time current_time = now();
    const double command_age = age(command_receive, current_time);
    const double odom_age = age(odom_receive, current_time);
    const double awareness_age = age(awareness_receive, current_time);
    const bool command_fresh = command_age <= command_timeout_sec_;
    const bool odom_fresh = odom.has_value() && odom_age <= odom_timeout_sec_;
    const bool awareness_fresh = awareness_age <= awareness_timeout_sec_;

    geometry_msgs::msg::TwistStamped output = command.value();
    output.header.stamp = current_time;
    obs_avoid::spatial_guard::GuardResult guard_result;
    double processing_ms = 0.0;
    const auto processing_start = std::chrono::steady_clock::now();

    std::string status = "ACTIVE";
    if (!command_fresh || !odom_fresh || !awareness_fresh) {
      output.twist = geometry_msgs::msg::Twist();
      status = !command_fresh ? "STOP_COMMAND_STALE" :
        (!odom_fresh ? "STOP_ODOM_STALE" : "STOP_AWARENESS_STALE");
    } else {
      double body_x = output.twist.linear.x;
      double body_y = output.twist.linear.y;
      const double yaw = yawFromOdom(odom.value());
      if (command_xy_frame_ == "world") {
        worldToBody(yaw, output.twist.linear.x, output.twist.linear.y, body_x, body_y);
      }

      obs_avoid::spatial_guard::BodyCommand body_command;
      body_command.x = body_x;
      body_command.y = body_y;
      body_command.z = output.twist.linear.z;
      body_command.yaw_rate = output.twist.angular.z;
      guard_result = obs_avoid::spatial_guard::applyDirectionalGuard(
        body_command, states, warning_speed_scale_, unknown_blocks_motion_);

      guard_result.command.x = clamp(
        guard_result.command.x, -max_horizontal_speed_mps_, max_horizontal_speed_mps_);
      guard_result.command.y = clamp(
        guard_result.command.y, -max_horizontal_speed_mps_, max_horizontal_speed_mps_);
      guard_result.command.z = clamp(
        guard_result.command.z, -max_vertical_speed_mps_, max_vertical_speed_mps_);
      guard_result.command.yaw_rate = clamp(
        guard_result.command.yaw_rate, -max_yaw_rate_rad_s_, max_yaw_rate_rad_s_);

      if (command_xy_frame_ == "world") {
        bodyToWorld(
          yaw, guard_result.command.x, guard_result.command.y,
          output.twist.linear.x, output.twist.linear.y);
      } else {
        output.twist.linear.x = guard_result.command.x;
        output.twist.linear.y = guard_result.command.y;
      }
      output.twist.linear.z = guard_result.command.z;
      output.twist.angular.x = 0.0;
      output.twist.angular.y = 0.0;
      output.twist.angular.z = guard_result.command.yaw_rate;
      if (std::any_of(
          guard_result.blocked.begin(), guard_result.blocked.end(),
          [](bool blocked) {return blocked;}))
      {
        status = "DIRECTION_BLOCKED";
      } else if (std::any_of(
          guard_result.warning_limited.begin(), guard_result.warning_limited.end(),
          [](bool limited) {return limited;}))
      {
        status = "WARNING_LIMITED";
      }
    }

    processing_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - processing_start).count();
    command_pub_->publish(output);
    updateGuardState(
      status, status == "ACTIVE" || status == "WARNING_LIMITED",
      guard_result.blocked, guard_result.warning_limited, states, processing_ms,
      command_age, odom_age, awareness_age);
  }

  void updateGuardState(
    const std::string & status,
    bool ready,
    const std::array<bool, 6> & blocked,
    const std::array<bool, 6> & limited,
    const std::array<DirectionState, 6> & states,
    double processing_ms,
    double command_age = std::numeric_limits<double>::infinity(),
    double odom_age = std::numeric_limits<double>::infinity(),
    double awareness_age = std::numeric_limits<double>::infinity())
  {
    std::lock_guard<std::mutex> lock(mutex_);
    guard_status_ = status;
    guard_ready_ = ready;
    last_blocked_ = blocked;
    last_limited_ = limited;
    last_guard_states_ = states;
    last_processing_ms_ = processing_ms;
    last_command_age_ = command_age;
    last_odom_age_ = odom_age;
    last_awareness_age_ = awareness_age;
    ++guard_publish_count_;
  }

  void publishDiagnostics()
  {
    std::string status_text;
    bool ready = false;
    std::array<bool, 6> blocked;
    std::array<bool, 6> limited;
    std::array<DirectionState, 6> states;
    std::array<double, 6> clearances;
    double processing_ms = 0.0;
    double command_age = 0.0;
    double odom_age = 0.0;
    double awareness_age = 0.0;
    std::uint64_t publish_count = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      status_text = guard_status_;
      ready = guard_ready_;
      blocked = last_blocked_;
      limited = last_limited_;
      states = last_guard_states_;
      clearances = direction_clearances_;
      processing_ms = last_processing_ms_;
      command_age = last_command_age_;
      odom_age = last_odom_age_;
      awareness_age = last_awareness_age_;
      publish_count = guard_publish_count_;
    }

    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "obstacle_avoidance/spatial_command_guard";
    status.hardware_id = "planner_spatial_guard";
    status.level = ready ? diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = status_text;
    status.values.push_back(keyValue("ready", ready ? "true" : "false"));
    status.values.push_back(keyValue("guard_status", status_text));
    status.values.push_back(keyValue("command_age_sec", formatDouble(command_age)));
    status.values.push_back(keyValue("odom_age_sec", formatDouble(odom_age)));
    status.values.push_back(keyValue("awareness_age_sec", formatDouble(awareness_age)));
    status.values.push_back(keyValue("processing_duration_ms", formatDouble(processing_ms)));
    status.values.push_back(keyValue("guard_publish_count", std::to_string(publish_count)));

    static const std::array<std::string, 6> names{
      "front", "rear", "left", "right", "top", "bottom"};
    for (std::size_t index = 0; index < names.size(); ++index) {
      status.values.push_back(keyValue(names[index] + "_state", stateName(states[index])));
      status.values.push_back(keyValue(
        names[index] + "_clearance_m", formatDouble(clearances[index])));
      status.values.push_back(keyValue(
        names[index] + "_blocked", blocked[index] ? "true" : "false"));
      status.values.push_back(keyValue(
        names[index] + "_warning_limited", limited[index] ? "true" : "false"));
    }
    array.status.push_back(status);
    diagnostics_pub_->publish(array);
  }

  std::string input_command_topic_;
  std::string output_command_topic_;
  std::string awareness_topic_;
  std::string odom_topic_;
  std::string diagnostics_topic_;
  std::string command_xy_frame_;
  double command_timeout_sec_{0.30};
  double awareness_timeout_sec_{0.65};
  double odom_timeout_sec_{0.35};
  double publish_rate_hz_{20.0};
  double diagnostics_rate_hz_{2.0};
  double warning_speed_scale_{0.25};
  bool unknown_blocks_motion_{true};
  double max_horizontal_speed_mps_{0.30};
  double max_vertical_speed_mps_{0.20};
  double max_yaw_rate_rad_s_{0.30};

  std::mutex mutex_;
  std::optional<geometry_msgs::msg::TwistStamped> latest_command_;
  std::optional<nav_msgs::msg::Odometry> latest_odom_;
  std::optional<rclcpp::Time> last_command_receive_;
  std::optional<rclcpp::Time> last_odom_receive_;
  std::optional<rclcpp::Time> last_awareness_receive_;
  std::array<DirectionState, 6> direction_states_{};
  std::array<double, 6> direction_clearances_{};
  std::string guard_status_{"WAITING_FOR_COMMAND"};
  bool guard_ready_{false};
  std::array<bool, 6> last_blocked_{};
  std::array<bool, 6> last_limited_{};
  std::array<DirectionState, 6> last_guard_states_{};
  double last_processing_ms_{0.0};
  double last_command_age_{std::numeric_limits<double>::infinity()};
  double last_odom_age_{std::numeric_limits<double>::infinity()};
  double last_awareness_age_{std::numeric_limits<double>::infinity()};
  std::uint64_t guard_publish_count_{0};

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr command_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr awareness_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr command_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr command_timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SpatialCommandGuardNode>());
  rclcpp::shutdown();
  return 0;
}
