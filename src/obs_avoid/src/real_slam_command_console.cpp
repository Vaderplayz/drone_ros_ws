#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>

#include <poll.h>
#include <unistd.h>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

namespace
{
using SteadyClock = std::chrono::steady_clock;

bool is_finite_value(double value)
{
  return std::isfinite(value);
}

double age_seconds(const SteadyClock::time_point & stamp)
{
  return std::chrono::duration<double>(SteadyClock::now() - stamp).count();
}

std::string yes_no(bool value)
{
  return value ? "yes" : "no";
}
}  // namespace

class RealSlamCommandConsole : public rclcpp::Node
{
public:
  RealSlamCommandConsole()
  : Node("real_slam_command_console")
  {
    setpoint_hz_ = declare_parameter<double>("setpoint_hz", 20.0);
    offboard_warmup_sec_ = declare_parameter<double>("offboard_warmup_sec", 2.0);
    odom_timeout_sec_ = declare_parameter<double>("odom_timeout_sec", 0.30);
    scan_timeout_sec_ = declare_parameter<double>("scan_timeout_sec", 0.30);
    planner_timeout_sec_ = declare_parameter<double>("planner_timeout_sec", 0.30);
    precland_mode_ = declare_parameter<std::string>("precland_mode", "");

    setpoint_hz_ = std::max(1.0, setpoint_hz_);
    offboard_warmup_sec_ = std::max(0.5, offboard_warmup_sec_);

    const auto sensor_qos = rclcpp::SensorDataQoS();
    state_sub_ = create_subscription<mavros_msgs::msg::State>(
      "/mavros/state", rclcpp::QoS(10),
      [this](mavros_msgs::msg::State::SharedPtr msg) { on_state(*msg); });
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/mavros/local_position/pose", sensor_qos,
      [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        pose_ = *msg;
        have_pose_ = true;
        pose_rx_ = SteadyClock::now();
      });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/mavros/local_position/odom", sensor_qos,
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {
        odom_ = *msg;
        have_odom_ = true;
        odom_rx_ = SteadyClock::now();
      });
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", sensor_qos,
      [this](sensor_msgs::msg::LaserScan::SharedPtr) {
        have_scan_ = true;
        scan_rx_ = SteadyClock::now();
      });
    planner_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "/planner_cmd_vel", rclcpp::QoS(10),
      [this](geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        if (!finite_twist(*msg)) {
          RCLCPP_ERROR(get_logger(), "Rejected non-finite planner command; forwarding disabled");
          disable_forwarding("non-finite planner command", true);
          return;
        }
        planner_cmd_ = *msg;
        have_planner_cmd_ = true;
        planner_rx_ = SteadyClock::now();
      });

    goal_pub_ = create_publisher<geometry_msgs::msg::Point>(
      "/drone_goal", rclcpp::QoS(10).reliable().transient_local());
    setpoint_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      "/mavros/setpoint_velocity/cmd_vel", rclcpp::QoS(10));
    arm_client_ = create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    mode_client_ = create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");

    const auto setpoint_period = std::chrono::duration<double>(1.0 / setpoint_hz_);
    setpoint_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(setpoint_period),
      [this]() { setpoint_tick(); });
    console_timer_ = create_wall_timer(100ms, [this]() { poll_console(); });
    health_timer_ = create_wall_timer(5s, [this]() { log_health_summary(); });

    RCLCPP_INFO(
      get_logger(),
      "Console ready: setpoint_hz=%.1f warmup=%.2fs timeouts odom/scan/planner=%.2f/%.2f/%.2fs precland_mode='%s'",
      setpoint_hz_, offboard_warmup_sec_, odom_timeout_sec_, scan_timeout_sec_,
      planner_timeout_sec_, precland_mode_.c_str());
    print_help();
    print_prompt();
  }

private:
  bool finite_twist(const geometry_msgs::msg::TwistStamped & msg) const
  {
    return is_finite_value(msg.twist.linear.x) && is_finite_value(msg.twist.linear.y) &&
           is_finite_value(msg.twist.linear.z) && is_finite_value(msg.twist.angular.x) &&
           is_finite_value(msg.twist.angular.y) && is_finite_value(msg.twist.angular.z);
  }

  bool odom_fresh() const
  {
    return have_odom_ && age_seconds(odom_rx_) <= odom_timeout_sec_;
  }

  bool scan_fresh() const
  {
    return have_scan_ && age_seconds(scan_rx_) <= scan_timeout_sec_;
  }

  bool planner_fresh() const
  {
    return have_planner_cmd_ && age_seconds(planner_rx_) <= planner_timeout_sec_;
  }

  void on_state(const mavros_msgs::msg::State & msg)
  {
    const bool changed = !have_state_ || msg.connected != state_.connected ||
      msg.armed != state_.armed || msg.mode != state_.mode;
    state_ = msg;
    have_state_ = true;
    state_rx_ = SteadyClock::now();

    if (changed) {
      RCLCPP_INFO(
        get_logger(), "MAVROS state: connected=%s armed=%s mode=%s",
        yes_no(state_.connected).c_str(), yes_no(state_.armed).c_str(), state_.mode.c_str());
    }

    if (forwarding_enabled_ && (!state_.connected || !state_.armed || state_.mode != "OFFBOARD")) {
      disable_forwarding("MAVROS state no longer permits planner forwarding", true);
    }
    if (offboard_warmup_active_ && (!state_.connected || !state_.armed)) {
      offboard_warmup_active_ = false;
      offboard_mode_requested_ = false;
      RCLCPP_WARN(get_logger(), "OFFBOARD warmup cancelled by MAVROS state change");
    }
  }

  void publish_zero()
  {
    geometry_msgs::msg::TwistStamped zero;
    zero.header.stamp = now();
    zero.header.frame_id = "map";
    setpoint_pub_->publish(zero);
  }

  void disable_forwarding(const std::string & reason, bool publish_stop)
  {
    const bool was_active = forwarding_enabled_ || forwarding_pending_;
    forwarding_enabled_ = false;
    forwarding_pending_ = false;
    if (publish_stop) {
      publish_zero();
    }
    if (was_active) {
      RCLCPP_WARN(get_logger(), "Planner forwarding disabled: %s", reason.c_str());
    }
  }

  void setpoint_tick()
  {
    if (offboard_warmup_active_) {
      if (!have_state_ || !state_.connected || !state_.armed || !odom_fresh() || !scan_fresh()) {
        offboard_warmup_active_ = false;
        publish_zero();
        RCLCPP_ERROR(get_logger(), "OFFBOARD warmup failed: state, odometry, or scan became invalid");
        return;
      }

      publish_zero();
      if (!offboard_mode_requested_ && age_seconds(offboard_warmup_start_) >= offboard_warmup_sec_) {
        offboard_mode_requested_ = true;
        RCLCPP_INFO(get_logger(), "OFFBOARD warmup complete; requesting OFFBOARD mode");
        request_mode("OFFBOARD");
      } else if (offboard_mode_requested_ && state_.mode == "OFFBOARD") {
        offboard_warmup_active_ = false;
        offboard_mode_requested_ = false;
        RCLCPP_INFO(get_logger(), "PX4 reports OFFBOARD; continuous zero setpoint hold remains active");
      } else if (offboard_mode_requested_ &&
        age_seconds(offboard_warmup_start_) > offboard_warmup_sec_ + 3.0)
      {
        offboard_warmup_active_ = false;
        offboard_mode_requested_ = false;
        RCLCPP_ERROR(get_logger(), "OFFBOARD request timed out; zero setpoint stream stopped");
      }
      return;
    }

    if (forwarding_pending_) {
      publish_zero();
      if (planner_fresh()) {
        forwarding_pending_ = false;
        forwarding_enabled_ = true;
        RCLCPP_INFO(get_logger(), "Fresh planner command received; forwarding enabled");
      } else if (age_seconds(goal_command_time_) > planner_timeout_sec_) {
        disable_forwarding("planner did not respond to goal before timeout", true);
      }
    }

    if (forwarding_enabled_) {
      const bool permitted = have_state_ && state_.connected && state_.armed &&
        state_.mode == "OFFBOARD" && odom_fresh() && scan_fresh() && planner_fresh();
      if (!permitted) {
        disable_forwarding("stale data or invalid MAVROS state", true);
        return;
      }

      auto command = planner_cmd_;
      command.header.stamp = now();
      setpoint_pub_->publish(command);
      return;
    }

    if (have_state_ && state_.connected && state_.mode == "OFFBOARD") {
      publish_zero();
    }
  }

  void poll_console()
  {
    pollfd input{};
    input.fd = STDIN_FILENO;
    input.events = POLLIN;
    const int result = ::poll(&input, 1, 0);
    if (result < 0) {
      RCLCPP_ERROR(get_logger(), "Terminal input poll failed");
      rclcpp::shutdown();
      return;
    }
    if (result == 0) {
      return;
    }
    if ((input.revents & (POLLHUP | POLLERR | POLLNVAL)) != 0) {
      RCLCPP_WARN(get_logger(), "Terminal input closed; shutting down console");
      rclcpp::shutdown();
      return;
    }
    if ((input.revents & POLLIN) == 0) {
      return;
    }

    std::string line;
    if (!std::getline(std::cin, line)) {
      RCLCPP_INFO(get_logger(), "Terminal EOF; shutting down console");
      rclcpp::shutdown();
      return;
    }
    process_command(line);
    if (rclcpp::ok()) {
      print_prompt();
    }
  }

  void process_command(const std::string & line)
  {
    std::istringstream stream(line);
    std::string command;
    stream >> command;
    if (command.empty()) {
      return;
    }
    RCLCPP_INFO(get_logger(), "User command: %s", line.c_str());

    if (command == "help") {
      if (!reject_trailing(stream, command)) print_help();
    } else if (command == "status") {
      if (!reject_trailing(stream, command)) print_status();
    } else if (command == "arm") {
      if (!reject_trailing(stream, command)) request_arm(true);
    } else if (command == "disarm") {
      if (!reject_trailing(stream, command)) request_arm(false);
    } else if (command == "offboard") {
      if (!reject_trailing(stream, command)) start_offboard_warmup();
    } else if (command == "goal") {
      command_goal(stream);
    } else if (command == "hold") {
      if (!reject_trailing(stream, command)) command_hold();
    } else if (command == "sethome") {
      if (!reject_trailing(stream, command)) command_sethome();
    } else if (command == "return") {
      if (!reject_trailing(stream, command)) command_return();
    } else if (command == "land") {
      if (!reject_trailing(stream, command)) command_land();
    } else if (command == "precland") {
      if (!reject_trailing(stream, command)) command_precland();
    } else if (command == "quit") {
      if (!reject_trailing(stream, command)) {
        disable_forwarding("console quit", true);
        RCLCPP_INFO(get_logger(), "Shutdown reason: user requested quit");
        rclcpp::shutdown();
      }
    } else {
      RCLCPP_ERROR(get_logger(), "Rejected unknown command '%s'; use 'help'", command.c_str());
    }
  }

  bool reject_trailing(std::istringstream & stream, const std::string & command)
  {
    std::string extra;
    if (stream >> extra) {
      RCLCPP_ERROR(get_logger(), "Rejected '%s': unexpected argument '%s'", command.c_str(), extra.c_str());
      return true;
    }
    return false;
  }

  void print_help() const
  {
    std::cout <<
      "Commands:\n"
      "  help                 Show this command list\n"
      "  status               Show MAVROS and sensor freshness\n"
      "  arm | disarm         Explicitly request arming state\n"
      "  offboard             Stream zero for warmup, then request OFFBOARD\n"
      "  goal X Y Z           Absolute PX4 local ENU goal\n"
      "  hold                 Disable planner forwarding and stream zero in OFFBOARD\n"
      "  sethome              Record current local ENU position\n"
      "  return               Send recorded home through obstacle planner\n"
      "  land                 Stop forwarding and request AUTO.LAND\n"
      "  precland             Stop forwarding and request configured precland_mode\n"
      "  quit                 Stop this navigation stack (not MAVROS/AprilTag)\n";
  }

  void print_prompt() const
  {
    std::cout << "real-slam> " << std::flush;
  }

  void print_status() const
  {
    std::ostringstream out;
    out << std::fixed << std::setprecision(2)
        << "Status: connected=" << yes_no(have_state_ && state_.connected)
        << " armed=" << yes_no(have_state_ && state_.armed)
        << " mode=" << (have_state_ ? state_.mode : "unknown")
        << " odom_fresh=" << yes_no(odom_fresh())
        << " scan_fresh=" << yes_no(scan_fresh())
        << " planner_fresh=" << yes_no(planner_fresh())
        << " forwarding=" << yes_no(forwarding_enabled_)
        << " warmup=" << yes_no(offboard_warmup_active_);
    if (have_odom_) {
      out << " local_enu=(" << odom_.pose.pose.position.x << ", "
          << odom_.pose.pose.position.y << ", " << odom_.pose.pose.position.z << ")";
    }
    if (home_.has_value()) {
      out << " home=(" << home_->x << ", " << home_->y << ", " << home_->z << ")";
    }
    RCLCPP_INFO(get_logger(), "%s", out.str().c_str());
  }

  void log_health_summary()
  {
    RCLCPP_INFO(
      get_logger(),
      "Health: connected=%s armed=%s mode=%s odom=%s scan=%s planner=%s forwarding=%s",
      yes_no(have_state_ && state_.connected).c_str(),
      yes_no(have_state_ && state_.armed).c_str(),
      have_state_ ? state_.mode.c_str() : "unknown",
      yes_no(odom_fresh()).c_str(), yes_no(scan_fresh()).c_str(),
      yes_no(planner_fresh()).c_str(), yes_no(forwarding_enabled_).c_str());
  }

  void request_arm(bool arm)
  {
    if (!arm) {
      offboard_warmup_active_ = false;
      offboard_mode_requested_ = false;
      disable_forwarding("explicit disarm request", true);
    }
    if (!have_state_ || !state_.connected) {
      RCLCPP_ERROR(get_logger(), "Rejected %s: MAVROS is not connected", arm ? "arm" : "disarm");
      return;
    }
    if (!arm_client_->service_is_ready()) {
      RCLCPP_ERROR(get_logger(), "Rejected %s: /mavros/cmd/arming unavailable", arm ? "arm" : "disarm");
      return;
    }
    auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    request->value = arm;
    arm_client_->async_send_request(
      request, [this, arm](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
        const auto response = future.get();
        RCLCPP_INFO(
          get_logger(), "%s service result: success=%s result=%u",
          arm ? "Arm" : "Disarm", yes_no(response->success).c_str(),
          static_cast<unsigned int>(response->result));
      });
    RCLCPP_INFO(get_logger(), "%s request sent", arm ? "Arm" : "Disarm");
  }

  void request_mode(const std::string & mode)
  {
    if (!have_state_ || !state_.connected) {
      RCLCPP_ERROR(get_logger(), "Rejected mode '%s': MAVROS is not connected", mode.c_str());
      return;
    }
    if (!mode_client_->service_is_ready()) {
      RCLCPP_ERROR(get_logger(), "Rejected mode '%s': /mavros/set_mode unavailable", mode.c_str());
      return;
    }
    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->base_mode = 0;
    request->custom_mode = mode;
    mode_client_->async_send_request(
      request, [this, mode](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
        const auto response = future.get();
        RCLCPP_INFO(
          get_logger(), "Mode service result: requested=%s mode_sent=%s",
          mode.c_str(), yes_no(response->mode_sent).c_str());
      });
    RCLCPP_INFO(get_logger(), "Mode request sent: %s", mode.c_str());
  }

  void start_offboard_warmup()
  {
    if (!have_state_ || !state_.connected || !state_.armed) {
      RCLCPP_ERROR(get_logger(), "Rejected offboard: vehicle must be connected and explicitly armed first");
      return;
    }
    if (!odom_fresh() || !scan_fresh()) {
      RCLCPP_ERROR(get_logger(), "Rejected offboard: odometry and scan must both be fresh");
      return;
    }
    disable_forwarding("OFFBOARD warmup requested", true);
    offboard_warmup_active_ = true;
    offboard_mode_requested_ = false;
    offboard_warmup_start_ = SteadyClock::now();
    RCLCPP_INFO(
      get_logger(), "Accepted offboard: streaming zero setpoints for %.2f seconds",
      offboard_warmup_sec_);
  }

  bool navigation_ready(const std::string & command) const
  {
    if (!have_state_ || !state_.connected || !state_.armed || state_.mode != "OFFBOARD") {
      RCLCPP_ERROR(
        get_logger(), "Rejected %s: requires connected, armed vehicle in OFFBOARD",
        command.c_str());
      return false;
    }
    if (!odom_fresh() || !scan_fresh()) {
      RCLCPP_ERROR(get_logger(), "Rejected %s: odometry or scan is stale", command.c_str());
      return false;
    }
    return true;
  }

  void publish_goal(const geometry_msgs::msg::Point & goal, const std::string & source)
  {
    goal_pub_->publish(goal);
    have_planner_cmd_ = false;
    forwarding_enabled_ = false;
    forwarding_pending_ = true;
    goal_command_time_ = SteadyClock::now();
    RCLCPP_INFO(
      get_logger(), "Accepted %s goal: absolute local ENU (%.3f, %.3f, %.3f); awaiting fresh planner command",
      source.c_str(), goal.x, goal.y, goal.z);
  }

  void command_goal(std::istringstream & stream)
  {
    geometry_msgs::msg::Point goal;
    std::string extra;
    if (!(stream >> goal.x >> goal.y >> goal.z) || (stream >> extra) ||
      !is_finite_value(goal.x) || !is_finite_value(goal.y) || !is_finite_value(goal.z))
    {
      RCLCPP_ERROR(get_logger(), "Rejected goal: expected finite 'goal X Y Z'");
      return;
    }
    if (!navigation_ready("goal")) {
      return;
    }
    publish_goal(goal, "navigation");
  }

  void command_hold()
  {
    offboard_warmup_active_ = false;
    offboard_mode_requested_ = false;
    disable_forwarding("hold command", true);
    RCLCPP_INFO(get_logger(), "Accepted hold: zero velocity setpoint active while OFFBOARD remains selected");
  }

  void command_sethome()
  {
    if (!odom_fresh()) {
      RCLCPP_ERROR(get_logger(), "Rejected sethome: local odometry is stale or unavailable");
      return;
    }
    geometry_msgs::msg::Point home;
    home.x = odom_.pose.pose.position.x;
    home.y = odom_.pose.pose.position.y;
    home.z = odom_.pose.pose.position.z;
    home_ = home;
    RCLCPP_INFO(get_logger(), "Home recorded in local ENU: (%.3f, %.3f, %.3f)", home.x, home.y, home.z);
  }

  void command_return()
  {
    if (!home_.has_value()) {
      RCLCPP_ERROR(get_logger(), "Rejected return: use sethome first");
      return;
    }
    if (!navigation_ready("return")) {
      return;
    }
    publish_goal(*home_, "return-through-obstacle-planner");
  }

  void command_land()
  {
    offboard_warmup_active_ = false;
    offboard_mode_requested_ = false;
    disable_forwarding("land command", true);
    RCLCPP_INFO(get_logger(), "Accepted land: planner forwarding stopped; requesting AUTO.LAND");
    request_mode("AUTO.LAND");
  }

  void command_precland()
  {
    if (precland_mode_.empty()) {
      RCLCPP_ERROR(
        get_logger(), "Rejected precland: precland_mode is not configured; refusing to guess a flight mode");
      return;
    }
    offboard_warmup_active_ = false;
    offboard_mode_requested_ = false;
    disable_forwarding("precland command", true);
    RCLCPP_INFO(
      get_logger(), "Accepted precland: planner forwarding stopped; requesting configured mode '%s'",
      precland_mode_.c_str());
    request_mode(precland_mode_);
  }

  double setpoint_hz_{20.0};
  double offboard_warmup_sec_{2.0};
  double odom_timeout_sec_{0.30};
  double scan_timeout_sec_{0.30};
  double planner_timeout_sec_{0.30};
  std::string precland_mode_;

  mavros_msgs::msg::State state_;
  geometry_msgs::msg::PoseStamped pose_;
  nav_msgs::msg::Odometry odom_;
  geometry_msgs::msg::TwistStamped planner_cmd_;
  std::optional<geometry_msgs::msg::Point> home_;
  bool have_state_{false};
  bool have_pose_{false};
  bool have_odom_{false};
  bool have_scan_{false};
  bool have_planner_cmd_{false};
  bool forwarding_pending_{false};
  bool forwarding_enabled_{false};
  bool offboard_warmup_active_{false};
  bool offboard_mode_requested_{false};
  SteadyClock::time_point state_rx_{};
  SteadyClock::time_point pose_rx_{};
  SteadyClock::time_point odom_rx_{};
  SteadyClock::time_point scan_rx_{};
  SteadyClock::time_point planner_rx_{};
  SteadyClock::time_point goal_command_time_{};
  SteadyClock::time_point offboard_warmup_start_{};

  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr planner_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr goal_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr setpoint_pub_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client_;
  rclcpp::TimerBase::SharedPtr setpoint_timer_;
  rclcpp::TimerBase::SharedPtr console_timer_;
  rclcpp::TimerBase::SharedPtr health_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RealSlamCommandConsole>());
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
