// DDS-native PX4 offboard velocity bridge for micro XRCE-DDS / px4_msgs.

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <iostream>
#include <limits>
#include <poll.h>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace {
std::string trim_copy(const std::string &s) {
  size_t a = 0;
  while (a < s.size() && std::isspace(static_cast<unsigned char>(s[a]))) ++a;
  size_t b = s.size();
  while (b > a && std::isspace(static_cast<unsigned char>(s[b - 1]))) --b;
  return s.substr(a, b - a);
}

std::string lower_copy(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return s;
}

inline double clamp(double x, double lo, double hi) { return std::max(lo, std::min(hi, x)); }
}  // namespace

class OffboardControlVelDds : public rclcpp::Node {
 public:
  OffboardControlVelDds() : Node("offboard_control_vel_dds") {
    setpoint_hz_ = declare_parameter<double>("setpoint_hz", 20.0);
    warmup_sec_ = declare_parameter<double>("warmup_sec", 2.0);
    state_check_hz_ = declare_parameter<double>("state_check_hz", 2.0);
    request_retry_sec_ = declare_parameter<double>("request_retry_sec", 1.0);
    ask_goal_on_start_ = declare_parameter<bool>("ask_goal_on_start", true);
    goal_input_poll_hz_ = declare_parameter<double>("goal_input_poll_hz", 10.0);
    print_input_help_on_start_ = declare_parameter<bool>("print_input_help_on_start", true);
    enable_internal_goal_nav_ = declare_parameter<bool>("enable_internal_goal_nav", true);
    nav_kp_xy_ = declare_parameter<double>("nav_kp_xy", 0.8);
    nav_max_speed_xy_ = declare_parameter<double>("nav_max_speed_xy", 1.0);
    nav_kp_z_ = declare_parameter<double>("nav_kp_z", 0.8);
    nav_max_speed_z_ = declare_parameter<double>("nav_max_speed_z", 0.6);
    goal_reach_radius_xy_ = declare_parameter<double>("goal_reach_radius_xy", 0.25);
    goal_reach_radius_z_ = declare_parameter<double>("goal_reach_radius_z", 0.20);
    planner_cmd_timeout_sec_ = declare_parameter<double>("planner_cmd_timeout_sec", 0.30);
    max_altitude_m_ = declare_parameter<double>("max_altitude_m", 5.0);
    min_altitude_m_ = declare_parameter<double>("min_altitude_m", 0.0);
    max_linear_speed_mps_ = declare_parameter<double>("max_linear_speed_mps", 3.0);
    max_yaw_rate_rad_s_ = declare_parameter<double>("max_yaw_rate_rad_s", 0.45);
    max_accel_xy_mps2_ = declare_parameter<double>("max_accel_xy_mps2", 0.8);
    max_accel_z_mps2_ = declare_parameter<double>("max_accel_z_mps2", 0.6);
    max_yaw_accel_rad_s2_ = declare_parameter<double>("max_yaw_accel_rad_s2", 1.0);
    altitude_guard_band_m_ = declare_parameter<double>("altitude_guard_band_m", 0.4);
    publish_heartbeat_ = declare_parameter<bool>("publish_heartbeat", true);

    auto qos_sensor = rclcpp::SensorDataQoS();
    auto qos_cmd = rclcpp::QoS(10).best_effort();
    auto qos_goal = rclcpp::QoS(10).reliable().transient_local();

    offboard_mode_pub_ =
        create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", qos_cmd);
    trajectory_pub_ =
        create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", qos_cmd);
    vehicle_command_pub_ =
        create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", qos_cmd);
    goal_pub_ = create_publisher<geometry_msgs::msg::Point>("/drone_goal", qos_goal);

    local_position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position", qos_sensor,
        std::bind(&OffboardControlVelDds::local_position_cb, this, std::placeholders::_1));
    planner_cmd_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        "/planner_cmd_vel", rclcpp::QoS(10).reliable(),
        std::bind(&OffboardControlVelDds::planner_cmd_cb, this, std::placeholders::_1));

    const auto sp_period = std::chrono::duration<double>(1.0 / std::max(1.0, setpoint_hz_));
    setpoint_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(sp_period),
        std::bind(&OffboardControlVelDds::publish_setpoint_cmd, this));

    const auto state_period = std::chrono::duration<double>(1.0 / std::max(0.2, state_check_hz_));
    state_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(state_period),
        std::bind(&OffboardControlVelDds::state_machine, this));

    if (ask_goal_on_start_) {
      const auto input_period = std::chrono::duration<double>(1.0 / std::max(1.0, goal_input_poll_hz_));
      input_timer_ = create_wall_timer(
          std::chrono::duration_cast<std::chrono::milliseconds>(input_period),
          std::bind(&OffboardControlVelDds::poll_goal_input, this));
      goal_input_active_ = true;
    }

    next_mode_request_time_ = now();
    next_arm_request_time_ = now();
    if (ask_goal_on_start_ && print_input_help_on_start_) print_goal_input_help();
    prompt_pending_ = ask_goal_on_start_;
    publish_initial_goal();
  }

 private:
  enum class InputParseResult { kNone, kGoal, kHelp, kQuit, kInvalid };

  uint64_t timestamp_us() const {
    return static_cast<uint64_t>(get_clock()->now().nanoseconds() / 1000ULL);
  }

  void local_position_cb(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
    have_position_ = true;
    current_x_ = msg->y;
    current_y_ = msg->x;
    current_z_ = -msg->z;
    if (!started_) {
      started_ = true;
      start_time_ = now();
      RCLCPP_INFO(get_logger(), "PX4 DDS local position received. Warmup heartbeat for %.1f s...",
                  warmup_sec_);
    }
  }

  void planner_cmd_cb(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    planner_cmd_cache_ = *msg;
    planner_cmd_seen_ = true;
    last_planner_cmd_time_ = now();
  }

  void publish_setpoint_cmd() {
    if (!started_) return;
    publish_offboard_mode();

    if (planner_override_active()) {
      publish_cmd(planner_cmd_cache_.twist.linear.x, planner_cmd_cache_.twist.linear.y,
                  planner_cmd_cache_.twist.linear.z, planner_cmd_cache_.twist.angular.z);
      return;
    }

    if (enable_internal_goal_nav_ && have_position_ && have_goal_) {
      publish_internal_goal_nav_cmd();
      return;
    }

    if (!publish_heartbeat_ && offboard_requested_ && arm_requested_) return;
    publish_cmd(0.0, 0.0, 0.0, 0.0);
  }

  void publish_offboard_mode() {
    px4_msgs::msg::OffboardControlMode msg{};
    msg.timestamp = timestamp_us();
    msg.position = false;
    msg.velocity = true;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    offboard_mode_pub_->publish(msg);
  }

  void publish_internal_goal_nav_cmd() {
    const double dx = active_goal_.x - current_x_;
    const double dy = active_goal_.y - current_y_;
    const double dz = active_goal_.z - current_z_;
    const double dxy = std::hypot(dx, dy);
    if (dxy < goal_reach_radius_xy_ && std::fabs(dz) < goal_reach_radius_z_) {
      publish_cmd(0.0, 0.0, 0.0, 0.0);
      return;
    }

    double vx = nav_kp_xy_ * dx;
    double vy = nav_kp_xy_ * dy;
    const double vxy = std::hypot(vx, vy);
    if (vxy > nav_max_speed_xy_ && vxy > 1e-6) {
      const double s = nav_max_speed_xy_ / vxy;
      vx *= s;
      vy *= s;
    }
    publish_cmd(vx, vy, clamp(nav_kp_z_ * dz, -nav_max_speed_z_, nav_max_speed_z_), 0.0);
  }

  bool planner_override_active() {
    if (!planner_cmd_seen_) return false;
    const bool active = (now() - last_planner_cmd_time_).seconds() <= planner_cmd_timeout_sec_;
    if (!active) {
      planner_override_logged_ = false;
    } else if (!planner_override_logged_) {
      planner_override_logged_ = true;
      RCLCPP_INFO(get_logger(), "Planner override active: forwarding /planner_cmd_vel to PX4 DDS.");
    }
    return active;
  }

  void publish_cmd(double vx, double vy, double vz, double wz) {
    apply_output_safety_limits(vx, vy, vz, wz);

    const float nan = std::numeric_limits<float>::quiet_NaN();
    px4_msgs::msg::TrajectorySetpoint sp{};
    sp.timestamp = timestamp_us();
    sp.position = {nan, nan, nan};
    sp.velocity = {static_cast<float>(vy), static_cast<float>(vx), static_cast<float>(-vz)};
    sp.acceleration = {nan, nan, nan};
    sp.yaw = nan;
    sp.yawspeed = static_cast<float>(-wz);
    trajectory_pub_->publish(sp);

    last_cmd_twist_.linear.x = vx;
    last_cmd_twist_.linear.y = vy;
    last_cmd_twist_.linear.z = vz;
    last_cmd_twist_.angular.z = wz;
    last_cmd_time_ = now();
  }

  void apply_output_safety_limits(double &vx, double &vy, double &vz, double &wz) {
    const double guard_band = std::max(0.05, altitude_guard_band_m_);
    if (have_position_) {
      if (current_z_ >= max_altitude_m_ && vz > 0.0) {
        vz = 0.0;
      } else if (current_z_ > (max_altitude_m_ - guard_band) && vz > 0.0) {
        vz *= clamp((max_altitude_m_ - current_z_) / guard_band, 0.0, 1.0);
      }
      if (current_z_ <= min_altitude_m_ && vz < 0.0) {
        vz = 0.0;
      } else if (current_z_ < (min_altitude_m_ + guard_band) && vz < 0.0) {
        vz *= clamp((current_z_ - min_altitude_m_) / guard_band, 0.0, 1.0);
      }
    }

    wz = clamp(wz, -std::fabs(max_yaw_rate_rad_s_), std::fabs(max_yaw_rate_rad_s_));
    const double v_norm = std::sqrt(vx * vx + vy * vy + vz * vz);
    const double v_cap = std::max(0.1, max_linear_speed_mps_);
    if (v_norm > v_cap) {
      const double s = v_cap / v_norm;
      vx *= s;
      vy *= s;
      vz *= s;
    }

    if (!last_cmd_valid_) {
      last_cmd_valid_ = true;
      return;
    }

    const double dt = std::max(1e-3, (now() - last_cmd_time_).seconds());
    const double dxy = std::hypot(vx - last_cmd_twist_.linear.x, vy - last_cmd_twist_.linear.y);
    const double dxy_cap = std::max(0.01, max_accel_xy_mps2_) * dt;
    if (dxy > dxy_cap) {
      const double s = dxy_cap / dxy;
      vx = last_cmd_twist_.linear.x + (vx - last_cmd_twist_.linear.x) * s;
      vy = last_cmd_twist_.linear.y + (vy - last_cmd_twist_.linear.y) * s;
    }

    const double dz_cap = std::max(0.01, max_accel_z_mps2_) * dt;
    const double dz = vz - last_cmd_twist_.linear.z;
    if (std::fabs(dz) > dz_cap) vz = last_cmd_twist_.linear.z + std::copysign(dz_cap, dz);

    const double dw_cap = std::max(0.01, max_yaw_accel_rad_s2_) * dt;
    const double dw = wz - last_cmd_twist_.angular.z;
    if (std::fabs(dw) > dw_cap) wz = last_cmd_twist_.angular.z + std::copysign(dw_cap, dw);
  }

  void state_machine() {
    if (!started_) return;
    const rclcpp::Time tnow = now();
    if ((tnow - start_time_).seconds() < warmup_sec_) return;

    if (!offboard_requested_) {
      try_request_offboard(tnow);
      return;
    }
    if (!arm_requested_) {
      try_request_arm(tnow);
      return;
    }
    if (!ready_logged_) {
      ready_logged_ = true;
      RCLCPP_INFO(get_logger(), "DDS OFFBOARD + ARM commands sent. Local planner can control velocity now.");
    }
  }

  void publish_vehicle_command(uint16_t command, float param1 = 0.0F, float param2 = 0.0F) {
    px4_msgs::msg::VehicleCommand msg{};
    msg.timestamp = timestamp_us();
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    vehicle_command_pub_->publish(msg);
  }

  void try_request_offboard(const rclcpp::Time &tnow) {
    if (tnow < next_mode_request_time_) return;
    next_mode_request_time_ = tnow + rclcpp::Duration::from_seconds(request_retry_sec_);
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0F, 6.0F);
    offboard_requested_ = true;
    RCLCPP_INFO(get_logger(), "PX4 OFFBOARD mode command sent over DDS.");
  }

  void try_request_arm(const rclcpp::Time &tnow) {
    if (tnow < next_arm_request_time_) return;
    next_arm_request_time_ = tnow + rclcpp::Duration::from_seconds(request_retry_sec_);
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0F);
    arm_requested_ = true;
    RCLCPP_INFO(get_logger(), "PX4 ARM command sent over DDS.");
  }

  void poll_goal_input() {
    if (!goal_input_active_) return;
    if (prompt_pending_) {
      std::cout << "\nEnter goal x y z ('h' for help, 'q' to stop input): " << std::flush;
      prompt_pending_ = false;
    }

    pollfd pfd{};
    pfd.fd = 0;
    pfd.events = POLLIN;
    const int ready = ::poll(&pfd, 1, 0);
    if (ready <= 0 || (pfd.revents & POLLIN) == 0) return;

    std::string line;
    if (!std::getline(std::cin, line)) {
      goal_input_active_ = false;
      RCLCPP_WARN(get_logger(), "Goal input stream closed; disabling CLI goal input.");
      return;
    }
    prompt_pending_ = true;

    geometry_msgs::msg::Point goal;
    const InputParseResult parsed = parse_goal_line(line, goal);
    if (parsed == InputParseResult::kNone) return;
    if (parsed == InputParseResult::kHelp) {
      print_goal_input_help();
      return;
    }
    if (parsed == InputParseResult::kQuit) {
      goal_input_active_ = false;
      RCLCPP_INFO(get_logger(), "CLI goal input disabled by user command.");
      return;
    }
    if (parsed == InputParseResult::kInvalid) {
      RCLCPP_WARN(get_logger(), "Invalid input. Expected: x y z  (example: 10 0 5)");
      return;
    }
    active_goal_ = goal;
    have_goal_ = true;
    goal_pub_->publish(goal);
    RCLCPP_INFO(get_logger(), "Goal published /drone_goal: [%.2f, %.2f, %.2f]", goal.x, goal.y, goal.z);
  }

  InputParseResult parse_goal_line(const std::string &raw, geometry_msgs::msg::Point &goal) const {
    const std::string line = trim_copy(raw);
    if (line.empty()) return InputParseResult::kNone;
    const std::string cmd = lower_copy(line);
    if (cmd == "q" || cmd == "quit" || cmd == "exit") return InputParseResult::kQuit;
    if (cmd == "h" || cmd == "help" || cmd == "?") return InputParseResult::kHelp;

    std::istringstream iss(line);
    double x = 0.0, y = 0.0, z = 0.0;
    char extra = '\0';
    if (!(iss >> x >> y >> z)) return InputParseResult::kInvalid;
    if (iss >> extra) return InputParseResult::kInvalid;
    goal.x = x;
    goal.y = y;
    goal.z = clamp(z, min_altitude_m_, max_altitude_m_);
    return InputParseResult::kGoal;
  }

  void print_goal_input_help() const {
    RCLCPP_INFO(get_logger(),
                "Goal input help:\n"
                "  - Type: x y z   (example: 10 0 5)\n"
                "  - Type: h       (show this help)\n"
                "  - Type: q       (stop CLI goal input)");
  }

  void publish_initial_goal() {
    active_goal_.x = 0.0;
    active_goal_.y = 0.0;
    active_goal_.z = clamp(5.0, min_altitude_m_, max_altitude_m_);
    have_goal_ = true;
    goal_pub_->publish(active_goal_);
  }

  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr goal_pub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr planner_cmd_sub_;
  rclcpp::TimerBase::SharedPtr setpoint_timer_;
  rclcpp::TimerBase::SharedPtr state_timer_;
  rclcpp::TimerBase::SharedPtr input_timer_;

  double setpoint_hz_{20.0};
  double warmup_sec_{2.0};
  double state_check_hz_{2.0};
  double request_retry_sec_{1.0};
  bool ask_goal_on_start_{true};
  double goal_input_poll_hz_{10.0};
  bool print_input_help_on_start_{true};
  bool enable_internal_goal_nav_{true};
  double nav_kp_xy_{0.8};
  double nav_max_speed_xy_{1.0};
  double nav_kp_z_{0.8};
  double nav_max_speed_z_{0.6};
  double goal_reach_radius_xy_{0.25};
  double goal_reach_radius_z_{0.20};
  double planner_cmd_timeout_sec_{0.30};
  double max_altitude_m_{5.0};
  double min_altitude_m_{0.0};
  double max_linear_speed_mps_{3.0};
  double max_yaw_rate_rad_s_{0.6};
  double max_accel_xy_mps2_{0.8};
  double max_accel_z_mps2_{0.6};
  double max_yaw_accel_rad_s2_{1.0};
  double altitude_guard_band_m_{0.4};
  bool publish_heartbeat_{true};

  bool started_{false};
  bool have_position_{false};
  bool offboard_requested_{false};
  bool arm_requested_{false};
  bool ready_logged_{false};
  bool goal_input_active_{false};
  bool prompt_pending_{false};
  bool have_goal_{false};
  bool planner_cmd_seen_{false};
  bool planner_override_logged_{false};
  bool last_cmd_valid_{false};
  double current_x_{0.0};
  double current_y_{0.0};
  double current_z_{0.0};
  geometry_msgs::msg::Point active_goal_;
  geometry_msgs::msg::TwistStamped planner_cmd_cache_;
  geometry_msgs::msg::Twist last_cmd_twist_;
  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time next_mode_request_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time next_arm_request_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_planner_cmd_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OffboardControlVelDds>());
  rclcpp::shutdown();
  return 0;
}
