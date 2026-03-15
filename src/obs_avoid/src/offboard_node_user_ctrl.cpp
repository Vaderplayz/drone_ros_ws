// Last modified: 2026-02-18 14:35:00 +07
// Added: startup initial goal publish, internal goal navigation fallback, planner command interception via /planner_cmd_vel, and poll()-based robust CLI input read
// Removed: dependence on planner for basic goal motion and fragile stdin availability checks

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <iostream>
#include <poll.h>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
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

class OffboardControlVel : public rclcpp::Node {
 public:
  OffboardControlVel() : Node("offboard_control_vel") {
    // Parameters
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

    // If true, keep publishing zero-velocity heartbeat forever.
    // If false, stop heartbeat after OFFBOARD + ARM succeed.
    publish_heartbeat_ = declare_parameter<bool>("publish_heartbeat", true);

    // ROS I/O
    auto qos_pose = rclcpp::SensorDataQoS();
    auto qos_cmd = rclcpp::QoS(10).reliable();
    auto qos_goal = rclcpp::QoS(10).reliable().transient_local();

    cmd_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
        "/mavros/setpoint_velocity/cmd_vel", qos_cmd);

    goal_pub_ = create_publisher<geometry_msgs::msg::Point>("/drone_goal", qos_goal);

    local_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/mavros/local_position/pose", qos_pose,
        std::bind(&OffboardControlVel::local_pose_cb, this, std::placeholders::_1));
    planner_cmd_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        "/planner_cmd_vel", qos_cmd,
        std::bind(&OffboardControlVel::planner_cmd_cb, this, std::placeholders::_1));

    arming_client_ = create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    set_mode_client_ = create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");

    const auto sp_period = std::chrono::duration<double>(1.0 / std::max(1.0, setpoint_hz_));
    setpoint_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(sp_period),
        std::bind(&OffboardControlVel::publish_setpoint_cmd, this));

    const auto state_period = std::chrono::duration<double>(1.0 / std::max(0.2, state_check_hz_));
    state_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(state_period),
        std::bind(&OffboardControlVel::state_machine, this));

    if (ask_goal_on_start_) {
      const auto input_period = std::chrono::duration<double>(1.0 / std::max(1.0, goal_input_poll_hz_));
      input_timer_ = create_wall_timer(
          std::chrono::duration_cast<std::chrono::milliseconds>(input_period),
          std::bind(&OffboardControlVel::poll_goal_input, this));
      goal_input_active_ = true;
    }

    next_mode_request_time_ = now();
    next_arm_request_time_ = now();

    RCLCPP_INFO(get_logger(),
                "OffboardControlVel started. setpoint_hz=%.1f warmup_sec=%.1f retry_sec=%.1f",
                setpoint_hz_, warmup_sec_, request_retry_sec_);

    if (ask_goal_on_start_) {
      if (print_input_help_on_start_) {
        print_goal_input_help();
      }
      prompt_pending_ = true;
    }

    publish_initial_goal();
  }

 private:
  enum class InputParseResult {
    kNone,
    kGoal,
    kHelp,
    kQuit,
    kInvalid,
  };

  void local_pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    got_pose_ = true;
    have_pose_ = true;
    current_pose_ = *msg;

    if (!started_) {
      started_ = true;
      start_time_ = now();
      RCLCPP_INFO(get_logger(), "Pose received. Warmup heartbeat for %.1f s...", warmup_sec_);
    }
  }

  void planner_cmd_cb(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    planner_cmd_cache_ = *msg;
    planner_cmd_seen_ = true;
    last_planner_cmd_time_ = now();
  }

  void publish_setpoint_cmd() {
    if (!started_) return;

    if (planner_override_active()) {
      publish_cmd(planner_cmd_cache_.twist.linear.x, planner_cmd_cache_.twist.linear.y,
                  planner_cmd_cache_.twist.linear.z, planner_cmd_cache_.twist.angular.z);
      return;
    }

    if (enable_internal_goal_nav_ && have_pose_ && have_goal_) {
      publish_internal_goal_nav_cmd();
      return;
    }

    if (!publish_heartbeat_ && offboard_set_ && armed_) return;
    publish_cmd(0.0, 0.0, 0.0, 0.0);
  }

  void publish_internal_goal_nav_cmd() {
    const auto &p = current_pose_.pose.position;
    const double dx = active_goal_.x - p.x;
    const double dy = active_goal_.y - p.y;
    const double dz = active_goal_.z - p.z;
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

    const double vz = clamp(nav_kp_z_ * dz, -nav_max_speed_z_, nav_max_speed_z_);
    publish_cmd(vx, vy, vz, 0.0);
  }

  bool planner_override_active() {
    if (!planner_cmd_seen_) return false;
    const double age = (now() - last_planner_cmd_time_).seconds();
    const bool active = age <= planner_cmd_timeout_sec_;
    if (!active) {
      planner_override_logged_ = false;
    } else if (!planner_override_logged_) {
      planner_override_logged_ = true;
      RCLCPP_INFO(get_logger(), "Planner override active: forwarding /planner_cmd_vel to MAVROS.");
    }
    return active;
  }

  void publish_cmd(double vx, double vy, double vz, double wz) {
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = now();
    cmd.header.frame_id = "map";
    cmd.twist.linear.x = vx;
    cmd.twist.linear.y = vy;
    cmd.twist.linear.z = vz;
    cmd.twist.angular.z = wz;
    cmd_pub_->publish(cmd);
  }

  void state_machine() {
    if (!started_ || !got_pose_) return;

    const rclcpp::Time tnow = now();
    const double warmup_elapsed = (tnow - start_time_).seconds();
    if (warmup_elapsed < warmup_sec_) return;

    if (!set_mode_client_->service_is_ready() || !arming_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Waiting for MAVROS services (/mavros/set_mode, /mavros/cmd/arming)...");
      return;
    }

    if (!offboard_set_) {
      try_request_offboard(tnow);
      return;
    }

    if (!armed_) {
      try_request_arm(tnow);
      return;
    }

    if (!ready_logged_) {
      ready_logged_ = true;
      RCLCPP_INFO(get_logger(), "OFFBOARD + ARM complete. Local planner can control velocity now.");
    }
  }

  void try_request_offboard(const rclcpp::Time &tnow) {
    if (mode_request_pending_) return;
    if (tnow < next_mode_request_time_) return;

    auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    req->custom_mode = "OFFBOARD";

    mode_request_pending_ = true;
    next_mode_request_time_ = tnow + rclcpp::Duration::from_seconds(request_retry_sec_);

    auto cb = [this](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
      bool ok = false;
      try {
        const auto resp = future.get();
        ok = (resp && resp->mode_sent);
      } catch (const std::exception &e) {
        RCLCPP_WARN(this->get_logger(), "OFFBOARD request exception: %s", e.what());
      }

      if (ok) {
        offboard_set_ = true;
        RCLCPP_INFO(this->get_logger(), "OFFBOARD accepted.");
      } else {
        RCLCPP_WARN(this->get_logger(), "OFFBOARD failed (will retry).");
      }
      mode_request_pending_ = false;
    };

    set_mode_client_->async_send_request(req, cb);
  }

  void try_request_arm(const rclcpp::Time &tnow) {
    if (arm_request_pending_) return;
    if (tnow < next_arm_request_time_) return;

    auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    req->value = true;

    arm_request_pending_ = true;
    next_arm_request_time_ = tnow + rclcpp::Duration::from_seconds(request_retry_sec_);

    auto cb = [this](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
      bool ok = false;
      try {
        const auto resp = future.get();
        ok = (resp && resp->success);
      } catch (const std::exception &e) {
        RCLCPP_WARN(this->get_logger(), "ARM request exception: %s", e.what());
      }

      if (ok) {
        armed_ = true;
        RCLCPP_INFO(this->get_logger(), "ARM accepted.");
      } else {
        RCLCPP_WARN(this->get_logger(), "ARM failed (will retry).");
      }
      arm_request_pending_ = false;
    };

    arming_client_->async_send_request(req, cb);
  }

  void poll_goal_input() {
    if (!goal_input_active_) return;

    if (prompt_pending_) {
      std::cout << "\nEnter goal x y z ('h' for help, 'q' to stop input): " << std::flush;
      prompt_pending_ = false;
    }

    pollfd pfd{};
    pfd.fd = 0;  // stdin
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
    goal.z = z;
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
    geometry_msgs::msg::Point goal;
    goal.x = 0.0;
    goal.y = 0.0;
    goal.z = 5.0;
    active_goal_ = goal;
    have_goal_ = true;
    goal_pub_->publish(goal);
    RCLCPP_INFO(get_logger(), "Initial goal published /drone_goal: [%.2f, %.2f, %.2f]", goal.x, goal.y,
                goal.z);
  }

 private:
  // ROS interfaces
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr goal_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr planner_cmd_sub_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;

  rclcpp::TimerBase::SharedPtr setpoint_timer_;
  rclcpp::TimerBase::SharedPtr state_timer_;
  rclcpp::TimerBase::SharedPtr input_timer_;

  // Parameters
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
  bool publish_heartbeat_{true};

  // State
  bool got_pose_{false};
  bool started_{false};
  bool offboard_set_{false};
  bool armed_{false};
  bool ready_logged_{false};

  bool mode_request_pending_{false};
  bool arm_request_pending_{false};

  bool goal_input_active_{false};
  bool prompt_pending_{false};
  bool have_pose_{false};
  bool have_goal_{false};
  bool planner_cmd_seen_{false};
  bool planner_override_logged_{false};

  geometry_msgs::msg::PoseStamped current_pose_;
  geometry_msgs::msg::Point active_goal_;
  geometry_msgs::msg::TwistStamped planner_cmd_cache_;

  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time next_mode_request_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time next_arm_request_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_planner_cmd_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OffboardControlVel>());
  rclcpp::shutdown();
  return 0;
}
