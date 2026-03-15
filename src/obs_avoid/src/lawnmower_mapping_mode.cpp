// Last modified: 2026-02-25 00:58:00 +07
// Added: keep return-to-center waypoint altitude at mission center z to prevent unintended descent/landing
// Removed: return-to-center z assignment tied to mission-start pose altitude

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cctype>
#include <iostream>
#include <limits>
#include <optional>
#include <poll.h>
#include <sstream>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"

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

std::string upper_copy(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(),
                 [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
  return s;
}

std::string parse_mode_from_dwa_state(const std::string &state_line) {
  const std::string key = "mode=";
  const auto pos = state_line.find(key);
  if (pos == std::string::npos) return {};
  const size_t begin = pos + key.size();
  size_t end = begin;
  while (end < state_line.size()) {
    const unsigned char c = static_cast<unsigned char>(state_line[end]);
    if (std::isspace(c) || c == ',' || c == ';' || c == ')') break;
    ++end;
  }
  return upper_copy(trim_copy(state_line.substr(begin, end - begin)));
}

inline double clamp(double x, double lo, double hi) { return std::max(lo, std::min(hi, x)); }
inline double wrap_pi(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

double yaw_from_pose(const geometry_msgs::msg::PoseStamped &pose) {
  const auto &q = pose.pose.orientation;
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}
}  // namespace

class LawnmowerMappingMode : public rclcpp::Node {
 public:
  LawnmowerMappingMode() : Node("lawnmower_mapping_mode") {
    // Offboard / timing
    setpoint_hz_ = declare_parameter<double>("setpoint_hz", 20.0);
    warmup_sec_ = declare_parameter<double>("warmup_sec", 2.0);
    state_check_hz_ = declare_parameter<double>("state_check_hz", 2.0);
    request_retry_sec_ = declare_parameter<double>("request_retry_sec", 1.0);
    publish_heartbeat_ = declare_parameter<bool>("publish_heartbeat", true);

    // Navigation
    nav_kp_xy_ = declare_parameter<double>("nav_kp_xy", 0.8);
    nav_max_speed_xy_ = declare_parameter<double>("nav_max_speed_xy", 1.0);
    nav_kp_z_ = declare_parameter<double>("nav_kp_z", 0.8);
    nav_max_speed_z_ = declare_parameter<double>("nav_max_speed_z", 0.6);
    waypoint_reach_radius_xy_ = declare_parameter<double>("waypoint_reach_radius_xy", 0.45);
    waypoint_reach_radius_z_ = declare_parameter<double>("waypoint_reach_radius_z", 0.30);
    enable_internal_goal_nav_ = declare_parameter<bool>("enable_internal_goal_nav", true);
    takeoff_priority_over_planner_ = declare_parameter<bool>("takeoff_priority_over_planner", true);
    hold_yaw_during_takeoff_ = declare_parameter<bool>("hold_yaw_during_takeoff", true);
    planner_cmd_timeout_sec_ = declare_parameter<double>("planner_cmd_timeout_sec", 0.30);
    face_goal_to_setpoint_ = declare_parameter<bool>("face_goal_to_setpoint", true);
    face_goal_k_yaw_ = declare_parameter<double>("face_goal_k_yaw", 0.9);
    face_goal_deadband_deg_ = declare_parameter<double>("face_goal_deadband_deg", 3.0);
    face_goal_turn_only_deg_ = declare_parameter<double>("face_goal_turn_only_deg", 70.0);
    face_goal_min_xy_scale_ = declare_parameter<double>("face_goal_min_xy_scale", 0.2);
    yaw_rate_max_ = declare_parameter<double>("yaw_rate_max", 0.8);
    waypoint_skip_if_blocked_ = declare_parameter<bool>("waypoint_skip_if_blocked", true);
    waypoint_skip_obstacle_dist_ =
        declare_parameter<double>("waypoint_skip_obstacle_dist", 0.9);
    waypoint_skip_scan_topic_ =
        declare_parameter<std::string>("waypoint_skip_scan_topic", "/scan_horizontal");
    waypoint_skip_scan_stride_ = declare_parameter<int>("waypoint_skip_scan_stride", 3);
    waypoint_skip_max_use_range_ =
        declare_parameter<double>("waypoint_skip_max_use_range", 12.0);
    waypoint_skip_hard_obstacle_dist_ =
        declare_parameter<double>("waypoint_skip_hard_obstacle_dist", 0.0);
    enable_obstacle_goal_standoff_reach_ =
        declare_parameter<bool>("enable_obstacle_goal_standoff_reach", true);
    obstacle_goal_standoff_fraction_ =
        declare_parameter<double>("obstacle_goal_standoff_fraction", 0.35);
    obstacle_goal_republish_min_delta_ =
        declare_parameter<double>("obstacle_goal_republish_min_delta", 0.20);
    obstacle_goal_republish_min_period_sec_ =
        declare_parameter<double>("obstacle_goal_republish_min_period_sec", 0.50);
    enable_waypoint_stall_skip_ = declare_parameter<bool>("enable_waypoint_stall_skip", true);
    waypoint_stall_timeout_sec_ = declare_parameter<double>("waypoint_stall_timeout_sec", 12.0);
    waypoint_stall_progress_delta_m_ =
        declare_parameter<double>("waypoint_stall_progress_delta_m", 0.20);
    waypoint_stall_skip_radius_xy_ =
        declare_parameter<double>("waypoint_stall_skip_radius_xy", 1.80);
    waypoint_stall_obstacle_gate_ =
        declare_parameter<double>("waypoint_stall_obstacle_gate", 1.15);
    enable_recovery_mode_timeout_skip_ =
        declare_parameter<bool>("enable_recovery_mode_timeout_skip", true);
    recovery_mode_timeout_sec_ = declare_parameter<double>("recovery_mode_timeout_sec", 35.0);
    dwa_state_stale_sec_ = declare_parameter<double>("dwa_state_stale_sec", 2.5);
    enable_waypoint_no_progress_skip_ =
        declare_parameter<bool>("enable_waypoint_no_progress_skip", true);
    waypoint_no_progress_timeout_sec_ =
        declare_parameter<double>("waypoint_no_progress_timeout_sec", 18.0);
    waypoint_no_progress_delta_m_ =
        declare_parameter<double>("waypoint_no_progress_delta_m", 0.10);
    waypoint_no_progress_obstacle_gate_ =
        declare_parameter<double>("waypoint_no_progress_obstacle_gate", 1.60);

    // Mapping mission generation
    mission_pattern_ = declare_parameter<std::string>("mission_pattern", "lawnmower");
    ask_mission_on_start_ = declare_parameter<bool>("ask_mission_on_start", true);
    input_poll_hz_ = declare_parameter<double>("input_poll_hz", 10.0);
    print_input_help_on_start_ = declare_parameter<bool>("print_input_help_on_start", true);
    auto_start_default_mission_ = declare_parameter<bool>("auto_start_default_mission", false);
    default_square_size_m_ = declare_parameter<double>("default_square_size_m", 30.0);
    default_z_layers_ = declare_parameter<int>("default_z_layers", 1);
    default_z_step_m_ = declare_parameter<double>("default_z_step_m", 2.0);
    lidar_range_m_ = declare_parameter<double>("lidar_range_m", 12.0);
    spiral_pitch_ratio_ = declare_parameter<double>("spiral_pitch_ratio", 0.5);
    return_to_center_ = declare_parameter<bool>("return_to_center", true);
    enable_initial_takeoff_setpoint_ =
        declare_parameter<bool>("enable_initial_takeoff_setpoint", true);
    initial_takeoff_x_ = declare_parameter<double>("initial_takeoff_x", 0.0);
    initial_takeoff_y_ = declare_parameter<double>("initial_takeoff_y", 0.0);
    initial_takeoff_z_ = declare_parameter<double>("initial_takeoff_z", 5.0);

    auto qos_pose = rclcpp::SensorDataQoS();
    auto qos_scan = rclcpp::SensorDataQoS();
    auto qos_cmd = rclcpp::QoS(10).reliable();
    auto qos_goal = rclcpp::QoS(10).reliable().transient_local();

    cmd_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
        "/mavros/setpoint_velocity/cmd_vel", qos_cmd);
    goal_pub_ = create_publisher<geometry_msgs::msg::Point>("/drone_goal", qos_goal);

    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/mavros/local_position/pose", qos_pose,
        std::bind(&LawnmowerMappingMode::pose_cb, this, std::placeholders::_1));
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        waypoint_skip_scan_topic_, qos_scan,
        [this](sensor_msgs::msg::LaserScan::SharedPtr msg) { scan_ = *msg; });
    planner_cmd_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        "/planner_cmd_vel", qos_cmd,
        std::bind(&LawnmowerMappingMode::planner_cmd_cb, this, std::placeholders::_1));
    dwa_state_sub_ = create_subscription<std_msgs::msg::String>(
        "/dwa/state", qos_cmd,
        std::bind(&LawnmowerMappingMode::dwa_state_cb, this, std::placeholders::_1));

    arming_client_ = create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    set_mode_client_ = create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");

    const auto sp_period = std::chrono::duration<double>(1.0 / std::max(1.0, setpoint_hz_));
    setpoint_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(sp_period),
        std::bind(&LawnmowerMappingMode::publish_setpoint_cmd, this));

    const auto state_period = std::chrono::duration<double>(1.0 / std::max(0.2, state_check_hz_));
    state_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(state_period),
        std::bind(&LawnmowerMappingMode::state_machine, this));

    if (ask_mission_on_start_) {
      const auto input_period = std::chrono::duration<double>(1.0 / std::max(1.0, input_poll_hz_));
      input_timer_ = create_wall_timer(
          std::chrono::duration_cast<std::chrono::milliseconds>(input_period),
          std::bind(&LawnmowerMappingMode::poll_mission_input, this));
      input_active_ = true;
    }

    next_mode_request_time_ = now();
    next_arm_request_time_ = now();

    RCLCPP_INFO(get_logger(),
                "LawnmowerMappingMode started. setpoint_hz=%.1f warmup_sec=%.1f retry_sec=%.1f",
                setpoint_hz_, warmup_sec_, request_retry_sec_);

    if (ask_mission_on_start_) {
      if (print_input_help_on_start_) print_input_help();
      prompt_pending_ = true;
    } else if (auto_start_default_mission_) {
      pending_request_ = MissionRequest::defaults(default_square_size_m_, default_z_layers_,
                                                  default_z_step_m_);
    }

    publish_initial_takeoff_goal();
  }

 private:
  struct MissionRequest {
    double side_m{30.0};
    int z_layers{1};
    double z_step_m{2.0};
    bool has_center_z{false};
    double center_z{0.0};

    static MissionRequest defaults(double side, int layers, double zstep) {
      MissionRequest r;
      r.side_m = side;
      r.z_layers = std::max(1, layers);
      r.z_step_m = std::max(0.0, zstep);
      return r;
    }
  };

  void pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    have_pose_ = true;
    pose_ = *msg;
    if (!started_) {
      started_ = true;
      start_time_ = now();
      RCLCPP_INFO(get_logger(), "Pose received. Warmup heartbeat for %.1f s...", warmup_sec_);
    }
    try_configure_mission_from_pending();
  }

  void planner_cmd_cb(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    planner_cmd_cache_ = *msg;
    planner_cmd_seen_ = true;
    last_planner_cmd_time_ = now();
  }

  void dwa_state_cb(const std_msgs::msg::String::SharedPtr msg) {
    last_dwa_state_time_ = now();
    const std::string mode = parse_mode_from_dwa_state(msg->data);
    if (!mode.empty()) dwa_mode_ = mode;
  }

  void publish_setpoint_cmd() {
    if (!started_) return;
    update_goal_progress_state();

    // Keep initial climb deterministic: internal takeoff control can override planner commands.
    if (enable_internal_goal_nav_ && takeoff_phase_active_ && takeoff_priority_over_planner_) {
      publish_takeoff_nav_cmd();
      return;
    }

    if (planner_override_active()) {
      publish_cmd(planner_cmd_cache_.twist.linear.x, planner_cmd_cache_.twist.linear.y,
                  planner_cmd_cache_.twist.linear.z, planner_cmd_cache_.twist.angular.z);
      return;
    }

    if (enable_internal_goal_nav_) {
      if (takeoff_phase_active_) {
        publish_takeoff_nav_cmd();
        return;
      }

      if (mission_active_ && current_wp_idx_ < waypoints_.size()) {
        publish_mission_nav_cmd();
        return;
      }
    }

    if (!publish_heartbeat_ && offboard_set_ && armed_) return;
    publish_cmd(0.0, 0.0, 0.0, 0.0);
  }

  void update_goal_progress_state() {
    if (!have_pose_) return;
    const auto &p = pose_.pose.position;

    if (takeoff_phase_active_) {
      const double dxy = std::hypot(initial_takeoff_goal_.x - p.x, initial_takeoff_goal_.y - p.y);
      const double dz = initial_takeoff_goal_.z - p.z;
      if (dxy < waypoint_reach_radius_xy_ && std::fabs(dz) < waypoint_reach_radius_z_) {
        takeoff_phase_active_ = false;
        RCLCPP_INFO(get_logger(), "[TAKEOFF] initial target reached.");
      }
    }

    if (mission_active_ && current_wp_idx_ < waypoints_.size()) {
      const auto &wp = waypoints_[current_wp_idx_];
      bool goal_near_obs = false;
      bool skip_waypoint = false;
      double active_gate = 0.0;
      if (waypoint_skip_if_blocked_) {
        active_gate = waypoint_skip_obstacle_dist_;
        goal_near_obs = waypoint_too_close_to_obstacle(wp, active_gate);
      } else if (waypoint_skip_hard_obstacle_dist_ > 0.0) {
        active_gate = waypoint_skip_hard_obstacle_dist_;
        goal_near_obs = waypoint_too_close_to_obstacle(wp, active_gate);
      }
      if (enable_recovery_mode_timeout_skip_ && check_recovery_mode_timeout(goal_near_obs)) return;
      if (goal_near_obs && !enable_obstacle_goal_standoff_reach_) {
        skip_waypoint = true;
      }
      if (skip_waypoint) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "[MISSION] waypoint %zu/%zu too close to obstacle (<%.2fm). Skipping.",
            current_wp_idx_ + 1, waypoints_.size(), active_gate);
        advance_waypoint();
        return;
      }
      const double dxy = std::hypot(wp.x - p.x, wp.y - p.y);
      const double dz = wp.z - p.z;
      double reach_xy = waypoint_reach_radius_xy_;
      if (goal_near_obs && enable_obstacle_goal_standoff_reach_) {
        const double frac = clamp(obstacle_goal_standoff_fraction_, 0.05, 0.95);
        const double standoff_xy = frac * std::max(0.5, waypoint_skip_max_use_range_);
        reach_xy = std::max(waypoint_reach_radius_xy_, standoff_xy);
      }
      if (dxy < reach_xy && std::fabs(dz) < waypoint_reach_radius_z_) {
        if (goal_near_obs && enable_obstacle_goal_standoff_reach_) {
          RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 1000,
              "[MISSION] waypoint %zu/%zu near obstacle: standoff reached (dxy=%.2f <= %.2f). Advancing.",
              current_wp_idx_ + 1, waypoints_.size(), dxy, reach_xy);
        }
        advance_waypoint();
        return;
      }
      publish_active_waypoint_goal(wp, goal_near_obs, reach_xy);
      if (check_no_progress_skip(wp, dxy, goal_near_obs)) return;

      if (enable_waypoint_stall_skip_) {
        const rclcpp::Time tnow = now();
        if (progress_track_wp_idx_ != current_wp_idx_) {
          progress_track_wp_idx_ = current_wp_idx_;
          progress_track_best_dxy_ = dxy;
          progress_track_last_improve_time_ = tnow;
        } else if (dxy < progress_track_best_dxy_ - std::max(0.01, waypoint_stall_progress_delta_m_)) {
          progress_track_best_dxy_ = dxy;
          progress_track_last_improve_time_ = tnow;
        }

        const double stall_sec = (tnow - progress_track_last_improve_time_).seconds();
        if (stall_sec > std::max(1.0, waypoint_stall_timeout_sec_) &&
            dxy < std::max(waypoint_reach_radius_xy_, waypoint_stall_skip_radius_xy_)) {
          const double wp_obs_min = waypoint_min_obstacle_distance(wp);
          if (std::isfinite(wp_obs_min) && wp_obs_min < std::max(0.05, waypoint_stall_obstacle_gate_)) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "[MISSION] waypoint %zu/%zu stalled %.1fs near obstacle (dxy=%.2f obs=%.2f). Skipping.",
                current_wp_idx_ + 1, waypoints_.size(), stall_sec, dxy, wp_obs_min);
            advance_waypoint();
            return;
          }
        }
      }
    }
  }

  bool check_recovery_mode_timeout(bool goal_near_obs) {
    if (!enable_recovery_mode_timeout_skip_) return false;
    if (!goal_near_obs) {
      reset_recovery_mode_timeout_state();
      return false;
    }
    if (current_wp_idx_ >= waypoints_.size()) {
      reset_recovery_mode_timeout_state();
      return false;
    }
    if (last_dwa_state_time_.nanoseconds() <= 0) {
      reset_recovery_mode_timeout_state();
      return false;
    }
    const double mode_age = (now() - last_dwa_state_time_).seconds();
    if (mode_age > std::max(0.2, dwa_state_stale_sec_)) {
      recovery_mode_last_eval_time_ = now();
      return false;
    }
    const bool recovery_mode = (dwa_mode_ == "BYPASS" || dwa_mode_ == "FALLBACK");
    const rclcpp::Time tnow = now();
    if (!recovery_mode_timer_active_ || recovery_mode_wp_idx_ != current_wp_idx_ ||
        recovery_mode_last_eval_time_.nanoseconds() <= 0) {
      recovery_mode_timer_active_ = true;
      recovery_mode_wp_idx_ = current_wp_idx_;
      recovery_mode_accum_sec_ = 0.0;
      recovery_mode_last_eval_time_ = tnow;
      recovery_mode_name_ = dwa_mode_;
      return false;
    }

    double dt = (tnow - recovery_mode_last_eval_time_).seconds();
    recovery_mode_last_eval_time_ = tnow;
    if (!std::isfinite(dt) || dt < 0.0) dt = 0.0;
    dt = clamp(dt, 0.0, 1.0);
    if (recovery_mode) {
      recovery_mode_accum_sec_ += dt;
      recovery_mode_name_ = dwa_mode_;
    }
    const double timeout_sec = std::max(1.0, recovery_mode_timeout_sec_);
    if (recovery_mode_accum_sec_ < timeout_sec) return false;

    RCLCPP_WARN(get_logger(),
                "[MISSION] waypoint %zu/%zu recovery dwell exceeded: mode=%s accum=%.1fs (>= %.1fs). Mark reached and advance.",
                current_wp_idx_ + 1, waypoints_.size(), recovery_mode_name_.c_str(),
                recovery_mode_accum_sec_,
                timeout_sec);
    reset_recovery_mode_timeout_state();
    advance_waypoint();
    return true;
  }

  bool check_no_progress_skip(const geometry_msgs::msg::Point &wp,
                              double dxy,
                              bool goal_near_obs) {
    if (!enable_waypoint_no_progress_skip_) {
      reset_no_progress_state();
      return false;
    }
    if (current_wp_idx_ >= waypoints_.size()) {
      reset_no_progress_state();
      return false;
    }

    const rclcpp::Time tnow = now();
    if (no_progress_wp_idx_ != current_wp_idx_ || no_progress_ref_time_.nanoseconds() <= 0) {
      no_progress_wp_idx_ = current_wp_idx_;
      no_progress_ref_dxy_ = dxy;
      no_progress_ref_time_ = tnow;
      return false;
    }

    const double improve_gate = std::max(0.01, waypoint_no_progress_delta_m_);
    if (dxy < no_progress_ref_dxy_ - improve_gate) {
      no_progress_ref_dxy_ = dxy;
      no_progress_ref_time_ = tnow;
      return false;
    }

    const double no_prog_sec = (tnow - no_progress_ref_time_).seconds();
    const double timeout_sec = std::max(1.0, waypoint_no_progress_timeout_sec_);
    if (no_prog_sec < timeout_sec) return false;

    const double wp_obs_min = waypoint_min_obstacle_distance(wp);
    const bool obstacle_likely =
        goal_near_obs ||
        (std::isfinite(wp_obs_min) &&
         wp_obs_min < std::max(0.05, waypoint_no_progress_obstacle_gate_));
    if (!obstacle_likely) return false;

    RCLCPP_WARN(get_logger(),
                "[MISSION] waypoint %zu/%zu no progress for %.1fs (dxy=%.2f, obs=%.2f). Skipping.",
                current_wp_idx_ + 1, waypoints_.size(), no_prog_sec, dxy, wp_obs_min);
    reset_no_progress_state();
    advance_waypoint();
    return true;
  }

  void reset_no_progress_state() {
    no_progress_wp_idx_ = std::numeric_limits<size_t>::max();
    no_progress_ref_dxy_ = std::numeric_limits<double>::infinity();
    no_progress_ref_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }

  void reset_recovery_mode_timeout_state() {
    recovery_mode_timer_active_ = false;
    recovery_mode_name_.clear();
    recovery_mode_wp_idx_ = std::numeric_limits<size_t>::max();
    recovery_mode_accum_sec_ = 0.0;
    recovery_mode_last_eval_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
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

  bool waypoint_too_close_to_obstacle(const geometry_msgs::msg::Point &wp, double gate) const {
    if (!scan_.has_value() || !have_pose_) return false;
    const auto &scan = scan_.value();
    if (scan.ranges.empty()) return false;

    const auto &p = pose_.pose.position;
    const double yaw = yaw_from_pose(pose_);
    const double gate_clamped = std::max(0.05, gate);
    const int stride = std::max(1, waypoint_skip_scan_stride_);
    const double max_use =
        std::max(scan.range_min + 0.01, std::min<double>(scan.range_max, waypoint_skip_max_use_range_));

    double ang = scan.angle_min;
    for (size_t i = 0; i < scan.ranges.size(); ++i, ang += scan.angle_increment) {
      if (static_cast<int>(i) % stride != 0) continue;

      const float rf = scan.ranges[i];
      if (!std::isfinite(rf)) continue;
      double r = clamp(static_cast<double>(rf), scan.range_min, max_use);
      if (r <= scan.range_min + 1e-3) continue;

      const double aw = yaw + ang;
      const double ox = p.x + r * std::cos(aw);
      const double oy = p.y + r * std::sin(aw);
      const double dxy = std::hypot(wp.x - ox, wp.y - oy);
      if (dxy < gate_clamped) return true;
    }
    return false;
  }

  double waypoint_min_obstacle_distance(const geometry_msgs::msg::Point &wp) const {
    if (!scan_.has_value() || !have_pose_) return std::numeric_limits<double>::infinity();
    const auto &scan = scan_.value();
    if (scan.ranges.empty()) return std::numeric_limits<double>::infinity();

    const auto &p = pose_.pose.position;
    const double yaw = yaw_from_pose(pose_);
    const int stride = std::max(1, waypoint_skip_scan_stride_);
    const double max_use =
        std::max(scan.range_min + 0.01, std::min<double>(scan.range_max, waypoint_skip_max_use_range_));

    double min_d = std::numeric_limits<double>::infinity();
    double ang = scan.angle_min;
    for (size_t i = 0; i < scan.ranges.size(); ++i, ang += scan.angle_increment) {
      if (static_cast<int>(i) % stride != 0) continue;
      const float rf = scan.ranges[i];
      if (!std::isfinite(rf)) continue;
      double r = clamp(static_cast<double>(rf), scan.range_min, max_use);
      if (r <= scan.range_min + 1e-3) continue;

      const double aw = yaw + ang;
      const double ox = p.x + r * std::cos(aw);
      const double oy = p.y + r * std::sin(aw);
      min_d = std::min(min_d, std::hypot(wp.x - ox, wp.y - oy));
    }
    return min_d;
  }

  void publish_takeoff_nav_cmd() {
    if (!have_pose_) return;

    const auto &p = pose_.pose.position;
    const double dx = initial_takeoff_goal_.x - p.x;
    const double dy = initial_takeoff_goal_.y - p.y;
    const double dz = initial_takeoff_goal_.z - p.z;

    double vx = nav_kp_xy_ * dx;
    double vy = nav_kp_xy_ * dy;
    const double vxy = std::hypot(vx, vy);
    if (vxy > nav_max_speed_xy_ && vxy > 1e-6) {
      const double s = nav_max_speed_xy_ / vxy;
      vx *= s;
      vy *= s;
    }

    const double vz = clamp(nav_kp_z_ * dz, -nav_max_speed_z_, nav_max_speed_z_);
    const double wz = hold_yaw_during_takeoff_
                          ? 0.0
                          : shape_yaw_to_target(initial_takeoff_goal_.x, initial_takeoff_goal_.y, vx, vy);
    publish_cmd(vx, vy, vz, wz);
  }

  void publish_mission_nav_cmd() {
    const auto &p = pose_.pose.position;
    if (!mission_active_ || current_wp_idx_ >= waypoints_.size()) {
      publish_cmd(0.0, 0.0, 0.0, 0.0);
      return;
    }

    const auto &target = waypoints_[current_wp_idx_];
    const double ex = target.x - p.x;
    const double ey = target.y - p.y;
    const double ez = target.z - p.z;

    double vx = nav_kp_xy_ * ex;
    double vy = nav_kp_xy_ * ey;
    const double vxy = std::hypot(vx, vy);
    if (vxy > nav_max_speed_xy_ && vxy > 1e-6) {
      const double s = nav_max_speed_xy_ / vxy;
      vx *= s;
      vy *= s;
    }
    const double vz = clamp(nav_kp_z_ * ez, -nav_max_speed_z_, nav_max_speed_z_);
    const double wz = shape_yaw_to_target(target.x, target.y, vx, vy);
    publish_cmd(vx, vy, vz, wz);
  }

  void advance_waypoint() {
    reset_no_progress_state();
    reset_recovery_mode_timeout_state();
    if (current_wp_idx_ + 1 < waypoints_.size()) {
      current_wp_idx_++;
      publish_goal_waypoint(current_wp_idx_);
      RCLCPP_INFO(get_logger(), "[MISSION] waypoint %zu/%zu", current_wp_idx_ + 1, waypoints_.size());
      return;
    }
    mission_active_ = false;
    RCLCPP_INFO(get_logger(), "[MISSION] complete. Holding at center.");
  }

  void state_machine() {
    if (!started_ || !have_pose_) return;

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
      RCLCPP_INFO(get_logger(), "OFFBOARD + ARM complete. Spiral mission navigation active.");
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
        RCLCPP_WARN(this->get_logger(), "OFFBOARD not accepted yet, will retry.");
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
        RCLCPP_WARN(this->get_logger(), "ARM not accepted yet, will retry.");
      }
      arm_request_pending_ = false;
    };
    arming_client_->async_send_request(req, cb);
  }

  void poll_mission_input() {
    if (!input_active_) return;
    if (prompt_pending_) {
      std::cout << "Enter mission: <side_m> [z_layers] [z_step_m] [center_z], "
                   "'d' for default, 'h' help, 'q' quit input > "
                << std::flush;
      prompt_pending_ = false;
    }

    pollfd pfd{};
    pfd.fd = 0;
    pfd.events = POLLIN;
    const int pr = ::poll(&pfd, 1, 0);
    if (pr <= 0) return;

    std::string line;
    if (!std::getline(std::cin, line)) {
      input_active_ = false;
      RCLCPP_WARN(get_logger(), "Mission input stream closed; disabling CLI mission input.");
      return;
    }
    line = trim_copy(line);
    if (line.empty()) {
      prompt_pending_ = true;
      return;
    }
    const std::string low = lower_copy(line);
    if (low == "h" || low == "help" || low == "?") {
      print_input_help();
      prompt_pending_ = true;
      return;
    }
    if (low == "q" || low == "quit" || low == "exit") {
      input_active_ = false;
      RCLCPP_INFO(get_logger(), "Mission input disabled.");
      return;
    }
    if (low == "d" || low == "default") {
      pending_request_ =
          MissionRequest::defaults(default_square_size_m_, default_z_layers_, default_z_step_m_);
      RCLCPP_INFO(get_logger(), "Default mission selected: side=%.2f layers=%d z_step=%.2f",
                  pending_request_->side_m, pending_request_->z_layers, pending_request_->z_step_m);
      try_configure_mission_from_pending();
      prompt_pending_ = true;
      return;
    }

    std::istringstream iss(line);
    std::vector<double> vals;
    double v = 0.0;
    while (iss >> v) vals.push_back(v);
    if (vals.empty()) {
      RCLCPP_WARN(get_logger(), "Invalid input. Type 'h' for help.");
      prompt_pending_ = true;
      return;
    }

    MissionRequest req{};
    req.side_m = vals[0];
    req.z_layers = (vals.size() >= 2) ? std::max(1, static_cast<int>(std::lround(vals[1])))
                                      : std::max(1, default_z_layers_);
    req.z_step_m = (vals.size() >= 3) ? std::max(0.0, vals[2]) : std::max(0.0, default_z_step_m_);
    if (vals.size() >= 4) {
      req.has_center_z = true;
      req.center_z = vals[3];
    }

    if (req.side_m <= 0.0) {
      RCLCPP_WARN(get_logger(), "side_m must be > 0.");
      prompt_pending_ = true;
      return;
    }
    pending_request_ = req;
    RCLCPP_INFO(get_logger(), "Mission accepted: side=%.2f layers=%d z_step=%.2f%s",
                req.side_m, req.z_layers, req.z_step_m, req.has_center_z ? " with center_z" : "");
    try_configure_mission_from_pending();
    prompt_pending_ = true;
  }

  void print_input_help() {
    std::cout << "\nMapping mission input:\n"
              << "  <side_m> [z_layers] [z_step_m] [center_z]\n"
              << "Examples:\n"
              << "  30\n"
              << "  40 3 2.0\n"
              << "  36 2 1.5 6.0\n"
              << "Special:\n"
              << "  d  -> start default mission\n"
              << "  q  -> disable input\n\n";
  }

  void publish_initial_takeoff_goal() {
    if (!enable_initial_takeoff_setpoint_) return;

    initial_takeoff_goal_.x = initial_takeoff_x_;
    initial_takeoff_goal_.y = initial_takeoff_y_;
    initial_takeoff_goal_.z = initial_takeoff_z_;
    takeoff_phase_active_ = true;
    goal_pub_->publish(initial_takeoff_goal_);
    RCLCPP_INFO(get_logger(), "[TAKEOFF] initial goal -> [%.2f, %.2f, %.2f]",
                initial_takeoff_goal_.x, initial_takeoff_goal_.y, initial_takeoff_goal_.z);
  }

  double shape_yaw_to_target(double tx, double ty, double &vx, double &vy) const {
    if (!face_goal_to_setpoint_ || !have_pose_) return 0.0;

    const auto &p = pose_.pose.position;
    const double ex = tx - p.x;
    const double ey = ty - p.y;
    if (std::hypot(ex, ey) < 1e-3) return 0.0;

    const double yaw = yaw_from_pose(pose_);
    double yaw_err = wrap_pi(std::atan2(ey, ex) - yaw);
    const double yaw_db = clamp(face_goal_deadband_deg_, 0.0, 45.0) * M_PI / 180.0;
    if (std::fabs(yaw_err) < yaw_db) yaw_err = 0.0;

    const double wz = clamp(face_goal_k_yaw_ * yaw_err, -std::max(0.1, yaw_rate_max_),
                            std::max(0.1, yaw_rate_max_));

    const double turn_only = clamp(face_goal_turn_only_deg_, 5.0, 179.0) * M_PI / 180.0;
    if (std::fabs(yaw_err) > turn_only) {
      const double t =
          clamp((std::fabs(yaw_err) - turn_only) / std::max(1e-3, (M_PI - turn_only)), 0.0, 1.0);
      const double min_xy = clamp(face_goal_min_xy_scale_, 0.0, 1.0);
      const double scale = 1.0 - (1.0 - min_xy) * t;
      vx *= scale;
      vy *= scale;
    }

    return wz;
  }

  void try_configure_mission_from_pending() {
    if (!pending_request_.has_value()) return;
    if (!have_pose_) return;
    configure_mission(pending_request_.value());
    pending_request_.reset();
  }

  void add_waypoint(std::vector<geometry_msgs::msg::Point> &wps, double x, double y, double z) {
    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    if (!wps.empty()) {
      const auto &q = wps.back();
      const double d = std::sqrt((q.x - p.x) * (q.x - p.x) + (q.y - p.y) * (q.y - p.y) +
                                 (q.z - p.z) * (q.z - p.z));
      if (d < 1e-3) return;
    }
    wps.push_back(p);
  }

  void configure_mission(const MissionRequest &req) {
    const auto &p0 = pose_.pose.position;
    center_ = p0;
    const double default_center_z =
        enable_initial_takeoff_setpoint_ ? initial_takeoff_goal_.z : p0.z;
    const double z_center = req.has_center_z ? req.center_z : default_center_z;
    center_.z = z_center;
    const int layers = std::max(1, req.z_layers);
    const double z_step = std::max(0.0, req.z_step_m);
    const double side = std::max(1.0, req.side_m);
    const double half = 0.5 * side;
    const double pitch = std::max(0.5, spiral_pitch_ratio_ * lidar_range_m_);
    const std::string mission_pattern = lower_copy(trim_copy(mission_pattern_));
    const bool use_lawnmower =
        (mission_pattern == "lawnmower") || (mission_pattern == "boustrophedon") ||
        (mission_pattern == "coverage");
    const bool deterministic_loops = (mission_pattern == "deterministic_loops");

    std::vector<geometry_msgs::msg::Point> wps;
    add_waypoint(wps, center_.x, center_.y, z_center);

    for (int layer = 0; layer < layers; ++layer) {
      const double z = z_center + static_cast<double>(layer) * z_step;
      add_waypoint(wps, wps.back().x, wps.back().y, z);

      if (use_lawnmower) {
        // Boustrophedon sweep across the square ROI for uniform area coverage.
        const double lane_step = std::max(0.8, pitch);
        const int lane_count =
            std::max(2, static_cast<int>(std::ceil(side / std::max(1e-3, lane_step))) + 1);
        const double x_min = center_.x - half;
        const double x_max = center_.x + half;
        const double y_min = center_.y - half;
        const double y_max = center_.y + half;

        for (int lane = 0; lane < lane_count; ++lane) {
          const double y = std::min(y_max, y_min + static_cast<double>(lane) * lane_step);
          if ((lane % 2) == 0) {
            add_waypoint(wps, x_min, y, z);
            add_waypoint(wps, x_max, y, z);
          } else {
            add_waypoint(wps, x_max, y, z);
            add_waypoint(wps, x_min, y, z);
          }
        }

        const double y_last =
            std::min(y_max, y_min + static_cast<double>(lane_count - 1) * lane_step);
        if (y_max - y_last > 0.25 * lane_step) {
          if ((lane_count % 2) == 0) {
            add_waypoint(wps, x_min, y_max, z);
            add_waypoint(wps, x_max, y_max, z);
          } else {
            add_waypoint(wps, x_max, y_max, z);
            add_waypoint(wps, x_min, y_max, z);
          }
        }
      } else if (deterministic_loops) {
        const double loop_step = std::max(1.0, pitch);
        const int loop_count =
            std::max(1, static_cast<int>(std::ceil(half / std::max(1e-3, loop_step))));
        for (int loop = 1; loop <= loop_count; ++loop) {
          const double ru = std::min(half, static_cast<double>(loop) * loop_step);
          add_waypoint(wps, center_.x + ru, center_.y + ru, z);
          add_waypoint(wps, center_.x - ru, center_.y + ru, z);
          add_waypoint(wps, center_.x - ru, center_.y - ru, z);
          add_waypoint(wps, center_.x + ru, center_.y - ru, z);
          add_waypoint(wps, center_.x + ru, center_.y + ru, z);
        }
        add_waypoint(wps, center_.x, center_.y, z);
      } else {
        bool made_ring = false;
        for (double r = pitch; r <= half + 1e-6; r += pitch) {
          const double ru = std::min(r, half);
          add_waypoint(wps, center_.x + ru, center_.y + ru, z);
          add_waypoint(wps, center_.x - ru, center_.y + ru, z);
          add_waypoint(wps, center_.x - ru, center_.y - ru, z);
          add_waypoint(wps, center_.x + ru, center_.y - ru, z);
          add_waypoint(wps, center_.x + ru, center_.y + ru, z);
          made_ring = true;
        }

        if (!made_ring) {
          add_waypoint(wps, center_.x + half, center_.y + half, z);
          add_waypoint(wps, center_.x - half, center_.y + half, z);
          add_waypoint(wps, center_.x - half, center_.y - half, z);
          add_waypoint(wps, center_.x + half, center_.y - half, z);
          add_waypoint(wps, center_.x + half, center_.y + half, z);
        }
      }
    }

    if (return_to_center_) {
      add_waypoint(wps, center_.x, center_.y, z_center);
    }

    waypoints_ = std::move(wps);
    current_wp_idx_ = (waypoints_.size() > 1) ? 1u : 0u;
    mission_active_ = !waypoints_.empty();

    const char *pattern_name =
        use_lawnmower ? "lawnmower" : (deterministic_loops ? "deterministic_loops" : "spiral");
    RCLCPP_INFO(get_logger(),
                "[MISSION] configured: pattern=%s side=%.2f pitch=%.2f layers=%d waypoints=%zu center=(%.2f %.2f %.2f)",
                pattern_name, side, pitch, layers, waypoints_.size(), center_.x, center_.y,
                center_.z);
    if (mission_active_) publish_goal_waypoint(current_wp_idx_);
  }

  void publish_goal_waypoint(size_t idx) {
    if (idx >= waypoints_.size()) return;
    publish_goal_point(waypoints_[idx], true);
    RCLCPP_INFO(get_logger(), "[MISSION] goal -> [%.2f, %.2f, %.2f]", waypoints_[idx].x,
                waypoints_[idx].y, waypoints_[idx].z);
  }

  void publish_active_waypoint_goal(const geometry_msgs::msg::Point &wp,
                                    bool goal_near_obs,
                                    double reach_xy) {
    if (!goal_pub_ || !have_pose_) return;

    geometry_msgs::msg::Point g = wp;
    if (goal_near_obs && enable_obstacle_goal_standoff_reach_) {
      const auto &p = pose_.pose.position;
      const double dxy = std::hypot(wp.x - p.x, wp.y - p.y);
      const double standoff = std::max(waypoint_reach_radius_xy_, reach_xy);
      if (dxy > standoff + 1e-3) {
        const double ux = (wp.x - p.x) / dxy;
        const double uy = (wp.y - p.y) / dxy;
        g.x = wp.x - ux * standoff;
        g.y = wp.y - uy * standoff;
        g.z = wp.z;
      }
    }

    publish_goal_point(g, false);
  }

  void publish_goal_point(const geometry_msgs::msg::Point &g, bool force) {
    if (!goal_pub_) return;
    const rclcpp::Time tnow = now();
    const bool have_last = have_last_goal_pub_;
    const double dg = have_last
                          ? std::hypot(g.x - last_goal_pub_.x, g.y - last_goal_pub_.y)
                          : std::numeric_limits<double>::infinity();
    const double age = have_last ? (tnow - last_goal_pub_time_).seconds()
                                 : std::numeric_limits<double>::infinity();
    const bool changed = dg >= std::max(0.01, obstacle_goal_republish_min_delta_);
    const bool timed_out = age >= std::max(0.05, obstacle_goal_republish_min_period_sec_);
    if (!force && !changed && !timed_out) return;

    goal_pub_->publish(g);
    last_goal_pub_ = g;
    last_goal_pub_time_ = tnow;
    have_last_goal_pub_ = true;
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

  // Params
  double setpoint_hz_{20.0};
  double warmup_sec_{2.0};
  double state_check_hz_{2.0};
  double request_retry_sec_{1.0};
  bool publish_heartbeat_{true};

  double nav_kp_xy_{0.8};
  double nav_max_speed_xy_{1.0};
  double nav_kp_z_{0.8};
  double nav_max_speed_z_{0.6};
  double waypoint_reach_radius_xy_{0.45};
  double waypoint_reach_radius_z_{0.30};
  bool enable_internal_goal_nav_{true};
  bool takeoff_priority_over_planner_{true};
  bool hold_yaw_during_takeoff_{true};
  double planner_cmd_timeout_sec_{0.30};
  bool face_goal_to_setpoint_{true};
  double face_goal_k_yaw_{0.9};
  double face_goal_deadband_deg_{3.0};
  double face_goal_turn_only_deg_{70.0};
  double face_goal_min_xy_scale_{0.2};
  double yaw_rate_max_{0.8};
  bool waypoint_skip_if_blocked_{true};
  double waypoint_skip_obstacle_dist_{0.9};
  double waypoint_skip_hard_obstacle_dist_{0.0};
  std::string waypoint_skip_scan_topic_{"/scan_horizontal"};
  int waypoint_skip_scan_stride_{3};
  double waypoint_skip_max_use_range_{12.0};
  bool enable_obstacle_goal_standoff_reach_{true};
  double obstacle_goal_standoff_fraction_{0.35};
  double obstacle_goal_republish_min_delta_{0.20};
  double obstacle_goal_republish_min_period_sec_{0.50};
  bool enable_waypoint_stall_skip_{true};
  double waypoint_stall_timeout_sec_{12.0};
  double waypoint_stall_progress_delta_m_{0.20};
  double waypoint_stall_skip_radius_xy_{1.80};
  double waypoint_stall_obstacle_gate_{1.15};
  bool enable_recovery_mode_timeout_skip_{true};
  double recovery_mode_timeout_sec_{35.0};
  double dwa_state_stale_sec_{2.5};
  bool enable_waypoint_no_progress_skip_{true};
  double waypoint_no_progress_timeout_sec_{18.0};
  double waypoint_no_progress_delta_m_{0.10};
  double waypoint_no_progress_obstacle_gate_{1.60};

  std::string mission_pattern_{"lawnmower"};
  bool ask_mission_on_start_{true};
  double input_poll_hz_{10.0};
  bool print_input_help_on_start_{true};
  bool auto_start_default_mission_{false};
  double default_square_size_m_{30.0};
  int default_z_layers_{1};
  double default_z_step_m_{2.0};
  double lidar_range_m_{12.0};
  double spiral_pitch_ratio_{0.5};
  bool return_to_center_{true};
  bool enable_initial_takeoff_setpoint_{true};
  double initial_takeoff_x_{0.0};
  double initial_takeoff_y_{0.0};
  double initial_takeoff_z_{5.0};

  // State
  bool started_{false};
  bool have_pose_{false};
  bool offboard_set_{false};
  bool armed_{false};
  bool ready_logged_{false};
  bool mode_request_pending_{false};
  bool arm_request_pending_{false};
  bool mission_active_{false};
  bool takeoff_phase_active_{false};
  bool input_active_{false};
  bool prompt_pending_{false};
  bool planner_cmd_seen_{false};
  bool planner_override_logged_{false};
  bool recovery_mode_timer_active_{false};
  size_t current_wp_idx_{0};

  geometry_msgs::msg::PoseStamped pose_;
  std::optional<sensor_msgs::msg::LaserScan> scan_;
  geometry_msgs::msg::Point center_;
  geometry_msgs::msg::Point initial_takeoff_goal_;
  geometry_msgs::msg::TwistStamped planner_cmd_cache_;
  std::string dwa_mode_{"UNKNOWN"};
  std::string recovery_mode_name_;
  std::vector<geometry_msgs::msg::Point> waypoints_;
  geometry_msgs::msg::Point last_goal_pub_;
  size_t no_progress_wp_idx_{std::numeric_limits<size_t>::max()};
  size_t recovery_mode_wp_idx_{std::numeric_limits<size_t>::max()};
  size_t progress_track_wp_idx_{std::numeric_limits<size_t>::max()};
  double progress_track_best_dxy_{std::numeric_limits<double>::infinity()};
  double no_progress_ref_dxy_{std::numeric_limits<double>::infinity()};
  double recovery_mode_accum_sec_{0.0};
  rclcpp::Time no_progress_ref_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time recovery_mode_last_eval_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time progress_track_last_improve_time_{0, 0, RCL_ROS_TIME};
  std::optional<MissionRequest> pending_request_;

  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time next_mode_request_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time next_arm_request_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_planner_cmd_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_dwa_state_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_goal_pub_time_{0, 0, RCL_ROS_TIME};
  bool have_last_goal_pub_{false};

  // ROS
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr planner_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr dwa_state_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr goal_pub_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
  rclcpp::TimerBase::SharedPtr setpoint_timer_;
  rclcpp::TimerBase::SharedPtr state_timer_;
  rclcpp::TimerBase::SharedPtr input_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LawnmowerMappingMode>());
  rclcpp::shutdown();
  return 0;
}
