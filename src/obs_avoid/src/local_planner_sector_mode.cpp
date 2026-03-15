// Last modified: 2026-02-18 15:26:00 +07
// Added: conservative default speed profile for sector planner to reduce overshoot in normal and terminal behavior
// Removed: overly aggressive default forward/yaw rates that could carry momentum past target path

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Matrix3x3.hpp"
#include "tf2/LinearMath/Quaternion.hpp"

using namespace std::chrono_literals;

namespace {
inline double clamp(double x, double lo, double hi) { return std::max(lo, std::min(hi, x)); }
inline double wrap_pi(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}
}  // namespace

class LocalPlannerSectorSelect : public rclcpp::Node {
 public:
  LocalPlannerSectorSelect() : Node("uav_local_planner_sector_select") {
    // ------------------- Motion -------------------
    control_dt_ = declare_parameter<double>("control_dt", 0.05);
    v_fwd_max_ = declare_parameter<double>("v_fwd_max", 1.2);
    v_fwd_min_ = declare_parameter<double>("v_fwd_min", 0.08);
    vz_max_ = declare_parameter<double>("vz_max", 1.0);

    k_yaw_ = declare_parameter<double>("k_yaw", 0.7);
    yaw_rate_max_ = declare_parameter<double>("yaw_rate_max", 0.45);
    yaw_deadband_deg_ = declare_parameter<double>("yaw_deadband_deg", 2.0);
    yaw_smooth_alpha_ = declare_parameter<double>("yaw_smooth_alpha", 0.20);
    turn_only_deg_ = declare_parameter<double>("turn_only_deg", 30.0);

    // ------------------- Sectors -------------------
    const int sectors = static_cast<int>(declare_parameter<int64_t>("sectors_n", 15));
    sectors_n_ = std::max(5, sectors);
    fov_deg_ = clamp(declare_parameter<double>("fov_deg", 270.0), 30.0, 359.0);
    occ_dist_ = declare_parameter<double>("occ_dist", 2.6);
    prefer_dist_ = declare_parameter<double>("prefer_dist", 7.0);
    max_use_range_ = declare_parameter<double>("max_use_range", 12.0);

    sector_clear_percentile_ = declare_parameter<double>("sector_clear_percentile", 0.50);
    sector_occ_percentile_ = declare_parameter<double>("sector_occ_percentile", 0.20);

    near_wall_override_dist_ = declare_parameter<double>("near_wall_override_dist", 2.8);
    commit_time_sec_ = declare_parameter<double>("commit_time_sec", 0.9);
    hold_margin_ = declare_parameter<double>("hold_margin", 0.12);

    // ------------------- Speed safety -------------------
    safety_stop_dist_ = declare_parameter<double>("safety_stop_dist", 1.4);
    safety_slow_dist_ = declare_parameter<double>("safety_slow_dist", 4.8);

    // ------------------- Cost weights -------------------
    w_goal_ = declare_parameter<double>("w_goal", 1.35);
    w_switch_ = declare_parameter<double>("w_switch", 0.30);
    w_clear_ = declare_parameter<double>("w_clear", 0.30);

    // ------------------- Goal / terminal behavior -------------------
    goal_hold_enter_ = declare_parameter<double>("goal_hold_enter", 0.50);
    goal_hold_exit_ = declare_parameter<double>("goal_hold_exit", 0.90);

    terminal_radius_ = declare_parameter<double>("terminal_radius", 1.0);
    terminal_turn_only_deg_ = declare_parameter<double>("terminal_turn_only_deg", 20.0);
    terminal_k_v_ = declare_parameter<double>("terminal_k_v", 0.45);
    terminal_v_max_ = declare_parameter<double>("terminal_v_max", 0.45);
    terminal_k_yaw_ = declare_parameter<double>("terminal_k_yaw", 1.0);

    yaw_hold_enter_ = declare_parameter<double>("yaw_hold_enter", 1.2);
    yaw_hold_exit_ = declare_parameter<double>("yaw_hold_exit", 1.8);
    yaw_stop_radius_ = declare_parameter<double>("yaw_stop_radius", 0.8);

    publish_rollout_path_ = declare_parameter<bool>("publish_rollout_path", true);
    rollout_path_frame_ = declare_parameter<std::string>("rollout_path_frame", "map");
    debug_hz_ = declare_parameter<double>("debug_hz", 2.0);

    auto qos_odom = rclcpp::SensorDataQoS();
    auto qos_scan = rclcpp::SensorDataQoS();
    auto qos_goal = rclcpp::QoS(10).reliable();
    auto qos_cmd = rclcpp::QoS(10).reliable();

    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
        "/mavros/local_position/odom", qos_odom,
        [this](nav_msgs::msg::Odometry::SharedPtr msg) { odom_ = *msg; });

    sub_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", qos_scan, [this](sensor_msgs::msg::LaserScan::SharedPtr msg) { scan_ = *msg; });

    sub_goal_ = create_subscription<geometry_msgs::msg::Point>(
        "/drone_goal", qos_goal,
        [this](geometry_msgs::msg::Point::SharedPtr msg) { goal_ = *msg; });

    pub_cmd_ = create_publisher<geometry_msgs::msg::TwistStamped>("/planner_cmd_vel", qos_cmd);
    pub_rollout_path_ = create_publisher<nav_msgs::msg::Path>("/sector/best_rollout_path", qos_cmd);

    const auto period = std::chrono::duration<double>(std::max(0.01, control_dt_));
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(period),
                               std::bind(&LocalPlannerSectorSelect::loop, this));

    last_debug_time_ = now();
    commit_until_ = now();
  }

 private:
  struct SectorTable {
    std::vector<double> center;
    std::vector<double> clear_r;
    std::vector<double> occ_r;
    std::vector<bool> free;
  };

  void loop() {
    if (!odom_.has_value() || !scan_.has_value() || !goal_.has_value()) {
      debug_throttle("[WAIT] waiting for odom/scan/goal");
      return;
    }

    const auto &odom = odom_.value();
    const auto &scan = scan_.value();
    const auto &goal = goal_.value();
    const auto tnow = now();

    const auto &p = odom.pose.pose.position;
    const double yaw = get_yaw_from_odom(odom);
    const double ex = goal.x - p.x;
    const double ey = goal.y - p.y;
    const double ez = goal.z - p.z;
    const double d_goal_xy = std::hypot(ex, ey);

    update_goal_latches(d_goal_xy, yaw);
    if (goal_hold_active_) {
      publish_cmd(0.0, 0.0, clamp(0.8 * ez, -vz_max_, vz_max_), 0.0);
      publish_rollout_path(p.x, p.y, p.z, yaw, 0.0, 0.0);
      debug_throttle("[GOAL] hold");
      return;
    }

    const double d_front = compute_front_clearance(scan, 20.0);
    if (run_terminal_mode(ex, ey, ez, yaw, d_goal_xy, d_front, p.x, p.y, p.z)) {
      return;
    }

    const double goal_yaw_world = std::atan2(ey, ex);
    const double goal_ang_body = wrap_pi(goal_yaw_world - yaw);

    SectorTable st = compute_sector_table(scan);

    int current_idx = (selected_sector_idx_ >= 0 && selected_sector_idx_ < sectors_n_)
                          ? selected_sector_idx_
                          : angle_to_sector(0.0, st.center);

    int best_idx = choose_sector(goal_ang_body, current_idx, st.center, st.clear_r, st.free);

    // Hysteresis: keep current if not clearly better.
    if (selected_sector_idx_ >= 0 && selected_sector_idx_ < sectors_n_ && st.free[selected_sector_idx_]) {
      const double cur_cost = sector_cost(goal_ang_body, selected_sector_idx_, current_idx, st.center, st.clear_r);
      const double best_cost = sector_cost(goal_ang_body, best_idx, current_idx, st.center, st.clear_r);
      if (cur_cost <= best_cost + hold_margin_) {
        best_idx = selected_sector_idx_;
      }
    }

    // Commitment: keep selected sector for a short duration unless occupied.
    if (committed_sector_ >= 0 && committed_sector_ < sectors_n_ && tnow < commit_until_) {
      if (st.occ_r[committed_sector_] > occ_dist_) {
        best_idx = committed_sector_;
      }
    } else {
      committed_sector_ = best_idx;
      commit_until_ = tnow + rclcpp::Duration::from_seconds(commit_time_sec_);
    }

    // Near wall: bias to maximum-clear sector.
    if (d_front < near_wall_override_dist_) {
      const int clear_idx = pick_max_clear_sector(st.clear_r, st.free, false);
      if (clear_idx >= 0) {
        best_idx = clear_idx;
      }
    }

    selected_sector_idx_ = best_idx;

    // ------------------- Yaw -------------------
    double desired_yaw_world = wrap_pi(yaw + st.center[best_idx]);
    if (yaw_hold_active_) {
      desired_yaw_world = yaw_hold_world_;
    }

    double yaw_err = wrap_pi(desired_yaw_world - yaw);
    const double yaw_rate_cmd = compute_yaw_rate_cmd(yaw_err, d_goal_xy);

    // ------------------- Forward speed -------------------
    const double align = std::fabs(yaw_err);
    const double clear = st.clear_r[best_idx];

    const double s_align = clamp(1.0 - (align / (70.0 * M_PI / 180.0)), 0.0, 1.0);
    const double s_clear = clamp(clear / std::max(0.1, prefer_dist_), 0.0, 1.0);

    double v_fwd = v_fwd_min_ + (v_fwd_max_ - v_fwd_min_) * (0.7 * s_align + 0.3 * s_clear);

    // Slow near goal to prevent overshoot/orbit.
    if (d_goal_xy < 2.0) {
      v_fwd *= clamp(d_goal_xy / 2.0, 0.0, 1.0);
    }

    // Turn-then-go: for large yaw error, rotate first.
    const double turn_only = turn_only_deg_ * M_PI / 180.0;
    if (std::fabs(yaw_err) > turn_only) {
      v_fwd = 0.0;
    }

    // If selected sector is currently occupied, only allow tiny creep with enough front margin.
    if (!st.free[best_idx]) {
      if (d_front > safety_stop_dist_ + 0.5) {
        v_fwd = std::min(v_fwd, 0.35);
      } else {
        v_fwd = 0.0;
      }
    }

    // Front-based smooth safety cap + hard stop.
    const double slow_span = std::max(0.1, safety_slow_dist_ - safety_stop_dist_);
    const double front_scale = clamp((d_front - safety_stop_dist_) / slow_span, 0.0, 1.0);
    v_fwd = std::min(v_fwd, v_fwd_max_ * front_scale);
    if (d_front <= safety_stop_dist_) {
      v_fwd = 0.0;
    }

    // Body-forward command converted to ENU.
    const double vx = v_fwd * std::cos(yaw);
    const double vy = v_fwd * std::sin(yaw);
    const double vz = clamp(0.8 * ez, -vz_max_, vz_max_);

    publish_cmd(vx, vy, vz, yaw_rate_cmd);
    publish_rollout_path(p.x, p.y, p.z, yaw, vx, vy);
    debug_status(yaw, d_goal_xy, d_front, goal_ang_body, best_idx, st, v_fwd, yaw_err, yaw_rate_cmd);
  }

  void update_goal_latches(double d_goal_xy, double yaw) {
    if (!goal_hold_active_ && d_goal_xy < goal_hold_enter_) {
      goal_hold_active_ = true;
    } else if (goal_hold_active_ && d_goal_xy > goal_hold_exit_) {
      goal_hold_active_ = false;
    }

    if (!yaw_hold_active_ && d_goal_xy < yaw_hold_enter_) {
      yaw_hold_active_ = true;
      yaw_hold_world_ = yaw;
    } else if (yaw_hold_active_ && d_goal_xy > yaw_hold_exit_) {
      yaw_hold_active_ = false;
    }
  }

  bool run_terminal_mode(double ex,
                         double ey,
                         double ez,
                         double yaw,
                         double d_goal_xy,
                         double d_front,
                         double x0,
                         double y0,
                         double z0) {
    if (d_goal_xy > terminal_radius_) return false;
    if (d_front <= occ_dist_) return false;

    const double goal_yaw = std::atan2(ey, ex);
    const double yaw_err = wrap_pi(goal_yaw - yaw);
    const double turn_only = terminal_turn_only_deg_ * M_PI / 180.0;

    double v = clamp(terminal_k_v_ * d_goal_xy, 0.0, terminal_v_max_);
    if (std::fabs(yaw_err) > turn_only) {
      v = 0.0;
    }

    const double gx = ex / (d_goal_xy + 1e-6);
    const double gy = ey / (d_goal_xy + 1e-6);
    const double vx = v * gx;
    const double vy = v * gy;
    const double vz = clamp(0.8 * ez, -vz_max_, vz_max_);
    const double yaw_rate = clamp(terminal_k_yaw_ * yaw_err, -yaw_rate_max_, yaw_rate_max_);

    publish_cmd(vx, vy, vz, yaw_rate);
    publish_rollout_path(x0, y0, z0, yaw, vx, vy);
    debug_throttle("[TERMINAL] active");
    return true;
  }

  double compute_front_clearance(const sensor_msgs::msg::LaserScan &scan, double cone_deg) const {
    double d = max_use_range_;
    const double half = cone_deg * M_PI / 180.0;
    double ang = scan.angle_min;

    for (float rf : scan.ranges) {
      double r = std::isfinite(rf) ? static_cast<double>(rf) : max_use_range_;
      r = clamp(r, scan.range_min, std::min<double>(scan.range_max, max_use_range_));
      if (std::fabs(ang) <= half) {
        d = std::min(d, r);
      }
      ang += scan.angle_increment;
    }
    return d;
  }

  SectorTable compute_sector_table(const sensor_msgs::msg::LaserScan &scan) const {
    SectorTable st;
    st.center.assign(sectors_n_, 0.0);
    st.clear_r.assign(sectors_n_, max_use_range_);
    st.occ_r.assign(sectors_n_, max_use_range_);
    st.free.assign(sectors_n_, true);

    const double fov = fov_deg_ * M_PI / 180.0;
    const double half = 0.5 * fov;
    const double sector_w = fov / static_cast<double>(sectors_n_);

    std::vector<std::vector<double>> samples(sectors_n_);

    for (int i = 0; i < sectors_n_; ++i) {
      st.center[i] = -half + (static_cast<double>(i) + 0.5) * sector_w;
    }

    double ang = scan.angle_min;
    for (float rf : scan.ranges) {
      double r = std::isfinite(rf) ? static_cast<double>(rf) : max_use_range_;
      r = clamp(r, scan.range_min, std::min<double>(scan.range_max, max_use_range_));

      if (ang >= -half && ang <= half) {
        int idx = static_cast<int>(std::floor((ang + half) / sector_w));
        idx = std::max(0, std::min(sectors_n_ - 1, idx));
        samples[idx].push_back(r);
      }
      ang += scan.angle_increment;
    }

    const double clear_q = clamp(sector_clear_percentile_, 0.0, 1.0);
    const double occ_q = clamp(sector_occ_percentile_, 0.0, 1.0);

    for (int i = 0; i < sectors_n_; ++i) {
      if (!samples[i].empty()) {
        st.clear_r[i] = percentile_from_samples(samples[i], clear_q);
        st.occ_r[i] = percentile_from_samples(samples[i], occ_q);
      }
      st.free[i] = (st.occ_r[i] > occ_dist_);
    }

    return st;
  }

  static double percentile_from_samples(std::vector<double> samples, double q) {
    if (samples.empty()) return std::numeric_limits<double>::infinity();
    q = clamp(q, 0.0, 1.0);
    const size_t idx = static_cast<size_t>(std::floor(q * static_cast<double>(samples.size() - 1)));
    std::nth_element(samples.begin(), samples.begin() + static_cast<long>(idx), samples.end());
    return samples[idx];
  }

  int angle_to_sector(double a, const std::vector<double> &center) const {
    int best = 0;
    double best_d = std::numeric_limits<double>::infinity();
    for (int i = 0; i < static_cast<int>(center.size()); ++i) {
      const double d = std::fabs(wrap_pi(a - center[i]));
      if (d < best_d) {
        best_d = d;
        best = i;
      }
    }
    return best;
  }

  double sector_cost(double goal_ang_body,
                     int cand,
                     int current_idx,
                     const std::vector<double> &center,
                     const std::vector<double> &clear_r) const {
    const double d_goal = std::fabs(wrap_pi(goal_ang_body - center[cand]));
    const double d_switch = std::fabs(center[cand] - center[current_idx]);

    const double c = clamp(clear_r[cand] / std::max(0.1, prefer_dist_), 0.0, 1.0);
    const double d_clear = 1.0 - c;

    return w_goal_ * d_goal + w_switch_ * d_switch + w_clear_ * d_clear;
  }

  int choose_sector(double goal_ang_body,
                    int current_idx,
                    const std::vector<double> &center,
                    const std::vector<double> &clear_r,
                    const std::vector<bool> &free_flag) const {
    int best = -1;
    double best_cost = std::numeric_limits<double>::infinity();

    for (int i = 0; i < sectors_n_; ++i) {
      if (!free_flag[i]) continue;
      const double c = sector_cost(goal_ang_body, i, current_idx, center, clear_r);
      if (c < best_cost) {
        best_cost = c;
        best = i;
      }
    }

    if (best >= 0) return best;
    return pick_max_clear_sector(clear_r, free_flag, false);
  }

  int pick_max_clear_sector(const std::vector<double> &clear_r,
                            const std::vector<bool> &free_flag,
                            bool require_free) const {
    int best = -1;
    double max_clear = -1.0;
    for (int i = 0; i < sectors_n_; ++i) {
      if (require_free && !free_flag[i]) continue;
      if (clear_r[i] > max_clear) {
        max_clear = clear_r[i];
        best = i;
      }
    }
    return (best >= 0) ? best : 0;
  }

  double compute_yaw_rate_cmd(double &yaw_err, double d_goal_xy) {
    const double yaw_db = yaw_deadband_deg_ * M_PI / 180.0;
    if (std::fabs(yaw_err) < yaw_db) yaw_err = 0.0;
    if (d_goal_xy < yaw_stop_radius_) yaw_err = 0.0;

    const double yaw_rate = clamp(k_yaw_ * yaw_err, -yaw_rate_max_, yaw_rate_max_);
    yaw_rate_filt_ = (1.0 - yaw_smooth_alpha_) * yaw_rate_filt_ + yaw_smooth_alpha_ * yaw_rate;
    return yaw_rate_filt_;
  }

  void publish_cmd(double vx, double vy, double vz, double yaw_rate) {
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = now();
    cmd.header.frame_id = "map";  // ENU
    cmd.twist.linear.x = vx;
    cmd.twist.linear.y = vy;
    cmd.twist.linear.z = vz;
    cmd.twist.angular.z = yaw_rate;
    pub_cmd_->publish(cmd);
  }

  void publish_rollout_path(double x0, double y0, double z0, double yaw, double vx, double vy) {
    if (!publish_rollout_path_ || !pub_rollout_path_) return;

    nav_msgs::msg::Path path;
    path.header.stamp = now();
    const bool use_base_link_frame = (rollout_path_frame_ == "base_link");
    path.header.frame_id = use_base_link_frame ? "base_link" : "map";

    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.orientation.w = 1.0;
    pose.pose.position.x = use_base_link_frame ? 0.0 : x0;
    pose.pose.position.y = use_base_link_frame ? 0.0 : y0;
    pose.pose.position.z = use_base_link_frame ? 0.0 : z0;
    path.poses.push_back(pose);

    const int n_steps = std::max(1, static_cast<int>(std::ceil(1.2 / std::max(0.01, control_dt_))));
    double px = pose.pose.position.x;
    double py = pose.pose.position.y;
    const double vx_step = use_base_link_frame ? (vx * std::cos(yaw) + vy * std::sin(yaw)) : vx;
    const double vy_step = use_base_link_frame ? (-vx * std::sin(yaw) + vy * std::cos(yaw)) : vy;

    for (int k = 0; k < n_steps; ++k) {
      px += vx_step * control_dt_;
      py += vy_step * control_dt_;
      pose.pose.position.x = px;
      pose.pose.position.y = py;
      path.poses.push_back(pose);
    }

    pub_rollout_path_->publish(path);
  }

  static double get_yaw_from_odom(const nav_msgs::msg::Odometry &odom) {
    tf2::Quaternion q(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
                      odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    (void)roll;
    (void)pitch;
    return yaw;
  }

  void debug_throttle(const std::string &msg) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "%s", msg.c_str());
  }

  void debug_status(double yaw,
                    double d_goal_xy,
                    double d_front,
                    double goal_ang_body,
                    int best_idx,
                    const SectorTable &st,
                    double v_fwd,
                    double yaw_err,
                    double yaw_rate_cmd) {
    const double period = 1.0 / std::max(0.1, debug_hz_);
    const rclcpp::Time t = now();
    if ((t - last_debug_time_).seconds() < period) return;
    last_debug_time_ = t;

    std::string sec;
    sec.reserve(static_cast<size_t>(sectors_n_) * 8);
    for (int i = 0; i < sectors_n_; ++i) {
      char buf[48];
      std::snprintf(buf, sizeof(buf), "%s%.1f", st.free[i] ? "" : "X", st.clear_r[i]);
      sec += buf;
      if (i != sectors_n_ - 1) sec += " ";
    }

    RCLCPP_INFO(get_logger(),
                "yaw=%.1fdeg | goal_body=%.1fdeg | sel=%d(%.1fdeg) commit=%d | d_goal=%.2f d_front=%.2f | "
                "v=%.2f | yaw_err=%.1fdeg yaw_rate=%.2f | sectors(minm): %s",
                yaw * 180.0 / M_PI, goal_ang_body * 180.0 / M_PI, best_idx,
                st.center[best_idx] * 180.0 / M_PI, committed_sector_, d_goal_xy, d_front, v_fwd,
                yaw_err * 180.0 / M_PI, yaw_rate_cmd, sec.c_str());
  }

 private:
  // Parameters
  double control_dt_{0.05};
  double v_fwd_max_{1.2};
  double v_fwd_min_{0.08};
  double vz_max_{1.0};

  double k_yaw_{0.7};
  double yaw_rate_max_{0.45};
  double yaw_deadband_deg_{2.0};
  double yaw_smooth_alpha_{0.20};
  double turn_only_deg_{30.0};

  int sectors_n_{15};
  double fov_deg_{270.0};
  double occ_dist_{2.6};
  double prefer_dist_{7.0};
  double max_use_range_{12.0};

  double sector_clear_percentile_{0.50};
  double sector_occ_percentile_{0.20};

  double near_wall_override_dist_{2.8};
  double commit_time_sec_{0.9};
  double hold_margin_{0.12};

  double safety_stop_dist_{1.4};
  double safety_slow_dist_{4.8};

  double w_goal_{1.35};
  double w_switch_{0.30};
  double w_clear_{0.30};

  double goal_hold_enter_{0.50};
  double goal_hold_exit_{0.90};

  double terminal_radius_{1.0};
  double terminal_turn_only_deg_{20.0};
  double terminal_k_v_{0.45};
  double terminal_v_max_{0.45};
  double terminal_k_yaw_{1.0};

  double yaw_hold_enter_{1.2};
  double yaw_hold_exit_{1.8};
  double yaw_stop_radius_{0.8};

  bool publish_rollout_path_{true};
  std::string rollout_path_frame_{"map"};
  double debug_hz_{2.0};

  // State
  std::optional<nav_msgs::msg::Odometry> odom_;
  std::optional<sensor_msgs::msg::LaserScan> scan_;
  std::optional<geometry_msgs::msg::Point> goal_;

  bool goal_hold_active_{false};
  bool yaw_hold_active_{false};
  double yaw_hold_world_{0.0};
  double yaw_rate_filt_{0.0};

  int selected_sector_idx_{-1};
  int committed_sector_{-1};

  rclcpp::Time last_debug_time_;
  rclcpp::Time commit_until_;

  // ROS I/O
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_goal_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_cmd_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_rollout_path_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalPlannerSectorSelect>());
  rclcpp::shutdown();
  return 0;
}
