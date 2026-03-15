// Last modified: 2026-02-19 13:05:00 +07
// Added: wall-follow takeover gating (stuck+weak/no DWA) and faster return to goal-seeking when path re-opens
// Removed: over-eager wall-follow entry that could suppress normal goal-directed DWA behavior

#include <algorithm>
#include <chrono>
#include <cmath>
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

namespace {
inline double clamp(double x, double lo, double hi) { return std::max(lo, std::min(hi, x)); }
inline double wrap_pi(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

struct ObPoint {
  double x;
  double y;
};

inline void rot_body_to_world(double yaw, double vx_b, double vy_b, double &vx_w, double &vy_w) {
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  vx_w = c * vx_b - s * vy_b;
  vy_w = s * vx_b + c * vy_b;
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
}  // namespace

class LocalPlannerHybrid : public rclcpp::Node {
 public:
  LocalPlannerHybrid() : Node("uav_local_planner_hybrid") {
    // ------------------- Motion -------------------
    control_dt_ = declare_parameter<double>("control_dt", 0.05);
    sim_dt_ = declare_parameter<double>("sim_dt", 0.10);
    horizon_sec_ = declare_parameter<double>("horizon_sec", 1.8);

    v_max_ = declare_parameter<double>("v_max", 2.5);
    vy_max_ = declare_parameter<double>("vy_max", 2.5);
    yaw_rate_max_ = declare_parameter<double>("yaw_rate_max", 0.45);
    ax_max_ = declare_parameter<double>("ax_max", 1.0);
    ay_max_ = declare_parameter<double>("ay_max", 1.0);
    yaw_accel_max_ = declare_parameter<double>("yaw_accel_max", 1.2);
    vz_max_ = declare_parameter<double>("vz_max", 1.0);

    // ------------------- DWA sampling -------------------
    nx_samples_ = declare_parameter<int>("nx_samples", 11);
    ny_samples_ = declare_parameter<int>("ny_samples", 9);
    nw_samples_ = declare_parameter<int>("nw_samples", 11);

    // ------------------- LiDAR / collision -------------------
    max_use_range_ = declare_parameter<double>("max_use_range", 12.0);
    obstacle_cloud_range_ = declare_parameter<double>("obstacle_cloud_range", 8.0);
    scan_stride_ = declare_parameter<int>("scan_stride", 2);
    safety_radius_ = declare_parameter<double>("safety_radius", 0.60);
    collision_radius_ = declare_parameter<double>("collision_radius", 0.30);
    occ_dist_ = declare_parameter<double>("occ_dist", 2.6);
    prefer_dist_ = declare_parameter<double>("prefer_dist", 7.0);

    // ------------------- Sector guidance -------------------
    sectors_n_ = std::max(5, static_cast<int>(declare_parameter<int64_t>("sectors_n", 15)));
    fov_deg_ = clamp(declare_parameter<double>("fov_deg", 270.0), 30.0, 359.0);
    sector_clear_percentile_ = declare_parameter<double>("sector_clear_percentile", 0.50);
    sector_occ_percentile_ = declare_parameter<double>("sector_occ_percentile", 0.20);
    sector_hold_margin_ = declare_parameter<double>("sector_hold_margin", 0.12);
    sector_commit_time_sec_ = declare_parameter<double>("sector_commit_time_sec", 0.8);
    near_wall_override_dist_ = declare_parameter<double>("near_wall_override_dist", 2.6);
    near_wall_override_free_only_ = declare_parameter<bool>("near_wall_override_free_only", true);
    near_wall_switch_min_gain_ = declare_parameter<double>("near_wall_switch_min_gain", 0.45);
    sector_gate_deg_ = declare_parameter<double>("sector_gate_deg", 75.0);

    // Sector-selection cost
    sector_w_goal_ = declare_parameter<double>("sector_w_goal", 1.35);
    sector_w_switch_ = declare_parameter<double>("sector_w_switch", 0.30);
    sector_w_clear_ = declare_parameter<double>("sector_w_clear", 0.30);

    // DWA trajectory cost
    w_goal_align_ = declare_parameter<double>("w_goal_align", 1.10);
    w_progress_ = declare_parameter<double>("w_progress", 2.60);
    w_clearance_ = declare_parameter<double>("w_clearance", 1.30);
    w_speed_ = declare_parameter<double>("w_speed", 0.20);
    w_smooth_ = declare_parameter<double>("w_smooth", 0.35);
    w_yaw_rate_ = declare_parameter<double>("w_yaw_rate", 0.70);
    w_sector_align_ = declare_parameter<double>("w_sector_align", 0.95);

    // Recovery stages
    use_relaxed_collision_stage_ = declare_parameter<bool>("use_relaxed_collision_stage", true);
    relaxed_collision_scale_ = declare_parameter<double>("relaxed_collision_scale", 0.75);
    use_full_window_stage_ = declare_parameter<bool>("use_full_window_stage", true);
    full_window_yaw_cap_ = declare_parameter<double>("full_window_yaw_cap", 0.25);
    full_window_collision_scale_ = declare_parameter<double>("full_window_collision_scale", 0.70);

    // Goal / terminal
    goal_hold_enter_ = declare_parameter<double>("goal_hold_enter", 0.35);
    goal_hold_exit_ = declare_parameter<double>("goal_hold_exit", 0.70);
    terminal_radius_ = declare_parameter<double>("terminal_radius", 1.1);
    terminal_k_v_ = declare_parameter<double>("terminal_k_v", 0.60);
    terminal_v_max_ = declare_parameter<double>("terminal_v_max", 0.60);
    terminal_k_yaw_ = declare_parameter<double>("terminal_k_yaw", 1.0);
    terminal_turn_only_deg_ = declare_parameter<double>("terminal_turn_only_deg", 20.0);

    // Fallback
    fallback_speed_ = declare_parameter<double>("fallback_speed", 0.28);
    fallback_yaw_rate_ = declare_parameter<double>("fallback_yaw_rate", 0.20);

    // Wall-follow / boundary-follow mode
    enable_wall_follow_mode_ = declare_parameter<bool>("enable_wall_follow_mode", true);
    goal_block_cone_deg_ = declare_parameter<double>("goal_block_cone_deg", 10.0);
    goal_block_dist_ = declare_parameter<double>("goal_block_dist", 2.6);
    wall_follow_side_cone_deg_ = declare_parameter<double>("wall_follow_side_cone_deg", 20.0);
    wall_follow_front_cone_deg_ = declare_parameter<double>("wall_follow_front_cone_deg", 20.0);
    wall_follow_d_ref_ = declare_parameter<double>("wall_follow_d_ref", 2.0);
    wall_follow_k_lat_ = declare_parameter<double>("wall_follow_k_lat", 0.9);
    wall_follow_k_yaw_ = declare_parameter<double>("wall_follow_k_yaw", 0.5);
    wall_follow_v_forward_ = declare_parameter<double>("wall_follow_v_forward", 0.34);
    wall_follow_vy_max_ = declare_parameter<double>("wall_follow_vy_max", 0.45);
    wall_follow_wz_max_ = declare_parameter<double>("wall_follow_wz_max", 0.25);
    wall_follow_front_stop_dist_ = declare_parameter<double>("wall_follow_front_stop_dist", 0.95);
    wall_follow_front_slow_dist_ = declare_parameter<double>("wall_follow_front_slow_dist", 2.0);
    wall_follow_side_switch_gain_ =
        declare_parameter<double>("wall_follow_side_switch_gain", 1.0);
    wall_follow_min_time_sec_ = declare_parameter<double>("wall_follow_min_time_sec", 0.8);
    wall_follow_exit_goal_clear_dist_ =
        declare_parameter<double>("wall_follow_exit_goal_clear_dist", 3.0);
    wall_follow_exit_hold_sec_ = declare_parameter<double>("wall_follow_exit_hold_sec", 0.6);
    wall_follow_exit_min_progress_ =
        declare_parameter<double>("wall_follow_exit_min_progress", 0.25);
    wall_follow_takeover_speed_ = declare_parameter<double>("wall_follow_takeover_speed", 0.12);
    stuck_window_sec_ = declare_parameter<double>("stuck_window_sec", 1.2);
    stuck_min_progress_ = declare_parameter<double>("stuck_min_progress", 0.12);
    stuck_goal_dist_min_ = declare_parameter<double>("stuck_goal_dist_min", 2.0);
    stuck_trigger_count_ = declare_parameter<int>("stuck_trigger_count", 2);

    // Output/debug
    publish_world_cmd_ = declare_parameter<bool>("publish_world_cmd", true);
    publish_rollout_path_ = declare_parameter<bool>("publish_rollout_path", true);
    rollout_path_frame_ = declare_parameter<std::string>("rollout_path_frame", "link");
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
    pub_rollout_path_ = create_publisher<nav_msgs::msg::Path>("/hybrid/best_rollout_path", qos_cmd);

    const auto period = std::chrono::duration<double>(std::max(0.01, control_dt_));
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(period),
                               std::bind(&LocalPlannerHybrid::loop, this));

    last_cmd_time_ = now();
    last_debug_time_ = now();
    commit_until_ = now();
    wall_follow_enter_time_ = now();
    wall_follow_exit_start_time_ = now();
    stuck_last_time_ = now();

    RCLCPP_INFO(get_logger(), "Hybrid planner started: sector-guided DWA active.");
  }

 private:
  struct SectorTable {
    std::vector<double> center;
    std::vector<double> clear_r;
    std::vector<double> occ_r;
    std::vector<bool> free;
  };

  struct Candidate {
    bool valid{false};
    double vx_b{0.0};
    double vy_b{0.0};
    double wz{0.0};
    double cost{std::numeric_limits<double>::infinity()};
    double min_clear{std::numeric_limits<double>::infinity()};
  };

  struct ConeClearance {
    double dist{0.0};
    bool covered{false};
  };

  void loop() {
    if (!odom_.has_value() || !scan_.has_value() || !goal_.has_value()) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "[WAIT] waiting for odom/scan/goal");
      return;
    }

    const auto &odom = odom_.value();
    const auto &scan = scan_.value();
    const auto &goal = goal_.value();
    const rclcpp::Time tnow = now();

    const auto &p = odom.pose.pose.position;
    const double x0 = p.x;
    const double y0 = p.y;
    const double z0 = p.z;
    const double yaw0 = get_yaw_from_odom(odom);

    const double ex = goal.x - x0;
    const double ey = goal.y - y0;
    const double ez = goal.z - z0;
    const double d_goal_xy = std::hypot(ex, ey);

    update_goal_hold(d_goal_xy);
    if (goal_hold_active_) {
      publish_cmd(0.0, 0.0, clamp(0.8 * ez, -vz_max_, vz_max_), 0.0);
      publish_rollout_path(x0, y0, z0, yaw0, 0.0, 0.0, 0.0);
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "[GOAL] hold");
      return;
    }

    const double vz = clamp(0.8 * ez, -vz_max_, vz_max_);
    const double goal_ang_body = wrap_pi(std::atan2(ey, ex) - yaw0);
    const ConeClearance goal_ray =
        compute_cone_clearance(scan, goal_ang_body, goal_block_cone_deg_);
    const bool goal_blocked = is_goal_direction_blocked(goal_ang_body, goal_ray);
    update_stuck_watchdog(d_goal_xy, tnow);
    const bool stuck_now = (stuck_counter_ >= std::max(1, stuck_trigger_count_));

    const double d_front = compute_front_clearance(scan, wall_follow_front_cone_deg_);
    if (run_terminal_mode(ex, ey, ez, yaw0, d_goal_xy, d_front, x0, y0, z0)) {
      return;
    }

    if (enable_wall_follow_mode_) {
      if (wall_follow_active_) {
        if (should_exit_wall_follow(scan, goal_ang_body, d_goal_xy, tnow)) {
          wall_follow_active_ = false;
          wall_follow_exit_pending_ = false;
          stuck_counter_ = 0;
          RCLCPP_INFO(get_logger(), "[WF] exit -> resume goal-seeking");
        } else {
          run_wall_follow_mode(scan, goal_ang_body, d_goal_xy, x0, y0, z0, yaw0, vz);
          return;
        }
      }
    }

    build_obstacle_cloud(scan, x0, y0, yaw0, obstacle_points_);
    SectorTable st = compute_sector_table(scan);

    const int selected_sector = select_guidance_sector(goal_ang_body, d_front, st, tnow);
    const double sector_center = st.center[selected_sector];

    const double dtw = clamp((tnow - last_cmd_time_).seconds(), 0.02, 0.20);
    const double vx_cur = cmd_inited_ ? vx_cmd_b_ : 0.0;
    const double vy_cur = cmd_inited_ ? vy_cmd_b_ : 0.0;
    const double wz_cur = cmd_inited_ ? wz_cmd_ : 0.0;

    const double vx_lo = clamp(vx_cur - ax_max_ * dtw, -v_max_, v_max_);
    const double vx_hi = clamp(vx_cur + ax_max_ * dtw, -v_max_, v_max_);
    const double vy_lo = clamp(vy_cur - ay_max_ * dtw, -vy_max_, vy_max_);
    const double vy_hi = clamp(vy_cur + ay_max_ * dtw, -vy_max_, vy_max_);
    const double wz_lo = clamp(wz_cur - yaw_accel_max_ * dtw, -yaw_rate_max_, yaw_rate_max_);
    const double wz_hi = clamp(wz_cur + yaw_accel_max_ * dtw, -yaw_rate_max_, yaw_rate_max_);

    Candidate best = evaluate_window(vx_lo, vx_hi, vy_lo, vy_hi, wz_lo, wz_hi, x0, y0, yaw0,
                                     ex, ey, d_goal_xy, sector_center, collision_radius_, true,
                                     w_sector_align_);

    if (!best.valid && use_relaxed_collision_stage_) {
      const double relaxed_collision = std::max(0.16, relaxed_collision_scale_ * collision_radius_);
      best = evaluate_window(vx_lo, vx_hi, vy_lo, vy_hi, wz_lo, wz_hi, x0, y0, yaw0, ex, ey,
                             d_goal_xy, sector_center, relaxed_collision, false,
                             0.8 * w_sector_align_);
      if (best.valid) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1200,
            "HYBRID: recovered with relaxed collision radius (strict=%.2f relaxed=%.2f).",
            collision_radius_, relaxed_collision);
      }
    }

    if (!best.valid && use_full_window_stage_) {
      const double wide_collision = std::max(0.16, full_window_collision_scale_ * collision_radius_);
      const double wz_cap = std::min(yaw_rate_max_, std::max(0.05, full_window_yaw_cap_));
      best = evaluate_window(-v_max_, v_max_, -vy_max_, vy_max_, -wz_cap, wz_cap, x0, y0, yaw0,
                             ex, ey, d_goal_xy, sector_center, wide_collision, false, 0.4);
      if (best.valid) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1200,
            "HYBRID: recovered with full-window stage (w_cap=%.2f collision=%.2f).", wz_cap,
            wide_collision);
      }
    }

    const double best_speed = best.valid ? std::hypot(best.vx_b, best.vy_b) : 0.0;
    const bool weak_dwa = best.valid && (best_speed < wall_follow_takeover_speed_);
    if (enable_wall_follow_mode_ && stuck_now && (goal_blocked || weak_dwa)) {
      start_wall_follow(scan, d_goal_xy, goal_ang_body, tnow,
                        goal_blocked ? "stuck-goal-blocked" : "stuck-weak-dwa");
      run_wall_follow_mode(scan, goal_ang_body, d_goal_xy, x0, y0, z0, yaw0, vz);
      return;
    }

    if (best.valid) {
      publish_cmd(best.vx_b, best.vy_b, vz, best.wz);
      publish_rollout_path(x0, y0, z0, yaw0, best.vx_b, best.vy_b, best.wz);
      last_plan_vx_ = best.vx_b;
      last_plan_vy_ = best.vy_b;
      last_plan_w_ = best.wz;
      debug_status(d_goal_xy, d_front, selected_sector, st.center[selected_sector], best, "DWA");
      return;
    }

    if (enable_wall_follow_mode_) {
      start_wall_follow(scan, d_goal_xy, goal_ang_body, tnow, "no-valid-dwa");
      run_wall_follow_mode(scan, goal_ang_body, d_goal_xy, x0, y0, z0, yaw0, vz);
      return;
    }

    run_sector_fallback(st, selected_sector, d_front, x0, y0, z0, yaw0, vz);
  }

  void update_goal_hold(double d_goal_xy) {
    if (!goal_hold_active_ && d_goal_xy < goal_hold_enter_) {
      goal_hold_active_ = true;
    } else if (goal_hold_active_ && d_goal_xy > goal_hold_exit_) {
      goal_hold_active_ = false;
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
    if (d_front <= std::max(occ_dist_, safety_radius_ + 0.15)) return false;

    const double goal_yaw = std::atan2(ey, ex);
    const double yaw_err = wrap_pi(goal_yaw - yaw);
    const double turn_only = terminal_turn_only_deg_ * M_PI / 180.0;

    double v = clamp(terminal_k_v_ * d_goal_xy, 0.0, terminal_v_max_);
    if (std::fabs(yaw_err) > turn_only) {
      v = 0.0;
    }

    const double gx = ex / (d_goal_xy + 1e-6);
    const double gy = ey / (d_goal_xy + 1e-6);
    const double vx_w = v * gx;
    const double vy_w = v * gy;
    const double wz = clamp(terminal_k_yaw_ * yaw_err, -yaw_rate_max_, yaw_rate_max_);

    // Convert terminal command to body-space storage for smoothing continuity.
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    const double vx_b = c * vx_w + s * vy_w;
    const double vy_b = -s * vx_w + c * vy_w;

    publish_cmd(vx_b, vy_b, clamp(0.8 * ez, -vz_max_, vz_max_), wz);
    publish_rollout_path(x0, y0, z0, yaw, vx_b, vy_b, wz);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "[TERMINAL] active");
    return true;
  }

  void build_obstacle_cloud(const sensor_msgs::msg::LaserScan &scan,
                            double px,
                            double py,
                            double yaw,
                            std::vector<ObPoint> &out) const {
    out.clear();
    out.reserve(scan.ranges.size() / std::max(1, scan_stride_) + 1);

    double ang = scan.angle_min;
    for (size_t i = 0; i < scan.ranges.size(); ++i, ang += scan.angle_increment) {
      if (static_cast<int>(i) % std::max(1, scan_stride_) != 0) continue;
      const float rf = scan.ranges[i];
      if (!std::isfinite(rf)) continue;
      double r =
          clamp(static_cast<double>(rf), scan.range_min, std::min<double>(scan.range_max, max_use_range_));
      if (r <= scan.range_min + 1e-2) continue;
      if (r > obstacle_cloud_range_) continue;

      const double aw = yaw + ang;
      out.push_back({px + r * std::cos(aw), py + r * std::sin(aw)});
    }
  }

  ConeClearance compute_cone_clearance(const sensor_msgs::msg::LaserScan &scan,
                                       double center_ang,
                                       double half_cone_deg) const {
    const double half = half_cone_deg * M_PI / 180.0;
    double ang = scan.angle_min;
    double d = max_use_range_;
    size_t covered = 0;
    for (float rf : scan.ranges) {
      if (std::fabs(wrap_pi(ang - center_ang)) <= half) {
        covered++;
        double r = std::isfinite(rf) ? static_cast<double>(rf) : max_use_range_;
        r = clamp(r, scan.range_min, std::min<double>(scan.range_max, max_use_range_));
        d = std::min(d, r);
      }
      ang += scan.angle_increment;
    }
    if (covered == 0) {
      return {0.0, false};
    }
    return {d, true};
  }

  double compute_front_clearance(const sensor_msgs::msg::LaserScan &scan, double cone_deg) const {
    return compute_cone_clearance(scan, 0.0, cone_deg).dist;
  }

  bool is_goal_direction_blocked(double goal_ang_body, const ConeClearance &goal_ray) const {
    const double scan_half = 0.5 * fov_deg_ * M_PI / 180.0;
    if (std::fabs(goal_ang_body) > scan_half) {
      return false;
    }
    if (!goal_ray.covered) {
      return false;
    }
    return goal_ray.dist < goal_block_dist_;
  }

  void update_stuck_watchdog(double d_goal_xy, const rclcpp::Time &tnow) {
    if (!stuck_watchdog_init_) {
      stuck_watchdog_init_ = true;
      stuck_last_goal_dist_ = d_goal_xy;
      stuck_last_time_ = tnow;
      return;
    }
    const double dt = (tnow - stuck_last_time_).seconds();
    if (dt < stuck_window_sec_) return;

    const double progress = stuck_last_goal_dist_ - d_goal_xy;
    const bool far_goal = d_goal_xy > stuck_goal_dist_min_;
    if (far_goal && progress < stuck_min_progress_) {
      stuck_counter_++;
    } else {
      stuck_counter_ = 0;
    }
    stuck_last_goal_dist_ = d_goal_xy;
    stuck_last_time_ = tnow;
  }

  void start_wall_follow(const sensor_msgs::msg::LaserScan &scan,
                         double d_goal_xy,
                         double goal_ang_body,
                         const rclcpp::Time &tnow,
                         const std::string &reason) {
    if (wall_follow_active_) return;

    const ConeClearance left = compute_cone_clearance(scan, M_PI_2, wall_follow_side_cone_deg_);
    const ConeClearance right = compute_cone_clearance(scan, -M_PI_2, wall_follow_side_cone_deg_);

    int side = wall_follow_side_;
    if (side == 0) side = 1;
    if (left.covered && right.covered) {
      side = (left.dist >= right.dist) ? 1 : -1;
    } else if (left.covered) {
      side = 1;
    } else if (right.covered) {
      side = -1;
    }

    wall_follow_side_ = side;
    wall_follow_active_ = true;
    wall_follow_entry_goal_dist_ = d_goal_xy;
    wall_follow_enter_time_ = tnow;
    wall_follow_exit_pending_ = false;

    RCLCPP_WARN(get_logger(),
                "[WF] enter (%s): side=%s goal_ang=%.1fdeg d_goal=%.2f",
                reason.c_str(), (wall_follow_side_ > 0) ? "left" : "right",
                goal_ang_body * 180.0 / M_PI, d_goal_xy);
  }

  bool should_exit_wall_follow(const sensor_msgs::msg::LaserScan &scan,
                               double goal_ang_body,
                               double d_goal_xy,
                               const rclcpp::Time &tnow) {
    const double elapsed = (tnow - wall_follow_enter_time_).seconds();
    if (elapsed < wall_follow_min_time_sec_) {
      wall_follow_exit_pending_ = false;
      return false;
    }

    const ConeClearance goal_ray =
        compute_cone_clearance(scan, goal_ang_body, goal_block_cone_deg_);
    const bool goal_clear = goal_ray.covered && (goal_ray.dist > wall_follow_exit_goal_clear_dist_);
    const bool progress_ok =
        (wall_follow_entry_goal_dist_ - d_goal_xy) > wall_follow_exit_min_progress_;
    const double wf_elapsed = (tnow - wall_follow_enter_time_).seconds();
    const bool timeout_release = wf_elapsed > std::max(3.0, 3.0 * wall_follow_min_time_sec_);

    if (goal_clear && (progress_ok || timeout_release)) {
      if (!wall_follow_exit_pending_) {
        wall_follow_exit_pending_ = true;
        wall_follow_exit_start_time_ = tnow;
      }
      const double hold = (tnow - wall_follow_exit_start_time_).seconds();
      return hold >= wall_follow_exit_hold_sec_;
    }

    wall_follow_exit_pending_ = false;
    return false;
  }

  void run_wall_follow_mode(const sensor_msgs::msg::LaserScan &scan,
                            double goal_ang_body,
                            double d_goal_xy,
                            double x0,
                            double y0,
                            double z0,
                            double yaw0,
                            double vz) {
    ConeClearance front = compute_cone_clearance(scan, 0.0, wall_follow_front_cone_deg_);
    ConeClearance left = compute_cone_clearance(scan, M_PI_2, wall_follow_side_cone_deg_);
    ConeClearance right = compute_cone_clearance(scan, -M_PI_2, wall_follow_side_cone_deg_);

    if (!left.covered) left.dist = 0.0;
    if (!right.covered) right.dist = 0.0;
    const int better_side = (left.dist >= right.dist) ? 1 : -1;

    if (wall_follow_side_ == 0) {
      wall_follow_side_ = better_side;
    } else if (better_side != wall_follow_side_) {
      const double cur_clear = (wall_follow_side_ > 0) ? left.dist : right.dist;
      const double new_clear = (better_side > 0) ? left.dist : right.dist;
      if (new_clear > cur_clear + wall_follow_side_switch_gain_) {
        wall_follow_side_ = better_side;
      }
    }

    ConeClearance side = (wall_follow_side_ > 0) ? left : right;
    const double side_dist = side.covered ? side.dist : wall_follow_d_ref_;

    const double e_lat = wall_follow_d_ref_ - side_dist;
    double vy_b = -static_cast<double>(wall_follow_side_) * wall_follow_k_lat_ * e_lat;
    vy_b = clamp(vy_b, -wall_follow_vy_max_, wall_follow_vy_max_);

    const double slow_dist =
        std::max(wall_follow_front_stop_dist_ + 0.10, wall_follow_front_slow_dist_);
    const double front_dist = front.covered ? front.dist : 0.0;
    const double front_scale = clamp((front_dist - wall_follow_front_stop_dist_) /
                                         std::max(1e-3, slow_dist - wall_follow_front_stop_dist_),
                                     0.0, 1.0);
    double vx_b = wall_follow_v_forward_ * front_scale;
    if (front_dist <= wall_follow_front_stop_dist_) {
      vx_b = 0.0;
    }

    const double side_front_ang = (wall_follow_side_ > 0) ? (60.0 * M_PI / 180.0)
                                                           : (-60.0 * M_PI / 180.0);
    const double side_back_ang = (wall_follow_side_ > 0) ? (120.0 * M_PI / 180.0)
                                                          : (-120.0 * M_PI / 180.0);
    const ConeClearance side_front =
        compute_cone_clearance(scan, side_front_ang, 0.6 * wall_follow_side_cone_deg_);
    const ConeClearance side_back =
        compute_cone_clearance(scan, side_back_ang, 0.6 * wall_follow_side_cone_deg_);

    double wz = 0.0;
    if (side_front.covered && side_back.covered) {
      const double e_parallel = side_back.dist - side_front.dist;
      wz = -static_cast<double>(wall_follow_side_) * wall_follow_k_yaw_ * e_parallel;
    }
    if (front_dist <= wall_follow_front_stop_dist_ + 0.20) {
      wz += -static_cast<double>(wall_follow_side_) * 0.12;
    }

    // When goal ray is open, bias yaw back toward the setpoint to hand back to DWA cleanly.
    const ConeClearance goal_ray =
        compute_cone_clearance(scan, goal_ang_body, goal_block_cone_deg_);
    if (goal_ray.covered && goal_ray.dist > wall_follow_exit_goal_clear_dist_) {
      const double wz_goal = clamp(0.7 * goal_ang_body, -wall_follow_wz_max_, wall_follow_wz_max_);
      wz = 0.6 * wz + 0.4 * wz_goal;
    }

    wz = clamp(wz, -wall_follow_wz_max_, wall_follow_wz_max_);

    publish_cmd(vx_b, vy_b, vz, wz);
    publish_rollout_path(x0, y0, z0, yaw0, vx_b, vy_b, wz);
    last_plan_vx_ = vx_b;
    last_plan_vy_ = vy_b;
    last_plan_w_ = wz;

    Candidate wf;
    wf.valid = true;
    wf.vx_b = vx_b;
    wf.vy_b = vy_b;
    wf.wz = wz;
    wf.min_clear = std::min({front_dist, left.dist, right.dist});
    wf.cost = 0.0;
    const double side_angle = (wall_follow_side_ > 0) ? M_PI_2 : -M_PI_2;
    debug_status(d_goal_xy, front_dist, wall_follow_side_, side_angle, wf, "WFOLLOW");
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

    for (int i = 0; i < sectors_n_; ++i) {
      if (!samples[i].empty()) {
        st.clear_r[i] = percentile(samples[i], clamp(sector_clear_percentile_, 0.0, 1.0));
        st.occ_r[i] = percentile(samples[i], clamp(sector_occ_percentile_, 0.0, 1.0));
      }
      st.free[i] = st.occ_r[i] > occ_dist_;
    }
    return st;
  }

  static double percentile(std::vector<double> samples, double q) {
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
                     int cand_idx,
                     int current_idx,
                     const std::vector<double> &center,
                     const std::vector<double> &clear_r) const {
    const double d_goal = std::fabs(wrap_pi(goal_ang_body - center[cand_idx]));
    const double d_switch = std::fabs(center[cand_idx] - center[current_idx]);
    const double d_clear = 1.0 - clamp(clear_r[cand_idx] / std::max(0.1, prefer_dist_), 0.0, 1.0);
    return sector_w_goal_ * d_goal + sector_w_switch_ * d_switch + sector_w_clear_ * d_clear;
  }

  int pick_max_clear_sector(const SectorTable &st, bool require_free) const {
    int best = -1;
    double max_clear = -1.0;
    for (int i = 0; i < sectors_n_; ++i) {
      if (require_free && !st.free[i]) continue;
      if (st.clear_r[i] > max_clear) {
        max_clear = st.clear_r[i];
        best = i;
      }
    }
    if (best >= 0) return best;
    return 0;
  }

  int select_guidance_sector(double goal_ang_body,
                             double d_front,
                             const SectorTable &st,
                             const rclcpp::Time &tnow) {
    int current_idx = (selected_sector_idx_ >= 0 && selected_sector_idx_ < sectors_n_)
                          ? selected_sector_idx_
                          : angle_to_sector(0.0, st.center);

    int best_idx = -1;
    double best_cost = std::numeric_limits<double>::infinity();
    for (int i = 0; i < sectors_n_; ++i) {
      if (!st.free[i]) continue;
      const double c = sector_cost(goal_ang_body, i, current_idx, st.center, st.clear_r);
      if (c < best_cost) {
        best_cost = c;
        best_idx = i;
      }
    }

    if (best_idx < 0) {
      best_idx = pick_max_clear_sector(st, false);
    }

    if (selected_sector_idx_ >= 0 && selected_sector_idx_ < sectors_n_ && st.free[selected_sector_idx_]) {
      const double cur_cost =
          sector_cost(goal_ang_body, selected_sector_idx_, current_idx, st.center, st.clear_r);
      const double nxt_cost = sector_cost(goal_ang_body, best_idx, current_idx, st.center, st.clear_r);
      if (cur_cost <= nxt_cost + sector_hold_margin_) {
        best_idx = selected_sector_idx_;
      }
    }

    if (d_front < near_wall_override_dist_) {
      int clear_idx = pick_max_clear_sector(st, near_wall_override_free_only_);
      if (clear_idx < 0 && near_wall_override_free_only_) {
        clear_idx = pick_max_clear_sector(st, false);
      }
      if (clear_idx >= 0) {
        int chosen_idx = clear_idx;
        if (selected_sector_idx_ >= 0 && selected_sector_idx_ < sectors_n_) {
          const int cur_idx = selected_sector_idx_;
          const bool cur_allowed = !near_wall_override_free_only_ || st.free[cur_idx];
          if (cur_allowed) {
            const int cur_side = (st.center[cur_idx] >= 0.0) ? 1 : -1;
            const int new_side = (st.center[clear_idx] >= 0.0) ? 1 : -1;
            const double clear_gain = st.clear_r[clear_idx] - st.clear_r[cur_idx];
            // In sharp corners, do not flip left<->right unless new side is meaningfully clearer.
            if (new_side != cur_side && clear_gain < near_wall_switch_min_gain_) {
              chosen_idx = cur_idx;
            }
          }
        }
        best_idx = chosen_idx;
      }
    }

    if (committed_sector_ >= 0 && committed_sector_ < sectors_n_ && tnow < commit_until_) {
      if (st.occ_r[committed_sector_] > occ_dist_) {
        best_idx = committed_sector_;
      }
    } else {
      committed_sector_ = best_idx;
      commit_until_ = tnow + rclcpp::Duration::from_seconds(sector_commit_time_sec_);
    }

    selected_sector_idx_ = best_idx;
    return best_idx;
  }

  Candidate evaluate_window(double vx_lo,
                            double vx_hi,
                            double vy_lo,
                            double vy_hi,
                            double wz_lo,
                            double wz_hi,
                            double x0,
                            double y0,
                            double yaw0,
                            double ex,
                            double ey,
                            double d_goal0,
                            double sector_center,
                            double collision_radius,
                            bool hard_sector_gate,
                            double sector_weight_scale) const {
    Candidate best;
    const int nx = std::max(2, nx_samples_);
    const int ny = std::max(2, ny_samples_);
    const int nw = std::max(3, nw_samples_);
    const double gate = sector_gate_deg_ * M_PI / 180.0;

    // Hover seed.
    Candidate hover =
        rollout_and_score(0.0, 0.0, 0.0, x0, y0, yaw0, ex, ey, d_goal0, sector_center,
                          collision_radius, sector_weight_scale);
    if (hover.valid) {
      best = hover;
    }

    for (int ix = 0; ix < nx; ++ix) {
      const double fx = static_cast<double>(ix) / static_cast<double>(nx - 1);
      const double vx = vx_lo + fx * (vx_hi - vx_lo);
      for (int iy = 0; iy < ny; ++iy) {
        const double fy = static_cast<double>(iy) / static_cast<double>(ny - 1);
        const double vy = vy_lo + fy * (vy_hi - vy_lo);
        const double speed = std::hypot(vx, vy);
        if (speed > v_max_) continue;

        if (speed > 0.08) {
          const double vel_dir = std::atan2(vy, vx);
          if (hard_sector_gate && std::fabs(wrap_pi(vel_dir - sector_center)) > gate) {
            continue;
          }
        }

        for (int iw = 0; iw < nw; ++iw) {
          const double fw = static_cast<double>(iw) / static_cast<double>(nw - 1);
          const double wz = wz_lo + fw * (wz_hi - wz_lo);
          Candidate c =
              rollout_and_score(vx, vy, wz, x0, y0, yaw0, ex, ey, d_goal0, sector_center,
                                collision_radius, sector_weight_scale);
          if (!c.valid) continue;
          if (c.cost < best.cost) best = c;
        }
      }
    }
    return best;
  }

  Candidate rollout_and_score(double vx_b,
                              double vy_b,
                              double wz,
                              double x0,
                              double y0,
                              double yaw0,
                              double ex,
                              double ey,
                              double d_goal0,
                              double sector_center,
                              double collision_radius,
                              double sector_weight_scale) const {
    Candidate c;
    c.vx_b = vx_b;
    c.vy_b = vy_b;
    c.wz = wz;

    double x = x0;
    double y = y0;
    double yaw = yaw0;
    double min_clear = std::numeric_limits<double>::infinity();
    const double safe2 = collision_radius * collision_radius;
    const int steps = std::max(1, static_cast<int>(std::ceil(horizon_sec_ / sim_dt_)));

    for (int k = 0; k < steps; ++k) {
      yaw = wrap_pi(yaw + wz * sim_dt_);
      double vx_w, vy_w;
      rot_body_to_world(yaw, vx_b, vy_b, vx_w, vy_w);
      x += vx_w * sim_dt_;
      y += vy_w * sim_dt_;

      for (const auto &ob : obstacle_points_) {
        const double dx = x - ob.x;
        const double dy = y - ob.y;
        const double d2 = dx * dx + dy * dy;
        if (d2 <= safe2) {
          c.valid = false;
          return c;
        }
        min_clear = std::min(min_clear, std::sqrt(d2));
      }
    }

    c.min_clear = std::isfinite(min_clear) ? min_clear : max_use_range_;

    const double mdx = x - x0;
    const double mdy = y - y0;
    const double gnorm = std::hypot(ex, ey);
    const double mnorm = std::hypot(mdx, mdy);
    const double align = (mdx * ex + mdy * ey) / (std::max(1e-3, gnorm) * std::max(1e-3, mnorm));
    const double cost_goal_align = clamp((1.0 - align) * 0.5, 0.0, 1.0);

    const double d1 = std::hypot(ex - mdx, ey - mdy);
    const double progress = d_goal0 - d1;
    const double cost_progress = clamp(1.0 - progress / std::max(0.5, v_max_ * horizon_sec_), 0.0, 3.0);

    const double knee = safety_radius_ + 0.20;
    const double fade = knee + 2.0;
    double cost_clear = 0.0;
    if (c.min_clear <= knee) {
      cost_clear = 1.0;
    } else {
      cost_clear = clamp((fade - c.min_clear) / std::max(1e-3, fade - knee), 0.0, 1.0);
    }

    const double speed = std::hypot(vx_b, vy_b);
    const double cost_speed = 1.0 - clamp(speed / std::max(1e-3, v_max_), 0.0, 1.0);

    const double dv = std::hypot(vx_b - last_plan_vx_, vy_b - last_plan_vy_) / std::max(1e-3, v_max_);
    const double dw = std::fabs(wz - last_plan_w_) / std::max(1e-3, yaw_rate_max_);
    const double cost_smooth = clamp(0.7 * dv + 0.3 * dw, 0.0, 1.5);

    const double cost_yaw_rate = std::fabs(wz) / std::max(1e-3, yaw_rate_max_);

    double cost_sector = 0.5;
    if (speed > 0.08) {
      const double vel_dir = std::atan2(vy_b, vx_b);
      cost_sector = std::fabs(wrap_pi(vel_dir - sector_center)) / M_PI;
    }

    c.cost = w_goal_align_ * cost_goal_align + w_progress_ * cost_progress +
             w_clearance_ * cost_clear + w_speed_ * cost_speed + w_smooth_ * cost_smooth +
             w_yaw_rate_ * cost_yaw_rate + sector_weight_scale * w_sector_align_ * cost_sector;

    if (progress < 0.0) {
      c.cost += 1.2 * std::fabs(progress);
    }

    c.valid = true;
    return c;
  }

  void run_sector_fallback(const SectorTable &st,
                           int selected_sector,
                           double d_front,
                           double x0,
                           double y0,
                           double z0,
                           double yaw0,
                           double vz) {
    int use_idx = selected_sector;
    if (use_idx < 0 || use_idx >= sectors_n_) {
      use_idx = pick_max_clear_sector(st, false);
    }

    if (!st.free[use_idx]) {
      use_idx = pick_max_clear_sector(st, false);
    }

    const double ang = st.center[use_idx];
    const double clear = st.clear_r[use_idx];
    const double scale = clamp(clear / std::max(0.1, prefer_dist_), 0.2, 1.0);

    double vx_b = fallback_speed_ * scale * std::cos(ang);
    double vy_b = fallback_speed_ * scale * std::sin(ang);
    double wz = clamp(0.8 * ang, -fallback_yaw_rate_, fallback_yaw_rate_);

    if (d_front < safety_radius_ + 0.20 && vx_b > 0.0) {
      vx_b = 0.0;
    }

    publish_cmd(vx_b, vy_b, vz, wz);
    publish_rollout_path(x0, y0, z0, yaw0, vx_b, vy_b, wz);
    debug_status(0.0, d_front, use_idx, ang, Candidate{}, "FALLBACK");
  }

  void publish_cmd(double vx_b, double vy_b, double vz, double wz) {
    vx_cmd_b_ = vx_b;
    vy_cmd_b_ = vy_b;
    wz_cmd_ = wz;
    cmd_inited_ = true;
    last_cmd_time_ = now();

    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = last_cmd_time_;

    double vx_out = vx_b;
    double vy_out = vy_b;
    if (publish_world_cmd_ && odom_.has_value()) {
      const double yaw = get_yaw_from_odom(odom_.value());
      rot_body_to_world(yaw, vx_b, vy_b, vx_out, vy_out);
      cmd.header.frame_id = "map";
    } else {
      cmd.header.frame_id = "base_link";
    }

    cmd.twist.linear.x = clamp(vx_out, -v_max_, v_max_);
    cmd.twist.linear.y = clamp(vy_out, -v_max_, v_max_);
    cmd.twist.linear.z = clamp(vz, -vz_max_, vz_max_);
    cmd.twist.angular.z = clamp(wz, -yaw_rate_max_, yaw_rate_max_);
    pub_cmd_->publish(cmd);
  }

  void publish_rollout_path(double x0,
                            double y0,
                            double z0,
                            double yaw0,
                            double vx_b,
                            double vy_b,
                            double wz) {
    if (!publish_rollout_path_ || !pub_rollout_path_) return;

    nav_msgs::msg::Path path;
    path.header.stamp = now();
    const bool use_base_link = (rollout_path_frame_ == "base_link");
    path.header.frame_id = use_base_link ? "base_link" : "map";

    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.orientation.w = 1.0;
    pose.pose.position.x = use_base_link ? 0.0 : x0;
    pose.pose.position.y = use_base_link ? 0.0 : y0;
    pose.pose.position.z = use_base_link ? 0.0 : z0;
    path.poses.push_back(pose);

    const int steps = std::max(1, static_cast<int>(std::ceil(horizon_sec_ / sim_dt_)));
    double px = pose.pose.position.x;
    double py = pose.pose.position.y;
    double yaw = yaw0;
    for (int k = 0; k < steps; ++k) {
      if (use_base_link) {
        px += vx_b * sim_dt_;
        py += vy_b * sim_dt_;
      } else {
        yaw = wrap_pi(yaw + wz * sim_dt_);
        double vx_w, vy_w;
        rot_body_to_world(yaw, vx_b, vy_b, vx_w, vy_w);
        px += vx_w * sim_dt_;
        py += vy_w * sim_dt_;
      }
      pose.pose.position.x = px;
      pose.pose.position.y = py;
      path.poses.push_back(pose);
    }
    pub_rollout_path_->publish(path);
  }

  void debug_status(double d_goal,
                    double d_front,
                    int selected_sector,
                    double sector_ang,
                    const Candidate &best,
                    const std::string &mode) {
    const double period = 1.0 / std::max(0.1, debug_hz_);
    const rclcpp::Time t = now();
    if ((t - last_debug_time_).seconds() < period) return;
    last_debug_time_ = t;

    if (best.valid) {
      RCLCPP_INFO(get_logger(),
                  "mode=%s d_goal=%.2f d_front=%.2f sec=%d(%.1fdeg) | cmd_b(vx=%.2f vy=%.2f w=%.2f) "
                  "clear=%.2f cost=%.3f",
                  mode.c_str(), d_goal, d_front, selected_sector, sector_ang * 180.0 / M_PI,
                  best.vx_b, best.vy_b, best.wz, best.min_clear, best.cost);
    } else {
      RCLCPP_INFO(get_logger(), "mode=%s d_goal=%.2f d_front=%.2f sec=%d(%.1fdeg)", mode.c_str(),
                  d_goal, d_front, selected_sector, sector_ang * 180.0 / M_PI);
    }
  }

 private:
  // Params
  double control_dt_{0.05};
  double sim_dt_{0.10};
  double horizon_sec_{1.8};

  double v_max_{0.85};
  double vy_max_{0.60};
  double yaw_rate_max_{0.45};
  double ax_max_{1.0};
  double ay_max_{1.0};
  double yaw_accel_max_{1.2};
  double vz_max_{1.0};

  int nx_samples_{11};
  int ny_samples_{9};
  int nw_samples_{11};

  double max_use_range_{12.0};
  double obstacle_cloud_range_{8.0};
  int scan_stride_{2};
  double safety_radius_{0.60};
  double collision_radius_{0.30};
  double occ_dist_{2.6};
  double prefer_dist_{7.0};

  int sectors_n_{15};
  double fov_deg_{270.0};
  double sector_clear_percentile_{0.50};
  double sector_occ_percentile_{0.20};
  double sector_hold_margin_{0.12};
  double sector_commit_time_sec_{0.8};
  double near_wall_override_dist_{2.6};
  bool near_wall_override_free_only_{true};
  double near_wall_switch_min_gain_{0.45};
  double sector_gate_deg_{75.0};

  double sector_w_goal_{1.35};
  double sector_w_switch_{0.30};
  double sector_w_clear_{0.30};

  double w_goal_align_{1.10};
  double w_progress_{2.60};
  double w_clearance_{1.30};
  double w_speed_{0.20};
  double w_smooth_{0.35};
  double w_yaw_rate_{0.70};
  double w_sector_align_{0.95};

  bool use_relaxed_collision_stage_{true};
  double relaxed_collision_scale_{0.75};
  bool use_full_window_stage_{true};
  double full_window_yaw_cap_{0.25};
  double full_window_collision_scale_{0.70};

  double goal_hold_enter_{0.35};
  double goal_hold_exit_{0.70};
  double terminal_radius_{1.1};
  double terminal_k_v_{0.60};
  double terminal_v_max_{0.60};
  double terminal_k_yaw_{1.0};
  double terminal_turn_only_deg_{20.0};

  double fallback_speed_{0.28};
  double fallback_yaw_rate_{0.20};

  bool enable_wall_follow_mode_{true};
  double goal_block_cone_deg_{10.0};
  double goal_block_dist_{2.6};
  double wall_follow_side_cone_deg_{20.0};
  double wall_follow_front_cone_deg_{20.0};
  double wall_follow_d_ref_{2.0};
  double wall_follow_k_lat_{0.9};
  double wall_follow_k_yaw_{0.5};
  double wall_follow_v_forward_{0.34};
  double wall_follow_vy_max_{0.45};
  double wall_follow_wz_max_{0.25};
  double wall_follow_front_stop_dist_{0.95};
  double wall_follow_front_slow_dist_{2.0};
  double wall_follow_side_switch_gain_{1.0};
  double wall_follow_min_time_sec_{0.8};
  double wall_follow_exit_goal_clear_dist_{3.0};
  double wall_follow_exit_hold_sec_{0.6};
  double wall_follow_exit_min_progress_{0.25};
  double wall_follow_takeover_speed_{0.12};
  double stuck_window_sec_{1.2};
  double stuck_min_progress_{0.12};
  double stuck_goal_dist_min_{2.0};
  int stuck_trigger_count_{2};

  bool publish_world_cmd_{true};
  bool publish_rollout_path_{true};
  std::string rollout_path_frame_{"map"};
  double debug_hz_{2.0};

  // State
  std::optional<nav_msgs::msg::Odometry> odom_;
  std::optional<sensor_msgs::msg::LaserScan> scan_;
  std::optional<geometry_msgs::msg::Point> goal_;

  bool goal_hold_active_{false};
  int selected_sector_idx_{-1};
  int committed_sector_{-1};
  rclcpp::Time commit_until_;
  bool wall_follow_active_{false};
  int wall_follow_side_{0};  // +1: left wall, -1: right wall
  double wall_follow_entry_goal_dist_{0.0};
  bool wall_follow_exit_pending_{false};
  rclcpp::Time wall_follow_enter_time_;
  rclcpp::Time wall_follow_exit_start_time_;
  bool stuck_watchdog_init_{false};
  double stuck_last_goal_dist_{0.0};
  int stuck_counter_{0};
  rclcpp::Time stuck_last_time_;

  bool cmd_inited_{false};
  double vx_cmd_b_{0.0};
  double vy_cmd_b_{0.0};
  double wz_cmd_{0.0};
  rclcpp::Time last_cmd_time_;

  double last_plan_vx_{0.0};
  double last_plan_vy_{0.0};
  double last_plan_w_{0.0};

  std::vector<ObPoint> obstacle_points_;
  rclcpp::Time last_debug_time_;

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
  rclcpp::spin(std::make_shared<LocalPlannerHybrid>());
  rclcpp::shutdown();
  return 0;
}
