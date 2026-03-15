// Last modified: 2026-02-25 12:02:00 +07
// Added: 2D map-based goal occupancy check (OccupancyGrid) with optional force-hold when goal cell neighborhood is occupied
// Removed: goal validation blind spot where planner could not explicitly detect an occupied target from the map

#include <chrono>
#include <cmath>
#include <limits>
#include <optional>
#include <vector>
#include <algorithm>
#include <sstream>
#include <iomanip>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/string.hpp"

#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/LinearMath/Matrix3x3.hpp"

using namespace std::chrono_literals;

namespace {
inline double clamp(double x, double lo, double hi) { return std::max(lo, std::min(hi, x)); }
inline double wrap_pi(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

inline void cap_xy_speed(double &vx, double &vy, double v_cap) {
  const double s = std::hypot(vx, vy);
  if (s <= std::max(1e-6, v_cap)) return;
  const double k = v_cap / s;
  vx *= k;
  vy *= k;
}

struct ObPoint { double x, y; };

static double yaw_from_odom(const nav_msgs::msg::Odometry &odom) {
  tf2::Quaternion q(
      odom.pose.pose.orientation.x,
      odom.pose.pose.orientation.y,
      odom.pose.pose.orientation.z,
      odom.pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double r, p, y;
  m.getRPY(r, p, y);
  return y;
}

inline void rot_body_to_world(double yaw, double vx_b, double vy_b, double &vx_w, double &vy_w) {
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  vx_w = c * vx_b - s * vy_b;
  vy_w = s * vx_b + c * vy_b;
}
} // namespace

class DwaLocalPlanner : public rclcpp::Node {
public:
  DwaLocalPlanner() : Node("dwa_local_planner_skeleton") {
    // -------- Parameters (keep simple first) --------
    v_max_  = declare_parameter<double>("v_max", 2.5);
    vy_max_ = declare_parameter<double>("vy_max", 2.0);
    w_max_  = declare_parameter<double>("w_max", 1.0);
    vz_max_ = declare_parameter<double>("vz_max", 1.0);

    ax_max_ = declare_parameter<double>("ax_max", 1.0);
    ay_max_ = declare_parameter<double>("ay_max", 1.0);
    aw_max_ = declare_parameter<double>("aw_max", 2.2);

    control_dt_ = declare_parameter<double>("control_dt", 0.05);
    sim_dt_     = declare_parameter<double>("sim_dt", 0.10);
    horizon_    = declare_parameter<double>("horizon_sec", 2.4);

    nx_ = declare_parameter<int>("nx_samples", 9);
    ny_ = declare_parameter<int>("ny_samples", 7);
    nw_ = declare_parameter<int>("nw_samples", 11);

    max_use_range_ = declare_parameter<double>("max_use_range", 12.0);
    scan_stride_   = declare_parameter<int>("scan_stride", 2);
    safety_radius_ = declare_parameter<double>("safety_radius", 0.65);
    collision_radius_ = declare_parameter<double>("collision_radius", 0.32);
    collision_from_safety_scale_ =
        declare_parameter<double>("collision_from_safety_scale", 0.85);
    hard_clearance_margin_ = declare_parameter<double>("hard_clearance_margin", 0.05);
    final_cmd_clearance_margin_ = declare_parameter<double>("final_cmd_clearance_margin", 0.08);
    obs_range_     = declare_parameter<double>("obstacle_cloud_range", 12.0);
    fallback_strafe_speed_ = declare_parameter<double>("fallback_strafe_speed", 0.32);
    fallback_forward_speed_ = declare_parameter<double>("fallback_forward_speed", 0.18);
    fallback_yaw_rate_ = declare_parameter<double>("fallback_yaw_rate", 0.45);
    fallback_front_margin_ = declare_parameter<double>("fallback_front_margin", 0.30);
    fallback_side_margin_ = declare_parameter<double>("fallback_side_margin", 0.00);
    enable_bypass_assist_ = declare_parameter<bool>("enable_bypass_assist", true);
    bypass_front_trigger_dist_ = declare_parameter<double>("bypass_front_trigger_dist", 1.8);
    bypass_side_open_dist_ = declare_parameter<double>("bypass_side_open_dist", 1.0);
    bypass_strafe_speed_ = declare_parameter<double>("bypass_strafe_speed", 0.55);
    bypass_forward_speed_ = declare_parameter<double>("bypass_forward_speed", 0.24);
    bypass_yaw_rate_ = declare_parameter<double>("bypass_yaw_rate", 0.35);
    bypass_lock_sec_ = declare_parameter<double>("bypass_lock_sec", 1.0);
    bypass_turn_hysteresis_ = declare_parameter<double>("bypass_turn_hysteresis", 0.60);
    bypass_max_continuous_sec_ = declare_parameter<double>("bypass_max_continuous_sec", 6.0);
    bypass_cooldown_sec_ = declare_parameter<double>("bypass_cooldown_sec", 2.5);
    bypass_disable_near_goal_dist_ =
        declare_parameter<double>("bypass_disable_near_goal_dist", 2.5);
    bypass_orbit_escape_dist_ = declare_parameter<double>("bypass_orbit_escape_dist", 1.8);
    bypass_outward_push_speed_ =
        declare_parameter<double>("bypass_outward_push_speed", 0.45);
    bypass_near_obs_wz_scale_ = declare_parameter<double>("bypass_near_obs_wz_scale", 0.35);
    bypass_goal_push_gain_ = declare_parameter<double>("bypass_goal_push_gain", 2.4);
    bypass_near_goal_dist_ = declare_parameter<double>("bypass_near_goal_dist", 4.5);
    bypass_near_goal_strafe_scale_ =
        declare_parameter<double>("bypass_near_goal_strafe_scale", 0.40);
    enable_sharp_turn_assist_ = declare_parameter<bool>("enable_sharp_turn_assist", true);
    sharp_turn_trigger_dist_ = declare_parameter<double>("sharp_turn_trigger_dist", 2.0);
    sharp_turn_yaw_boost_ = declare_parameter<double>("sharp_turn_yaw_boost", 1.6);
    sharp_turn_min_fwd_scale_ = declare_parameter<double>("sharp_turn_min_fwd_scale", 0.25);
    sharp_turn_side_speed_ = declare_parameter<double>("sharp_turn_side_speed", 0.28);
    final_gate_turn_rate_ = declare_parameter<double>("final_gate_turn_rate", 0.60);
    final_gate_strafe_speed_ = declare_parameter<double>("final_gate_strafe_speed", 0.22);
    recovery_side_lock_sec_ = declare_parameter<double>("recovery_side_lock_sec", 1.20);
    recovery_side_hysteresis_ = declare_parameter<double>("recovery_side_hysteresis", 0.40);
    deorbit_trigger_dist_ = declare_parameter<double>("deorbit_trigger_dist", 1.8);
    deorbit_outward_speed_ = declare_parameter<double>("deorbit_outward_speed", 0.30);
    deorbit_max_xy_speed_ = declare_parameter<double>("deorbit_max_xy_speed", 0.42);
    deorbit_min_turn_scale_ = declare_parameter<double>("deorbit_min_turn_scale", 0.25);
    steer_first_mode_ = declare_parameter<bool>("steer_first_mode", true);
    steer_first_yaw_cap_ = declare_parameter<double>("steer_first_yaw_cap", 0.10);
    steer_first_forward_speed_ = declare_parameter<double>("steer_first_forward_speed", 0.12);
    publish_debug_state_ = declare_parameter<bool>("publish_debug_state", true);
    debug_state_hz_ = declare_parameter<double>("debug_state_hz", 8.0);
    use_map_goal_occupancy_ = declare_parameter<bool>("use_map_goal_occupancy", true);
    map_topic_ = declare_parameter<std::string>("map_topic", "/map");
    map_occ_threshold_ = declare_parameter<int>("map_occ_threshold", 50);
    map_unknown_as_occupied_ = declare_parameter<bool>("map_unknown_as_occupied", false);
    map_goal_inflate_radius_ = declare_parameter<double>("map_goal_inflate_radius", 0.60);
    map_goal_force_hold_ = declare_parameter<bool>("map_goal_force_hold", false);
    enable_goal_obstacle_offset_ = declare_parameter<bool>("enable_goal_obstacle_offset", true);
    goal_obstacle_near_dist_ = declare_parameter<double>("goal_obstacle_near_dist", 0.85);
    goal_obstacle_target_clearance_ =
        declare_parameter<double>("goal_obstacle_target_clearance", 1.05);
    goal_obstacle_max_offset_ = declare_parameter<double>("goal_obstacle_max_offset", 0.90);

    // weights (lower cost is better)
    w_goal_heading_ = declare_parameter<double>("w_goal_heading", 1.0);
    w_progress_     = declare_parameter<double>("w_progress", 2.5);
    w_clearance_    = declare_parameter<double>("w_clearance", 1.2);
    w_speed_        = declare_parameter<double>("w_speed", 0.2);
    w_smooth_       = declare_parameter<double>("w_smooth", 0.3);
    w_yaw_rate_     = declare_parameter<double>("w_yaw_rate", 0.25);
    w_anti_orbit_   = declare_parameter<double>("w_anti_orbit", 1.4);
    enable_anti_orbit_cost_ = declare_parameter<bool>("enable_anti_orbit_cost", true);
    anti_orbit_near_dist_ = declare_parameter<double>("anti_orbit_near_dist", 2.2);
    anti_orbit_goal_gate_ = declare_parameter<double>("anti_orbit_goal_gate", 8.0);

    // Extra recovery stage before fallback.
    full_search_if_empty_ = declare_parameter<bool>("full_search_if_empty", true);
    full_search_w_max_ = declare_parameter<double>("full_search_w_max", 0.70);
    full_search_collision_scale_ =
        declare_parameter<double>("full_search_collision_scale", 0.70);

    // Heading behavior: face the setpoint direction while moving.
    face_goal_to_setpoint_ = declare_parameter<bool>("face_goal_to_setpoint", true);
    face_goal_k_yaw_ = declare_parameter<double>("face_goal_k_yaw", 0.9);
    face_goal_deadband_deg_ = declare_parameter<double>("face_goal_deadband_deg", 3.0);
    face_goal_turn_only_deg_ = declare_parameter<double>("face_goal_turn_only_deg", 45.0);
    face_goal_mix_with_dwa_ = declare_parameter<double>("face_goal_mix_with_dwa", 0.45);
    face_goal_min_xy_scale_ = declare_parameter<double>("face_goal_min_xy_scale", 0.20);
    enable_min_cruise_kick_ = declare_parameter<bool>("enable_min_cruise_kick", true);
    min_cruise_goal_dist_ = declare_parameter<double>("min_cruise_goal_dist", 2.5);
    min_cruise_speed_ = declare_parameter<double>("min_cruise_speed", 0.42);
    min_cruise_front_margin_ = declare_parameter<double>("min_cruise_front_margin", 0.45);
    min_cruise_blend_ = declare_parameter<double>("min_cruise_blend", 0.85);

    // If true: publish vx,vy in world/map frame (recommended for PX4 offboard velocity in ENU).
    publish_world_cmd_ = declare_parameter<bool>("publish_world_cmd", true);
    publish_rollout_path_ = declare_parameter<bool>("publish_rollout_path", true);
    rollout_path_frame_ = declare_parameter<std::string>("rollout_path_frame", "base_link");

    // -------- ROS I/O --------
    auto qos_sensor = rclcpp::SensorDataQoS();
    // Match spiral goal publisher durability so late-joining planner still receives
    // the last published goal waypoint.
    auto qos_goal   = rclcpp::QoS(10).reliable().transient_local();

    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
        "/mavros/local_position/odom", qos_sensor,
        [this](nav_msgs::msg::Odometry::SharedPtr msg){ odom_ = *msg; });

    sub_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan_horizontal", qos_sensor,
        [this](sensor_msgs::msg::LaserScan::SharedPtr msg){ scan_ = *msg; });

    sub_goal_ = create_subscription<geometry_msgs::msg::Point>(
        "/drone_goal", qos_goal,
        [this](geometry_msgs::msg::Point::SharedPtr msg){ goal_ = *msg; });
    auto qos_map = rclcpp::QoS(1).reliable().transient_local();
    sub_map_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        map_topic_, qos_map,
        [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg){ map_ = *msg; });

    pub_cmd_ = create_publisher<geometry_msgs::msg::TwistStamped>("/planner_cmd_vel", 10);
    pub_rollout_path_ = create_publisher<nav_msgs::msg::Path>("/dwa/best_rollout_path", 10);
    pub_state_ = create_publisher<std_msgs::msg::String>("/dwa/state", 10);

    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(control_dt_)),
        std::bind(&DwaLocalPlanner::loop, this));

    last_cmd_time_ = now();
    last_state_pub_time_ = now();
    cmd_inited_ = false;
    RCLCPP_INFO(get_logger(), "DWA skeleton started.");
  }

private:
  struct Candidate {
    bool valid{false};
    double vx_b{0}, vy_b{0}, wz{0};
    double cost{std::numeric_limits<double>::infinity()};
    double min_clear{std::numeric_limits<double>::infinity()};
    double end_x{0}, end_y{0}, end_yaw{0};
  };

  struct ConeClearance {
    double dist{0.0};
    bool covered{false};
  };

  void loop() {
    if (!has_required_inputs()) return;

    const auto &odom = odom_.value();
    const auto &scan = scan_.value();
    const auto &goal = goal_.value();

    const double x0 = odom.pose.pose.position.x;
    const double y0 = odom.pose.pose.position.y;
    const double z0 = odom.pose.pose.position.z;
    const double yaw0 = yaw_from_odom(odom);

    const double gx = goal.x;
    const double gy = goal.y;
    const double gz = goal.z;
    const double ex = gx - x0;
    const double ey = gy - y0;
    const double ez = gz - z0;
    const double d_goal = std::hypot(ex, ey);
    const double cb = std::cos(yaw0);
    const double sb = std::sin(yaw0);

    const ConeClearance front = cone_clearance_info(scan, 0.0, 22.0);
    const ConeClearance left = cone_clearance_info(scan, M_PI_2, 30.0);
    const ConeClearance right = cone_clearance_info(scan, -M_PI_2, 30.0);

    if (maybe_hold_for_occupied_goal(gx, gy, ez, d_goal, x0, y0, yaw0, front, left, right)) return;

    // 1) Build obstacle points in WORLD frame from scan
    build_obstacles_world(scan, x0, y0, yaw0, obstacles_);

    double gx_plan = gx;
    double gy_plan = gy;
    compute_goal_plan(gx, gy, gx_plan, gy_plan);
    const double ex_plan = gx_plan - x0;
    const double ey_plan = gy_plan - y0;
    const double d_goal_plan = std::hypot(ex_plan, ey_plan);
    const double goal_bx_plan = cb * ex_plan + sb * ey_plan;
    const double goal_by_plan = -sb * ex_plan + cb * ey_plan;

    if (maybe_run_bypass(scan, x0, y0, yaw0, gx_plan, gy_plan, ez, goal_by_plan, goal_bx_plan,
                         d_goal, d_goal_plan, front, left, right)) {
      return;
    }

    // 2) Dynamic window around current command (or odom if you want)
    const rclcpp::Time t = now();
    const double dtw = clamp((t - last_cmd_time_).seconds(), 0.02, 0.20);

    const double vx_cur = cmd_inited_ ? vx_cmd_b_ : 0.0;
    const double vy_cur = cmd_inited_ ? vy_cmd_b_ : 0.0;
    const double wz_cur = cmd_inited_ ? wz_cmd_   : 0.0;

    const double vx_lo = clamp(vx_cur - ax_max_ * dtw, -v_max_,  v_max_);
    const double vx_hi = clamp(vx_cur + ax_max_ * dtw, -v_max_,  v_max_);
    const double vy_lo = clamp(vy_cur - ay_max_ * dtw, -vy_max_, vy_max_);
    const double vy_hi = clamp(vy_cur + ay_max_ * dtw, -vy_max_, vy_max_);
    const double wz_lo = clamp(wz_cur - aw_max_ * dtw, -w_max_,  w_max_);
    const double wz_hi = clamp(wz_cur + aw_max_ * dtw, -w_max_,  w_max_);

    // 3) Evaluate samples
    const double strict_collision_radius =
        std::max(collision_radius_, collision_from_safety_scale_ * safety_radius_);
    Candidate best = evaluate_window(vx_lo, vx_hi, vy_lo, vy_hi, wz_lo, wz_hi,
                                     x0, y0, yaw0, gx_plan, gy_plan, d_goal_plan, strict_collision_radius);

    if (!best.valid) {
      // If strict envelope finds no candidate, retry with a smaller hard-collision radius.
      const double relaxed_collision_radius = std::max(0.18, 0.80 * strict_collision_radius);
      best = evaluate_window(vx_lo, vx_hi, vy_lo, vy_hi, wz_lo, wz_hi,
                             x0, y0, yaw0, gx_plan, gy_plan, d_goal_plan, relaxed_collision_radius);
      if (best.valid) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1500,
            "DWA: strict search empty, using relaxed collision radius (strict=%.2f, relaxed=%.2f).",
            strict_collision_radius, relaxed_collision_radius);
      }
    }

    if (!best.valid && full_search_if_empty_) {
      // Last DWA attempt: widen search to full command envelope with limited yaw-rate.
      const double wide_collision_radius =
          std::max(0.16, full_search_collision_scale_ * strict_collision_radius);
      best = evaluate_window(
          -v_max_, v_max_, -vy_max_, vy_max_,
          -std::min(w_max_, full_search_w_max_), std::min(w_max_, full_search_w_max_),
          x0, y0, yaw0, gx_plan, gy_plan, d_goal_plan, wide_collision_radius);
      if (best.valid) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1500,
            "DWA: recovered with full-envelope search (w_max=%.2f, collision=%.2f).",
            std::min(w_max_, full_search_w_max_), wide_collision_radius);
      }
    }

    // 4) Fallback if no valid plan: LiDAR-aware wall-follow sidestep
    if (!best.valid) {
      run_wall_follow_fallback(scan, x0, y0, yaw0, gx_plan, gy_plan, ez);
      return;
    }

    // 5) Publish chosen command (translation from DWA, yaw shaped to face setpoint)
    double vx_cmd = best.vx_b;
    double vy_cmd = best.vy_b;
    double wz_cmd = best.wz;

    apply_heading_shaping(ex_plan, ey_plan, d_goal, yaw0, best.wz, vx_cmd, vy_cmd, wz_cmd);
    apply_sharp_turn_assist(front, left, right, vx_cmd, vy_cmd, wz_cmd);
    apply_min_cruise_kick(front, d_goal, d_goal_plan, goal_bx_plan, goal_by_plan, vx_cmd, vy_cmd);

    // Final anti-crash gate on the chosen command.
    // If blocked, switch to wall-follow fallback instead of just freezing in place.
    const double pred_clear = predict_min_clearance(vx_cmd, vy_cmd, wz_cmd, x0, y0, yaw0);
    if (maybe_run_final_gate_escape(pred_clear, x0, y0, yaw0, ez, d_goal, goal_by_plan, front, left, right)) {
      return;
    }

    const double vz_cmd = clamp(0.8 * ez, -vz_max_, vz_max_);
    publish_cmd(vx_cmd, vy_cmd, vz_cmd, wz_cmd);
    publish_rollout_path(x0, y0, yaw0, vx_cmd, vy_cmd, wz_cmd);
    publish_debug_state("DWA", d_goal, front.dist, left.dist, right.dist, pred_clear,
                        best.cost, vx_cmd, vy_cmd, wz_cmd);
  }

  bool has_required_inputs() {
    if (odom_ && scan_ && goal_) return true;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for odom/scan/goal...");
    publish_debug_state("WAIT", 0.0, 0.0, 0.0, 0.0, 0.0, std::numeric_limits<double>::infinity(),
                        0.0, 0.0, 0.0);
    return false;
  }

  bool maybe_hold_for_occupied_goal(double gx,
                                    double gy,
                                    double ez,
                                    double d_goal,
                                    double x0,
                                    double y0,
                                    double yaw0,
                                    const ConeClearance &front,
                                    const ConeClearance &left,
                                    const ConeClearance &right) {
    map_ready_cached_ = use_map_goal_occupancy_ && map_.has_value();
    goal_occupied_cached_ = false;
    goal_occ_value_cached_ = -1;
    if (map_ready_cached_) {
      const double check_r = std::max(collision_radius_, map_goal_inflate_radius_);
      goal_occupied_cached_ = is_goal_occupied_in_map(gx, gy, check_r, &goal_occ_value_cached_);
    }
    if (!goal_occupied_cached_) return false;

    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1500,
                         "Goal appears occupied in /map (occ=%d).", goal_occ_value_cached_);
    if (!map_goal_force_hold_) return false;

    const double vz_hold = clamp(0.8 * ez, -vz_max_, vz_max_);
    publish_cmd(0.0, 0.0, vz_hold, 0.0);
    publish_rollout_path(x0, y0, yaw0, 0.0, 0.0, 0.0);
    publish_debug_state("GOAL_OCCUPIED_HOLD", d_goal, front.dist, left.dist, right.dist,
                        std::numeric_limits<double>::infinity(),
                        std::numeric_limits<double>::infinity(), 0.0, 0.0, 0.0);
    return true;
  }

  void compute_goal_plan(double gx, double gy, double &gx_plan, double &gy_plan) {
    gx_plan = gx;
    gy_plan = gy;
    if (!enable_goal_obstacle_offset_) return;

    const double near_gate = std::max(collision_radius_ + 0.05, goal_obstacle_near_dist_);
    double ox = 0.0, oy = 0.0, d_goal_obs = max_use_range_;
    if (!(nearest_obstacle_point(gx, gy, ox, oy, d_goal_obs) && d_goal_obs < near_gate)) return;

    const double target_clear = std::max(collision_radius_ + 0.10, goal_obstacle_target_clearance_);
    const double push = std::min(std::max(0.0, goal_obstacle_max_offset_),
                                 std::max(0.0, target_clear - d_goal_obs));
    if (push <= 1e-3) return;
    const double ux = (gx - ox) / std::max(1e-3, d_goal_obs);
    const double uy = (gy - oy) / std::max(1e-3, d_goal_obs);
    gx_plan = gx + push * ux;
    gy_plan = gy + push * uy;
  }

  bool maybe_run_bypass(const sensor_msgs::msg::LaserScan &scan,
                        double x0,
                        double y0,
                        double yaw0,
                        double gx_plan,
                        double gy_plan,
                        double ez,
                        double goal_by_plan,
                        double goal_bx_plan,
                        double d_goal,
                        double d_goal_plan,
                        const ConeClearance &front,
                        const ConeClearance &left,
                        const ConeClearance &right) {
    if (!enable_bypass_assist_ || d_goal <= 0.8) return false;

    const bool front_blocked = front.covered && (front.dist < std::max(0.5, bypass_front_trigger_dist_));
    const double left_score = left.covered ? left.dist : 0.0;
    const double right_score = right.covered ? right.dist : 0.0;
    const bool side_open = std::max(left_score, right_score) > std::max(0.4, bypass_side_open_dist_);
    const rclcpp::Time tnow = now();
    const bool bypass_condition = front_blocked && side_open && goal_bx_plan > 0.1 &&
                                  d_goal_plan > std::max(0.6, bypass_disable_near_goal_dist_);

    if (!bypass_condition) bypass_mode_active_ = false;
    if (!(bypass_condition && tnow >= bypass_cooldown_until_)) return false;

    if (!bypass_mode_active_) {
      bypass_mode_active_ = true;
      bypass_mode_start_ = tnow;
    }

    const double by_elapsed = (tnow - bypass_mode_start_).seconds();
    if (by_elapsed <= std::max(0.2, bypass_max_continuous_sec_)) {
      run_bypass_assist(scan, x0, y0, yaw0, gx_plan, gy_plan, ez, goal_by_plan, front, left, right);
      return true;
    }

    bypass_mode_active_ = false;
    bypass_cooldown_until_ = tnow + rclcpp::Duration::from_seconds(std::max(0.2, bypass_cooldown_sec_));
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "BYPASS timeout (%.1fs) -> forcing DWA replan cooldown %.1fs", by_elapsed,
                         std::max(0.2, bypass_cooldown_sec_));
    return false;
  }

  void apply_heading_shaping(double ex_plan,
                             double ey_plan,
                             double d_goal,
                             double yaw0,
                             double wz_best,
                             double &vx_cmd,
                             double &vy_cmd,
                             double &wz_cmd) const {
    wz_cmd = wz_best;
    if (!face_goal_to_setpoint_ || d_goal <= 0.15) return;

    const double goal_yaw = std::atan2(ey_plan, ex_plan);
    double yaw_err = wrap_pi(goal_yaw - yaw0);
    const double yaw_db = face_goal_deadband_deg_ * M_PI / 180.0;
    if (std::fabs(yaw_err) < yaw_db) yaw_err = 0.0;

    const double wz_goal = clamp(face_goal_k_yaw_ * yaw_err, -w_max_, w_max_);
    const double mix = clamp(face_goal_mix_with_dwa_, 0.0, 1.0);
    wz_cmd = (1.0 - mix) * wz_best + mix * wz_goal;

    const double turn_only = face_goal_turn_only_deg_ * M_PI / 180.0;
    if (std::fabs(yaw_err) <= turn_only) return;
    const double t = clamp((std::fabs(yaw_err) - turn_only) / std::max(1e-3, (M_PI - turn_only)),
                           0.0, 1.0);
    const double min_xy = clamp(face_goal_min_xy_scale_, 0.0, 1.0);
    const double scale = 1.0 - (1.0 - min_xy) * t;
    vx_cmd *= scale;
    vy_cmd *= scale;
  }

  void apply_sharp_turn_assist(const ConeClearance &front,
                               const ConeClearance &left,
                               const ConeClearance &right,
                               double &vx_cmd,
                               double &vy_cmd,
                               double &wz_cmd) const {
    if (!enable_sharp_turn_assist_ || !front.covered) return;
    const double trigger = std::max(0.3, sharp_turn_trigger_dist_);
    const double ratio = clamp((trigger - front.dist) / trigger, 0.0, 1.0);
    if (ratio <= 0.0) return;

    const double yaw_gain = 1.0 + std::max(0.0, sharp_turn_yaw_boost_) * ratio;
    wz_cmd = clamp(wz_cmd * yaw_gain, -w_max_, w_max_);

    const double min_fwd = clamp(sharp_turn_min_fwd_scale_, 0.0, 1.0);
    const double fwd_scale = 1.0 - (1.0 - min_fwd) * ratio;
    vx_cmd *= fwd_scale;

    const double left_score = left.covered ? left.dist : 0.0;
    const double right_score = right.covered ? right.dist : 0.0;
    const int side_sign = (left_score >= right_score) ? 1 : -1;
    const double side_inject = static_cast<double>(side_sign) * std::max(0.0, sharp_turn_side_speed_) * ratio;
    vy_cmd = clamp(vy_cmd + side_inject, -vy_max_, vy_max_);
  }

  void apply_min_cruise_kick(const ConeClearance &front,
                             double d_goal,
                             double d_goal_plan,
                             double goal_bx_plan,
                             double goal_by_plan,
                             double &vx_cmd,
                             double &vy_cmd) const {
    if (!enable_min_cruise_kick_ || d_goal <= std::max(0.5, min_cruise_goal_dist_)) return;

    const double cmd_sp = std::hypot(vx_cmd, vy_cmd);
    const double front_gate = collision_radius_ + std::max(0.05, min_cruise_front_margin_);
    const bool front_open = (!front.covered) || (front.dist > front_gate);
    if (!front_open || cmd_sp >= std::max(0.05, min_cruise_speed_)) return;

    const double inv_d = 1.0 / std::max(1e-3, d_goal_plan);
    const double vx_goal = goal_bx_plan * inv_d * std::max(0.05, min_cruise_speed_);
    const double vy_goal = goal_by_plan * inv_d * std::max(0.05, min_cruise_speed_);
    const double blend = clamp(min_cruise_blend_, 0.0, 1.0);
    vx_cmd = (1.0 - blend) * vx_cmd + blend * vx_goal;
    vy_cmd = (1.0 - blend) * vy_cmd + blend * vy_goal;
    cap_xy_speed(vx_cmd, vy_cmd, v_max_);
  }

  bool maybe_run_final_gate_escape(double pred_clear,
                                   double x0,
                                   double y0,
                                   double yaw0,
                                   double ez,
                                   double d_goal,
                                   double goal_by_plan,
                                   const ConeClearance &front,
                                   const ConeClearance &left,
                                   const ConeClearance &right) {
    const double hard_gate = collision_radius_ + final_cmd_clearance_margin_;
    if (pred_clear >= hard_gate) return false;

    const double left_score = left.covered ? left.dist : 0.0;
    const double right_score = right.covered ? right.dist : 0.0;
    const int side_sign = select_recovery_side(left_score, right_score, goal_by_plan, now());

    double vx_esc = 0.0;
    double vy_esc = 0.0;
    double wz_esc = static_cast<double>(side_sign) * std::min(std::max(0.1, final_gate_turn_rate_), w_max_);
    if (steer_first_mode_) {
      wz_esc = static_cast<double>(side_sign) * std::min(std::max(0.0, steer_first_yaw_cap_), w_max_);
    }

    if (std::max(left_score, right_score) > (collision_radius_ + 0.12)) {
      vy_esc = static_cast<double>(side_sign) * std::max(0.0, final_gate_strafe_speed_);
    }
    if (steer_first_mode_ && front.covered) {
      const double fwd_scale = clamp((front.dist - (collision_radius_ + 0.10)) / 1.0, 0.0, 1.0);
      vx_esc += std::max(0.0, steer_first_forward_speed_) * fwd_scale;
    }

    const double nearest_d = apply_deorbit_push(x0, y0, yaw0, 1.0, vx_esc, vy_esc);
    if (nearest_d < std::max(collision_radius_ + 0.05, deorbit_trigger_dist_)) {
      const double turn_scale = clamp(
          (nearest_d - collision_radius_) / std::max(1e-3, deorbit_trigger_dist_ - collision_radius_),
          clamp(deorbit_min_turn_scale_, 0.0, 1.0), 1.0);
      if (!steer_first_mode_) wz_esc *= turn_scale;
    }
    cap_xy_speed(vx_esc, vy_esc, std::max(0.12, deorbit_max_xy_speed_));

    double pred_esc = predict_min_clearance(vx_esc, vy_esc, wz_esc, x0, y0, yaw0);
    if (pred_esc < collision_radius_ + 0.02) {
      // Prefer translation-only peel-off over spinning in place.
      double vx_alt = vx_esc;
      double vy_alt = vy_esc;
      if (std::hypot(vx_alt, vy_alt) < 0.08) {
        vy_alt = static_cast<double>(side_sign) * std::max(0.10, 0.70 * final_gate_strafe_speed_);
      }
      cap_xy_speed(vx_alt, vy_alt, std::max(0.12, deorbit_max_xy_speed_));
      const double pred_alt = predict_min_clearance(vx_alt, vy_alt, 0.0, x0, y0, yaw0);
      if (pred_alt >= collision_radius_ + 0.02) {
        vx_esc = vx_alt;
        vy_esc = vy_alt;
        wz_esc = 0.0;
        pred_esc = pred_alt;
      } else {
        // Keep a forced escape command (never hard-stop), but bias to sidestep.
        vx_esc = 0.0;
        vy_esc = static_cast<double>(side_sign) * std::max(0.10, 0.60 * final_gate_strafe_speed_);
        wz_esc = steer_first_mode_ ? 0.0 : static_cast<double>(side_sign) * std::min(0.35, w_max_);
        pred_esc = predict_min_clearance(vx_esc, vy_esc, wz_esc, x0, y0, yaw0);
      }
    }

    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "DWA final gate blocked: pred=%.2f gate=%.2f -> turn-escape side=%s cmd(vx=%.2f vy=%.2f wz=%.2f)",
        pred_clear, hard_gate, (side_sign > 0) ? "left" : "right", vx_esc, vy_esc, wz_esc);
    const double vz_esc = clamp(0.8 * ez, -vz_max_, vz_max_);
    publish_cmd(vx_esc, vy_esc, vz_esc, wz_esc);
    publish_rollout_path(x0, y0, yaw0, vx_esc, vy_esc, wz_esc);
    publish_debug_state("FINAL_GATE_ESCAPE", d_goal, front.dist, left_score, right_score, pred_esc,
                        std::numeric_limits<double>::infinity(), vx_esc, vy_esc, wz_esc);
    return true;
  }

  void build_obstacles_world(const sensor_msgs::msg::LaserScan &scan,
                             double px, double py, double yaw,
                             std::vector<ObPoint> &out) const {
    out.clear();
    out.reserve(scan.ranges.size() / std::max(1, scan_stride_) + 1);

    double ang = scan.angle_min;
    for (size_t i = 0; i < scan.ranges.size(); ++i, ang += scan.angle_increment) {
      if (static_cast<int>(i) % std::max(1, scan_stride_) != 0) continue;

      const float rf = scan.ranges[i];
      if (!std::isfinite(rf)) continue;
      double r = clamp(static_cast<double>(rf), scan.range_min, std::min<double>(scan.range_max, max_use_range_));
      if (r <= scan.range_min + 1e-2) continue;
      if (r > obs_range_) continue;

      // ray angle is in sensor frame; assume scan is in base_link aligned with yaw
      const double aw = yaw + ang;
      out.push_back({px + r * std::cos(aw), py + r * std::sin(aw)});
    }
  }

  Candidate evaluate_window(double vx_lo, double vx_hi,
                            double vy_lo, double vy_hi,
                            double wz_lo, double wz_hi,
                            double x0, double y0, double yaw0,
                            double gx, double gy, double d_goal0,
                            double collision_radius) const {
    Candidate best;

    // Seed with an explicit hover candidate so planner always has a conservative fallback
    // when dynamic-window samples all get rejected.
    Candidate hover = rollout_and_score(0.0, 0.0, 0.0, x0, y0, yaw0, gx, gy, d_goal0, collision_radius);
    if (hover.valid) {
      best = hover;
    }

    const int nx = std::max(2, nx_);
    const int ny = std::max(2, ny_);
    const int nw = std::max(3, nw_);

    for (int ix = 0; ix < nx; ++ix) {
      const double fx = double(ix) / double(nx - 1);
      const double vx = vx_lo + fx * (vx_hi - vx_lo);

      for (int iy = 0; iy < ny; ++iy) {
        const double fy = double(iy) / double(ny - 1);
        const double vy = vy_lo + fy * (vy_hi - vy_lo);

        // cap body speed
        if (std::hypot(vx, vy) > v_max_) continue;

        for (int iw = 0; iw < nw; ++iw) {
          const double fw = double(iw) / double(nw - 1);
          const double wz = wz_lo + fw * (wz_hi - wz_lo);

          Candidate c = rollout_and_score(vx, vy, wz, x0, y0, yaw0, gx, gy, d_goal0, collision_radius);
          if (!c.valid) continue;
          if (c.cost < best.cost) best = c;
        }
      }
    }
    return best;
  }

  Candidate rollout_and_score(double vx_b, double vy_b, double wz,
                              double x0, double y0, double yaw0,
                              double gx, double gy, double d_goal0,
                              double collision_radius) const {
    Candidate c;
    c.vx_b = vx_b; c.vy_b = vy_b; c.wz = wz;

    double x = x0, y = y0, yaw = yaw0;
    double min_clear = std::numeric_limits<double>::infinity();
    const double safe2 = collision_radius * collision_radius;

    const int steps = std::max(1, int(std::ceil(horizon_ / sim_dt_)));

    // integrate forward
    for (int k = 0; k < steps; ++k) {
      yaw = wrap_pi(yaw + wz * sim_dt_);

      double vx_w, vy_w;
      rot_body_to_world(yaw, vx_b, vy_b, vx_w, vy_w);

      x += vx_w * sim_dt_;
      y += vy_w * sim_dt_;

      // collision check vs obstacle points
      for (const auto &ob : obstacles_) {
        const double dx = x - ob.x;
        const double dy = y - ob.y;
        const double d2 = dx*dx + dy*dy;
        if (d2 <= safe2) {
          c.valid = false;
          return c;
        }
        min_clear = std::min(min_clear, std::sqrt(d2));
      }
    }

    c.end_x = x; c.end_y = y; c.end_yaw = yaw;
    c.min_clear = std::isfinite(min_clear) ? min_clear : max_use_range_;

    // Hard safety validity: reject any candidate whose predicted clearance is below safety envelope.
    const double hard_clear = collision_radius_ + hard_clearance_margin_;
    if (c.min_clear < hard_clear) {
      c.valid = false;
      return c;
    }

    // ---- Costs (normalized-ish) ----
    // 1) Goal alignment: prefer displacement direction aligned with goal ray.
    const double gdx0 = gx - x0;
    const double gdy0 = gy - y0;
    const double gnorm = std::hypot(gdx0, gdy0);
    const double mdx = x - x0;
    const double mdy = y - y0;
    const double mnorm = std::hypot(mdx, mdy);
    const double align =
        (mdx * gdx0 + mdy * gdy0) / (std::max(1e-3, gnorm) * std::max(1e-3, mnorm));
    const double cost_goal_heading = clamp((1.0 - align) * 0.5, 0.0, 1.0);

    // 2) Progress: reduce distance-to-goal (must not go backwards too much)
    const double d1 = std::hypot(gx - x, gy - y);
    const double progress = (d_goal0 - d1);
    // Soft-penalize backward progress instead of hard reject to avoid empty-search lockups.
    const double cost_progress = clamp(1.0 - (progress / std::max(0.5, v_max_ * horizon_)), 0.0, 3.0);

    // 3) Clearance: penalize being close
    // knee near safety radius; fades out by ~ (safety + 2m)
    const double knee = safety_radius_ + 0.20;
    const double fade = knee + 2.0;
    double cost_clear = 0.0;
    if (c.min_clear <= knee) cost_clear = 1.0;
    else cost_clear = clamp((fade - c.min_clear) / std::max(1e-3, fade - knee), 0.0, 1.0);

    // 4) Speed: prefer higher body speed
    const double sp = std::hypot(vx_b, vy_b);
    const double cost_speed = 1.0 - clamp(sp / std::max(1e-3, v_max_), 0.0, 1.0);

    // 5) Smoothness: penalize switching from last cmd
    const double dv = std::hypot(vx_b - vx_cmd_b_, vy_b - vy_cmd_b_) / std::max(1e-3, v_max_);
    const double dw = std::fabs(wz - wz_cmd_) / std::max(1e-3, w_max_);
    const double cost_smooth = clamp(0.7 * dv + 0.3 * dw, 0.0, 1.5);

    // 6) Explicit anti-spin: keep yaw rate low unless truly needed.
    const double cost_yaw_rate = std::fabs(wz) / std::max(1e-3, w_max_);

    // 7) Anti-orbit: near an obstacle, penalize tangential circling motion.
    double cost_orbit = 0.0;
    if (enable_anti_orbit_cost_ && d_goal0 < std::max(0.5, anti_orbit_goal_gate_)) {
      double ox = 0.0, oy = 0.0, d_obs0 = max_use_range_;
      if (nearest_obstacle_point(x0, y0, ox, oy, d_obs0) &&
          d_obs0 < std::max(collision_radius_ + 0.05, anti_orbit_near_dist_)) {
        const double rx = (x0 - ox) / std::max(1e-3, d_obs0);
        const double ry = (y0 - oy) / std::max(1e-3, d_obs0);
        double vx_w0 = 0.0, vy_w0 = 0.0;
        rot_body_to_world(yaw0, vx_b, vy_b, vx_w0, vy_w0);
        const double sp_w = std::hypot(vx_w0, vy_w0);
        if (sp_w > 1e-3) {
          const double radial = (vx_w0 * rx + vy_w0 * ry) / sp_w;             // +outward, -inward
          const double tangential = std::fabs(vx_w0 * ry - vy_w0 * rx) / sp_w; // [0..1]
          double orbit = tangential;
          if (radial > 0.0) orbit *= (1.0 - 0.7 * clamp(radial, 0.0, 1.0));
          if (radial < 0.0) orbit += 0.6 * std::fabs(radial);
          if (progress < 0.0) orbit += 0.4;
          cost_orbit = clamp(orbit, 0.0, 2.0);
        }
      }
    }

    c.cost =
        w_goal_heading_ * cost_goal_heading +
        w_progress_     * cost_progress +
        w_clearance_    * cost_clear +
        w_speed_        * cost_speed +
        w_smooth_       * cost_smooth +
        w_yaw_rate_     * cost_yaw_rate +
        w_anti_orbit_   * cost_orbit;

    if (progress < 0.0) {
      c.cost += 1.2 * std::fabs(progress);
    }

    c.valid = true;
    return c;
  }

  double predict_min_clearance(double vx_b,
                               double vy_b,
                               double wz,
                               double x0,
                               double y0,
                               double yaw0) const {
    double x = x0;
    double y = y0;
    double yaw = yaw0;
    double min_clear = nearest_obstacle_distance(x0, y0);
    const int steps = std::max(1, static_cast<int>(std::ceil(horizon_ / sim_dt_)));

    for (int k = 0; k < steps; ++k) {
      yaw = wrap_pi(yaw + wz * sim_dt_);
      double vx_w = 0.0, vy_w = 0.0;
      rot_body_to_world(yaw, vx_b, vy_b, vx_w, vy_w);
      x += vx_w * sim_dt_;
      y += vy_w * sim_dt_;
      min_clear = std::min(min_clear, nearest_obstacle_distance(x, y));
    }

    return std::isfinite(min_clear) ? min_clear : max_use_range_;
  }

  double nearest_obstacle_distance(double x, double y) const {
    if (obstacles_.empty()) return max_use_range_;
    double min_d = std::numeric_limits<double>::infinity();
    for (const auto &ob : obstacles_) {
      const double dx = x - ob.x;
      const double dy = y - ob.y;
      min_d = std::min(min_d, std::sqrt(dx * dx + dy * dy));
    }
    return std::isfinite(min_d) ? min_d : max_use_range_;
  }

  ConeClearance cone_clearance_info(const sensor_msgs::msg::LaserScan &scan,
                                    double center_ang,
                                    double half_cone_deg) const {
    const double half = half_cone_deg * M_PI / 180.0;
    double ang = scan.angle_min;
    double d = max_use_range_;
    size_t count = 0;

    for (float rf : scan.ranges) {
      if (std::fabs(wrap_pi(ang - center_ang)) <= half) {
        count++;
        double r = std::isfinite(rf) ? static_cast<double>(rf) : max_use_range_;
        r = clamp(r, scan.range_min, std::min<double>(scan.range_max, max_use_range_));
        d = std::min(d, r);
      }
      ang += scan.angle_increment;
    }

    if (count == 0) {
      return {0.0, false};
    }
    return {d, true};
  }

  void run_wall_follow_fallback(const sensor_msgs::msg::LaserScan &scan,
                                double x0, double y0, double yaw0,
                                double gx, double gy, double ez) {
    const ConeClearance front = cone_clearance_info(scan, 0.0, 20.0);
    const ConeClearance left = cone_clearance_info(scan, M_PI_2, 30.0);
    const ConeClearance right = cone_clearance_info(scan, -M_PI_2, 30.0);

    const double ex = gx - x0;
    const double ey = gy - y0;
    const double c = std::cos(yaw0);
    const double s = std::sin(yaw0);
    const double goal_by = -s * ex + c * ey;

    const double left_score = left.covered ? left.dist : 0.0;
    const double right_score = right.covered ? right.dist : 0.0;
    const int side_sign = select_recovery_side(left_score, right_score, goal_by, now());

    const double best_side = std::max(left_score, right_score);
    const bool side_ok = best_side > (collision_radius_ + fallback_side_margin_);

    double vx_b = 0.0;
    double vy_b = 0.0;
    double wz = 0.0;

    if (side_ok) {
      const double side_scale =
          clamp((best_side - (collision_radius_ + fallback_side_margin_)) / 0.9, 0.0, 1.0);
      vy_b = static_cast<double>(side_sign) * fallback_strafe_speed_ * (0.35 + 0.65 * side_scale);

      const double front_dist = front.covered ? front.dist : 0.0;
      const double front_scale =
          clamp((front_dist - (collision_radius_ + fallback_front_margin_)) / 1.2, 0.0, 1.0);
      vx_b = fallback_forward_speed_ * front_scale;
    } else {
      // Boxed in: avoid blind backward motion; only small yaw to re-open free side.
      if (left.covered || right.covered) {
        if (steer_first_mode_) {
          vy_b = static_cast<double>(side_sign) * std::max(0.10, 0.60 * fallback_strafe_speed_);
          wz = 0.0;
        } else {
          wz = static_cast<double>(side_sign) * fallback_yaw_rate_;
        }
      }
    }

    const double pred_clear = predict_min_clearance(vx_b, vy_b, wz, x0, y0, yaw0);
    const double hard_gate = collision_radius_ + final_cmd_clearance_margin_;
    if (pred_clear < hard_gate) {
      // Never freeze here; always issue a peel-off move to break local trap.
      double vx_alt = 0.0;
      double vy_alt = static_cast<double>(side_sign) * std::max(0.10, 0.65 * fallback_strafe_speed_);
      double wz_alt = 0.0;
      apply_deorbit_push(x0, y0, yaw0, 0.85, vx_alt, vy_alt);
      cap_xy_speed(vx_alt, vy_alt, std::max(0.12, deorbit_max_xy_speed_));
      const double pred_alt = predict_min_clearance(vx_alt, vy_alt, wz_alt, x0, y0, yaw0);
      if (pred_alt >= collision_radius_ + 0.02) {
        vx_b = vx_alt;
        vy_b = vy_alt;
        wz = wz_alt;
      } else {
        vx_b = 0.0;
        vy_b = static_cast<double>(side_sign) * std::max(0.08, 0.50 * fallback_strafe_speed_);
        wz = steer_first_mode_ ? 0.0 : static_cast<double>(side_sign) * std::min(0.25, w_max_);
      }
    }

    const double vz = clamp(0.8 * ez, -vz_max_, vz_max_);
    publish_cmd(vx_b, vy_b, vz, wz);
    publish_rollout_path(x0, y0, yaw0, vx_b, vy_b, wz);
    publish_debug_state("FALLBACK",
                        std::hypot(gx - x0, gy - y0), front.dist, left.dist, right.dist,
                        pred_clear, std::numeric_limits<double>::infinity(), vx_b, vy_b, wz);

    const double min_clear_now = nearest_obstacle_distance(x0, y0);
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "No valid DWA candidate -> wall-follow fallback. side=%s front=%.2f left=%.2f right=%.2f min_clear=%.2f",
        (side_sign > 0) ? "left" : "right", front.dist, left.dist, right.dist, min_clear_now);
  }

  int select_recovery_side(double left_score,
                           double right_score,
                           double goal_by,
                           const rclcpp::Time &tnow) {
    int desired_sign = (left_score >= right_score) ? 1 : -1;
    if (std::fabs(left_score - right_score) < 0.25) {
      desired_sign = (goal_by >= 0.0) ? 1 : -1;
    }

    if (!recovery_sign_inited_) {
      recovery_sign_ = desired_sign;
      recovery_sign_inited_ = true;
      last_recovery_flip_time_ = tnow;
      return recovery_sign_;
    }

    if (desired_sign == recovery_sign_) return recovery_sign_;

    const bool lock_elapsed =
        (tnow - last_recovery_flip_time_).seconds() >= std::max(0.2, recovery_side_lock_sec_);
    const bool strong_better =
        std::fabs(left_score - right_score) > std::max(0.0, recovery_side_hysteresis_);

    if (lock_elapsed && strong_better) {
      recovery_sign_ = desired_sign;
      last_recovery_flip_time_ = tnow;
    }

    return recovery_sign_;
  }

  double apply_deorbit_push(double x0,
                            double y0,
                            double yaw0,
                            double gain,
                            double &vx_b,
                            double &vy_b) const {
    double ox = 0.0, oy = 0.0, d = max_use_range_;
    if (!nearest_obstacle_point(x0, y0, ox, oy, d)) return max_use_range_;

    const double trigger = std::max(collision_radius_ + 0.10, deorbit_trigger_dist_);
    if (d >= trigger) return d;

    const double ux_w = (x0 - ox) / std::max(1e-6, d);
    const double uy_w = (y0 - oy) / std::max(1e-6, d);

    const double c = std::cos(yaw0);
    const double s = std::sin(yaw0);
    const double ux_b = c * ux_w + s * uy_w;
    const double uy_b = -s * ux_w + c * uy_w;

    const double ratio = clamp((trigger - d) / std::max(1e-3, trigger - collision_radius_), 0.0, 1.0);
    const double k = std::max(0.0, deorbit_outward_speed_) * ratio * std::max(0.0, gain);
    vx_b += k * ux_b;
    vy_b += k * uy_b;
    return d;
  }

  bool nearest_obstacle_point(double x,
                              double y,
                              double &ox,
                              double &oy,
                              double &dist) const {
    if (obstacles_.empty()) return false;
    double best_d2 = std::numeric_limits<double>::infinity();
    bool ok = false;
    for (const auto &ob : obstacles_) {
      const double dx = x - ob.x;
      const double dy = y - ob.y;
      const double d2 = dx * dx + dy * dy;
      if (d2 < best_d2) {
        best_d2 = d2;
        ox = ob.x;
        oy = ob.y;
        ok = true;
      }
    }
    if (!ok) return false;
    dist = std::sqrt(best_d2);
    return true;
  }

  bool is_goal_occupied_in_map(double gx,
                               double gy,
                               double radius,
                               int *worst_occ) const {
    if (!map_.has_value()) return false;
    const auto &map = map_.value();
    if (map.info.resolution <= 1e-6 || map.info.width == 0 || map.info.height == 0 ||
        map.data.empty()) {
      return false;
    }

    const double res = static_cast<double>(map.info.resolution);
    const double ox = map.info.origin.position.x;
    const double oy = map.info.origin.position.y;

    const auto world_to_cell = [&](double wx, double wy, int &mx, int &my) -> bool {
      mx = static_cast<int>(std::floor((wx - ox) / res));
      my = static_cast<int>(std::floor((wy - oy) / res));
      return (mx >= 0 && my >= 0 && mx < static_cast<int>(map.info.width) &&
              my < static_cast<int>(map.info.height));
    };

    int cx = 0, cy = 0;
    if (!world_to_cell(gx, gy, cx, cy)) {
      if (worst_occ) *worst_occ = -1;
      return false;
    }

    const int r_cells = std::max(0, static_cast<int>(std::ceil(radius / res)));
    int worst = -1;
    for (int dy = -r_cells; dy <= r_cells; ++dy) {
      for (int dx = -r_cells; dx <= r_cells; ++dx) {
        const int mx = cx + dx;
        const int my = cy + dy;
        if (mx < 0 || my < 0 || mx >= static_cast<int>(map.info.width) ||
            my >= static_cast<int>(map.info.height)) {
          continue;
        }

        const double wx = ox + (static_cast<double>(mx) + 0.5) * res;
        const double wy = oy + (static_cast<double>(my) + 0.5) * res;
        if (std::hypot(wx - gx, wy - gy) > radius) continue;

        const int idx = my * static_cast<int>(map.info.width) + mx;
        const int occ = static_cast<int>(map.data[static_cast<size_t>(idx)]);
        worst = std::max(worst, occ);

        if (occ < 0) {
          if (map_unknown_as_occupied_) {
            if (worst_occ) *worst_occ = -1;
            return true;
          }
          continue;
        }
        if (occ >= std::max(0, map_occ_threshold_)) {
          if (worst_occ) *worst_occ = occ;
          return true;
        }
      }
    }

    if (worst_occ) *worst_occ = worst;
    return false;
  }

  void run_bypass_assist(const sensor_msgs::msg::LaserScan &scan,
                         double x0, double y0, double yaw0,
                         double gx, double gy, double ez,
                         double goal_by,
                         const ConeClearance &front,
                         const ConeClearance &left,
                         const ConeClearance &right) {
    (void)scan;
    const double ex = gx - x0;
    const double ey = gy - y0;
    const double d_goal = std::hypot(ex, ey);
    const double cb = std::cos(yaw0);
    const double sb = std::sin(yaw0);
    const double goal_bx = cb * ex + sb * ey;

    const double left_score = left.covered ? left.dist : 0.0;
    const double right_score = right.covered ? right.dist : 0.0;

    int desired_sign = (left_score >= right_score) ? 1 : -1;
    if (std::fabs(left_score - right_score) < 0.25) {
      desired_sign = (goal_by >= 0.0) ? 1 : -1;
    }

    const rclcpp::Time tnow = now();
    if (!bypass_sign_inited_) {
      bypass_sign_ = desired_sign;
      bypass_sign_inited_ = true;
      last_bypass_flip_time_ = tnow;
    } else {
      const bool lock_elapsed =
          (tnow - last_bypass_flip_time_).seconds() >= std::max(0.2, bypass_lock_sec_);
      const bool strong = std::fabs(left_score - right_score) > std::max(0.0, bypass_turn_hysteresis_);
      if (desired_sign != bypass_sign_ && lock_elapsed && strong) {
        bypass_sign_ = desired_sign;
        last_bypass_flip_time_ = tnow;
      }
    }
    const int side_sign = bypass_sign_;

    const double side_clear = (side_sign > 0) ? left_score : right_score;
    const double side_scale =
        clamp((side_clear - std::max(0.2, bypass_side_open_dist_)) / 1.0, 0.0, 1.0);

    double vy_b = static_cast<double>(side_sign) * bypass_strafe_speed_ * (0.45 + 0.55 * side_scale);
    const double front_scale =
        clamp((front.dist - collision_radius_) / std::max(0.2, bypass_front_trigger_dist_), 0.0, 1.0);
    double vx_b = bypass_forward_speed_ * front_scale;

    // Keep BYPASS goal-directed to avoid orbiting near local minima.
    if (goal_bx > 0.05) {
      const double goal_pull = clamp(goal_bx / std::max(0.5, d_goal), 0.0, 1.0);
      const double vx_goal_floor = bypass_forward_speed_ *
                                   (0.35 + std::max(0.0, bypass_goal_push_gain_) * goal_pull);
      vx_b = std::max(vx_b, std::min(v_max_, vx_goal_floor));
    }

    const bool near_goal = d_goal < std::max(1.0, bypass_near_goal_dist_);
    const bool front_has_room = front.covered && front.dist > (collision_radius_ + 0.35);
    if (near_goal && front_has_room) {
      vy_b *= clamp(bypass_near_goal_strafe_scale_, 0.1, 1.0);
      vx_b = std::max(vx_b, std::min(v_max_, 0.12 + 0.25 * front_scale));
    }

    double wz = static_cast<double>(side_sign) * std::min(std::fabs(bypass_yaw_rate_), w_max_);
    if (steer_first_mode_) {
      wz = static_cast<double>(side_sign) * std::min(std::max(0.0, steer_first_yaw_cap_), w_max_);
    }

    // Anti-pole-hug: when too close to the nearest obstacle, bias command outward (radially away).
    double ox = 0.0, oy = 0.0, d_near = max_use_range_;
    if (nearest_obstacle_point(x0, y0, ox, oy, d_near) &&
        d_near < std::max(collision_radius_ + 0.05, bypass_orbit_escape_dist_)) {
      const double ux_w = (x0 - ox) / std::max(1e-3, d_near);
      const double uy_w = (y0 - oy) / std::max(1e-3, d_near);
      const double cb = std::cos(yaw0);
      const double sb = std::sin(yaw0);
      const double ux_b = cb * ux_w + sb * uy_w;
      const double uy_b = -sb * ux_w + cb * uy_w;
      const double esc = std::max(0.10, bypass_outward_push_speed_);
      const double blend = clamp((std::max(collision_radius_ + 0.05, bypass_orbit_escape_dist_) - d_near) /
                                     std::max(1e-3, bypass_orbit_escape_dist_ - collision_radius_),
                                 0.0, 1.0);
      vx_b = (1.0 - 0.75 * blend) * vx_b + (0.75 * blend) * esc * ux_b;
      vy_b = (1.0 - 0.75 * blend) * vy_b + (0.75 * blend) * esc * uy_b;
      wz *= clamp(bypass_near_obs_wz_scale_, 0.0, 1.0);
    }

    const double pred_clear = predict_min_clearance(vx_b, vy_b, wz, x0, y0, yaw0);
    const double critical_gate = collision_radius_ + std::max(0.03, hard_clearance_margin_);
    if (pred_clear < critical_gate) {
      vx_b = 0.0;
      vy_b = static_cast<double>(side_sign) * std::max(0.10, 0.55 * bypass_strafe_speed_);
      wz = 0.0;
    }
    cap_xy_speed(vx_b, vy_b, v_max_);

    const double vz = clamp(0.8 * ez, -vz_max_, vz_max_);
    publish_cmd(vx_b, vy_b, vz, wz);
    publish_rollout_path(x0, y0, yaw0, vx_b, vy_b, wz);
    publish_debug_state("BYPASS",
                        std::hypot(gx - x0, gy - y0), front.dist, left.dist, right.dist,
                        pred_clear, std::numeric_limits<double>::infinity(), vx_b, vy_b, wz);

    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "DWA bypass-assist: side=%s front=%.2f left=%.2f right=%.2f cmd(vx=%.2f vy=%.2f wz=%.2f)",
        (side_sign > 0) ? "left" : "right", front.dist, left.dist, right.dist, vx_b, vy_b, wz);
  }

  void publish_debug_state(const std::string &mode,
                           double d_goal,
                           double front,
                           double left,
                           double right,
                           double pred_clear,
                           double cost,
                           double vx_b,
                           double vy_b,
                           double wz) {
    if (!publish_debug_state_ || !pub_state_) return;

    const double hz = std::max(0.5, debug_state_hz_);
    const double dt = (now() - last_state_pub_time_).seconds();
    if (dt < (1.0 / hz)) return;
    last_state_pub_time_ = now();

    std_msgs::msg::String msg;
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2)
       << "mode=" << mode
       << " d_goal=" << d_goal
       << " front=" << front
       << " left=" << left
       << " right=" << right
       << " pred_clear=" << pred_clear
       << " cost=" << cost
       << " goal_occ=" << (goal_occupied_cached_ ? 1 : 0)
       << " goal_occ_val=" << goal_occ_value_cached_
       << " map_ready=" << (map_ready_cached_ ? 1 : 0)
       << " cmd(vx=" << vx_b << " vy=" << vy_b << " wz=" << wz << ")";
    msg.data = ss.str();
    pub_state_->publish(msg);
  }

  void publish_cmd(double vx_b, double vy_b, double vz, double wz) {
    // store cmd state (used for window + smoothing)
    vx_cmd_b_ = vx_b;
    vy_cmd_b_ = vy_b;
    wz_cmd_   = wz;
    cmd_inited_ = true;
    last_cmd_time_ = now();

    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = last_cmd_time_;

    // Publish in WORLD/ENU (recommended for MAVROS local velocity setpoints),
    // but planned variables are in body frame, so rotate by current yaw from odom.
    double vx_out = vx_b, vy_out = vy_b;
    if (publish_world_cmd_ && odom_) {
      const double yaw = yaw_from_odom(odom_.value());
      rot_body_to_world(yaw, vx_b, vy_b, vx_out, vy_out);
      msg.header.frame_id = "map";
    } else {
      msg.header.frame_id = "base_link";
    }

    msg.twist.linear.x  = clamp(vx_out, -v_max_, v_max_);
    msg.twist.linear.y  = clamp(vy_out, -v_max_, v_max_);
    msg.twist.linear.z  = clamp(vz, -vz_max_, vz_max_);
    msg.twist.angular.z = clamp(wz, -w_max_, w_max_);

    pub_cmd_->publish(msg);
  }

  void publish_rollout_path(double x0, double y0, double yaw0,
                            double vx_b, double vy_b, double wz) {
    if (!publish_rollout_path_ || !pub_rollout_path_) return;

    nav_msgs::msg::Path path;
    path.header.stamp = now();
    const bool use_base_link_frame =
        (rollout_path_frame_ == "base_link") || (rollout_path_frame_ == "link");
    path.header.frame_id = use_base_link_frame ? "base_link" : "map";

    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.orientation.w = 1.0;
    pose.pose.position.x = use_base_link_frame ? 0.0 : x0;
    pose.pose.position.y = use_base_link_frame ? 0.0 : y0;
    pose.pose.position.z = 0.0;
    path.poses.push_back(pose);

    const int steps = std::max(1, static_cast<int>(std::ceil(horizon_ / sim_dt_)));
    double x = pose.pose.position.x;
    double y = pose.pose.position.y;
    double yaw = yaw0;
    for (int k = 0; k < steps; ++k) {
      if (use_base_link_frame) {
        x += vx_b * sim_dt_;
        y += vy_b * sim_dt_;
      } else {
        yaw = wrap_pi(yaw + wz * sim_dt_);
        double vx_w = 0.0, vy_w = 0.0;
        rot_body_to_world(yaw, vx_b, vy_b, vx_w, vy_w);
        x += vx_w * sim_dt_;
        y += vy_w * sim_dt_;
      }
      pose.pose.position.x = x;
      pose.pose.position.y = y;
      path.poses.push_back(pose);
    }

    pub_rollout_path_->publish(path);
  }

private:
  // params
  double v_max_{0.75}, vy_max_{0.55}, w_max_{0.45}, vz_max_{1.0};
  double ax_max_{1.0}, ay_max_{1.0}, aw_max_{2.2};
  double control_dt_{0.05}, sim_dt_{0.10}, horizon_{2.4};
  int nx_{9}, ny_{7}, nw_{11};
  double max_use_range_{10.0}, safety_radius_{0.65}, collision_radius_{0.32}, obs_range_{12.0};
  double collision_from_safety_scale_{0.85};
  double hard_clearance_margin_{0.05};
  double final_cmd_clearance_margin_{0.08};
  double fallback_strafe_speed_{0.32}, fallback_forward_speed_{0.18}, fallback_yaw_rate_{0.45};
  double fallback_front_margin_{0.30}, fallback_side_margin_{0.00};
  bool enable_bypass_assist_{true};
  double bypass_front_trigger_dist_{1.8};
  double bypass_side_open_dist_{1.0};
  double bypass_strafe_speed_{0.55};
  double bypass_forward_speed_{0.24};
  double bypass_yaw_rate_{0.35};
  double bypass_lock_sec_{1.0};
  double bypass_turn_hysteresis_{0.60};
  double bypass_max_continuous_sec_{6.0};
  double bypass_cooldown_sec_{2.5};
  double bypass_disable_near_goal_dist_{2.5};
  double bypass_orbit_escape_dist_{1.8};
  double bypass_outward_push_speed_{0.45};
  double bypass_near_obs_wz_scale_{0.35};
  double bypass_goal_push_gain_{2.4};
  double bypass_near_goal_dist_{4.5};
  double bypass_near_goal_strafe_scale_{0.40};
  bool enable_sharp_turn_assist_{true};
  double sharp_turn_trigger_dist_{2.0};
  double sharp_turn_yaw_boost_{1.6};
  double sharp_turn_min_fwd_scale_{0.25};
  double sharp_turn_side_speed_{0.28};
  double final_gate_turn_rate_{0.60};
  double final_gate_strafe_speed_{0.22};
  double recovery_side_lock_sec_{1.20};
  double recovery_side_hysteresis_{0.40};
  double deorbit_trigger_dist_{1.8};
  double deorbit_outward_speed_{0.30};
  double deorbit_max_xy_speed_{0.42};
  double deorbit_min_turn_scale_{0.25};
  bool steer_first_mode_{true};
  double steer_first_yaw_cap_{0.10};
  double steer_first_forward_speed_{0.12};
  bool publish_debug_state_{true};
  double debug_state_hz_{8.0};
  bool use_map_goal_occupancy_{true};
  std::string map_topic_{"/map"};
  int map_occ_threshold_{50};
  bool map_unknown_as_occupied_{false};
  double map_goal_inflate_radius_{0.60};
  bool map_goal_force_hold_{false};
  bool enable_goal_obstacle_offset_{true};
  double goal_obstacle_near_dist_{0.85};
  double goal_obstacle_target_clearance_{1.05};
  double goal_obstacle_max_offset_{0.90};
  int scan_stride_{2};
  double w_goal_heading_{1.0}, w_progress_{2.5}, w_clearance_{1.2}, w_speed_{0.2}, w_smooth_{0.3};
  double w_yaw_rate_{0.25};
  double w_anti_orbit_{1.4};
  bool enable_anti_orbit_cost_{true};
  double anti_orbit_near_dist_{2.2};
  double anti_orbit_goal_gate_{8.0};
  bool full_search_if_empty_{true};
  double full_search_w_max_{0.70};
  double full_search_collision_scale_{0.70};
  bool face_goal_to_setpoint_{true};
  double face_goal_k_yaw_{0.9};
  double face_goal_deadband_deg_{3.0};
  double face_goal_turn_only_deg_{45.0};
  double face_goal_mix_with_dwa_{0.45};
  double face_goal_min_xy_scale_{0.20};
  bool enable_min_cruise_kick_{true};
  double min_cruise_goal_dist_{2.5};
  double min_cruise_speed_{0.42};
  double min_cruise_front_margin_{0.45};
  double min_cruise_blend_{0.85};
  bool publish_world_cmd_{true};
  bool publish_rollout_path_{true};
  std::string rollout_path_frame_{"base_link"};

  // state
  std::optional<nav_msgs::msg::Odometry> odom_;
  std::optional<sensor_msgs::msg::LaserScan> scan_;
  std::optional<geometry_msgs::msg::Point> goal_;
  std::optional<nav_msgs::msg::OccupancyGrid> map_;

  std::vector<ObPoint> obstacles_;

  bool cmd_inited_{false};
  double vx_cmd_b_{0.0}, vy_cmd_b_{0.0}, wz_cmd_{0.0};
  rclcpp::Time last_cmd_time_;
  bool bypass_sign_inited_{false};
  int bypass_sign_{1};
  rclcpp::Time last_bypass_flip_time_{0, 0, RCL_ROS_TIME};
  bool bypass_mode_active_{false};
  rclcpp::Time bypass_mode_start_{0, 0, RCL_ROS_TIME};
  rclcpp::Time bypass_cooldown_until_{0, 0, RCL_ROS_TIME};
  bool recovery_sign_inited_{false};
  int recovery_sign_{1};
  rclcpp::Time last_recovery_flip_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_state_pub_time_;
  bool goal_occupied_cached_{false};
  int goal_occ_value_cached_{-1};
  bool map_ready_cached_{false};

  // ros
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_goal_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_cmd_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_rollout_path_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_state_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DwaLocalPlanner>());
  rclcpp::shutdown();
  return 0;
}
