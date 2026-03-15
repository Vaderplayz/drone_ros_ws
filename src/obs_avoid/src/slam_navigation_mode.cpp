#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace {
inline double clamp(double x, double lo, double hi) { return std::max(lo, std::min(hi, x)); }

double yaw_from_quat(const geometry_msgs::msg::Quaternion &q) {
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}
}  // namespace

class SlamNavigationMode : public rclcpp::Node {
 public:
  SlamNavigationMode() : Node("slam_navigation_mode") {
    map_topic_ = declare_parameter<std::string>("map_topic", "/map");
    pose_topic_ = declare_parameter<std::string>("pose_topic", "/mavros/local_position/pose");
    raw_goal_topic_ = declare_parameter<std::string>("raw_goal_topic", "/mission_goal");
    nav_goal_topic_ = declare_parameter<std::string>("nav_goal_topic", "/drone_goal");
    path_topic_ = declare_parameter<std::string>("path_topic", "/slam_nav/global_path");

    planner_hz_ = declare_parameter<double>("planner_hz", 4.0);
    replan_min_period_sec_ = declare_parameter<double>("replan_min_period_sec", 0.6);
    lookahead_distance_m_ = declare_parameter<double>("lookahead_distance_m", 1.8);
    max_nav_goal_distance_m_ = declare_parameter<double>("max_nav_goal_distance_m", 6.0);
    goal_reached_radius_m_ = declare_parameter<double>("goal_reached_radius_m", 0.65);
    map_occ_threshold_ = declare_parameter<int>("map_occ_threshold", 50);
    map_unknown_as_occupied_ = declare_parameter<bool>("map_unknown_as_occupied", false);
    inflate_radius_m_ = declare_parameter<double>("inflate_radius_m", 0.40);
    start_search_radius_m_ = declare_parameter<double>("start_search_radius_m", 1.5);
    goal_search_radius_m_ = declare_parameter<double>("goal_search_radius_m", 2.5);
    publish_passthrough_without_map_ =
        declare_parameter<bool>("publish_passthrough_without_map", true);
    clamp_out_of_bounds_start_ = declare_parameter<bool>("clamp_out_of_bounds_start", true);
    clamp_out_of_bounds_goal_ = declare_parameter<bool>("clamp_out_of_bounds_goal", true);
    clamp_border_margin_cells_ = declare_parameter<int>("clamp_border_margin_cells", 2);

    auto qos_sensor = rclcpp::SensorDataQoS();
    auto qos_map = rclcpp::QoS(1).reliable().transient_local();
    auto qos_goal = rclcpp::QoS(10).reliable().transient_local();
    auto qos_path = rclcpp::QoS(10).reliable();

    sub_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        pose_topic_, qos_sensor,
        std::bind(&SlamNavigationMode::pose_cb, this, std::placeholders::_1));
    sub_map_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        map_topic_, qos_map, std::bind(&SlamNavigationMode::map_cb, this, std::placeholders::_1));
    sub_raw_goal_ = create_subscription<geometry_msgs::msg::Point>(
        raw_goal_topic_, qos_goal,
        std::bind(&SlamNavigationMode::raw_goal_cb, this, std::placeholders::_1));

    pub_nav_goal_ = create_publisher<geometry_msgs::msg::Point>(nav_goal_topic_, qos_goal);
    pub_path_ = create_publisher<nav_msgs::msg::Path>(path_topic_, qos_path);

    const auto period =
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(
            1.0 / std::max(0.5, planner_hz_)));
    timer_ = create_wall_timer(period, std::bind(&SlamNavigationMode::on_timer, this));

    RCLCPP_INFO(get_logger(),
                "slam_navigation_mode started: map=%s pose=%s raw_goal=%s nav_goal=%s",
                map_topic_.c_str(), pose_topic_.c_str(), raw_goal_topic_.c_str(),
                nav_goal_topic_.c_str());
  }

 private:
  struct Cell {
    int x{0};
    int y{0};
  };

  struct OpenNode {
    double f{0.0};
    int idx{0};
    bool operator>(const OpenNode &o) const { return f > o.f; }
  };

  void pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    pose_ = *msg;
    have_pose_ = true;
  }

  void raw_goal_cb(const geometry_msgs::msg::Point::SharedPtr msg) {
    raw_goal_ = *msg;
    have_raw_goal_ = true;
    ++goal_seq_;
  }

  void map_cb(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    map_ = *msg;
    have_map_ = true;
    ++map_seq_;
    rebuild_blocked_grid();
  }

  void on_timer() {
    if (!have_pose_ || !have_raw_goal_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Waiting for pose/raw_goal for SLAM navigation...");
      return;
    }

    const auto &p = pose_.pose.position;
    const auto &g = raw_goal_;
    const double d_goal = std::hypot(g.x - p.x, g.y - p.y);

    if (d_goal < std::max(0.2, goal_reached_radius_m_)) {
      publish_nav_goal(g);
      publish_path({});
      return;
    }

    if (!have_map_ || blocked_.empty()) {
      if (publish_passthrough_without_map_) {
        publish_nav_goal(g);
      }
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                           "No usable /map yet, passthrough raw mission goal.");
      return;
    }

    Cell start{};
    Cell goal{};
    const bool start_in = world_to_cell(p.x, p.y, start.x, start.y);
    const bool goal_in = world_to_cell(g.x, g.y, goal.x, goal.y);
    bool clamped_start = false;
    bool clamped_goal = false;

    if (!start_in) {
      if (!clamp_out_of_bounds_start_ ||
          !world_to_cell_clamped(p.x, p.y, start, clamped_start)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Start outside current map bounds. Passthrough.");
        publish_nav_goal(g);
        return;
      }
    }
    if (!goal_in) {
      if (!clamp_out_of_bounds_goal_ ||
          !world_to_cell_clamped(g.x, g.y, goal, clamped_goal)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Goal outside current map bounds. Passthrough.");
        publish_nav_goal(g);
        return;
      }
    }
    if (clamped_start || clamped_goal) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Start/goal outside current map bounds. Using clamped in-map target.");
    }

    const int start_search_cells = meters_to_cells(start_search_radius_m_);
    const int goal_search_cells = meters_to_cells(goal_search_radius_m_);
    if (!find_nearest_free_cell(start, start_search_cells, start)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1500,
                           "No free start cell around current pose. Passthrough.");
      publish_nav_goal(g);
      return;
    }
    if (!find_nearest_free_cell(goal, goal_search_cells, goal)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1500,
                           "No free goal cell around mission goal. Passthrough.");
      publish_nav_goal(g);
      return;
    }

    const rclcpp::Time tnow = now();
    const bool need_replan = path_cells_.empty() || (goal_seq_ != planned_goal_seq_) ||
                             (map_seq_ != planned_map_seq_) ||
                             ((tnow - last_plan_time_).seconds() >=
                              std::max(0.1, replan_min_period_sec_));

    if (need_replan) {
      std::vector<Cell> planned_cells;
      if (!plan_astar(start, goal, planned_cells)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1500,
                             "A* failed to find route. Passthrough raw mission goal.");
        publish_nav_goal(g);
        return;
      }
      path_cells_ = std::move(planned_cells);
      planned_goal_seq_ = goal_seq_;
      planned_map_seq_ = map_seq_;
      last_plan_time_ = tnow;
    }

    if (path_cells_.empty()) {
      publish_nav_goal(g);
      return;
    }

    publish_path(path_cells_);

    geometry_msgs::msg::Point nav_goal = choose_lookahead_goal(path_cells_, p.x, p.y, g.z);
    if (std::hypot(g.x - p.x, g.y - p.y) < std::max(0.2, lookahead_distance_m_)) {
      nav_goal = g;
    }

    const double max_nav_dist = std::max(0.5, max_nav_goal_distance_m_);
    const double d_nav = std::hypot(nav_goal.x - p.x, nav_goal.y - p.y);
    if (d_nav > max_nav_dist) {
      const double scale = max_nav_dist / d_nav;
      nav_goal.x = p.x + (nav_goal.x - p.x) * scale;
      nav_goal.y = p.y + (nav_goal.y - p.y) * scale;
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "SLAM nav goal capped to %.2fm from pose (raw planned=%.2fm).",
          max_nav_dist, d_nav);
    }
    publish_nav_goal(nav_goal);
  }

  void rebuild_blocked_grid() {
    blocked_.clear();
    if (!have_map_) return;
    const auto &m = map_;
    const int w = static_cast<int>(m.info.width);
    const int h = static_cast<int>(m.info.height);
    if (w <= 0 || h <= 0 || m.info.resolution <= 1e-6) return;
    if (static_cast<int>(m.data.size()) != w * h) return;

    blocked_.assign(static_cast<size_t>(w * h), 0);
    const int inflate_cells = meters_to_cells(inflate_radius_m_);

    for (int y = 0; y < h; ++y) {
      for (int x = 0; x < w; ++x) {
        const int idx = y * w + x;
        if (!is_occupied_value(m.data[static_cast<size_t>(idx)])) continue;
        if (inflate_cells <= 0) {
          blocked_[static_cast<size_t>(idx)] = 1;
          continue;
        }

        for (int dy = -inflate_cells; dy <= inflate_cells; ++dy) {
          for (int dx = -inflate_cells; dx <= inflate_cells; ++dx) {
            if (dx * dx + dy * dy > inflate_cells * inflate_cells) continue;
            const int nx = x + dx;
            const int ny = y + dy;
            if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
            blocked_[static_cast<size_t>(ny * w + nx)] = 1;
          }
        }
      }
    }
  }

  bool is_occupied_value(int8_t occ) const {
    if (occ < 0) return map_unknown_as_occupied_;
    return occ >= map_occ_threshold_;
  }

  int meters_to_cells(double meters) const {
    if (!have_map_ || map_.info.resolution <= 1e-6) return 0;
    return std::max(0, static_cast<int>(std::ceil(meters / map_.info.resolution)));
  }

  bool in_bounds(int x, int y) const {
    const int w = static_cast<int>(map_.info.width);
    const int h = static_cast<int>(map_.info.height);
    return (x >= 0 && y >= 0 && x < w && y < h);
  }

  int flat(int x, int y) const { return y * static_cast<int>(map_.info.width) + x; }

  bool is_blocked(int x, int y) const {
    if (!in_bounds(x, y)) return true;
    const int idx = flat(x, y);
    if (idx < 0 || idx >= static_cast<int>(blocked_.size())) return true;
    return blocked_[static_cast<size_t>(idx)] != 0;
  }

  bool world_to_cell(double wx, double wy, int &cx, int &cy) const {
    if (!have_map_ || map_.info.resolution <= 1e-6) return false;
    const auto &o = map_.info.origin;
    const double yaw = yaw_from_quat(o.orientation);
    const double dx = wx - o.position.x;
    const double dy = wy - o.position.y;
    const double c = std::cos(-yaw);
    const double s = std::sin(-yaw);
    const double mx = c * dx - s * dy;
    const double my = s * dx + c * dy;
    cx = static_cast<int>(std::floor(mx / map_.info.resolution));
    cy = static_cast<int>(std::floor(my / map_.info.resolution));
    return in_bounds(cx, cy);
  }

  bool world_to_cell_clamped(double wx, double wy, Cell &out, bool &clamped) const {
    clamped = false;
    int cx = 0;
    int cy = 0;
    if (world_to_cell(wx, wy, cx, cy)) {
      out = Cell{cx, cy};
      return true;
    }
    if (!have_map_ || map_.info.resolution <= 1e-6) return false;

    const int w = static_cast<int>(map_.info.width);
    const int h = static_cast<int>(map_.info.height);
    if (w <= 0 || h <= 0) return false;

    const auto &o = map_.info.origin;
    const double yaw = yaw_from_quat(o.orientation);
    const double dx = wx - o.position.x;
    const double dy = wy - o.position.y;
    const double c = std::cos(-yaw);
    const double s = std::sin(-yaw);
    const double mx = c * dx - s * dy;
    const double my = s * dx + c * dy;
    const int gx = static_cast<int>(std::floor(mx / map_.info.resolution));
    const int gy = static_cast<int>(std::floor(my / map_.info.resolution));

    const int max_x = w - 1;
    const int max_y = h - 1;
    const int margin = std::max(0, clamp_border_margin_cells_);
    const int lo_x = std::min(std::max(0, margin), max_x);
    const int hi_x = std::max(lo_x, max_x - margin);
    const int lo_y = std::min(std::max(0, margin), max_y);
    const int hi_y = std::max(lo_y, max_y - margin);

    out.x = static_cast<int>(clamp(static_cast<double>(gx), static_cast<double>(lo_x),
                                   static_cast<double>(hi_x)));
    out.y = static_cast<int>(clamp(static_cast<double>(gy), static_cast<double>(lo_y),
                                   static_cast<double>(hi_y)));
    clamped = true;
    return true;
  }

  void cell_to_world(int cx, int cy, double &wx, double &wy) const {
    const auto &o = map_.info.origin;
    const double yaw = yaw_from_quat(o.orientation);
    const double mx = (static_cast<double>(cx) + 0.5) * map_.info.resolution;
    const double my = (static_cast<double>(cy) + 0.5) * map_.info.resolution;
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    wx = o.position.x + c * mx - s * my;
    wy = o.position.y + s * mx + c * my;
  }

  bool find_nearest_free_cell(const Cell &seed, int radius_cells, Cell &out) const {
    if (!have_map_) return false;
    if (in_bounds(seed.x, seed.y) && !is_blocked(seed.x, seed.y)) {
      out = seed;
      return true;
    }

    const int rmax = std::max(1, radius_cells);
    for (int r = 1; r <= rmax; ++r) {
      for (int dy = -r; dy <= r; ++dy) {
        for (int dx = -r; dx <= r; ++dx) {
          if (std::abs(dx) != r && std::abs(dy) != r) continue;
          Cell c{seed.x + dx, seed.y + dy};
          if (!in_bounds(c.x, c.y)) continue;
          if (!is_blocked(c.x, c.y)) {
            out = c;
            return true;
          }
        }
      }
    }
    return false;
  }

  bool plan_astar(const Cell &start, const Cell &goal, std::vector<Cell> &out_path) const {
    out_path.clear();
    if (!have_map_) return false;
    const int w = static_cast<int>(map_.info.width);
    const int h = static_cast<int>(map_.info.height);
    if (w <= 0 || h <= 0) return false;
    if (is_blocked(start.x, start.y) || is_blocked(goal.x, goal.y)) return false;

    const int n = w * h;
    std::vector<double> g_score(static_cast<size_t>(n), std::numeric_limits<double>::infinity());
    std::vector<int> parent(static_cast<size_t>(n), -1);
    std::vector<uint8_t> closed(static_cast<size_t>(n), 0);

    const auto heuristic = [&](int x, int y) {
      return std::hypot(static_cast<double>(goal.x - x), static_cast<double>(goal.y - y));
    };

    std::priority_queue<OpenNode, std::vector<OpenNode>, std::greater<OpenNode>> open;
    const int sidx = flat(start.x, start.y);
    const int gidx = flat(goal.x, goal.y);
    g_score[static_cast<size_t>(sidx)] = 0.0;
    open.push(OpenNode{heuristic(start.x, start.y), sidx});

    static const int dx8[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    static const int dy8[8] = {0, 0, 1, -1, 1, -1, 1, -1};
    static const double step8[8] = {1.0, 1.0, 1.0, 1.0, std::sqrt(2.0), std::sqrt(2.0),
                                    std::sqrt(2.0), std::sqrt(2.0)};

    bool found = false;
    while (!open.empty()) {
      OpenNode cur = open.top();
      open.pop();
      const int idx = cur.idx;
      if (closed[static_cast<size_t>(idx)]) continue;
      closed[static_cast<size_t>(idx)] = 1;
      if (idx == gidx) {
        found = true;
        break;
      }

      const int x = idx % w;
      const int y = idx / w;
      for (int k = 0; k < 8; ++k) {
        const int nx = x + dx8[k];
        const int ny = y + dy8[k];
        if (!in_bounds(nx, ny) || is_blocked(nx, ny)) continue;

        // Disallow cutting corners through obstacles on diagonal motion.
        if (k >= 4 && (is_blocked(x + dx8[k], y) || is_blocked(x, y + dy8[k]))) continue;

        const int nidx = flat(nx, ny);
        if (closed[static_cast<size_t>(nidx)]) continue;
        const double tg = g_score[static_cast<size_t>(idx)] + step8[k];
        if (tg >= g_score[static_cast<size_t>(nidx)]) continue;
        g_score[static_cast<size_t>(nidx)] = tg;
        parent[static_cast<size_t>(nidx)] = idx;
        const double f = tg + heuristic(nx, ny);
        open.push(OpenNode{f, nidx});
      }
    }

    if (!found) return false;

    std::vector<Cell> rev;
    int idx = gidx;
    while (idx >= 0) {
      rev.push_back(Cell{idx % w, idx / w});
      if (idx == sidx) break;
      idx = parent[static_cast<size_t>(idx)];
    }
    if (rev.empty() || rev.back().x != start.x || rev.back().y != start.y) return false;

    out_path.assign(rev.rbegin(), rev.rend());
    return !out_path.empty();
  }

  geometry_msgs::msg::Point choose_lookahead_goal(const std::vector<Cell> &path_cells,
                                                  double px,
                                                  double py,
                                                  double target_z) const {
    geometry_msgs::msg::Point out;
    out.x = px;
    out.y = py;
    out.z = target_z;
    if (path_cells.empty()) return out;

    std::vector<std::pair<double, double>> path_xy;
    path_xy.reserve(path_cells.size());
    for (const auto &c : path_cells) {
      double wx = 0.0, wy = 0.0;
      cell_to_world(c.x, c.y, wx, wy);
      path_xy.emplace_back(wx, wy);
    }

    int nearest = 0;
    double best_d2 = std::numeric_limits<double>::infinity();
    for (int i = 0; i < static_cast<int>(path_xy.size()); ++i) {
      const double dx = path_xy[static_cast<size_t>(i)].first - px;
      const double dy = path_xy[static_cast<size_t>(i)].second - py;
      const double d2 = dx * dx + dy * dy;
      if (d2 < best_d2) {
        best_d2 = d2;
        nearest = i;
      }
    }

    const double lookahead = std::max(0.2, lookahead_distance_m_);
    double acc = 0.0;
    int pick = nearest;
    for (int i = nearest; i + 1 < static_cast<int>(path_xy.size()); ++i) {
      const auto &a = path_xy[static_cast<size_t>(i)];
      const auto &b = path_xy[static_cast<size_t>(i + 1)];
      acc += std::hypot(b.first - a.first, b.second - a.second);
      pick = i + 1;
      if (acc >= lookahead) break;
    }

    out.x = path_xy[static_cast<size_t>(pick)].first;
    out.y = path_xy[static_cast<size_t>(pick)].second;
    out.z = target_z;
    return out;
  }

  void publish_nav_goal(const geometry_msgs::msg::Point &g) {
    if (!pub_nav_goal_) return;
    pub_nav_goal_->publish(g);
  }

  void publish_path(const std::vector<Cell> &cells) {
    if (!pub_path_) return;
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = "map";
    for (const auto &c : cells) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path.header;
      cell_to_world(c.x, c.y, ps.pose.position.x, ps.pose.position.y);
      ps.pose.position.z = 0.0;
      ps.pose.orientation.w = 1.0;
      path.poses.push_back(ps);
    }
    pub_path_->publish(path);
  }

  // Params
  std::string map_topic_{"/map"};
  std::string pose_topic_{"/mavros/local_position/pose"};
  std::string raw_goal_topic_{"/mission_goal"};
  std::string nav_goal_topic_{"/drone_goal"};
  std::string path_topic_{"/slam_nav/global_path"};
  double planner_hz_{4.0};
  double replan_min_period_sec_{0.6};
  double lookahead_distance_m_{1.8};
  double max_nav_goal_distance_m_{6.0};
  double goal_reached_radius_m_{0.65};
  int map_occ_threshold_{50};
  bool map_unknown_as_occupied_{false};
  double inflate_radius_m_{0.40};
  double start_search_radius_m_{1.5};
  double goal_search_radius_m_{2.5};
  bool publish_passthrough_without_map_{true};
  bool clamp_out_of_bounds_start_{true};
  bool clamp_out_of_bounds_goal_{true};
  int clamp_border_margin_cells_{2};

  // State
  bool have_pose_{false};
  bool have_raw_goal_{false};
  bool have_map_{false};
  uint64_t goal_seq_{0};
  uint64_t map_seq_{0};
  uint64_t planned_goal_seq_{0};
  uint64_t planned_map_seq_{0};
  geometry_msgs::msg::PoseStamped pose_;
  geometry_msgs::msg::Point raw_goal_;
  nav_msgs::msg::OccupancyGrid map_;
  std::vector<uint8_t> blocked_;
  std::vector<Cell> path_cells_;
  rclcpp::Time last_plan_time_{0, 0, RCL_ROS_TIME};

  // ROS
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_raw_goal_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_nav_goal_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SlamNavigationMode>());
  rclcpp::shutdown();
  return 0;
}
