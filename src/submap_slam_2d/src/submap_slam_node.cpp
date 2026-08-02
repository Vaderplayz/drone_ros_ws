#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <ctime>
#include <deque>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <map>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "submap_slam_2d/mapping_core.hpp"
#include "submap_slam_2d/descriptor.hpp"
#include "submap_slam_2d/pose_graph.hpp"
#include "submap_slam_2d/scan_matcher.hpp"
#include "submap_slam_2d/msg/external_loop_closure.hpp"

namespace submap_slam_2d
{
namespace
{

diagnostic_msgs::msg::KeyValue kv(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue result;
  result.key = key;
  result.value = value;
  return result;
}

std::string number(double value, int precision = 3)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(precision) << value;
  return stream.str();
}

geometry_msgs::msg::Quaternion yawQuaternion(double yaw)
{
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(quaternion);
}

Pose2 poseFromTransform(const geometry_msgs::msg::TransformStamped & transform)
{
  const auto & q = transform.transform.rotation;
  tf2::Quaternion quaternion(q.x, q.y, q.z, q.w);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
  return {transform.transform.translation.x, transform.transform.translation.y, yaw};
}

}  // namespace

class SubmapSlamNode : public rclcpp::Node
{
public:
  SubmapSlamNode()
  : Node("submap_slam_2d"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan_slam");
    full_odom_topic_ = declare_parameter<std::string>(
      "full_odom_topic", "/mavros/local_position/odom");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    map_frame_ = declare_parameter<std::string>("map_frame", "submap_map");
    publish_map_to_odom_tf_ = declare_parameter<bool>("publish_map_to_odom_tf", false);
    min_range_ = declare_parameter<double>("min_range_m", 0.2);
    max_range_ = declare_parameter<double>("max_range_m", 8.0);
    max_scans_per_submap_ = declare_parameter<int>("submap.max_scans", 60);
    max_completed_submaps_ = declare_parameter<int>("submap.max_completed_submaps", 200);
    overlap_ratio_ = declare_parameter<double>("submap.overlap_ratio", 0.5);
      max_submap_duration_sec_ = declare_parameter<double>("submap.max_duration_sec", 6.0);
    max_submap_translation_m_ = declare_parameter<double>("submap.max_translation_m", 2.0);
    max_submap_yaw_rad_ = declare_parameter<double>("submap.max_yaw_rad", 1.0472);
    global_map_publish_hz_ = declare_parameter<double>("global_map_publish_hz", 1.0);
    diagnostics_publish_hz_ = declare_parameter<double>("diagnostics_publish_hz", 1.0);

    grid_config_.resolution = declare_parameter<double>("submap.resolution", 0.05);
    grid_config_.size_x_m = declare_parameter<double>("submap.size_x_m", 20.0);
    grid_config_.size_y_m = declare_parameter<double>("submap.size_y_m", 20.0);
    grid_config_.occupied_log_odds = static_cast<float>(
      declare_parameter<double>("submap.occupied_log_odds", 0.85));
    grid_config_.free_log_odds = static_cast<float>(
      declare_parameter<double>("submap.free_log_odds", -0.40));
    grid_config_.occupied_threshold = static_cast<float>(
      declare_parameter<double>("submap.occupied_threshold", 0.50));

    matcher_config_.min_points = static_cast<std::size_t>(
      declare_parameter<int>("local_match.min_points", 60));
    matcher_config_.point_stride = static_cast<std::size_t>(
      declare_parameter<int>("local_match.point_stride", 3));
    matcher_config_.coarse_xy_window_m = declare_parameter<double>(
      "local_match.coarse_xy_window_m", 0.15);
    matcher_config_.coarse_xy_step_m = declare_parameter<double>(
      "local_match.coarse_xy_step_m", 0.05);
    matcher_config_.coarse_yaw_window_rad = declare_parameter<double>(
      "local_match.coarse_yaw_window_rad", 0.12);
    matcher_config_.coarse_yaw_step_rad = declare_parameter<double>(
      "local_match.coarse_yaw_step_rad", 0.03);
    matcher_config_.distance_sigma_m = declare_parameter<double>(
      "local_match.distance_sigma_m", 0.10);
    matcher_config_.min_score = declare_parameter<double>("local_match.min_score", 0.30);
    matcher_config_.max_rmse_m = declare_parameter<double>("local_match.max_rmse_m", 0.25);
    matcher_config_.max_translation_correction_m = declare_parameter<double>(
      "local_match.max_translation_correction_m", 0.35);
    matcher_config_.max_yaw_correction_rad = declare_parameter<double>(
      "local_match.max_yaw_correction_rad", 0.25);
    matcher_config_.huber_scale_m = declare_parameter<double>(
      "local_match.huber_scale_m", 0.10);
    matcher_config_.odom_translation_weight = declare_parameter<double>(
      "local_match.odom_translation_weight", 2.0);
    matcher_config_.odom_yaw_weight = declare_parameter<double>(
      "local_match.odom_yaw_weight", 2.0);
    matcher_config_.max_iterations = declare_parameter<int>("local_match.max_iterations", 20);

    registration_config_.point_stride = static_cast<std::size_t>(
      declare_parameter<int>("registration.point_stride", 2));
    registration_config_.min_points = static_cast<std::size_t>(
      declare_parameter<int>("registration.min_points", 100));
    registration_config_.coarse_xy_window_m = declare_parameter<double>(
      "registration.coarse_xy_window_m", 0.75);
    registration_config_.coarse_xy_step_m = declare_parameter<double>(
      "registration.coarse_xy_step_m", 0.10);
    registration_config_.coarse_yaw_window_rad = declare_parameter<double>(
      "registration.coarse_yaw_window_rad", 0.35);
    registration_config_.coarse_yaw_step_rad = declare_parameter<double>(
      "registration.coarse_yaw_step_rad", 0.05);
    registration_config_.distance_sigma_m = 0.10;
    registration_config_.min_score = 0.25;
    registration_config_.max_rmse_m = declare_parameter<double>(
      "registration.max_rmse_m", 0.20);
    registration_min_inlier_ratio_ = declare_parameter<double>(
      "registration.min_inlier_ratio", 0.30);
    registration_config_.max_translation_correction_m = declare_parameter<double>(
      "registration.max_translation_correction_m", 0.75);
    registration_config_.max_yaw_correction_rad = declare_parameter<double>(
      "registration.max_yaw_correction_rad", 0.35);
    registration_config_.huber_scale_m = 0.10;
    registration_config_.odom_translation_weight = 0.5;
    registration_config_.odom_yaw_weight = 0.5;
    registration_config_.max_iterations = 30;

    optimize_every_submap_ = declare_parameter<bool>("graph.optimize_every_submap", true);
    graph_config_.max_iterations = declare_parameter<int>("graph.max_iterations", 40);
    graph_config_.loop_huber_scale = declare_parameter<double>("graph.loop_huber_scale", 1.0);
    odom_translation_stddev_ = declare_parameter<double>(
      "graph.odom_translation_stddev_m", 0.20);
    odom_yaw_stddev_ = declare_parameter<double>("graph.odom_yaw_stddev_rad", 0.15);
    registration_translation_stddev_ = declare_parameter<double>(
      "graph.registration_translation_stddev_m", 0.10);
    registration_yaw_stddev_ = declare_parameter<double>(
      "graph.registration_yaw_stddev_rad", 0.08);

    descriptor_config_.angular_bins = declare_parameter<int>("loop.angular_bins", 60);
    descriptor_config_.radial_bins = declare_parameter<int>("loop.radial_bins", 20);
    descriptor_config_.max_radius_m = max_range_;
    loop_search_radius_m_ = declare_parameter<double>("loop.search_radius_m", 8.0);
    loop_descriptor_min_similarity_ = declare_parameter<double>(
      "loop.descriptor_min_similarity", 0.65);
    loop_min_index_separation_ = declare_parameter<int>("loop.min_index_separation", 5);
    loop_max_closures_per_submap_ = declare_parameter<int>(
      "loop.max_closures_per_submap", 1);
    loop_min_inlier_ratio_ = declare_parameter<double>("loop.min_inlier_ratio", 0.40);
    loop_match_config_.min_points = static_cast<std::size_t>(
      declare_parameter<int>("loop.min_points", 150));
    loop_match_config_.point_stride = 2;
    loop_match_config_.coarse_xy_window_m = declare_parameter<double>(
      "loop.coarse_xy_window_m", 3.0);
    loop_match_config_.coarse_xy_step_m = declare_parameter<double>(
      "loop.coarse_xy_step_m", 0.20);
    loop_match_config_.coarse_yaw_window_rad = declare_parameter<double>(
      "loop.coarse_yaw_window_rad", 1.2);
    loop_match_config_.coarse_yaw_step_rad = declare_parameter<double>(
      "loop.coarse_yaw_step_rad", 0.10);
    loop_match_config_.distance_sigma_m = 0.08;
    loop_match_config_.min_score = 0.35;
    loop_match_config_.max_rmse_m = declare_parameter<double>("loop.max_rmse_m", 0.15);
    loop_match_config_.max_translation_correction_m = declare_parameter<double>(
      "loop.max_translation_correction_m", 3.0);
    loop_match_config_.max_yaw_correction_rad = declare_parameter<double>(
      "loop.max_yaw_correction_rad", 1.2);
    loop_match_config_.huber_scale_m = 0.08;
    loop_match_config_.odom_translation_weight = 0.05;
    loop_match_config_.odom_yaw_weight = 0.05;
    loop_match_config_.max_iterations = 40;

    if (max_scans_per_submap_ < 4 || max_completed_submaps_ < 1 ||
      overlap_ratio_ <= 0.0 || overlap_ratio_ >= 1.0 ||
      min_range_ < 0.0 || max_range_ <= min_range_ || global_map_publish_hz_ <= 0.0)
    {
      throw std::invalid_argument("invalid submap, range, overlap, or publish-rate parameter");
    }
    scans_between_submaps_ = std::max(
      1, static_cast<int>(std::lround(max_scans_per_submap_ * (1.0 - overlap_ratio_))));
    matcher_ = std::make_unique<DistanceFieldMatcher2D>(matcher_config_);
    registration_matcher_ = std::make_unique<DistanceFieldMatcher2D>(registration_config_);
    pose_graph_ = std::make_unique<PoseGraph2D>(graph_config_);
    descriptor_ = std::make_unique<PolarDescriptor>(descriptor_config_);
    loop_matcher_ = std::make_unique<DistanceFieldMatcher2D>(loop_match_config_);

    const auto map_qos = rclcpp::QoS(1).reliable().transient_local();
    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/submap_slam/map", map_qos);
    local_map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/submap_slam/local_map", map_qos);
    trajectory_pub_ = create_publisher<nav_msgs::msg::Path>(
      "/submap_slam/trajectory", map_qos);
    corrected_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/submap_slam/corrected_pose", 10);
    corrected_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      "/submap_slam/corrected_odom", 10);
    submaps_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/submap_slam/submaps", map_qos);
    constraints_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/submap_slam/constraints", map_qos);
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/submap_slam/diagnostics", 10);

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&SubmapSlamNode::scanCallback, this, std::placeholders::_1));
    full_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      full_odom_topic_, rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_full_odom_ = *message;
      });
    external_loop_sub_ = create_subscription<submap_slam_2d::msg::ExternalLoopClosure>(
      "/submap_slam/external_loop_closure", 10,
      std::bind(&SubmapSlamNode::externalLoopCallback, this, std::placeholders::_1));

    map_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / global_map_publish_hz_),
      std::bind(&SubmapSlamNode::publishMaps, this));
    diagnostics_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / diagnostics_publish_hz_),
      std::bind(&SubmapSlamNode::publishDiagnostics, this));
    if (publish_map_to_odom_tf_) {
      map_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
      tf_guard_timer_ = create_wall_timer(
        std::chrono::milliseconds(500), std::bind(&SubmapSlamNode::guardedMapTfPublish, this));
    }
    graph_worker_ = std::thread(&SubmapSlamNode::graphWorker, this);

    RCLCPP_INFO(
      get_logger(),
      "Observer-only submap mapper: scan=%s prediction=%s->%s output_frame=%s map_tf=%s",
      scan_topic_.c_str(), odom_frame_.c_str(), base_frame_.c_str(), map_frame_.c_str(),
      publish_map_to_odom_tf_ ? "explicitly requested" : "disabled");
  }

  ~SubmapSlamNode() override
  {
    {
      std::lock_guard<std::mutex> lock(worker_mutex_);
      worker_stop_ = true;
    }
    worker_cv_.notify_all();
    if (graph_worker_.joinable()) {
      graph_worker_.join();
    }
  }

private:
  std::vector<Point2> scanPointsInBase(
    const sensor_msgs::msg::LaserScan & scan,
    const geometry_msgs::msg::TransformStamped & base_from_laser) const
  {
    tf2::Transform transform;
    tf2::fromMsg(base_from_laser.transform, transform);
    std::vector<Point2> points;
    points.reserve(scan.ranges.size());
    for (std::size_t index = 0; index < scan.ranges.size(); ++index) {
      const double range = static_cast<double>(scan.ranges[index]);
      if (!std::isfinite(range) || range < std::max(min_range_, static_cast<double>(scan.range_min)) ||
        range > std::min(max_range_, static_cast<double>(scan.range_max)))
      {
        continue;
      }
      const double angle = static_cast<double>(scan.angle_min) +
        static_cast<double>(index) * static_cast<double>(scan.angle_increment);
      const tf2::Vector3 point_laser(range * std::cos(angle), range * std::sin(angle), 0.0);
      const tf2::Vector3 point_base = transform * point_laser;
      if (std::isfinite(point_base.x()) && std::isfinite(point_base.y())) {
        points.push_back({point_base.x(), point_base.y()});
      }
    }
    return points;
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    const auto processing_start = std::chrono::steady_clock::now();
    const std::clock_t cpu_start = std::clock();
    const rclcpp::Time stamp(scan->header.stamp);
    if (stamp.nanoseconds() == 0 || (!scan->header.frame_id.empty() &&
      have_last_scan_stamp_ && stamp <= last_scan_stamp_))
    {
      ++scan_timestamp_regressions_;
      reject("invalid_or_regressed_scan_timestamp");
      return;
    }
    have_last_scan_stamp_ = true;
    last_scan_stamp_ = stamp;
    ++scan_count_;
    updateRate(stamp.seconds(), last_input_stamp_sec_, scan_input_rate_hz_);

    try {
      const auto timeout = tf2::durationFromSec(0.15);
      const auto odom_from_base = tf_buffer_.lookupTransform(
        odom_frame_, base_frame_, stamp, timeout);
      const auto base_from_laser = tf_buffer_.lookupTransform(
        base_frame_, scan->header.frame_id, stamp, timeout);
      const Pose2 odom_pose = poseFromTransform(odom_from_base);
      const std::vector<Point2> points = scanPointsInBase(*scan, base_from_laser);
      if (points.size() < 30U) {
        reject("too_few_valid_points");
        return;
      }

      std::lock_guard<std::mutex> lock(mutex_);
      if (!first_odom_pose_) {
        first_odom_pose_ = odom_pose;
      }
      if (map_capacity_reached_) {
        reject("map_capacity_reached");
        return;
      }
      if (active_submaps_.empty()) {
        createSubmap(odom_pose, stamp.seconds());
      }

      const auto & matching_submap = active_submaps_.back();
      const Pose2 prediction = between(matching_submap->odom_pose, odom_pose);
      Pose2 matched_local = prediction;
      if (matching_submap->scan_count > 0U) {
        last_match_ = matcher_->match(matching_submap->distance_field, points, prediction);
        if (!last_match_.accepted) {
          reject(last_match_.reason);
          return;
        }
        matched_local = last_match_.pose;
      } else {
        last_match_ = MatchResult{};
        last_match_.accepted = true;
        last_match_.pose = prediction;
        last_match_.reason = "bootstrap";
      }
      const Pose2 current_global = matching_submap->optimized_pose * matched_local;
      for (const auto & submap : active_submaps_) {
        const Pose2 local_pose = between(submap->optimized_pose, current_global);
        submap->insert(local_pose, points, stamp.seconds());
      }
      active_submaps_.back()->distance_field.build(active_submaps_.back()->grid);
      ++accepted_scan_count_;
      updateRate(stamp.seconds(), last_accepted_stamp_sec_, accepted_scan_rate_hz_);
      current_corrected_pose_ = current_global;

      const auto & newest = active_submaps_.back();
      const Pose2 motion = between(newest->odom_pose, odom_pose);
      const bool transition_due = newest->scan_count >= static_cast<std::size_t>(scans_between_submaps_) ||
        stamp.seconds() - newest->start_time >= max_submap_duration_sec_ ||
        std::hypot(motion.x, motion.y) >= max_submap_translation_m_ ||
        std::abs(motion.yaw) >= max_submap_yaw_rad_;
      if (transition_due) {
        createSubmap(odom_pose, stamp.seconds());
      }
      publishCurrentPose(stamp);
    } catch (const tf2::TransformException & exception) {
      ++tf_lookup_failures_;
      reject("tf_lookup_failed");
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Submap scan dropped: %s", exception.what());
      return;
    }
    const double duration_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - processing_start).count();
    processing_duration_ms_ = duration_ms;
    cpu_processing_duration_ms_ = 1000.0 * static_cast<double>(std::clock() - cpu_start) /
      static_cast<double>(CLOCKS_PER_SEC);
  }

  void createSubmap(const Pose2 & odom_pose, double stamp)
  {
    if (active_submaps_.size() == 2U) {
      if (completed_submaps_.size() >= static_cast<std::size_t>(max_completed_submaps_)) {
        map_capacity_reached_ = true;
        RCLCPP_ERROR(
          get_logger(), "Completed-submap capacity %zu reached; mapping paused to keep memory bounded",
          static_cast<std::size_t>(max_completed_submaps_));
        return;
      }
      auto completed = active_submaps_.front();
      active_submaps_.pop_front();
      completed->finish();
      completed_submaps_.push_back(std::move(completed));
      enqueueCompleted(completed_submaps_.back());
      global_map_dirty_ = true;
    }
    auto submap = std::make_shared<Submap2D>(next_submap_id_++, grid_config_, odom_pose, stamp);
    submap->optimized_pose = between(*first_odom_pose_, odom_pose);
    active_submaps_.push_back(std::move(submap));
  }

  void enqueueCompleted(const std::shared_ptr<Submap2D> & submap)
  {
    std::lock_guard<std::mutex> lock(worker_mutex_);
    if (graph_processing_disabled_) {
      return;
    }
    if (graph_queue_.size() >= 8U) {
      ++graph_queue_overflows_;
      graph_processing_disabled_ = true;
      RCLCPP_ERROR(
        get_logger(), "Pose-graph queue overflow; global correction disabled to preserve ordering");
      return;
    }
    graph_queue_.push_back(submap);
    worker_cv_.notify_one();
  }

  void graphWorker()
  {
    while (true) {
      std::shared_ptr<Submap2D> current;
      {
        std::unique_lock<std::mutex> lock(worker_mutex_);
        worker_cv_.wait(lock, [this]() {
          return worker_stop_ || !graph_queue_.empty() || !external_loop_queue_.empty();
        });
        if (worker_stop_) {
          return;
        }
        if (graph_queue_.empty()) {
          const auto proposal = external_loop_queue_.front();
          external_loop_queue_.pop_front();
          lock.unlock();
          processExternalLoop(proposal);
          continue;
        }
        current = graph_queue_.front();
        graph_queue_.pop_front();
      }

      const int vertex = pose_graph_->addVertex(current->optimized_pose);
      descriptors_.push_back(descriptor_->compute(current->grid));
      MatchResult adjacent;
      if (vertex > 0) {
        std::shared_ptr<Submap2D> previous;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          previous = completed_submaps_[static_cast<std::size_t>(vertex - 1)];
        }
        pose_graph_->addEdge({
          vertex - 1, vertex, between(previous->odom_pose, current->odom_pose),
          odom_translation_stddev_, odom_yaw_stddev_, EdgeType::Odometry});
        const Pose2 prediction = between(previous->optimized_pose, current->optimized_pose);
        adjacent = registration_matcher_->match(
          previous->distance_field, current->grid.surfacePoints(registration_config_.point_stride),
          prediction);
        if (adjacent.accepted && adjacent.inlier_ratio >= registration_min_inlier_ratio_) {
          pose_graph_->addEdge({
            vertex - 1, vertex, adjacent.pose, registration_translation_stddev_,
            registration_yaw_stddev_, EdgeType::Registration});
          ++adjacent_registrations_accepted_;
        } else {
          ++adjacent_registrations_rejected_;
          last_registration_rejection_ = adjacent.accepted ?
            "low_inlier_ratio" : adjacent.reason;
        }
      }

      OptimizationSummary optimization;
      if (optimize_every_submap_) {
        optimization = pose_graph_->optimize();
      }
      if (vertex >= loop_min_index_separation_) {
        const auto loop_result = findAndVerifyLoop(vertex, current);
        if (loop_result) {
          pose_graph_->addEdge(*loop_result);
          ++loop_closures_accepted_;
          optimization = pose_graph_->optimize();
        }
      }
      {
        std::lock_guard<std::mutex> lock(mutex_);
        last_optimization_ = optimization;
        graph_edges_snapshot_ = pose_graph_->edges();
        const auto & poses = pose_graph_->poses();
        graph_vertex_count_ = poses.size();
        for (std::size_t index = 0;
          index < poses.size() && index < completed_submaps_.size(); ++index)
        {
          completed_submaps_[index]->optimized_pose = poses[index];
        }
        if (!completed_submaps_.empty()) {
          const auto & anchor = completed_submaps_.back();
          for (const auto & active : active_submaps_) {
            active->optimized_pose = anchor->optimized_pose *
              between(anchor->odom_pose, active->odom_pose);
          }
        }
        global_map_dirty_ = true;
      }
    }
  }

  struct Bounds
  {
    double min_x{0.0};
    double min_y{0.0};
    double max_x{0.0};
    double max_y{0.0};
  };

  static Bounds bounds(const Submap2D & submap)
  {
    const auto & grid = submap.grid;
    const Point2 corners[4] = {
      {grid.originX(), grid.originY()},
      {grid.originX() + grid.width() * grid.config().resolution, grid.originY()},
      {grid.originX(), grid.originY() + grid.height() * grid.config().resolution},
      {grid.originX() + grid.width() * grid.config().resolution,
        grid.originY() + grid.height() * grid.config().resolution}};
    Bounds result{std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(),
      -std::numeric_limits<double>::infinity()};
    for (const Point2 & corner : corners) {
      const Point2 point = submap.optimized_pose.transform(corner);
      result.min_x = std::min(result.min_x, point.x);
      result.min_y = std::min(result.min_y, point.y);
      result.max_x = std::max(result.max_x, point.x);
      result.max_y = std::max(result.max_y, point.y);
    }
    return result;
  }

  static bool overlaps(const Bounds & a, const Bounds & b)
  {
    return a.min_x <= b.max_x && a.max_x >= b.min_x &&
      a.min_y <= b.max_y && a.max_y >= b.min_y;
  }

  std::optional<GraphEdge> findAndVerifyLoop(
    int vertex, const std::shared_ptr<Submap2D> & current)
  {
    if (loop_max_closures_per_submap_ <= 0) {
      return std::nullopt;
    }
    struct Candidate {int index; DescriptorMatch descriptor;};
    std::vector<Candidate> candidates;
    const Bounds current_bounds = bounds(*current);
    for (int index = 0; index <= vertex - loop_min_index_separation_; ++index) {
      std::shared_ptr<Submap2D> target;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        target = completed_submaps_[static_cast<std::size_t>(index)];
      }
      const double distance = std::hypot(
        target->optimized_pose.x - current->optimized_pose.x,
        target->optimized_pose.y - current->optimized_pose.y);
      const DescriptorMatch descriptor_match = descriptor_->compare(
        descriptors_[static_cast<std::size_t>(index)], descriptors_.back());
      if ((distance <= loop_search_radius_m_ && overlaps(bounds(*target), current_bounds)) ||
        descriptor_match.similarity >= loop_descriptor_min_similarity_)
      {
        candidates.push_back({index, descriptor_match});
      }
    }
    std::sort(candidates.begin(), candidates.end(), [](const Candidate & a, const Candidate & b) {
      return a.descriptor.similarity > b.descriptor.similarity;
    });
    if (candidates.size() > 3U) {
      candidates.resize(3U);
    }

    std::optional<GraphEdge> best_edge;
    double best_rmse = std::numeric_limits<double>::infinity();
    const auto source_points = current->grid.surfacePoints(loop_match_config_.point_stride);
    if (!hasNonDegenerateGeometry(source_points, loop_match_config_.min_points)) {
      last_loop_rejection_reason_ = "ambiguous_source_geometry";
      loop_closures_rejected_ += candidates.size();
      return std::nullopt;
    }
    for (const Candidate & candidate : candidates) {
      ++loop_candidates_tested_;
      std::shared_ptr<Submap2D> target;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        target = completed_submaps_[static_cast<std::size_t>(candidate.index)];
      }
      if (!hasNonDegenerateGeometry(
          target->grid.surfacePoints(loop_match_config_.point_stride),
          loop_match_config_.min_points))
      {
        ++loop_closures_rejected_;
        last_loop_rejection_reason_ = "ambiguous_target_geometry";
        continue;
      }
      Pose2 prediction = between(target->optimized_pose, current->optimized_pose);
      const MatchResult match = loop_matcher_->match(target->distance_field, source_points, prediction);
      if (!match.accepted || match.inlier_ratio < loop_min_inlier_ratio_ ||
        !match.covariance_valid)
      {
        ++loop_closures_rejected_;
        last_loop_rejection_reason_ = !match.accepted ? match.reason :
          (match.inlier_ratio < loop_min_inlier_ratio_ ? "low_inlier_ratio" : "singular_covariance");
        continue;
      }
      if (match.rmse_m < best_rmse) {
        best_rmse = match.rmse_m;
        best_edge = GraphEdge{
          candidate.index, vertex, match.pose, 0.06, 0.05, EdgeType::Loop};
      }
    }
    if (!best_edge && candidates.empty()) {
      last_loop_rejection_reason_ = "no_candidate";
    }
    return best_edge;
  }

  void externalLoopCallback(
    const submap_slam_2d::msg::ExternalLoopClosure::SharedPtr proposal)
  {
    const bool covariance_finite = std::all_of(
      proposal->covariance.begin(), proposal->covariance.end(),
      [](double value) {return std::isfinite(value);});
    if (proposal->source_submap_id == proposal->target_submap_id ||
      proposal->source_submap_id < 0 || proposal->target_submap_id < 0 ||
      !std::isfinite(proposal->relative_x + proposal->relative_y + proposal->relative_yaw +
      proposal->confidence) || proposal->confidence <= 0.0 || !covariance_finite ||
      proposal->source_type.empty())
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Rejected malformed external loop proposal");
      return;
    }
    std::lock_guard<std::mutex> lock(worker_mutex_);
    if (external_loop_queue_.size() >= 8U) {
      external_loop_queue_.pop_front();
      ++external_loop_queue_overflows_;
    }
    external_loop_queue_.push_back(*proposal);
    worker_cv_.notify_one();
  }

  void processExternalLoop(const submap_slam_2d::msg::ExternalLoopClosure & proposal)
  {
    const int source = proposal.source_submap_id;
    const int target = proposal.target_submap_id;
    std::shared_ptr<Submap2D> source_submap;
    std::shared_ptr<Submap2D> target_submap;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (source >= static_cast<int>(graph_vertex_count_) ||
        target >= static_cast<int>(graph_vertex_count_))
      {
        ++loop_closures_rejected_;
        last_loop_rejection_reason_ = "external_unknown_submap";
        return;
      }
      source_submap = completed_submaps_[static_cast<std::size_t>(source)];
      target_submap = completed_submaps_[static_cast<std::size_t>(target)];
    }
    const auto points = source_submap->grid.surfacePoints(loop_match_config_.point_stride);
    if (!hasNonDegenerateGeometry(points, loop_match_config_.min_points)) {
      ++loop_closures_rejected_;
      last_loop_rejection_reason_ = "external_ambiguous_geometry";
      return;
    }
    const Pose2 proposal_pose{
      proposal.relative_x, proposal.relative_y, wrap_angle(proposal.relative_yaw)};
    ++loop_candidates_tested_;
    const MatchResult match = loop_matcher_->match(
      target_submap->distance_field, points, proposal_pose);
    if (!match.accepted || match.inlier_ratio < loop_min_inlier_ratio_ ||
      !match.covariance_valid)
    {
      ++loop_closures_rejected_;
      last_loop_rejection_reason_ = "external_geometric_verification_failed";
      return;
    }
    pose_graph_->addEdge({target, source, match.pose, 0.06, 0.05, EdgeType::Loop});
    ++loop_closures_accepted_;
    const OptimizationSummary optimization = pose_graph_->optimize();
    std::lock_guard<std::mutex> lock(mutex_);
    last_optimization_ = optimization;
    graph_edges_snapshot_ = pose_graph_->edges();
    const auto & poses = pose_graph_->poses();
    for (std::size_t index = 0; index < poses.size() && index < completed_submaps_.size(); ++index) {
      completed_submaps_[index]->optimized_pose = poses[index];
    }
    global_map_dirty_ = true;
  }

  void reject(const std::string & reason)
  {
    ++rejected_scan_count_;
    last_rejection_reason_ = reason;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Rejected scan: %s", reason.c_str());
  }

  static void updateRate(double stamp, double & previous, double & rate)
  {
    if (previous > 0.0 && stamp > previous) {
      const double instantaneous = 1.0 / (stamp - previous);
      rate = rate > 0.0 ? 0.9 * rate + 0.1 * instantaneous : instantaneous;
    }
    previous = stamp;
  }

  nav_msgs::msg::OccupancyGrid gridMessage(
    const OccupancyGrid2D & grid, const std::string & frame, const rclcpp::Time & stamp) const
  {
    nav_msgs::msg::OccupancyGrid message;
    message.header.stamp = stamp;
    message.header.frame_id = frame;
    message.info.resolution = static_cast<float>(grid.config().resolution);
    message.info.width = static_cast<std::uint32_t>(grid.width());
    message.info.height = static_cast<std::uint32_t>(grid.height());
    message.info.origin.position.x = grid.originX();
    message.info.origin.position.y = grid.originY();
    message.info.origin.orientation.w = 1.0;
    message.data.resize(static_cast<std::size_t>(grid.width() * grid.height()));
    for (int y = 0; y < grid.height(); ++y) {
      for (int x = 0; x < grid.width(); ++x) {
        message.data[static_cast<std::size_t>(y * grid.width() + x)] = grid.occupancy(x, y);
      }
    }
    return message;
  }

  nav_msgs::msg::OccupancyGrid renderGlobal(
    const rclcpp::Time & stamp, const std::vector<RenderSubmapView> & submaps)
  {
    nav_msgs::msg::OccupancyGrid message;
    message.header.stamp = stamp;
    message.header.frame_id = map_frame_;
    const RenderedGrid rendered = renderGlobalMap(submaps, grid_config_.resolution);
    message.info.resolution = static_cast<float>(rendered.resolution);
    message.info.width = static_cast<std::uint32_t>(rendered.width);
    message.info.height = static_cast<std::uint32_t>(rendered.height);
    message.info.origin.position.x = rendered.origin_x;
    message.info.origin.position.y = rendered.origin_y;
    message.info.origin.orientation.w = 1.0;
    message.data = rendered.data;
    return message;
  }

  void publishMaps()
  {
    const auto start = std::chrono::steady_clock::now();
    const rclcpp::Time stamp = now();
    std::vector<RenderSubmapView> views;
    std::vector<OccupancyGrid2D> active_grid_snapshots;
    std::optional<OccupancyGrid2D> local_grid;
    Pose2 local_pose;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      views.reserve(completed_submaps_.size() + active_submaps_.size());
      for (const auto & submap : completed_submaps_) {
        views.push_back({&submap->grid, submap->optimized_pose});
      }
      active_grid_snapshots.reserve(active_submaps_.size());
      for (const auto & submap : active_submaps_) {
        active_grid_snapshots.push_back(submap->grid);
      }
      for (std::size_t index = 0; index < active_submaps_.size(); ++index) {
        views.push_back({&active_grid_snapshots[index], active_submaps_[index]->optimized_pose});
      }
      if (!active_submaps_.empty()) {
        local_grid = active_grid_snapshots.back();
        local_pose = active_submaps_.back()->optimized_pose;
      }
    }
    if (local_grid) {
      auto local = gridMessage(*local_grid, map_frame_, stamp);
      local.info.origin.position.x += local_pose.x;
      local.info.origin.position.y += local_pose.y;
      local.info.origin.orientation = yawQuaternion(local_pose.yaw);
      local_map_pub_->publish(local);
    }
    if (!views.empty()) {
      map_pub_->publish(renderGlobal(stamp, views));
      std::lock_guard<std::mutex> lock(mutex_);
      global_map_dirty_ = false;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      publishTrajectory(stamp);
      publishSubmapMarkers(stamp);
    }
    last_map_render_ms_ = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count();
  }

  void publishCurrentPose(const rclcpp::Time & stamp)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = map_frame_;
    pose.pose.position.x = current_corrected_pose_.x;
    pose.pose.position.y = current_corrected_pose_.y;
    pose.pose.orientation = yawQuaternion(current_corrected_pose_.yaw);
    corrected_pose_pub_->publish(pose);

    if (latest_full_odom_) {
      nav_msgs::msg::Odometry corrected = *latest_full_odom_;
      corrected.header.stamp = stamp;
      corrected.header.frame_id = map_frame_;
      corrected.child_frame_id = base_frame_;
      corrected.pose.pose.position.x = current_corrected_pose_.x;
      corrected.pose.pose.position.y = current_corrected_pose_.y;
      const auto & source_q = latest_full_odom_->pose.pose.orientation;
      tf2::Quaternion q(source_q.x, source_q.y, source_q.z, source_q.w);
      double roll = 0.0;
      double pitch = 0.0;
      double ignored_yaw = 0.0;
      tf2::Matrix3x3(q).getRPY(roll, pitch, ignored_yaw);
      tf2::Quaternion corrected_q;
      corrected_q.setRPY(roll, pitch, current_corrected_pose_.yaw);
      corrected.pose.pose.orientation = tf2::toMsg(corrected_q);
      corrected_odom_pub_->publish(corrected);
    }
  }

  void publishTrajectory(const rclcpp::Time & stamp)
  {
    nav_msgs::msg::Path path;
    path.header.stamp = stamp;
    path.header.frame_id = map_frame_;
    std::map<double, Pose2> corrected;
    auto collect = [&corrected](const std::shared_ptr<Submap2D> & submap) {
        for (std::size_t index = 0;
          index < submap->scan_poses.size() && index < submap->scan_timestamps.size(); ++index)
        {
          corrected[submap->scan_timestamps[index]] =
            submap->optimized_pose * submap->scan_poses[index];
        }
      };
    for (const auto & submap : completed_submaps_) {
      collect(submap);
    }
    for (const auto & submap : active_submaps_) {
      collect(submap);
    }
    path.poses.reserve(corrected.size());
    for (const auto & item : corrected) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = rclcpp::Time(
        static_cast<std::int64_t>(item.first * 1e9), get_clock()->get_clock_type());
      pose.header.frame_id = map_frame_;
      pose.pose.position.x = item.second.x;
      pose.pose.position.y = item.second.y;
      pose.pose.orientation = yawQuaternion(item.second.yaw);
      path.poses.push_back(std::move(pose));
    }
    trajectory_pub_->publish(path);
  }

  void publishSubmapMarkers(const rclcpp::Time & stamp)
  {
    visualization_msgs::msg::MarkerArray array;
    auto append = [&](const std::shared_ptr<Submap2D> & submap, bool completed) {
        visualization_msgs::msg::Marker marker;
        marker.header.stamp = stamp;
        marker.header.frame_id = map_frame_;
        marker.ns = "submap_bounds";
        marker.id = submap->id;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = submap->optimized_pose.x;
        marker.pose.position.y = submap->optimized_pose.y;
        marker.pose.position.z = -0.02;
        marker.pose.orientation = yawQuaternion(submap->optimized_pose.yaw);
        marker.scale.x = submap->grid.config().size_x_m;
        marker.scale.y = submap->grid.config().size_y_m;
        marker.scale.z = 0.01;
        marker.color.r = completed ? 0.1F : 0.1F;
        marker.color.g = completed ? 0.5F : 0.9F;
        marker.color.b = completed ? 0.9F : 0.2F;
        marker.color.a = 0.08F;
        array.markers.push_back(marker);

        visualization_msgs::msg::Marker center = marker;
        center.ns = "submap_centers";
        center.id = 10000 + submap->id;
        center.type = visualization_msgs::msg::Marker::SPHERE;
        center.pose.position.z = 0.03;
        center.pose.orientation.w = 1.0;
        center.scale.x = 0.10;
        center.scale.y = 0.10;
        center.scale.z = 0.10;
        center.color.a = 0.9F;
        array.markers.push_back(center);

        visualization_msgs::msg::Marker label = center;
        label.ns = "submap_ids";
        label.id = 20000 + submap->id;
        label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        label.pose.position.z = 0.20;
        label.scale.z = 0.16;
        label.color.r = 1.0F;
        label.color.g = 1.0F;
        label.color.b = 1.0F;
        label.color.a = 1.0F;
        label.text = std::to_string(submap->id);
        array.markers.push_back(label);
      };
    for (const auto & submap : completed_submaps_) {
      append(submap, true);
    }
    for (const auto & submap : active_submaps_) {
      append(submap, false);
    }
    submaps_pub_->publish(array);
    visualization_msgs::msg::MarkerArray constraints;
    visualization_msgs::msg::Marker lines;
    lines.header.stamp = stamp;
    lines.header.frame_id = map_frame_;
    lines.ns = "pose_graph_constraints";
    lines.id = 0;
    lines.type = visualization_msgs::msg::Marker::LINE_LIST;
    lines.action = visualization_msgs::msg::Marker::ADD;
    lines.scale.x = 0.025;
    lines.pose.orientation.w = 1.0;
    for (const GraphEdge & edge : graph_edges_snapshot_) {
      if (edge.from < 0 || edge.to < 0 ||
        edge.from >= static_cast<int>(completed_submaps_.size()) ||
        edge.to >= static_cast<int>(completed_submaps_.size()))
      {
        continue;
      }
      geometry_msgs::msg::Point from;
      geometry_msgs::msg::Point to;
      from.x = completed_submaps_[static_cast<std::size_t>(edge.from)]->optimized_pose.x;
      from.y = completed_submaps_[static_cast<std::size_t>(edge.from)]->optimized_pose.y;
      to.x = completed_submaps_[static_cast<std::size_t>(edge.to)]->optimized_pose.x;
      to.y = completed_submaps_[static_cast<std::size_t>(edge.to)]->optimized_pose.y;
      lines.points.push_back(from);
      lines.points.push_back(to);
      const std_msgs::msg::ColorRGBA color = edge.type == EdgeType::Odometry ?
        std_msgs::msg::ColorRGBA().set__r(0.3F).set__g(0.6F).set__b(1.0F).set__a(0.8F) :
        (edge.type == EdgeType::Registration ?
        std_msgs::msg::ColorRGBA().set__r(0.2F).set__g(0.9F).set__b(0.3F).set__a(0.9F) :
        std_msgs::msg::ColorRGBA().set__r(1.0F).set__g(0.3F).set__b(0.2F).set__a(1.0F));
      lines.colors.push_back(color);
      lines.colors.push_back(color);
    }
    constraints.markers.push_back(std::move(lines));
    constraints_pub_->publish(constraints);
  }

  void guardedMapTfPublish()
  {
    if (map_tf_refused_) {
      return;
    }
    try {
      for (const std::string & node_name : get_node_names()) {
        if (!map_tf_started_ && node_name.find("slam_toolbox") != std::string::npos) {
          map_tf_refused_ = true;
          RCLCPP_ERROR(
            get_logger(), "Refusing %s -> %s publication while slam_toolbox is running",
            map_frame_.c_str(), odom_frame_.c_str());
          return;
        }
      }
      if (!map_tf_started_ && tf_buffer_.canTransform(
          map_frame_, odom_frame_, tf2::TimePointZero, tf2::durationFromSec(0.05)))
      {
        map_tf_refused_ = true;
        RCLCPP_ERROR(
          get_logger(), "Refusing %s -> %s publication: transform already exists",
          map_frame_.c_str(), odom_frame_.c_str());
        return;
      }
    } catch (const tf2::TransformException &) {
    }
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now();
    transform.header.frame_id = map_frame_;
    transform.child_frame_id = odom_frame_;
    transform.transform.rotation.w = 1.0;
    map_tf_broadcaster_->sendTransform(transform);
    map_tf_started_ = true;
  }

  void publishDiagnostics()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "submap_slam_2d";
    status.hardware_id = "horizontal_lidar_mapping_only";
    status.level = tf_lookup_failures_ > 0 && accepted_scan_count_ == 0 ?
      diagnostic_msgs::msg::DiagnosticStatus::WARN : diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = active_submaps_.empty() ? "waiting_for_scans" : "mapping";
    status.values = {
      kv("scan_input_rate_hz", number(scan_input_rate_hz_)),
      kv("accepted_scan_rate_hz", number(accepted_scan_rate_hz_)),
      kv("rejected_scan_count", std::to_string(rejected_scan_count_)),
      kv("last_rejection_reason", last_rejection_reason_),
      kv("active_submap_id", active_submaps_.empty() ? "-1" :
        std::to_string(active_submaps_.back()->id)),
      kv("active_submap_scan_count", active_submaps_.empty() ? "0" :
        std::to_string(active_submaps_.back()->scan_count)),
      kv("completed_submap_count", std::to_string(completed_submaps_.size())),
      kv("local_match_score", number(last_match_.score)),
      kv("local_match_rmse_m", number(last_match_.rmse_m)),
      kv("local_match_inlier_ratio", number(last_match_.inlier_ratio)),
      kv("local_match_status", last_match_.reason),
      kv("local_match_duration_ms", number(last_match_.duration_ms)),
      kv("loop_candidates_tested", std::to_string(loop_candidates_tested_)),
      kv("loop_closures_accepted", std::to_string(loop_closures_accepted_)),
      kv("loop_closures_rejected", std::to_string(loop_closures_rejected_)),
      kv("last_loop_rejection_reason", last_loop_rejection_reason_),
      kv("graph_vertex_count", std::to_string(graph_vertex_count_)),
      kv("graph_odometry_edges", std::to_string(std::count_if(
        graph_edges_snapshot_.begin(), graph_edges_snapshot_.end(),
        [](const GraphEdge & edge) {return edge.type == EdgeType::Odometry;}))),
      kv("graph_registration_edges", std::to_string(std::count_if(
        graph_edges_snapshot_.begin(), graph_edges_snapshot_.end(),
        [](const GraphEdge & edge) {return edge.type == EdgeType::Registration;}))),
      kv("graph_loop_edges", std::to_string(std::count_if(
        graph_edges_snapshot_.begin(), graph_edges_snapshot_.end(),
        [](const GraphEdge & edge) {return edge.type == EdgeType::Loop;}))),
      kv("last_optimization_duration_ms", number(last_optimization_.duration_ms)),
      kv("last_optimization_initial_cost", number(last_optimization_.initial_cost)),
      kv("last_optimization_final_cost", number(last_optimization_.final_cost)),
      kv("adjacent_registrations_accepted", std::to_string(adjacent_registrations_accepted_)),
      kv("adjacent_registrations_rejected", std::to_string(adjacent_registrations_rejected_)),
      kv("last_registration_rejection", last_registration_rejection_),
      kv("graph_queue_overflows", std::to_string(graph_queue_overflows_)),
      kv("graph_processing_disabled", graph_processing_disabled_ ? "true" : "false"),
      kv("map_capacity_reached", map_capacity_reached_ ? "true" : "false"),
      kv("external_loop_queue_overflows", std::to_string(external_loop_queue_overflows_)),
      kv("map_render_duration_ms", number(last_map_render_ms_)),
      kv("tf_lookup_failures", std::to_string(tf_lookup_failures_)),
      kv("scan_timestamp_regressions", std::to_string(scan_timestamp_regressions_)),
      kv("process_memory_kib", processMemoryKiB()),
      kv("scan_processing_duration_ms", number(processing_duration_ms_)),
      kv("scan_cpu_time_ms", number(cpu_processing_duration_ms_)),
      kv("publish_map_to_odom_tf", publish_map_to_odom_tf_ ? "true" : "false"),
      kv("map_tf_refused", map_tf_refused_ ? "true" : "false")};
    array.status.push_back(std::move(status));
    diagnostics_pub_->publish(array);
  }

  static std::string processMemoryKiB()
  {
    std::ifstream status_file("/proc/self/status");
    std::string key;
    while (status_file >> key) {
      if (key == "VmRSS:") {
        std::string value;
        std::string units;
        status_file >> value >> units;
        return value;
      }
      status_file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }
    return "unknown";
  }

  std::string scan_topic_;
  std::string full_odom_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string map_frame_;
  bool publish_map_to_odom_tf_{false};
  bool map_tf_started_{false};
  bool map_tf_refused_{false};
  double min_range_{0.2};
  double max_range_{8.0};
  int max_scans_per_submap_{60};
  int max_completed_submaps_{200};
  int scans_between_submaps_{30};
  double overlap_ratio_{0.5};
  double max_submap_duration_sec_{6.0};
  double max_submap_translation_m_{2.0};
  double max_submap_yaw_rad_{1.0472};
  double global_map_publish_hz_{1.0};
  double diagnostics_publish_hz_{1.0};
  GridConfig grid_config_;
  MatcherConfig matcher_config_;
  MatcherConfig registration_config_;
  std::unique_ptr<DistanceFieldMatcher2D> matcher_;
  std::unique_ptr<DistanceFieldMatcher2D> registration_matcher_;
  PoseGraphConfig graph_config_;
  std::unique_ptr<PoseGraph2D> pose_graph_;
  DescriptorConfig descriptor_config_;
  std::unique_ptr<PolarDescriptor> descriptor_;
  MatcherConfig loop_match_config_;
  std::unique_ptr<DistanceFieldMatcher2D> loop_matcher_;
  double loop_search_radius_m_{8.0};
  double loop_descriptor_min_similarity_{0.65};
  double loop_min_inlier_ratio_{0.40};
  int loop_min_index_separation_{5};
  int loop_max_closures_per_submap_{1};
  bool optimize_every_submap_{true};
  double registration_min_inlier_ratio_{0.30};
  double odom_translation_stddev_{0.20};
  double odom_yaw_stddev_{0.15};
  double registration_translation_stddev_{0.10};
  double registration_yaw_stddev_{0.08};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> map_tf_broadcaster_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr full_odom_sub_;
  rclcpp::Subscription<submap_slam_2d::msg::ExternalLoopClosure>::SharedPtr external_loop_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr local_map_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr corrected_pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr corrected_odom_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr submaps_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr constraints_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr map_timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  rclcpp::TimerBase::SharedPtr tf_guard_timer_;

  std::mutex mutex_;
  std::optional<nav_msgs::msg::Odometry> latest_full_odom_;
  std::optional<Pose2> first_odom_pose_;
  std::deque<std::shared_ptr<Submap2D>> active_submaps_;
  std::vector<std::shared_ptr<Submap2D>> completed_submaps_;
  Pose2 current_corrected_pose_;
  int next_submap_id_{0};
  bool global_map_dirty_{false};
  bool map_capacity_reached_{false};
  bool have_last_scan_stamp_{false};
  rclcpp::Time last_scan_stamp_{0, 0, RCL_ROS_TIME};
  std::uint64_t scan_count_{0};
  std::uint64_t accepted_scan_count_{0};
  std::uint64_t rejected_scan_count_{0};
  std::uint64_t tf_lookup_failures_{0};
  std::uint64_t scan_timestamp_regressions_{0};
  std::string last_rejection_reason_{"none"};
  double last_input_stamp_sec_{0.0};
  double last_accepted_stamp_sec_{0.0};
  double scan_input_rate_hz_{0.0};
  double accepted_scan_rate_hz_{0.0};
  double processing_duration_ms_{0.0};
  double cpu_processing_duration_ms_{0.0};
  double last_map_render_ms_{0.0};
  MatchResult last_match_;
  OptimizationSummary last_optimization_;
  std::vector<GraphEdge> graph_edges_snapshot_;
  std::vector<std::vector<float>> descriptors_;
  std::size_t graph_vertex_count_{0};
  std::uint64_t adjacent_registrations_accepted_{0};
  std::uint64_t adjacent_registrations_rejected_{0};
  std::string last_registration_rejection_{"none"};

  std::mutex worker_mutex_;
  std::condition_variable worker_cv_;
  std::deque<std::shared_ptr<Submap2D>> graph_queue_;
  std::thread graph_worker_;
  bool worker_stop_{false};
  bool graph_processing_disabled_{false};
  std::uint64_t graph_queue_overflows_{0};
  std::deque<submap_slam_2d::msg::ExternalLoopClosure> external_loop_queue_;
  std::uint64_t external_loop_queue_overflows_{0};
  std::uint64_t loop_candidates_tested_{0};
  std::uint64_t loop_closures_accepted_{0};
  std::uint64_t loop_closures_rejected_{0};
  std::string last_loop_rejection_reason_{"none"};
};

}  // namespace submap_slam_2d

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<submap_slam_2d::SubmapSlamNode>());
  rclcpp::shutdown();
  return 0;
}
