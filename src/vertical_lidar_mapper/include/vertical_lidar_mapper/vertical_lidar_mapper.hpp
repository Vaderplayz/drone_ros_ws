// Timestamp: 2026-02-25 09:45:00 +07+0700
// Most Recent Update: Added floor-point exclusion parameters and diagnostics counters.
#ifndef VERTICAL_LIDAR_MAPPER__VERTICAL_LIDAR_MAPPER_HPP_
#define VERTICAL_LIDAR_MAPPER__VERTICAL_LIDAR_MAPPER_HPP_

#include <chrono>
#include <deque>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <message_filters/subscriber.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Transform.hpp>
#if __has_include(<tf2_ros/buffer.hpp>)
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/create_timer_ros.hpp>
#include <tf2_ros/message_filter.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/transform_listener.hpp>
#else
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#endif

namespace vertical_lidar_mapper
{

class VerticalLidarMapper : public rclcpp::Node
{
public:
  explicit VerticalLidarMapper(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~VerticalLidarMapper() override;

private:
  struct TimedScan
  {
    rclcpp::Time stamp;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    std::size_t points;
  };

  struct TrajectoryPoint
  {
    rclcpp::Time stamp;
    float x{0.0F};
    float y{0.0F};
    float z{0.0F};
  };

  struct Keyframe
  {
    std::uint64_t id{0};
    rclcpp::Time stamp;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_local;
    tf2::Transform pose_odom_source;
    tf2::Transform pose_self_aligned_source;
    tf2::Transform pose_target_source;
    std::string source_frame;
    bool deskewed{false};
    std::size_t input_points{0};
  };

  struct PoseSample
  {
    rclcpp::Time stamp;
    tf2::Transform pose_odom_base;
  };

  struct PendingScan
  {
    sensor_msgs::msg::LaserScan::ConstSharedPtr message;
    rclcpp::Time reference_stamp;
    std::chrono::steady_clock::time_point enqueued_at;
  };

  void loadParameters();
  void motionOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void scanCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);
  void processScan(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg,
    const rclcpp::Time & scan_stamp);
  void onDeskewQueueTimer();

  std::string resolveSourceFrame(const sensor_msgs::msg::LaserScan & scan_msg) const;
  rclcpp::Time resolveScanStamp(const sensor_msgs::msg::LaserScan & scan_msg) const;
  double scanDurationSec(const sensor_msgs::msg::LaserScan & scan_msg) const;
  void scanPoseWindow(
    const sensor_msgs::msg::LaserScan & scan_msg,
    const rclcpp::Time & reference_stamp,
    rclcpp::Time & first_pose_stamp,
    rclcpp::Time & last_pose_stamp) const;

  void pruneOldScans(const rclcpp::Time & newest_stamp);
  void enforceRawPointCap();
  void recordTfLookupSampleMs(double duration_ms);
  std::vector<PoseSample> snapshotPoseBuffer() const;
  bool interpolateFullPose(
    const std::vector<PoseSample> & samples,
    const rclcpp::Time & stamp,
    tf2::Transform & pose_odom_base,
    double & bracket_gap_sec) const;
  bool lookupTargetFromOdom(
    const rclcpp::Time & stamp,
    bool latest,
    tf2::Transform & pose_target_odom,
    std::string & error_message) const;
  bool buildDeskewedLocalCloud(
    const sensor_msgs::msg::LaserScan & scan,
    const rclcpp::Time & reference_stamp,
    const std::string & source_frame,
    pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_local,
    tf2::Transform & pose_odom_source,
    std::size_t & valid_input_points,
    std::string & error_message);
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_cloud,
    const tf2::Transform & transform) const;
  bool estimateFloorHeight(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
    double & floor_z,
    double & floor_tilt_rad,
    tf2::Quaternion & leveling_rotation) const;
  void applyFloorLevelingRotation(
    const tf2::Quaternion & leveling_rotation,
    pcl::PointCloud<pcl::PointXYZ>::Ptr & scan_cloud,
    tf2::Transform & pose_target_source,
    tf2::Transform & pose_odom_source) const;
  bool shouldDropGlobalIntegration(
    double & yaw_rate,
    double & vertical_speed,
    double & tilt_rate,
    double & odom_age_sec) const;
  bool shouldDropByRelativePoseConsistency(
    const rclcpp::Time & scan_stamp,
    double & translation_error_m,
    double & yaw_error_rad,
    double & motion_xy_m,
    double & map_step_xy_m,
    double & map_step_yaw_rad);
  bool maybeApplyMapFrameCorrection(
    const rclcpp::Time & stamp,
    double & correction_translation_m,
    double & correction_yaw_rad);
  void integrateGlobalCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr & scan_cloud);
  void enforceGlobalCloudLimits();
  bool alignKeyframeToSubmap(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & predicted_cloud,
    const tf2::Transform & predicted_pose,
    tf2::Transform & aligned_pose,
    pcl::PointCloud<pcl::PointXYZ>::Ptr & aligned_cloud,
    std::string & failure_reason);
  pcl::PointCloud<pcl::PointXYZ>::Ptr buildScanMatchingSubmap(
    const tf2::Vector3 & center) const;
  pcl::PointCloud<pcl::PointXYZ>::Ptr prepareScanMatchingCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_cloud,
    double voxel_leaf_size) const;
  bool maybeIntegrateAsKeyframe(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_local,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_target,
    const tf2::Transform & pose_odom_source,
    const tf2::Transform & pose_target_source,
    const rclcpp::Time & scan_stamp,
    const std::string & source_frame,
    bool deskewed,
    std::size_t input_points);
  bool rebuildGlobalCloudFromKeyframes(std::string & error_message);
  void recordTrajectoryPoint(
    const rclcpp::Time & stamp,
    const tf2::Transform & pose_target);
  std::string globalCloudFrame() const;
  void publishScanMatchingFrame(const rclcpp::Time & stamp);
  void publishGlobalMap(const rclcpp::Time & stamp);
  void publishStructuralCloud(const rclcpp::Time & stamp);
  void publishStatus();
  void onGlobalPublishTimer();
  void onStatusTimer();
  void handleSavePcdRequest(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void handleRebuildGlobalRequest(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void tryAutosaveOnExit(const char * reason);
  bool saveGlobalCloudToPcd(std::string & output_path, std::string & error_message);
  bool save2DMapToPgm(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_copy,
    const std::filesystem::path & export_dir,
    int64_t stamp_sec,
    int64_t stamp_nsec,
    std::string & pgm_path,
    std::string & yaml_path,
    std::string & error_message) const;
  bool saveSlam2DMap(
    const std::filesystem::path & export_dir,
    int64_t stamp_sec,
    int64_t stamp_nsec,
    std::string & pgm_path,
    std::string & yaml_path,
    std::string & error_message) const;
  bool saveTrajectoryToCsv(
    const std::filesystem::path & export_dir,
    int64_t stamp_sec,
    int64_t stamp_nsec,
    std::string & output_path,
    std::string & error_message) const;
  bool saveStructuralModelToGlb(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_copy,
    const std::filesystem::path & export_dir,
    int64_t stamp_sec,
    int64_t stamp_nsec,
    std::string & output_path,
    std::string & error_message);
  pcl::PointCloud<pcl::PointXYZ>::Ptr voxelDownsample(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_cloud,
    double leaf_size) const;
  pcl::PointCloud<pcl::PointXYZ>::Ptr radiusOutlierFilter(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_cloud) const;

  pcl::PointCloud<pcl::PointXYZ>::Ptr buildVoxelizedMapCloud() const;
  void publishMap(const rclcpp::Time & stamp);
  void warnIfTimeMismatch(const rclcpp::Time & scan_stamp) const;

  laser_geometry::LaserProjection projector_;

  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::LaserScan>> scan_filter_sub_;
  std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>> scan_filter_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr motion_odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr slam_map_sub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr deskewed_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr structural_cloud_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_pcd_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr rebuild_global_service_;
  rclcpp::TimerBase::SharedPtr deskew_queue_timer_;
  rclcpp::TimerBase::SharedPtr global_publish_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::CreateTimerROS> tf_timer_interface_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> scan_matching_tf_broadcaster_;

  std::deque<TimedScan> scan_queue_;
  std::size_t raw_points_total_{0};
  pcl::PointCloud<pcl::PointXYZ>::Ptr global_cloud_;
  std::size_t global_points_total_{0};
  std::deque<double> tf_lookup_samples_ms_;
  std::optional<nav_msgs::msg::Odometry> motion_odom_;
  mutable std::mutex pose_buffer_mutex_;
  std::deque<PoseSample> pose_buffer_;
  std::deque<PendingScan> pending_deskew_scans_;
  mutable std::mutex slam_map_mutex_;
  std::optional<nav_msgs::msg::OccupancyGrid> latest_slam_map_;
  std::optional<rclcpp::Time> last_scan_stamp_;
  std::optional<rclcpp::Time> last_local_map_publish_stamp_;
  std::optional<rclcpp::Time> last_structural_cloud_publish_stamp_;
  std::optional<geometry_msgs::msg::TransformStamped> last_applied_map_to_odom_tf_;
  std::optional<geometry_msgs::msg::TransformStamped> last_relative_pose_map_base_tf_;
  std::optional<geometry_msgs::msg::TransformStamped> last_relative_pose_odom_base_tf_;
  std::optional<tf2::Transform> last_keyframe_pose_;
  std::optional<rclcpp::Time> last_keyframe_stamp_;
  std::optional<double> floor_reference_z_;
  std::vector<TrajectoryPoint> trajectory_points_;
  std::deque<Keyframe> keyframes_;
  std::optional<Eigen::Vector3f> last_trajectory_point_;
  tf2::Transform scan_matching_correction_;
  std::optional<rclcpp::Time> last_scan_matching_attempt_stamp_;

  std::string scan_topic_;
  std::string target_frame_;
  std::string base_frame_;
  std::string lidar_frame_override_;
  std::string map_topic_;
  std::string cloud_topic_;
  std::string deskewed_cloud_topic_;
  std::string global_map_topic_;
  std::string structural_cloud_topic_{"/mapping/structural_cloud"};
  std::string status_topic_;
  std::string motion_odom_topic_;
  std::string pcd_export_dir_;
  std::string pcd_export_prefix_;
  std::string map2d_export_prefix_;
  std::string trajectory_export_prefix_;
  std::string structural_mesh_export_prefix_;
  std::string integration_mode_{"keyframe"};
  std::string motion_odom_frame_{"odom"};
  std::string scan_stamp_reference_{"start"};
  std::string scan_matching_map_frame_{"vertical_map"};

  double voxel_leaf_{0.15};
  int max_points_{500000};
  double keep_seconds_{30.0};
  double local_map_publish_hz_{2.0};
  double min_range_{0.2};
  double max_range_{12.0};
  double tf_timeout_{0.05};
  bool enable_full_pose_deskew_{false};
  bool require_full_pose_deskew_{true};
  double pose_buffer_duration_sec_{5.0};
  double pose_interpolation_max_gap_sec_{0.20};
  double deskew_min_valid_ratio_{0.90};
  double deskew_wait_for_pose_timeout_sec_{0.35};
  double deskew_queue_poll_hz_{100.0};
  int deskew_pending_queue_size_{30};
  int deskew_max_scans_per_cycle_{2};
  bool debug_{false};
  bool exclude_floor_points_{false};
  double floor_z_max_{0.15};
  bool enable_floor_stabilization_{false};
  double floor_stabilization_percentile_{0.10};
  double floor_stabilization_band_m_{0.08};
  int floor_stabilization_min_points_{30};
  double floor_stabilization_target_z_m_{0.0};
  double floor_stabilization_max_correction_m_{0.25};
  double floor_stabilization_max_step_m_{0.15};
  double floor_stabilization_min_xy_span_m_{0.40};
  double floor_stabilization_max_tilt_rad_{0.14};
  bool enable_floor_tilt_correction_{false};
  double floor_tilt_correction_min_rad_{0.0087};
  bool drop_scan_on_floor_stabilization_failure_{true};
  bool require_floor_for_global_integration_{false};

  double keyframe_min_translation_m_{0.10};
  double keyframe_min_yaw_rad_{0.06};
  double keyframe_max_interval_sec_{0.8};
  int max_keyframes_{5000};
  double global_voxel_leaf_size_{0.12};
  int max_global_points_{1500000};
  double global_publish_hz_{2.0};
  int global_revoxelize_every_n_scans_{10};
  bool enable_global_radius_outlier_filter_{false};
  double global_outlier_radius_m_{0.20};
  int global_outlier_min_neighbors_{3};
  int global_outlier_min_points_{500};
  int global_outlier_filter_every_n_revoxelizations_{3};
  bool drop_scan_on_excess_motion_{true};
  double max_integration_yaw_rate_{0.5};
  double max_integration_vertical_speed_{0.5};
  double max_integration_tilt_rate_{0.5};
  double max_motion_odom_age_sec_{0.25};
  bool rebuild_on_map_correction_{true};
  double rebuild_correction_translation_threshold_m_{0.03};
  double rebuild_correction_yaw_threshold_rad_{0.01};
  int rebuild_freeze_scans_after_correction_{10};
  int rebuild_freeze_remaining_scans_{0};
  bool tilt_compensation_{true};
  bool enable_relative_pose_gate_{true};
  bool enable_map_rebase_{true};
  bool enable_scan_matching_{false};
  bool scan_matching_drop_on_failure_{false};
  bool scan_matching_lock_valid_{false};
  bool pcd_export_binary_{true};
  bool autosave_on_exit_{true};
  bool export_map2d_on_save_{true};
  bool export_slam_map2d_on_save_{true};
  bool export_trajectory_on_save_{true};
  bool export_structural_mesh_on_save_{false};
  bool structural_mesh_auto_height_{true};
  bool structural_mesh_include_ceiling_{false};
  bool structural_mesh_use_obstacle_heights_{false};
  bool enable_structural_cloud_{false};
  double structural_cloud_publish_hz_{1.0};
  double structural_cloud_resolution_m_{0.15};
  double structural_cloud_vertical_resolution_m_{0.15};
  int structural_cloud_max_points_{200000};
  bool loop_closure_dedup_only_{false};
  double map2d_resolution_{0.10};
  double map2d_padding_m_{1.0};
  double trajectory_min_step_{0.05};
  int structural_mesh_occupied_threshold_{65};
  int structural_mesh_free_threshold_{19};
  double structural_mesh_default_floor_z_{0.0};
  double structural_mesh_default_ceiling_z_{2.5};
  double structural_mesh_floor_quantile_{0.02};
  double structural_mesh_ceiling_quantile_{0.98};
  double structural_mesh_min_room_height_{1.8};
  double structural_mesh_max_room_height_{4.0};
  double structural_mesh_min_obstacle_height_{0.25};
  double structural_mesh_obstacle_height_padding_{0.05};
  double structural_mesh_height_quantization_{0.10};
  int structural_mesh_height_search_radius_cells_{1};
  int structural_mesh_max_grid_cells_{2000000};
  int structural_mesh_max_height_samples_{200000};
  int structural_mesh_max_quads_{250000};
  std::string slam_map_topic_{"/map"};
  std::string slam_map2d_export_prefix_{"slam2d_map"};
  std::string map_rebase_map_frame_{"map"};
  std::string map_rebase_odom_frame_{"odom"};
  std::string relative_pose_map_frame_{"map"};
  std::string relative_pose_odom_frame_{"odom"};
  double map_rebase_translation_threshold_{0.03};
  double map_rebase_yaw_threshold_{0.01};
  double relative_pose_translation_error_threshold_{0.08};
  double relative_pose_yaw_error_threshold_{0.06};
  double relative_pose_min_motion_xy_{0.02};
  double relative_pose_max_map_step_xy_{0.50};
  double relative_pose_max_map_yaw_step_{0.25};
  double scan_matching_max_rate_hz_{4.0};
  double scan_matching_submap_radius_m_{4.0};
  double scan_matching_submap_half_height_m_{3.0};
  double scan_matching_voxel_leaf_size_m_{0.12};
  double scan_matching_max_correspondence_distance_m_{0.35};
  double scan_matching_min_overlap_ratio_{0.35};
  double scan_matching_max_rmse_m_{0.16};
  double scan_matching_max_translation_correction_m_{0.20};
  double scan_matching_max_yaw_correction_rad_{0.20};
  double scan_matching_max_z_correction_m_{0.08};
  double scan_matching_max_tilt_correction_rad_{0.08};
  double scan_matching_min_height_above_floor_m_{0.15};
  int scan_matching_max_iterations_{20};
  int scan_matching_max_submap_points_{12000};
  int scan_matching_min_source_points_{80};
  int scan_matching_min_submap_points_{400};
  int scan_matching_min_correspondences_{50};
  int scan_matching_warmup_keyframes_{8};
  int map_rebase_cooldown_scans_{10};
  int map_rebase_cooldown_remaining_scans_{0};
  int tf_filter_queue_size_{50};
  std::size_t tf_lookup_window_size_{200};
  std::size_t tf_failures_{0};
  std::size_t tf_filter_drop_count_{0};
  std::size_t dropped_excess_motion_count_{0};
  std::size_t dropped_stale_odom_count_{0};
  std::size_t dropped_yaw_motion_count_{0};
  std::size_t dropped_vertical_motion_count_{0};
  std::size_t dropped_tilt_motion_count_{0};
  std::size_t global_integrations_since_revoxel_{0};
  std::size_t global_revoxelization_count_{0};
  std::size_t total_scans_seen_{0};
  std::size_t total_scans_processed_{0};
  std::size_t total_scans_global_integrated_{0};
  std::size_t total_scans_keyframe_integrated_{0};
  std::size_t keyframe_drop_non_keyframe_count_{0};
  std::size_t keyframe_eviction_count_{0};
  std::size_t floor_points_dropped_{0};
  std::size_t floor_stabilization_corrections_{0};
  std::size_t floor_stabilization_estimate_failures_{0};
  std::size_t floor_stabilization_rejections_{0};
  std::size_t floor_tilt_corrections_{0};
  std::size_t floor_tilt_correction_failures_{0};
  std::size_t dropped_floor_unstable_count_{0};
  std::size_t global_outlier_filter_runs_{0};
  std::size_t global_outlier_points_removed_{0};
  std::size_t structural_cloud_publish_count_{0};
  std::size_t last_structural_cloud_points_{0};
  double last_structural_cloud_duration_ms_{0.0};
  double last_observed_floor_z_{0.0};
  double last_floor_residual_m_{0.0};
  double last_floor_correction_m_{0.0};
  double last_floor_tilt_rad_{0.0};
  double last_floor_residual_tilt_rad_{0.0};
  double last_floor_tilt_correction_rad_{0.0};
  double floor_stabilization_bias_m_{0.0};
  bool floor_stabilization_initialized_{false};
  std::size_t pcd_export_count_{0};
  bool autosave_completed_{false};
  std::string last_pcd_export_path_;
  std::size_t map2d_export_count_{0};
  std::size_t trajectory_export_count_{0};
  std::string last_map2d_export_path_;
  std::string last_map2d_yaml_path_;
  std::size_t slam_map2d_export_count_{0};
  std::string last_slam_map2d_export_path_;
  std::string last_slam_map2d_yaml_path_;
  std::string last_trajectory_export_path_;
  std::size_t structural_mesh_export_count_{0};
  std::size_t last_structural_mesh_vertices_{0};
  std::size_t last_structural_mesh_triangles_{0};
  std::size_t last_structural_mesh_bytes_{0};
  double last_structural_mesh_floor_z_{0.0};
  double last_structural_mesh_ceiling_z_{0.0};
  double last_structural_mesh_duration_ms_{0.0};
  std::string last_structural_mesh_export_path_;
  std::string last_structural_mesh_export_error_;
  std::size_t map_rebase_count_{0};
  double last_map_rebase_translation_m_{0.0};
  double last_map_rebase_yaw_rad_{0.0};
  std::size_t rebuild_count_{0};
  double rebuild_last_duration_ms_{0.0};
  std::size_t relative_pose_gate_drop_count_{0};
  std::size_t relative_pose_gate_lookup_failures_{0};
  std::size_t scan_matching_attempt_count_{0};
  std::size_t scan_matching_accept_count_{0};
  std::size_t scan_matching_reject_count_{0};
  std::size_t scan_matching_drop_count_{0};
  std::size_t scan_matching_insufficient_submap_count_{0};
  std::size_t scan_matching_nonconverged_count_{0};
  std::size_t scan_matching_quality_reject_count_{0};
  std::size_t scan_matching_bounds_reject_count_{0};
  std::size_t last_scan_matching_source_points_{0};
  std::size_t last_scan_matching_submap_points_{0};
  std::size_t last_scan_matching_correspondences_{0};
  double last_scan_matching_overlap_ratio_{0.0};
  double last_scan_matching_rmse_m_{0.0};
  double last_scan_matching_correction_translation_m_{0.0};
  double last_scan_matching_correction_yaw_rad_{0.0};
  double last_scan_matching_duration_ms_{0.0};
  std::string last_scan_matching_status_{"disabled"};
  double last_relative_pose_translation_error_m_{0.0};
  double last_relative_pose_yaw_error_rad_{0.0};
  double last_relative_pose_motion_xy_m_{0.0};
  double last_relative_pose_map_step_xy_m_{0.0};
  double last_relative_pose_map_step_yaw_rad_{0.0};
  std::size_t map_rebase_cooldown_drop_count_{0};
  std::size_t rebuild_freeze_drop_count_{0};
  std::uint64_t next_keyframe_id_{1};
  std::size_t accepted_scan_count_{0};
  std::size_t deskewed_scan_count_{0};
  std::size_t deskew_failure_count_{0};
  std::size_t pose_interpolation_failure_count_{0};
  std::size_t deskew_pose_wait_timeout_count_{0};
  std::size_t deskew_pose_history_miss_count_{0};
  std::size_t deskew_queue_overflow_count_{0};
  std::size_t deskew_queue_max_depth_{0};
  std::size_t deskewed_points_total_{0};
  std::size_t last_deskewed_point_count_{0};
  std::size_t last_valid_input_point_count_{0};
  std::size_t last_local_voxel_point_count_{0};
  double last_scan_age_sec_{0.0};
  double last_pose_bracket_gap_sec_{0.0};
  double last_pose_buffer_oldest_age_sec_{0.0};
  double last_pose_buffer_newest_age_sec_{0.0};
  double last_deskew_pose_wait_sec_{0.0};
  double max_deskew_pose_wait_sec_{0.0};
  double last_deskew_pose_lag_sec_{0.0};
  double last_motion_yaw_rate_{0.0};
  double last_motion_vertical_speed_{0.0};
  double last_motion_tilt_rate_{0.0};
  double last_motion_odom_age_sec_{0.0};
  double last_motion_roll_rad_{0.0};
  double last_motion_pitch_rad_{0.0};
  std::string last_scan_drop_reason_{"none"};
  std::optional<rclcpp::Time> last_status_stamp_;
  std::size_t last_status_scans_seen_{0};
  std::size_t last_status_scans_accepted_{0};
  std::size_t last_status_keyframes_{0};
};

}  // namespace vertical_lidar_mapper

#endif  // VERTICAL_LIDAR_MAPPER__VERTICAL_LIDAR_MAPPER_HPP_
