#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/LinearMath/Transform.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.hpp"

namespace {
inline bool starts_with(const std::string &value, const std::string &prefix) {
  return value.rfind(prefix, 0) == 0;
}
}  // namespace

class AprilTagPrecisionLandingNode : public rclcpp::Node {
 public:
  AprilTagPrecisionLandingNode() : Node("apriltag_precision_landing") {
    tf_topic_ = declare_parameter<std::string>("tf_topic", "/tf");
    camera_tag_pose_topic_ = declare_parameter<std::string>("camera_tag_pose_topic", "/precision_landing/tag_pose_camera");
    relay_image_stream_ = declare_parameter<bool>("relay_image_stream", true);
    image_input_topic_ = declare_parameter<std::string>("image_input_topic", "/camera/image_raw");
    image_output_topic_ = declare_parameter<std::string>("image_output_topic", "/image_raw");
    drone_pose_topic_ = declare_parameter<std::string>("drone_pose_topic", "/mavros/local_position/pose");
    landing_target_topic_ = declare_parameter<std::string>("landing_target_topic", "/mavros/landing_target/pose");
    world_frame_ = declare_parameter<std::string>("world_frame", "map");
    drone_frame_ = declare_parameter<std::string>("drone_frame", "base_link");
    camera_mount_frame_ = declare_parameter<std::string>("camera_mount_frame", "camera_link");
    camera_optical_frame_ = declare_parameter<std::string>("camera_optical_frame", "camera_optical_frame");
    tag_detection_frame_ = declare_parameter<std::string>("tag_detection_frame", "apriltag_detection");
    tag_world_frame_ = declare_parameter<std::string>("tag_world_frame", "apriltag_world");

    input_mode_ = declare_parameter<std::string>("input_mode", "camera_pose");
    output_mode_ = declare_parameter<std::string>("output_mode", "camera_pose");
    tag_frame_prefix_ = declare_parameter<std::string>("tag_frame_prefix", "id");
    tag_frame_exact_ = declare_parameter<std::string>("tag_frame_exact", "");

    const double publish_rate_hz = declare_parameter<double>("publish_rate_hz", 20.0);
    input_timeout_sec_ = declare_parameter<double>("input_timeout_sec", 0.30);
    normalize_input_stamps_ = declare_parameter<bool>("normalize_input_stamps", false);
    fixed_target_ned_x_ = declare_parameter<double>("fixed_target_ned_x", 0.0);
    fixed_target_ned_y_ = declare_parameter<double>("fixed_target_ned_y", 0.0);
    fixed_target_ned_z_ = declare_parameter<double>("fixed_target_ned_z", 0.0);
    fixed_target_frame_ = declare_parameter<std::string>("fixed_target_frame", "map");

    camera_offset_x_ = declare_parameter<double>("camera_offset_x", 0.10);
    camera_offset_y_ = declare_parameter<double>("camera_offset_y", 0.0);
    camera_offset_z_ = declare_parameter<double>("camera_offset_z", -0.06);
    camera_roll_ = declare_parameter<double>("camera_roll", 0.0);
    camera_pitch_ = declare_parameter<double>("camera_pitch", M_PI);
    camera_yaw_ = declare_parameter<double>("camera_yaw", M_PI_2);
    publish_camera_mount_to_optical_tf_ =
        declare_parameter<bool>("publish_camera_mount_to_optical_tf", false);
    optical_roll_ = declare_parameter<double>("optical_roll", 0.0);
    optical_pitch_ = declare_parameter<double>("optical_pitch", 0.0);
    optical_yaw_ = declare_parameter<double>("optical_yaw", 0.0);

    publish_world_to_drone_tf_ = declare_parameter<bool>("publish_world_to_drone_tf", true);
    publish_static_camera_tf_ = declare_parameter<bool>("publish_static_camera_tf", true);
    publish_camera_tag_tf_ = declare_parameter<bool>("publish_camera_tag_tf", true);
    publish_debug_tf_ = declare_parameter<bool>("publish_debug_tf", true);

    tf2::Quaternion q_cam_optical_from_drone;
    q_cam_optical_from_drone.setRPY(camera_roll_, camera_pitch_, camera_yaw_);
    drone_t_camera_optical_.setOrigin(tf2::Vector3(camera_offset_x_, camera_offset_y_, camera_offset_z_));
    drone_t_camera_optical_.setRotation(q_cam_optical_from_drone);

    tf2::Quaternion q_cam_optical;
    q_cam_optical.setRPY(optical_roll_, optical_pitch_, optical_yaw_);
    camera_mount_t_optical_.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
    camera_mount_t_optical_.setRotation(q_cam_optical);

    const auto qos_sensor = rclcpp::SensorDataQoS();

    if (relay_image_stream_) {
      if (image_input_topic_ == image_output_topic_) {
        RCLCPP_INFO(get_logger(),
                    "image relay enabled but input==output (%s), skipping relay to avoid topic loop",
                    image_output_topic_.c_str());
      } else {
        pub_image_relay_ = create_publisher<sensor_msgs::msg::Image>(image_output_topic_, qos_sensor);
        sub_image_relay_ = create_subscription<sensor_msgs::msg::Image>(
            image_input_topic_, qos_sensor,
            std::bind(&AprilTagPrecisionLandingNode::imageRelayCb, this, std::placeholders::_1));
        RCLCPP_INFO(get_logger(), "Relaying image stream %s -> %s",
                    image_input_topic_.c_str(), image_output_topic_.c_str());
      }
    }

    if (input_mode_ == "tf" || input_mode_ == "auto") {
      sub_tf_ = create_subscription<tf2_msgs::msg::TFMessage>(
          tf_topic_, qos_sensor,
          std::bind(&AprilTagPrecisionLandingNode::tfCallback, this, std::placeholders::_1));
    }

    if (input_mode_ == "camera_pose" || input_mode_ == "auto") {
      sub_camera_tag_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
          camera_tag_pose_topic_, qos_sensor,
          std::bind(&AprilTagPrecisionLandingNode::cameraTagPoseCb, this, std::placeholders::_1));
    }

    sub_drone_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        drone_pose_topic_, qos_sensor,
        std::bind(&AprilTagPrecisionLandingNode::dronePoseCallback, this, std::placeholders::_1));

    pub_landing_target_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>(landing_target_topic_, 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
    publishStaticTransforms();

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz));
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        std::bind(&AprilTagPrecisionLandingNode::publishLandingPose, this));
    diagnostics_timer_ = create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&AprilTagPrecisionLandingNode::publishDiagnostics, this));

    RCLCPP_INFO(get_logger(),
                "apriltag_precision_landing started input_mode=%s output_mode=%s tf_topic=%s camera_tag_pose_topic=%s use_sim_time=%s transform=direct_composition",
                input_mode_.c_str(), output_mode_.c_str(), tf_topic_.c_str(), camera_tag_pose_topic_.c_str(),
                useSimTime() ? "true" : "false");
  }

 private:
  bool frameMatchesTag(const std::string &frame_id) const {
    if (!tag_frame_exact_.empty()) {
      return frame_id == tag_frame_exact_;
    }
    if (tag_frame_prefix_.empty()) {
      return true;
    }
    return starts_with(frame_id, tag_frame_prefix_);
  }

  rclcpp::Time stampOrNow(const builtin_interfaces::msg::Time &stamp) const {
    const auto now_time = now();
    const rclcpp::Time stamp_time(stamp);
    if (stamp_time.nanoseconds() == 0) {
      return now_time;
    }
    if (normalize_input_stamps_ &&
        std::fabs((now_time - stamp_time).seconds()) > input_timeout_sec_) {
      return now_time;
    }
    return stamp_time;
  }

  geometry_msgs::msg::TransformStamped makeTransformStamped(
      const std::string &parent,
      const std::string &child,
      const tf2::Transform &tf,
      const rclcpp::Time &stamp) const {
    geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = parent;
    msg.child_frame_id = child;
    msg.transform.translation.x = tf.getOrigin().x();
    msg.transform.translation.y = tf.getOrigin().y();
    msg.transform.translation.z = tf.getOrigin().z();
    msg.transform.rotation.x = tf.getRotation().x();
    msg.transform.rotation.y = tf.getRotation().y();
    msg.transform.rotation.z = tf.getRotation().z();
    msg.transform.rotation.w = tf.getRotation().w();
    return msg;
  }

  void publishStaticTransforms() {
    if (!publish_static_camera_tf_ || !static_tf_broadcaster_) {
      return;
    }

    const auto stamp = now();
    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    transforms.push_back(makeTransformStamped(
        drone_frame_, camera_optical_frame_, drone_t_camera_optical_, stamp));
    if (camera_mount_frame_ != camera_optical_frame_) {
      transforms.push_back(makeTransformStamped(
          drone_frame_, camera_mount_frame_, drone_t_camera_optical_, stamp));
    }
    if (publish_camera_mount_to_optical_tf_) {
      transforms.push_back(makeTransformStamped(
          camera_mount_frame_, camera_optical_frame_, camera_mount_t_optical_, stamp));
    }
    static_tf_broadcaster_->sendTransform(transforms);
  }

  void dronePoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    ++drone_pose_input_count_;
    tf2::Quaternion q(msg->pose.orientation.x,
                      msg->pose.orientation.y,
                      msg->pose.orientation.z,
                      msg->pose.orientation.w);
    tf2::Vector3 t(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    world_t_drone_ = tf2::Transform(q, t);

    if (!msg->header.frame_id.empty()) {
      world_frame_from_pose_ = msg->header.frame_id;
    }

    got_world_t_drone_ = true;
    last_drone_pose_time_ = stampOrNow(msg->header.stamp);
  }

  void imageRelayCb(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (!pub_image_relay_) {
      return;
    }
    auto out = *msg;
    if (out.header.frame_id.empty()) {
      out.header.frame_id = camera_optical_frame_;
    }
    pub_image_relay_->publish(out);
  }

  void cameraTagPoseCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    const rclcpp::Time tag_time = stampOrNow(msg->header.stamp);
    if (!recordTagInput(tag_time)) {
      return;
    }

    tf2::Quaternion q(msg->pose.orientation.x,
                      msg->pose.orientation.y,
                      msg->pose.orientation.z,
                      msg->pose.orientation.w);
    tf2::Vector3 t(msg->pose.position.x,
                   msg->pose.position.y,
                   msg->pose.position.z);

    cam_t_tag_ = tf2::Transform(q, t);
    got_cam_t_tag_ = true;
    last_tag_time_ = tag_time;
    input_source_ = "camera_pose";
    camera_frame_id_ = msg->header.frame_id.empty() ? camera_optical_frame_ : msg->header.frame_id;
    tag_frame_id_ = tag_frame_exact_.empty() ? tag_detection_frame_ : tag_frame_exact_;
  }

  void tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg) {
    if (msg->transforms.empty()) {
      return;
    }

    for (const auto &tr : msg->transforms) {
      if (!frameMatchesTag(tr.child_frame_id)) {
        continue;
      }

      const rclcpp::Time tag_time = stampOrNow(tr.header.stamp);
      if (!recordTagInput(tag_time)) {
        continue;
      }

      tf2::Quaternion q(tr.transform.rotation.x,
                        tr.transform.rotation.y,
                        tr.transform.rotation.z,
                        tr.transform.rotation.w);
      tf2::Vector3 t(tr.transform.translation.x,
                     tr.transform.translation.y,
                     tr.transform.translation.z);

      cam_t_tag_ = tf2::Transform(q, t);
      got_cam_t_tag_ = true;
      last_tag_time_ = tag_time;
      input_source_ = "tf";
      tag_frame_id_ = tr.child_frame_id;
      camera_frame_id_ = tr.header.frame_id.empty() ? camera_optical_frame_ : tr.header.frame_id;
      return;
    }
  }

  bool stale(const rclcpp::Time &stamp) const {
    return (now() - stamp).seconds() > input_timeout_sec_;
  }

  void publishLandingPose() {
    if (!got_cam_t_tag_) {
      ++missing_tag_drop_count_;
      return;
    }

    if (stale(last_tag_time_)) {
      ++stale_tag_drop_count_;
      if (!tag_stale_active_) {
        ++stale_tag_episode_count_;
        tag_stale_active_ = true;
      }
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Tag pose stale (timeout %.2fs).", input_timeout_sec_);
      return;
    }

    if (!tag_update_pending_) {
      ++duplicate_publish_suppressed_count_;
      return;
    }

    if (output_mode_ != "fixed_local_ned" && !got_world_t_drone_) {
      ++missing_drone_pose_drop_count_;
      return;
    }

    if (output_mode_ != "fixed_local_ned" && stale(last_drone_pose_time_)) {
      ++stale_drone_pose_drop_count_;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Drone pose stale (timeout %.2fs).", input_timeout_sec_);
      return;
    }

    // Preserve the camera detection stamp. One image detection must never be
    // presented downstream as several newer visual measurements.
    const rclcpp::Time publish_stamp = last_tag_time_;
    const std::string world_frame = output_mode_ == "fixed_local_ned"
                                        ? fixed_target_frame_
                                        : (world_frame_from_pose_.empty() ? world_frame_ : world_frame_from_pose_);
    tf2::Transform world_t_tag;
    if (output_mode_ != "fixed_local_ned") {
      world_t_tag = world_t_drone_ * drone_t_camera_optical_ * cam_t_tag_;
    }

    geometry_msgs::msg::PoseStamped out;
    out.header.stamp = publish_stamp;
    out.header.frame_id = world_frame;

    if (output_mode_ == "fixed_local_ned") {
      // MAVROS PoseStamped landing_target input is ROS ENU. Convert the desired
      // PX4 local NED target into ROS ENU before publishing to MAVROS.
      out.pose.position.x = fixed_target_ned_y_;
      out.pose.position.y = fixed_target_ned_x_;
      out.pose.position.z = -fixed_target_ned_z_;
      out.pose.orientation.x = 0.0;
      out.pose.orientation.y = 0.0;
      out.pose.orientation.z = 0.0;
      out.pose.orientation.w = 1.0;
    } else {
      out.pose.position.x = world_t_tag.getOrigin().x();
      out.pose.position.y = world_t_tag.getOrigin().y();
      out.pose.position.z = world_t_tag.getOrigin().z();
      out.pose.orientation.x = world_t_tag.getRotation().x();
      out.pose.orientation.y = world_t_tag.getRotation().y();
      out.pose.orientation.z = world_t_tag.getRotation().z();
      out.pose.orientation.w = world_t_tag.getRotation().w();
    }

    if (!poseFinite(out.pose)) {
      ++invalid_transform_count_;
      tag_update_pending_ = false;
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                            "Rejecting non-finite landing-target transform output.");
      return;
    }

    pub_landing_target_pose_->publish(out);
    tag_update_pending_ = false;
    ++landing_target_output_count_;
    ++transform_success_count_;
    last_output_stamp_ = publish_stamp;
    const double output_latency_ms = std::max(0.0, (now() - publish_stamp).seconds() * 1000.0);
    last_output_latency_ms_ = output_latency_ms;
    max_output_latency_ms_ = std::max(max_output_latency_ms_, output_latency_ms);
    output_latency_total_ms_ += output_latency_ms;
    if (!have_output_bounds_) {
      min_output_x_ = max_output_x_ = out.pose.position.x;
      min_output_y_ = max_output_y_ = out.pose.position.y;
      have_output_bounds_ = true;
    } else {
      min_output_x_ = std::min(min_output_x_, out.pose.position.x);
      max_output_x_ = std::max(max_output_x_, out.pose.position.x);
      min_output_y_ = std::min(min_output_y_, out.pose.position.y);
      max_output_y_ = std::max(max_output_y_, out.pose.position.y);
    }

    std::vector<geometry_msgs::msg::TransformStamped> tf_msgs;
    if (publish_world_to_drone_tf_ && got_world_t_drone_) {
      tf_msgs.push_back(makeTransformStamped(
          world_frame, drone_frame_, world_t_drone_, publish_stamp));
    }
    if (publish_camera_tag_tf_) {
      const std::string camera_parent = camera_frame_id_.empty() ? camera_optical_frame_ : camera_frame_id_;
      const std::string tag_child = tag_frame_id_.empty() ? tag_detection_frame_ : tag_frame_id_;
      tf_msgs.push_back(makeTransformStamped(camera_parent, tag_child, cam_t_tag_, publish_stamp));
    }
    if (publish_debug_tf_ && output_mode_ != "fixed_local_ned") {
      tf_msgs.push_back(makeTransformStamped(
          world_frame, tag_world_frame_, world_t_tag, publish_stamp));
    }
    if (!tf_msgs.empty()) {
      tf_broadcaster_->sendTransform(tf_msgs);
    }

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                         "Target source=%s output_mode=%s optical_frame=%s world_frame=%s ros_enu=[%.3f %.3f %.3f] px4_ned=[%.3f %.3f %.3f]",
                         input_source_.c_str(),
                         output_mode_.c_str(),
                         (camera_frame_id_.empty() ? camera_optical_frame_.c_str() : camera_frame_id_.c_str()),
                         world_frame.c_str(),
                         out.pose.position.x, out.pose.position.y, out.pose.position.z,
                         out.pose.position.y, out.pose.position.x, -out.pose.position.z);
  }

  bool recordTagInput(const rclcpp::Time &stamp) {
    ++tag_input_count_;
    if (previous_tag_input_stamp_.nanoseconds() != 0) {
      if (stamp == previous_tag_input_stamp_) {
        ++duplicate_tag_stamp_count_;
        return false;
      } else if (stamp < previous_tag_input_stamp_) {
        ++nonmonotonic_tag_stamp_count_;
        return false;
      }
      longest_tag_gap_sec_ = std::max(
          longest_tag_gap_sec_, (stamp - previous_tag_input_stamp_).seconds());
    }
    previous_tag_input_stamp_ = stamp;
    tag_stale_active_ = false;
    tag_update_pending_ = true;
    return true;
  }

  bool useSimTime() const {
    bool use_sim_time = false;
    get_parameter("use_sim_time", use_sim_time);
    return use_sim_time;
  }

  bool poseFinite(const geometry_msgs::msg::Pose &pose) const {
    return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
           std::isfinite(pose.position.z) && std::isfinite(pose.orientation.x) &&
           std::isfinite(pose.orientation.y) && std::isfinite(pose.orientation.z) &&
           std::isfinite(pose.orientation.w);
  }

  void publishDiagnostics() {
    const auto current_time = now();
    const double tag_age = last_tag_time_.nanoseconds() == 0
                               ? std::numeric_limits<double>::infinity()
                               : std::max(0.0, (current_time - last_tag_time_).seconds());
    const double drone_pose_age = last_drone_pose_time_.nanoseconds() == 0
                                      ? std::numeric_limits<double>::infinity()
                                      : std::max(0.0, (current_time - last_drone_pose_time_).seconds());
    const double average_output_latency_ms = landing_target_output_count_ == 0
                                                 ? 0.0
                                                 : output_latency_total_ms_ /
                                                       static_cast<double>(landing_target_output_count_);

    RCLCPP_INFO(
        get_logger(),
        "LANDING_PIPELINE_DIAG use_sim_time=%s tag_input_hz=%llu drone_pose_hz=%llu local_target_output_hz=%llu mavros_input_topic_publish_hz=%llu mavros_serial_tx=not_observed "
        "last_detection_age_s=%.3f longest_detection_gap_s=%.3f last_detection_stamp_ns=%lld last_output_stamp_ns=%lld "
        "drone_pose_age_s=%.3f output_latency_ms[last=%.1f,avg=%.1f,max=%.1f] "
        "transform=direct tf_lookup=not_used success_hz=%llu tf_failure_total=%llu stale_episodes=%llu "
        "local_xy_spread_enu_m=[%.3f,%.3f] "
        "drop_total[no_tag=%llu,stale_tag=%llu,no_drone_pose=%llu,stale_drone_pose=%llu] duplicate_publish_suppressed=%llu "
        "stamp_total[duplicate=%llu,nonmonotonic=%llu]",
        useSimTime() ? "true" : "false",
        static_cast<unsigned long long>(tag_input_count_ - previous_tag_input_count_),
        static_cast<unsigned long long>(drone_pose_input_count_ - previous_drone_pose_input_count_),
        static_cast<unsigned long long>(landing_target_output_count_ - previous_landing_target_output_count_),
        static_cast<unsigned long long>(landing_target_output_count_ - previous_landing_target_output_count_),
        tag_age, longest_tag_gap_sec_,
        static_cast<long long>(last_tag_time_.nanoseconds()),
        static_cast<long long>(last_output_stamp_.nanoseconds()),
        drone_pose_age, last_output_latency_ms_, average_output_latency_ms, max_output_latency_ms_,
        static_cast<unsigned long long>(transform_success_count_ - previous_transform_success_count_),
        static_cast<unsigned long long>(invalid_transform_count_),
        static_cast<unsigned long long>(stale_tag_episode_count_),
        have_output_bounds_ ? max_output_x_ - min_output_x_ : 0.0,
        have_output_bounds_ ? max_output_y_ - min_output_y_ : 0.0,
        static_cast<unsigned long long>(missing_tag_drop_count_),
        static_cast<unsigned long long>(stale_tag_drop_count_),
        static_cast<unsigned long long>(missing_drone_pose_drop_count_),
        static_cast<unsigned long long>(stale_drone_pose_drop_count_),
        static_cast<unsigned long long>(duplicate_publish_suppressed_count_),
        static_cast<unsigned long long>(duplicate_tag_stamp_count_),
        static_cast<unsigned long long>(nonmonotonic_tag_stamp_count_));

    previous_tag_input_count_ = tag_input_count_;
    previous_drone_pose_input_count_ = drone_pose_input_count_;
    previous_landing_target_output_count_ = landing_target_output_count_;
    previous_transform_success_count_ = transform_success_count_;
  }

  std::string tf_topic_;
  std::string camera_tag_pose_topic_;
  bool relay_image_stream_{true};
  std::string image_input_topic_;
  std::string image_output_topic_;
  std::string drone_pose_topic_;
  std::string landing_target_topic_;
  std::string world_frame_;
  std::string drone_frame_;
  std::string camera_mount_frame_;
  std::string camera_optical_frame_;
  std::string tag_detection_frame_;
  std::string tag_world_frame_;
  std::string input_mode_;
  std::string output_mode_;
  std::string input_source_{"none"};
  std::string tag_frame_prefix_;
  std::string tag_frame_exact_;

  double input_timeout_sec_{0.30};
  bool normalize_input_stamps_{false};
  double fixed_target_ned_x_{0.0};
  double fixed_target_ned_y_{0.0};
  double fixed_target_ned_z_{0.0};
  std::string fixed_target_frame_;
  double camera_offset_x_{0.0};
  double camera_offset_y_{0.0};
  double camera_offset_z_{0.0};
  double camera_roll_{0.0};
  double camera_pitch_{M_PI};
  double camera_yaw_{M_PI_2};
  bool publish_camera_mount_to_optical_tf_{false};
  double optical_roll_{0.0};
  double optical_pitch_{0.0};
  double optical_yaw_{0.0};

  bool publish_world_to_drone_tf_{true};
  bool publish_static_camera_tf_{true};
  bool publish_camera_tag_tf_{true};
  bool publish_debug_tf_{true};
  bool got_world_t_drone_{false};
  bool got_cam_t_tag_{false};

  std::string world_frame_from_pose_;
  std::string tag_frame_id_;
  std::string camera_frame_id_;

  tf2::Transform world_t_drone_;
  tf2::Transform cam_t_tag_;
  tf2::Transform drone_t_camera_optical_;
  tf2::Transform camera_mount_t_optical_;

  rclcpp::Time last_drone_pose_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_tag_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time previous_tag_input_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_output_stamp_{0, 0, RCL_ROS_TIME};

  uint64_t tag_input_count_{0};
  uint64_t drone_pose_input_count_{0};
  uint64_t landing_target_output_count_{0};
  uint64_t transform_success_count_{0};
  uint64_t previous_tag_input_count_{0};
  uint64_t previous_drone_pose_input_count_{0};
  uint64_t previous_landing_target_output_count_{0};
  uint64_t previous_transform_success_count_{0};
  uint64_t missing_tag_drop_count_{0};
  uint64_t stale_tag_drop_count_{0};
  uint64_t stale_tag_episode_count_{0};
  uint64_t missing_drone_pose_drop_count_{0};
  uint64_t stale_drone_pose_drop_count_{0};
  uint64_t invalid_transform_count_{0};
  uint64_t duplicate_tag_stamp_count_{0};
  uint64_t nonmonotonic_tag_stamp_count_{0};
  uint64_t duplicate_publish_suppressed_count_{0};
  bool tag_stale_active_{false};
  bool tag_update_pending_{false};
  bool have_output_bounds_{false};
  double longest_tag_gap_sec_{0.0};
  double last_output_latency_ms_{0.0};
  double max_output_latency_ms_{0.0};
  double output_latency_total_ms_{0.0};
  double min_output_x_{0.0};
  double max_output_x_{0.0};
  double min_output_y_{0.0};
  double max_output_y_{0.0};

  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sub_tf_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_relay_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_camera_tag_pose_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_drone_pose_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_relay_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_landing_target_pose_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AprilTagPrecisionLandingNode>());
  rclcpp::shutdown();
  return 0;
}
