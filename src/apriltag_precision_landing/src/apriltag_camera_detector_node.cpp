#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "cv_bridge/cv_bridge.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_srvs/srv/trigger.hpp"

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

namespace {
cv::aruco::PREDEFINED_DICTIONARY_NAME dictionaryFromString(const std::string &name) {
  if (name == "16h5") return cv::aruco::DICT_APRILTAG_16h5;
  if (name == "25h9") return cv::aruco::DICT_APRILTAG_25h9;
  if (name == "36h10") return cv::aruco::DICT_APRILTAG_36h10;
  return cv::aruco::DICT_APRILTAG_36h11;
}

double quadArea(const std::vector<cv::Point2f> &c) {
  if (c.size() != 4) return 0.0;
  return std::fabs(cv::contourArea(c));
}

double pixelAreaToSquareMeters(double area_px, double z_m, double fx, double fy) {
  if (area_px <= 0.0 || z_m <= 0.0 || fx <= 1e-9 || fy <= 1e-9) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return area_px * (z_m * z_m) / (fx * fy);
}

struct LandingTagConfig {
  int id{-1};
  double size_m{0.0};
  // Translation from this tag's centre to the common landing point, expressed
  // in the tag frame. Tags are expected to lie flat and share the same axes.
  cv::Vec3d tag_to_landing{0.0, 0.0, 0.0};
  double yaw_rad{0.0};
};

struct TagCandidate {
  int id{-1};
  size_t detection_index{0};
  double area_px{0.0};
  double reprojection_rmse_px{std::numeric_limits<double>::infinity()};
  double centre_distance_norm{std::numeric_limits<double>::infinity()};
  double score{0.0};
  cv::Vec3d landing_tvec{0.0, 0.0, 0.0};
  cv::Matx33d landing_rotation{cv::Matx33d::eye()};
};

struct PreviewTagOverlay {
  int id{-1};
  bool active{false};
  std::vector<cv::Point2f> corners;
};

cv::Vec4d rotationToQuaternion(const cv::Matx33d &r) {
  const double trace = r(0, 0) + r(1, 1) + r(2, 2);
  cv::Vec4d q;
  if (trace > 0.0) {
    const double s = std::sqrt(trace + 1.0) * 2.0;
    q = {(r(2, 1) - r(1, 2)) / s, (r(0, 2) - r(2, 0)) / s,
         (r(1, 0) - r(0, 1)) / s, 0.25 * s};
  } else if (r(0, 0) > r(1, 1) && r(0, 0) > r(2, 2)) {
    const double s = std::sqrt(1.0 + r(0, 0) - r(1, 1) - r(2, 2)) * 2.0;
    q = {0.25 * s, (r(0, 1) + r(1, 0)) / s, (r(0, 2) + r(2, 0)) / s,
         (r(2, 1) - r(1, 2)) / s};
  } else if (r(1, 1) > r(2, 2)) {
    const double s = std::sqrt(1.0 + r(1, 1) - r(0, 0) - r(2, 2)) * 2.0;
    q = {(r(0, 1) + r(1, 0)) / s, 0.25 * s, (r(1, 2) + r(2, 1)) / s,
         (r(0, 2) - r(2, 0)) / s};
  } else {
    const double s = std::sqrt(1.0 + r(2, 2) - r(0, 0) - r(1, 1)) * 2.0;
    q = {(r(0, 2) + r(2, 0)) / s, (r(1, 2) + r(2, 1)) / s, 0.25 * s,
         (r(1, 0) - r(0, 1)) / s};
  }
  const double norm = cv::norm(q);
  return norm > 1e-12 ? q / norm : cv::Vec4d(0.0, 0.0, 0.0, 1.0);
}

cv::Vec4d blendQuaternion(const cv::Vec4d &from, cv::Vec4d to, double alpha) {
  if (from.dot(to) < 0.0) {
    to = -to;
  }
  cv::Vec4d result = from * (1.0 - alpha) + to * alpha;
  const double norm = cv::norm(result);
  return norm > 1e-12 ? result / norm : to;
}
}  // namespace

class AprilTagCameraDetectorNode : public rclcpp::Node {
 public:
  AprilTagCameraDetectorNode() : Node("apriltag_camera_detector") {
    input_source_ = declare_parameter<std::string>("input_source", "device");

    image_topic_ = declare_parameter<std::string>("image_topic", "/image_raw");
    camera_info_topic_ = declare_parameter<std::string>("camera_info_topic", "/camera_info");
    image_output_topic_ = declare_parameter<std::string>("image_output_topic", "/image_raw");
    camera_info_output_topic_ = declare_parameter<std::string>("camera_info_output_topic", "/camera_info");
    publish_image_stream_ = declare_parameter<bool>("publish_image_stream", true);
    preview_start_enabled_ = declare_parameter<bool>("preview_start_enabled", false);
    preview_rate_hz_ = declare_parameter<double>("preview_rate_hz", 5.0);
    preview_width_ = declare_parameter<int>("preview_width", 320);
    preview_height_ = declare_parameter<int>("preview_height", 240);
    preview_start_service_ = declare_parameter<std::string>(
        "preview_start_service", "/precision_landing/preview/start");
    preview_stop_service_ = declare_parameter<std::string>(
        "preview_stop_service", "/precision_landing/preview/stop");
    camera_frame_id_ = declare_parameter<std::string>("camera_frame_id", "camera_optical_frame");

    video_device_ = declare_parameter<std::string>("video_device", "/dev/video0");
    device_width_ = declare_parameter<int>("device_width", 640);
    device_height_ = declare_parameter<int>("device_height", 480);
    device_fps_ = declare_parameter<double>("device_fps", 30.0);
    detect_rate_hz_ = declare_parameter<double>("detect_rate_hz", 20.0);
    capture_buffer_size_ = declare_parameter<int>("capture_buffer_size", 1);

    fx_ = declare_parameter<double>("fx", 815.850981);
    fy_ = declare_parameter<double>("fy", 812.573400);
    cx_ = declare_parameter<double>("cx", 313.820838);
    cy_ = declare_parameter<double>("cy", 232.071864);
    calibration_width_ = declare_parameter<int>("calibration_width", 640);
    calibration_height_ = declare_parameter<int>("calibration_height", 480);
    hfov_deg_ = declare_parameter<double>("hfov_deg", 78.0);
    dist_coeffs_vec_ =
    declare_parameter<std::vector<double>>(
        "dist_coeffs",
        std::vector<double>{
            0.317619,
           -1.031399,
            0.001990,
           -0.002757,
            0.0
        });

    tag_pose_topic_ = declare_parameter<std::string>("tag_pose_topic", "/precision_landing/tag_pose_camera");
    tag_corners_topic_ = declare_parameter<std::string>(
        "tag_corners_topic", "/precision_landing/tag_corners");
    tag_metadata_topic_ = declare_parameter<std::string>(
        "tag_metadata_topic", "/precision_landing/tag_detection");
    tag_size_m_ = declare_parameter<double>("tag_size_m", std::numeric_limits<double>::quiet_NaN());
    target_tag_id_ = declare_parameter<int>("target_tag_id", -1);
    min_tag_area_px_ = declare_parameter<double>("min_tag_area_px", 80.0);
    dictionary_name_ = declare_parameter<std::string>("dictionary", "36h11");
    reference_tag_id_ = declare_parameter<int>("reference_tag_id", 0);
    landing_tag_ids_ = declare_parameter<std::vector<int64_t>>(
        "landing_tag_ids", std::vector<int64_t>{});
    landing_tag_sizes_m_ = declare_parameter<std::vector<double>>(
        "landing_tag_sizes_m", std::vector<double>{});
    landing_tag_offset_x_m_ = declare_parameter<std::vector<double>>(
        "landing_tag_offset_x_m", std::vector<double>{});
    landing_tag_offset_y_m_ = declare_parameter<std::vector<double>>(
        "landing_tag_offset_y_m", std::vector<double>{});
    landing_tag_offset_z_m_ = declare_parameter<std::vector<double>>(
        "landing_tag_offset_z_m", std::vector<double>{});
    landing_tag_yaw_rad_ = declare_parameter<std::vector<double>>(
        "landing_tag_yaw_rad", std::vector<double>{});
    switch_confirm_frames_ = static_cast<int>(
        std::max<int64_t>(1, declare_parameter<int64_t>("switch_confirm_frames", 4)));
    switch_score_ratio_ = std::max(1.0, declare_parameter<double>("switch_score_ratio", 1.10));
    pose_filter_alpha_ = std::clamp(declare_parameter<double>("pose_filter_alpha", 0.45), 0.01, 1.0);
    switch_filter_alpha_ = std::clamp(declare_parameter<double>("switch_filter_alpha", 0.20), 0.01, 1.0);
    filter_reset_timeout_sec_ = std::max(0.0, declare_parameter<double>("filter_reset_timeout_sec", 1.0));
    max_filtered_step_m_ = std::max(0.0, declare_parameter<double>("max_filtered_step_m", 0.12));
    uncertainty_weight_ = std::max(0.0, declare_parameter<double>("uncertainty_weight", 0.35));
    image_center_weight_ = std::max(0.0, declare_parameter<double>("image_center_weight", 0.15));

    loadLandingPadConfiguration();

    detector_dict_ = cv::aruco::getPredefinedDictionary(dictionaryFromString(dictionary_name_));
    detector_params_ = cv::aruco::DetectorParameters::create();
    const auto qos_sensor = rclcpp::SensorDataQoS();
    pub_image_ = create_publisher<sensor_msgs::msg::Image>(image_output_topic_, qos_sensor);
    pub_camera_info_ = create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_output_topic_, qos_sensor);
    pub_tag_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>(tag_pose_topic_, 10);
    pub_tag_corners_ = create_publisher<geometry_msgs::msg::PolygonStamped>(
        tag_corners_topic_, qos_sensor);
    pub_tag_metadata_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        tag_metadata_topic_, rclcpp::QoS(10).reliable());
    preview_enabled_.store(preview_start_enabled_ && publish_image_stream_);
    preview_start_server_ = create_service<std_srvs::srv::Trigger>(
        preview_start_service_,
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          if (!publish_image_stream_) {
            response->success = false;
            response->message = "preview capability is disabled by configuration";
            return;
          }
          preview_enabled_.store(true);
          response->success = true;
          response->message = "camera preview enabled";
        });
    preview_stop_server_ = create_service<std_srvs::srv::Trigger>(
        preview_stop_service_,
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          preview_enabled_.store(false);
          response->success = true;
          response->message = "camera preview disabled";
        });

    if (input_source_ == "ros_topics") {
      initRosTopicMode();
    } else if (input_source_ == "device") {
      initDeviceMode();
    } else {
      RCLCPP_FATAL(get_logger(), "Unknown input_source=%s (use ros_topics or device)", input_source_.c_str());
      throw std::runtime_error("invalid input_source");
    }

    diagnostics_timer_ = create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&AprilTagCameraDetectorNode::publishDiagnostics, this));

    RCLCPP_INFO(get_logger(),
                "apriltag_camera_detector started source=%s dict=%s tags=%zu reference_id=%d min_area_px=%.1f out=%s use_sim_time=%s",
                input_source_.c_str(), dictionary_name_.c_str(), landing_tags_.size(), reference_tag_id_,
                min_tag_area_px_, tag_pose_topic_.c_str(), useSimTime() ? "true" : "false");
  }

  ~AprilTagCameraDetectorNode() override {
    capture_running_.store(false);
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }
    if (preview_thread_.joinable()) {
      preview_thread_.join();
    }
    if (cap_.isOpened()) {
      cap_.release();
    }
  }

 private:
  void loadLandingPadConfiguration() {
    if (landing_tag_ids_.empty()) {
      if (!std::isfinite(tag_size_m_) || tag_size_m_ <= 0.0) {
        throw std::runtime_error("tag_size_m must be provided and > 0.0");
      }
      const int id = target_tag_id_ >= 0 ? target_tag_id_ : reference_tag_id_;
      landing_tags_[id] = LandingTagConfig{id, tag_size_m_, {0.0, 0.0, 0.0}, 0.0};
      reference_tag_id_ = id;
      return;
    }

    const size_t count = landing_tag_ids_.size();
    const auto require_count = [count](const auto &values, const char *name) {
      if (values.size() != count) {
        throw std::runtime_error(std::string(name) + " must match landing_tag_ids length");
      }
    };
    require_count(landing_tag_sizes_m_, "landing_tag_sizes_m");
    require_count(landing_tag_offset_x_m_, "landing_tag_offset_x_m");
    require_count(landing_tag_offset_y_m_, "landing_tag_offset_y_m");
    if (landing_tag_offset_z_m_.empty()) landing_tag_offset_z_m_.assign(count, 0.0);
    if (landing_tag_yaw_rad_.empty()) landing_tag_yaw_rad_.assign(count, 0.0);
    require_count(landing_tag_offset_z_m_, "landing_tag_offset_z_m");
    require_count(landing_tag_yaw_rad_, "landing_tag_yaw_rad");

    for (size_t i = 0; i < count; ++i) {
      const int id = static_cast<int>(landing_tag_ids_[i]);
      if (landing_tag_sizes_m_[i] <= 0.0 || landing_tags_.count(id) != 0) {
        throw std::runtime_error("landing tag IDs must be unique and sizes must be positive");
      }
      landing_tags_[id] = LandingTagConfig{
          id, landing_tag_sizes_m_[i],
          {landing_tag_offset_x_m_[i], landing_tag_offset_y_m_[i], landing_tag_offset_z_m_[i]},
          landing_tag_yaw_rad_[i]};
    }
    if (landing_tags_.count(reference_tag_id_) == 0) {
      throw std::runtime_error("reference_tag_id is not present in landing_tag_ids");
    }
  }

  void initRosTopicMode() {
    const auto qos_sensor = rclcpp::SensorDataQoS();
    sub_camera_info_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic_, qos_sensor,
        std::bind(&AprilTagCameraDetectorNode::cameraInfoCb, this, std::placeholders::_1));

    sub_image_ = create_subscription<sensor_msgs::msg::Image>(
        image_topic_, qos_sensor,
        std::bind(&AprilTagCameraDetectorNode::imageCb, this, std::placeholders::_1));
    capture_running_.store(true);
    preview_thread_ = std::thread(&AprilTagCameraDetectorNode::previewLoop, this);

    RCLCPP_INFO(get_logger(), "ROS-topic mode image=%s camera_info=%s",
                image_topic_.c_str(), camera_info_topic_.c_str());
    if (publish_image_stream_ && image_output_topic_ != image_topic_) {
      RCLCPP_INFO(get_logger(), "Republishing image stream to %s", image_output_topic_.c_str());
    }
    if (publish_image_stream_ && camera_info_output_topic_ != camera_info_topic_) {
      RCLCPP_INFO(get_logger(), "Republishing camera_info to %s", camera_info_output_topic_.c_str());
    }
  }

  void initDeviceMode() {
    setupCameraModelFromParams();

    cap_.open(video_device_, cv::CAP_V4L2);
    if (!cap_.isOpened()) {
      cap_.open(video_device_);
    }

    if (!cap_.isOpened()) {
      RCLCPP_FATAL(get_logger(), "Cannot open video device: %s", video_device_.c_str());
      throw std::runtime_error("video device open failed");
    }

    if (device_width_ > 0) cap_.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(device_width_));
    if (device_height_ > 0) cap_.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(device_height_));
    if (device_fps_ > 0.0) cap_.set(cv::CAP_PROP_FPS, device_fps_);
    const bool buffer_size_set = capture_buffer_size_ > 0
                                     ? cap_.set(cv::CAP_PROP_BUFFERSIZE, static_cast<double>(capture_buffer_size_))
                                     : false;

    const double actual_width = cap_.get(cv::CAP_PROP_FRAME_WIDTH);
    const double actual_height = cap_.get(cv::CAP_PROP_FRAME_HEIGHT);
    const double actual_fps = cap_.get(cv::CAP_PROP_FPS);

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, detect_rate_hz_));
    capture_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        std::bind(&AprilTagCameraDetectorNode::processLatestFrame, this));

    capture_running_.store(true);
    capture_thread_ = std::thread(&AprilTagCameraDetectorNode::captureLoop, this);
    preview_thread_ = std::thread(&AprilTagCameraDetectorNode::previewLoop, this);

    RCLCPP_INFO(get_logger(),
                "Device mode video=%s requested=%dx%d@%.1f negotiated=%.0fx%.0f@%.1f buffer_request=%d accepted=%s capture=dedicated_thread queue=latest_only",
                video_device_.c_str(), device_width_, device_height_, device_fps_,
                actual_width, actual_height, actual_fps, capture_buffer_size_, buffer_size_set ? "true" : "false");
    if (publish_image_stream_) {
      RCLCPP_INFO(get_logger(), "Publishing device stream image=%s camera_info=%s",
                  image_output_topic_.c_str(), camera_info_output_topic_.c_str());
    }
  }

  void setupCameraModelFromParams() {
    double fx = fx_;
    double fy = fy_;
    double cx = cx_;
    double cy = cy_;

    if (fx <= 0.0 || fy <= 0.0) {
      const double width = device_width_ > 0 ? static_cast<double>(device_width_) : 640.0;
      const double hfov_rad = std::max(1.0, hfov_deg_) * M_PI / 180.0;
      const double fx_auto = width / (2.0 * std::tan(hfov_rad * 0.5));
      fx = fy = fx_auto;
      RCLCPP_WARN(get_logger(),
                  "fx/fy not provided. Using approximate intrinsics from hfov_deg=%.1f -> fx=fy=%.1f",
                  hfov_deg_, fx_auto);
    }

    if (cx <= 0.0 || cy <= 0.0) {
      const double width = device_width_ > 0 ? static_cast<double>(device_width_) : 640.0;
      const double height = device_height_ > 0 ? static_cast<double>(device_height_) : 480.0;
      cx = width * 0.5;
      cy = height * 0.5;
      RCLCPP_WARN(get_logger(),
                  "cx/cy not provided. Using image center approximation cx=%.1f cy=%.1f", cx, cy);
    }

    param_camera_matrix_ = cv::Mat::zeros(3, 3, CV_64F);
    param_camera_matrix_.at<double>(0, 0) = fx;
    param_camera_matrix_.at<double>(0, 2) = cx;
    param_camera_matrix_.at<double>(1, 1) = fy;
    param_camera_matrix_.at<double>(1, 2) = cy;
    param_camera_matrix_.at<double>(2, 2) = 1.0;

    camera_matrix_ = param_camera_matrix_.clone();

    dist_coeffs_ = cv::Mat::zeros(1, static_cast<int>(dist_coeffs_vec_.size()), CV_64F);
    for (size_t i = 0; i < dist_coeffs_vec_.size(); ++i) {
      dist_coeffs_.at<double>(0, static_cast<int>(i)) = dist_coeffs_vec_[i];
    }

    got_camera_info_ = true;
  }

  void updateScaledCameraModelForFrame(int width, int height) {
    if (input_source_ != "device" || param_camera_matrix_.empty()) {
      return;
    }

    const int calib_w = calibration_width_ > 0 ? calibration_width_ : std::max(1, device_width_);
    const int calib_h = calibration_height_ > 0 ? calibration_height_ : std::max(1, device_height_);
    const double sx = static_cast<double>(width) / static_cast<double>(calib_w);
    const double sy = static_cast<double>(height) / static_cast<double>(calib_h);

    camera_matrix_ = param_camera_matrix_.clone();
    camera_matrix_.at<double>(0, 0) *= sx;
    camera_matrix_.at<double>(0, 2) *= sx;
    camera_matrix_.at<double>(1, 1) *= sy;
    camera_matrix_.at<double>(1, 2) *= sy;

    if (width != last_frame_width_ || height != last_frame_height_) {
      RCLCPP_INFO(get_logger(),
                  "Using frame %dx%d with calibration %dx%d -> scaled fx=%.2f fy=%.2f cx=%.2f cy=%.2f",
                  width, height, calib_w, calib_h,
                  camera_matrix_.at<double>(0, 0),
                  camera_matrix_.at<double>(1, 1),
                  camera_matrix_.at<double>(0, 2),
                  camera_matrix_.at<double>(1, 2));
      last_frame_width_ = width;
      last_frame_height_ = height;
    }
  }

  void cameraInfoCb(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    ++camera_info_count_;

    if (msg->k.size() != 9) {
      ++invalid_camera_info_count_;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "CameraInfo K is invalid (size=%zu)", msg->k.size());
      return;
    }

    camera_matrix_ = cv::Mat::zeros(3, 3, CV_64F);
    camera_matrix_.at<double>(0, 0) = msg->k[0];
    camera_matrix_.at<double>(0, 1) = msg->k[1];
    camera_matrix_.at<double>(0, 2) = msg->k[2];
    camera_matrix_.at<double>(1, 0) = msg->k[3];
    camera_matrix_.at<double>(1, 1) = msg->k[4];
    camera_matrix_.at<double>(1, 2) = msg->k[5];
    camera_matrix_.at<double>(2, 0) = msg->k[6];
    camera_matrix_.at<double>(2, 1) = msg->k[7];
    camera_matrix_.at<double>(2, 2) = msg->k[8];

    dist_coeffs_ = cv::Mat::zeros(1, static_cast<int>(msg->d.size()), CV_64F);
    for (size_t i = 0; i < msg->d.size(); ++i) {
      dist_coeffs_.at<double>(0, static_cast<int>(i)) = msg->d[i];
    }

    if (!msg->header.frame_id.empty()) {
      camera_frame_id_ = msg->header.frame_id;
    }

    got_camera_info_ = true;

    if (publish_image_stream_ && camera_info_output_topic_ != camera_info_topic_) {
      auto out = *msg;
      if (out.header.frame_id.empty()) {
        out.header.frame_id = camera_frame_id_;
      }
      pub_camera_info_->publish(out);
    }
  }

  void imageCb(const sensor_msgs::msg::Image::SharedPtr msg) {
    ++camera_frame_count_;

    if (!got_camera_info_) {
      ++missing_camera_info_count_;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Waiting camera_info before AprilTag detection.");
      return;
    }

    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(msg);
    } catch (const cv_bridge::Exception &e) {
      ++image_conversion_fail_count_;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "cv_bridge exception: %s", e.what());
      return;
    }

    if (preview_enabled_.load() && pub_image_->get_subscription_count() > 0) {
      std::lock_guard<std::mutex> lock(latest_frame_mutex_);
      latest_frame_ = cv_ptr->image.clone();
      latest_frame_stamp_ = rclcpp::Time(msg->header.stamp);
      ++latest_frame_sequence_;
    }

    detectAndPublish(cv_ptr->image, rclcpp::Time(msg->header.stamp), msg->header.frame_id);
  }

  void captureLoop() {
    auto previous_capture = std::chrono::steady_clock::time_point{};

    while (capture_running_.load()) {
      if (!cap_.isOpened()) {
        ++camera_not_open_count_;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Video device not open: %s", video_device_.c_str());
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        continue;
      }

      cv::Mat frame;
      if (!cap_.read(frame) || frame.empty()) {
        ++camera_read_fail_count_;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Failed to read frame from %s", video_device_.c_str());
        continue;
      }

      const auto capture_time = std::chrono::steady_clock::now();
      const auto stamp = now();
      const uint64_t sequence = ++camera_frame_count_;
      std::lock_guard<std::mutex> lock(latest_frame_mutex_);
      latest_frame_ = frame;
      latest_frame_stamp_ = stamp;
      latest_frame_sequence_ = sequence;

      if (previous_capture != std::chrono::steady_clock::time_point{}) {
        const double gap_ms = std::chrono::duration<double, std::milli>(capture_time - previous_capture).count();
        longest_capture_gap_ms_ = std::max(longest_capture_gap_ms_, gap_ms);
      }
      previous_capture = capture_time;
    }
  }

  void previewLoop() {
    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, preview_rate_hz_));
    auto next_publish = std::chrono::steady_clock::now();
    while (capture_running_.load()) {
      if (!preview_enabled_.load() || pub_image_->get_subscription_count() == 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        next_publish = std::chrono::steady_clock::now();
        continue;
      }

      cv::Mat frame;
      rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
      {
        std::lock_guard<std::mutex> lock(latest_frame_mutex_);
        if (!latest_frame_.empty()) {
          frame = latest_frame_;
          stamp = latest_frame_stamp_;
        }
      }
      if (!frame.empty()) {
        publishPreviewFrame(frame, stamp, camera_frame_id_);
      }
      next_publish += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
      std::this_thread::sleep_until(next_publish);
    }
  }

  void publishPreviewFrame(
      const cv::Mat &frame, const rclcpp::Time &stamp, const std::string &frame_id) {
    cv::Mat preview;
    const cv::Size output_size(
        preview_width_ > 0 ? preview_width_ : frame.cols,
        preview_height_ > 0 ? preview_height_ : frame.rows);
    if (frame.size() == output_size) {
      preview = frame;
    } else {
      cv::resize(frame, preview, output_size, 0.0, 0.0, cv::INTER_AREA);
    }

    cv::Mat bgr;
    if (preview.channels() == 1) {
      cv::cvtColor(preview, bgr, cv::COLOR_GRAY2BGR);
    } else if (preview.channels() == 3) {
      bgr = preview.clone();
    } else if (preview.channels() == 4) {
      cv::cvtColor(preview, bgr, cv::COLOR_BGRA2BGR);
    } else {
      return;
    }

    std::vector<PreviewTagOverlay> overlays;
    std::optional<cv::Point2f> landing_point;
    int active_id = -1;
    double confidence = 0.0;
    int source_width = frame.cols;
    int source_height = frame.rows;
    {
      std::lock_guard<std::mutex> lock(preview_overlay_mutex_);
      overlays = preview_overlays_;
      landing_point = preview_landing_point_;
      active_id = preview_active_tag_id_;
      confidence = preview_confidence_;
      source_width = std::max(1, preview_source_width_);
      source_height = std::max(1, preview_source_height_);
    }
    const float sx = static_cast<float>(bgr.cols) / source_width;
    const float sy = static_cast<float>(bgr.rows) / source_height;
    for (const auto &overlay : overlays) {
      std::vector<cv::Point> polygon;
      for (const auto &point : overlay.corners) {
        polygon.emplace_back(cvRound(point.x * sx), cvRound(point.y * sy));
      }
      const cv::Scalar color = overlay.active ? cv::Scalar(40, 220, 40) : cv::Scalar(0, 210, 255);
      if (polygon.size() == 4) cv::polylines(bgr, polygon, true, color, overlay.active ? 2 : 1);
      if (!polygon.empty()) {
        cv::putText(bgr, "ID " + std::to_string(overlay.id), polygon.front(),
                    cv::FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv::LINE_AA);
      }
    }
    if (landing_point.has_value()) {
      const cv::Point point(cvRound(landing_point->x * sx), cvRound(landing_point->y * sy));
      cv::drawMarker(bgr, point, cv::Scalar(0, 0, 255), cv::MARKER_CROSS, 18, 2);
    }
    if (active_id >= 0) {
      std::ostringstream text;
      text << "ACTIVE ID " << active_id << "  CONF " << std::fixed << std::setprecision(2)
           << confidence;
      cv::putText(bgr, text.str(), cv::Point(8, bgr.rows - 10), cv::FONT_HERSHEY_SIMPLEX,
                  0.45, cv::Scalar(40, 220, 40), 1, cv::LINE_AA);
    }

    std_msgs::msg::Header header;
    header.stamp = stamp;
    header.frame_id = frame_id.empty() ? camera_frame_id_ : frame_id;
    pub_image_->publish(*cv_bridge::CvImage(header, "bgr8", bgr).toImageMsg());
  }

  void processLatestFrame() {
    cv::Mat frame;
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    uint64_t sequence = 0;
    {
      std::lock_guard<std::mutex> lock(latest_frame_mutex_);
      sequence = latest_frame_sequence_;
      if (sequence == 0 || sequence == processed_frame_sequence_ || latest_frame_.empty()) {
        ++no_new_frame_count_;
        return;
      }
      frame = latest_frame_;
      stamp = latest_frame_stamp_;
    }

    if (processed_frame_sequence_ != 0 && sequence > processed_frame_sequence_ + 1) {
      frames_replaced_before_detection_ += sequence - processed_frame_sequence_ - 1;
    }
    processed_frame_sequence_ = sequence;

    detectAndPublish(frame, stamp, camera_frame_id_);
  }

  void publishDeviceStream(const cv::Mat &frame, const rclcpp::Time &stamp, const std::string &frame_id) {
    updateScaledCameraModelForFrame(frame.cols, frame.rows);

    cv::Mat bgr;
    if (frame.channels() == 1) {
      cv::cvtColor(frame, bgr, cv::COLOR_GRAY2BGR);
    } else if (frame.channels() == 3) {
      bgr = frame;
    } else if (frame.channels() == 4) {
      cv::cvtColor(frame, bgr, cv::COLOR_BGRA2BGR);
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Unsupported channel count=%d for image publish", frame.channels());
      return;
    }

    std_msgs::msg::Header header;
    header.stamp = stamp;
    header.frame_id = frame_id.empty() ? camera_frame_id_ : frame_id;

    auto img_msg = cv_bridge::CvImage(header, "bgr8", bgr).toImageMsg();
    pub_image_->publish(*img_msg);

    sensor_msgs::msg::CameraInfo info;
    info.header = header;
    info.width = static_cast<uint32_t>(bgr.cols);
    info.height = static_cast<uint32_t>(bgr.rows);
    info.distortion_model = "plumb_bob";

    info.k = {camera_matrix_.at<double>(0, 0), camera_matrix_.at<double>(0, 1), camera_matrix_.at<double>(0, 2),
              camera_matrix_.at<double>(1, 0), camera_matrix_.at<double>(1, 1), camera_matrix_.at<double>(1, 2),
              camera_matrix_.at<double>(2, 0), camera_matrix_.at<double>(2, 1), camera_matrix_.at<double>(2, 2)};

    info.r = {1.0, 0.0, 0.0,
              0.0, 1.0, 0.0,
              0.0, 0.0, 1.0};

    info.p = {camera_matrix_.at<double>(0, 0), camera_matrix_.at<double>(0, 1), camera_matrix_.at<double>(0, 2), 0.0,
              camera_matrix_.at<double>(1, 0), camera_matrix_.at<double>(1, 1), camera_matrix_.at<double>(1, 2), 0.0,
              camera_matrix_.at<double>(2, 0), camera_matrix_.at<double>(2, 1), camera_matrix_.at<double>(2, 2), 0.0};

    info.d.resize(static_cast<size_t>(dist_coeffs_.cols), 0.0);
    for (int i = 0; i < dist_coeffs_.cols; ++i) {
      info.d[static_cast<size_t>(i)] = dist_coeffs_.at<double>(0, i);
    }

    pub_camera_info_->publish(info);
    ++camera_info_count_;
  }

  bool estimateCandidate(
      int id, size_t detection_index, const std::vector<cv::Point2f> &corners,
      int image_width, int image_height, TagCandidate &candidate) const {
    const auto config_it = landing_tags_.find(id);
    if (config_it == landing_tags_.end()) return false;
    const auto &config = config_it->second;
    const double area = quadArea(corners);
    if (area < min_tag_area_px_) return false;

    std::vector<std::vector<cv::Point2f>> one_marker{corners};
    std::vector<cv::Vec3d> rvecs;
    std::vector<cv::Vec3d> tvecs;
    cv::aruco::estimatePoseSingleMarkers(
        one_marker, static_cast<float>(config.size_m), camera_matrix_, dist_coeffs_, rvecs, tvecs);
    if (rvecs.empty() || tvecs.empty()) return false;
    const cv::Vec3d &rvec = rvecs.front();
    const cv::Vec3d &tvec = tvecs.front();
    for (int axis = 0; axis < 3; ++axis) {
      if (!std::isfinite(rvec[axis]) || !std::isfinite(tvec[axis])) return false;
    }

    cv::Mat rotation_mat;
    cv::Rodrigues(rvec, rotation_mat);
    cv::Matx33d tag_rotation;
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        tag_rotation(row, col) = rotation_mat.at<double>(row, col);
      }
    }
    const double c = std::cos(config.yaw_rad);
    const double s = std::sin(config.yaw_rad);
    const cv::Matx33d tag_to_landing_rotation(c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0);

    const double half = config.size_m * 0.5;
    const std::vector<cv::Point3f> object_points{
        {-static_cast<float>(half), static_cast<float>(half), 0.0F},
        {static_cast<float>(half), static_cast<float>(half), 0.0F},
        {static_cast<float>(half), -static_cast<float>(half), 0.0F},
        {-static_cast<float>(half), -static_cast<float>(half), 0.0F}};
    std::vector<cv::Point2f> projected;
    cv::projectPoints(object_points, rvec, tvec, camera_matrix_, dist_coeffs_, projected);
    double squared_error = 0.0;
    for (size_t i = 0; i < corners.size(); ++i) {
      const cv::Point2f delta = projected[i] - corners[i];
      squared_error += delta.dot(delta);
    }
    const double rmse = std::sqrt(squared_error / 4.0);
    const cv::Point2f centre = 0.25F * (corners[0] + corners[1] + corners[2] + corners[3]);
    const double centre_distance = std::hypot(
        centre.x - image_width * 0.5, centre.y - image_height * 0.5) /
        std::max(1.0, 0.5 * std::hypot(image_width, image_height));

    candidate.id = id;
    candidate.detection_index = detection_index;
    candidate.area_px = area;
    candidate.reprojection_rmse_px = rmse;
    candidate.centre_distance_norm = centre_distance;
    candidate.score = area /
                      ((1.0 + uncertainty_weight_ * rmse) *
                       (1.0 + image_center_weight_ * centre_distance));
    candidate.landing_tvec = tvec + tag_rotation * config.tag_to_landing;
    candidate.landing_rotation = tag_rotation * tag_to_landing_rotation;
    return candidate.landing_tvec[2] > 0.0;
  }

  const TagCandidate *selectCandidate(const std::vector<TagCandidate> &candidates, bool &switched) {
    switched = false;
    if (candidates.empty()) return nullptr;
    const auto best_it = std::max_element(
        candidates.begin(), candidates.end(),
        [](const TagCandidate &a, const TagCandidate &b) { return a.score < b.score; });
    const TagCandidate *active = nullptr;
    for (const auto &candidate : candidates) {
      visible_streaks_[candidate.id] += 1;
      if (candidate.id == active_tag_id_) active = &candidate;
    }
    for (auto &[id, streak] : visible_streaks_) {
      const bool visible = std::any_of(candidates.begin(), candidates.end(),
                                       [id](const TagCandidate &c) { return c.id == id; });
      if (!visible) streak = 0;
    }

    if (active == nullptr) {
      const int previous_id = active_tag_id_;
      switched = previous_id >= 0 && previous_id != best_it->id;
      active_tag_id_ = best_it->id;
      if (switched) ++tag_switch_count_;
      return &*best_it;
    }
    if (best_it->id != active_tag_id_ &&
        best_it->score >= active->score * switch_score_ratio_ &&
        visible_streaks_[best_it->id] >= switch_confirm_frames_) {
      active_tag_id_ = best_it->id;
      switched = true;
      ++tag_switch_count_;
      return &*best_it;
    }
    return active;
  }

  void filterLandingPose(
      const TagCandidate &candidate, const rclcpp::Time &stamp, bool switched,
      cv::Vec3d &position, cv::Vec4d &orientation) {
    const cv::Vec4d raw_orientation = rotationToQuaternion(candidate.landing_rotation);
    const bool reset = !filter_initialized_ ||
                       (last_filter_stamp_.nanoseconds() != 0 &&
                        (stamp - last_filter_stamp_).seconds() > filter_reset_timeout_sec_);
    if (reset) {
      filtered_position_ = candidate.landing_tvec;
      filtered_orientation_ = raw_orientation;
      filter_initialized_ = true;
    } else {
      const double alpha = switched ? switch_filter_alpha_ : pose_filter_alpha_;
      cv::Vec3d correction = (candidate.landing_tvec - filtered_position_) * alpha;
      const double correction_norm = cv::norm(correction);
      if (max_filtered_step_m_ > 0.0 && correction_norm > max_filtered_step_m_) {
        correction *= max_filtered_step_m_ / correction_norm;
        ++filter_step_limit_count_;
      }
      filtered_position_ += correction;
      filtered_orientation_ = blendQuaternion(filtered_orientation_, raw_orientation, alpha);
    }
    last_filter_stamp_ = stamp;
    position = filtered_position_;
    orientation = filtered_orientation_;
  }

  void updatePreviewOverlay(
      const std::vector<int> &ids, const std::vector<std::vector<cv::Point2f>> &corners,
      int active_id, const cv::Vec3d *landing_point, double confidence,
      int image_width, int image_height) {
    std::lock_guard<std::mutex> lock(preview_overlay_mutex_);
    preview_overlays_.clear();
    for (size_t i = 0; i < ids.size(); ++i) {
      if (landing_tags_.count(ids[i]) != 0) {
        preview_overlays_.push_back({ids[i], ids[i] == active_id, corners[i]});
      }
    }
    preview_active_tag_id_ = active_id;
    preview_confidence_ = confidence;
    preview_source_width_ = image_width;
    preview_source_height_ = image_height;
    preview_landing_point_.reset();
    if (landing_point != nullptr && (*landing_point)[2] > 1e-6) {
      std::vector<cv::Point3f> point{{static_cast<float>((*landing_point)[0]),
                                     static_cast<float>((*landing_point)[1]),
                                     static_cast<float>((*landing_point)[2])}};
      std::vector<cv::Point2f> projected;
      cv::projectPoints(point, cv::Vec3d::all(0.0), cv::Vec3d::all(0.0),
                        camera_matrix_, dist_coeffs_, projected);
      if (!projected.empty()) preview_landing_point_ = projected.front();
    }
  }

  void detectAndPublish(const cv::Mat &image, const rclcpp::Time &stamp, const std::string &frame_id) {
    const auto processing_start = std::chrono::steady_clock::now();
    const double image_age_start_ms = std::max(0.0, (now() - stamp).seconds() * 1000.0);
    const auto record_processing = [this, processing_start, image_age_start_ms]() {
      const double processing_ms = std::chrono::duration<double, std::milli>(
                                       std::chrono::steady_clock::now() - processing_start).count();
      last_processing_ms_ = processing_ms;
      max_processing_ms_ = std::max(max_processing_ms_, processing_ms);
      processing_total_ms_ += processing_ms;
      ++processing_sample_count_;
      last_image_age_start_ms_ = image_age_start_ms;
      max_image_age_start_ms_ = std::max(max_image_age_start_ms_, image_age_start_ms);
    };

    ++detector_input_count_;
    if (last_input_stamp_.nanoseconds() != 0 && stamp <= last_input_stamp_) {
      ++nonmonotonic_stamp_count_;
    }
    last_input_stamp_ = stamp;

    updateScaledCameraModelForFrame(image.cols, image.rows);

    cv::Mat gray;
    if (image.channels() == 1) {
      gray = image;
    } else {
      cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    }

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<std::vector<cv::Point2f>> rejected;
    cv::aruco::detectMarkers(gray, detector_dict_, corners, ids, detector_params_, rejected);

    std::vector<TagCandidate> candidates;
    double target_max_area_px = 0.0;
    bool configured_id_seen = false;
    for (size_t i = 0; i < ids.size(); ++i) {
      if (landing_tags_.count(ids[i]) == 0) continue;
      configured_id_seen = true;
      target_max_area_px = std::max(target_max_area_px, quadArea(corners[i]));
      TagCandidate candidate;
      if (estimateCandidate(ids[i], i, corners[i], image.cols, image.rows, candidate)) {
        candidates.push_back(candidate);
      }
    }
    window_max_candidate_area_px_ = std::max(window_max_candidate_area_px_, target_max_area_px);
    if (candidates.empty()) {
      if (ids.empty()) ++no_marker_count_;
      else if (configured_id_seen) ++below_area_count_;
      else ++wrong_id_count_;
      for (auto &[id, streak] : visible_streaks_) streak = 0;
      updatePreviewOverlay(ids, corners, -1, nullptr, 0.0, image.cols, image.rows);
      record_processing();
      return;
    }

    bool switched = false;
    const TagCandidate *selected = selectCandidate(candidates, switched);
    if (selected == nullptr) {
      ++pose_fail_count_;
      record_processing();
      return;
    }

    cv::Vec3d filtered_position;
    cv::Vec4d filtered_orientation;
    filterLandingPose(*selected, stamp, switched, filtered_position, filtered_orientation);
    const size_t best_idx = selected->detection_index;
    const double best_area_px = selected->area_px;
    const double confidence = std::clamp(
        best_area_px / std::max(1.0, image.cols * image.rows * 0.10) /
            (1.0 + selected->reprojection_rmse_px),
        0.0, 1.0);

    geometry_msgs::msg::PoseStamped out;
    out.header.stamp = stamp;
    out.header.frame_id = frame_id.empty() ? camera_frame_id_ : frame_id;

    out.pose.position.x = filtered_position[0];
    out.pose.position.y = filtered_position[1];
    out.pose.position.z = filtered_position[2];
    out.pose.orientation.x = filtered_orientation[0];
    out.pose.orientation.y = filtered_orientation[1];
    out.pose.orientation.z = filtered_orientation[2];
    out.pose.orientation.w = filtered_orientation[3];

    pub_tag_pose_->publish(out);
    publishDetectionMetadata(
        out.header, corners[best_idx], selected->id, image.cols, image.rows,
        best_area_px, out.pose.position.x, out.pose.position.y, out.pose.position.z,
        candidates.size(), selected->reprojection_rmse_px, confidence, switched);
    updatePreviewOverlay(
        ids, corners, selected->id, &filtered_position, confidence, image.cols, image.rows);
    ++detector_output_count_;
    if (last_detection_stamp_.nanoseconds() != 0 && stamp > last_detection_stamp_) {
      longest_detection_gap_sec_ = std::max(
          longest_detection_gap_sec_, (stamp - last_detection_stamp_).seconds());
    }
    last_detection_stamp_ = stamp;
    last_detection_area_px_ = best_area_px;
    last_edge_distance_px_ = std::numeric_limits<double>::infinity();
    for (const auto &point : corners[best_idx]) {
      last_edge_distance_px_ = std::min(
          last_edge_distance_px_,
          std::min({static_cast<double>(point.x), static_cast<double>(point.y),
                    static_cast<double>(image.cols - 1) - point.x,
                    static_cast<double>(image.rows - 1) - point.y}));
    }
    window_min_edge_distance_px_ = std::min(window_min_edge_distance_px_, last_edge_distance_px_);
    record_processing();

    const double fx = camera_matrix_.at<double>(0, 0);
    const double fy = camera_matrix_.at<double>(1, 1);
    const double z_abs = std::fabs(out.pose.position.z);
    const double area_m2 = pixelAreaToSquareMeters(best_area_px, z_abs, fx, fy);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                         "Landing target active_id=%d visible=%zu switched=%s pos_cam=[%.2f %.2f %.2f] rmse=%.2fpx area_m2=%.6f",
                         selected->id, candidates.size(), switched ? "true" : "false",
                         out.pose.position.x, out.pose.position.y, out.pose.position.z,
                         selected->reprojection_rmse_px, area_m2);
  }

  bool useSimTime() const {
    bool use_sim_time = false;
    get_parameter("use_sim_time", use_sim_time);
    return use_sim_time;
  }

  void publishDetectionMetadata(
      const std_msgs::msg::Header &header,
      const std::vector<cv::Point2f> &corners,
      int tag_id,
      int image_width,
      int image_height,
      double area_px,
      double target_x,
      double target_y,
      double target_z,
      size_t visible_tag_count,
      double reprojection_rmse_px,
      double confidence,
      bool switched) {
    if (corners.size() != 4) {
      return;
    }

    geometry_msgs::msg::PolygonStamped polygon;
    polygon.header = header;
    polygon.polygon.points.reserve(corners.size());
    double center_x = 0.0;
    double center_y = 0.0;
    for (const auto &corner : corners) {
      geometry_msgs::msg::Point32 point;
      point.x = corner.x;
      point.y = corner.y;
      point.z = 0.0F;
      polygon.polygon.points.push_back(point);
      center_x += corner.x;
      center_y += corner.y;
    }
    center_x /= 4.0;
    center_y /= 4.0;
    pub_tag_corners_->publish(polygon);

    diagnostic_msgs::msg::DiagnosticArray metadata;
    metadata.header = header;
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.name = "apriltag_detection";
    status.hardware_id = video_device_;
    status.message = "TARGET_DETECTED";
    const auto add_value = [&status](const std::string &key, const std::string &value) {
      diagnostic_msgs::msg::KeyValue item;
      item.key = key;
      item.value = value;
      status.values.push_back(item);
    };
    add_value("tag_id", std::to_string(tag_id));
    add_value("quality", std::to_string(confidence));
    add_value("active_tag_id", std::to_string(tag_id));
    add_value("reference_tag_id", std::to_string(reference_tag_id_));
    add_value("visible_tag_count", std::to_string(visible_tag_count));
    add_value("tag_switched", switched ? "true" : "false");
    add_value("reprojection_rmse_px", std::to_string(reprojection_rmse_px));
    add_value("tag_area_px", std::to_string(area_px));
    add_value("center_x_px", std::to_string(center_x));
    add_value("center_y_px", std::to_string(center_y));
    add_value("image_width", std::to_string(image_width));
    add_value("image_height", std::to_string(image_height));
    add_value("pixel_error_x", std::to_string(center_x - static_cast<double>(image_width) * 0.5));
    add_value("pixel_error_y", std::to_string(center_y - static_cast<double>(image_height) * 0.5));
    add_value("target_x_m", std::to_string(target_x));
    add_value("target_y_m", std::to_string(target_y));
    add_value("target_z_m", std::to_string(target_z));
    add_value("source_stamp_ns", std::to_string(rclcpp::Time(header.stamp).nanoseconds()));
    metadata.status.push_back(status);
    pub_tag_metadata_->publish(metadata);
  }

  void publishDiagnostics() {
    const auto current_time = now();
    const uint64_t input_delta = detector_input_count_ - previous_detector_input_count_;
    const uint64_t output_delta = detector_output_count_ - previous_detector_output_count_;
    const double last_age = last_detection_stamp_.nanoseconds() == 0
                                ? std::numeric_limits<double>::infinity()
                                : std::max(0.0, (current_time - last_detection_stamp_).seconds());
    const double visible_percent = input_delta == 0
                                       ? 0.0
                                       : 100.0 * static_cast<double>(output_delta) /
                                             static_cast<double>(input_delta);
    const double total_visible_percent = detector_input_count_ == 0
                                             ? 0.0
                                             : 100.0 * static_cast<double>(detector_output_count_) /
                                                   static_cast<double>(detector_input_count_);
    const double average_processing_ms = processing_sample_count_ == 0
                                             ? 0.0
                                             : processing_total_ms_ / static_cast<double>(processing_sample_count_);
    double longest_capture_gap_ms = 0.0;
    {
      std::lock_guard<std::mutex> lock(latest_frame_mutex_);
      longest_capture_gap_ms = longest_capture_gap_ms_;
    }

    RCLCPP_INFO(
        get_logger(),
        "DETECTOR_DIAG use_sim_time=%s camera_hz=%llu info_hz=%llu detect_hz=%llu output_hz=%llu "
        "expected_tag_detected=%s visibility_pct=%.1f visibility_total_pct=%.1f "
        "active_tag_id=%d configured_tags=%zu tag_switches=%llu filter_step_limits=%llu "
        "last_detection_age_s=%.3f longest_detection_gap_s=%.3f last_stamp_ns=%lld last_area_px=%.1f window_max_area_px=%.1f "
        "edge_px[last=%.1f,window_min=%.1f] latency_ms[last=%.1f,avg=%.1f,max=%.1f,image_age_start=%.1f,max_image_age_start=%.1f] "
        "capture_gap_max_ms=%.1f queue=latest_only drop_total[read=%llu,replaced=%llu,no_new=%llu,no_marker=%llu,wrong_id=%llu,below_area=%llu,pose=%llu,no_info=%llu,convert=%llu,stamp=%llu]",
        useSimTime() ? "true" : "false",
        static_cast<unsigned long long>(camera_frame_count_.load() - previous_camera_frame_count_),
        static_cast<unsigned long long>(camera_info_count_ - previous_camera_info_count_),
        static_cast<unsigned long long>(input_delta),
        static_cast<unsigned long long>(output_delta),
        output_delta > 0 ? "true" : "false", visible_percent, total_visible_percent,
        active_tag_id_, landing_tags_.size(),
        static_cast<unsigned long long>(tag_switch_count_),
        static_cast<unsigned long long>(filter_step_limit_count_),
        last_age, longest_detection_gap_sec_,
        static_cast<long long>(last_detection_stamp_.nanoseconds()),
        last_detection_area_px_, window_max_candidate_area_px_,
        last_edge_distance_px_,
        std::isfinite(window_min_edge_distance_px_) ? window_min_edge_distance_px_ : -1.0,
        last_processing_ms_, average_processing_ms, max_processing_ms_,
        last_image_age_start_ms_, max_image_age_start_ms_, longest_capture_gap_ms,
        static_cast<unsigned long long>(camera_read_fail_count_.load() + camera_not_open_count_.load()),
        static_cast<unsigned long long>(frames_replaced_before_detection_),
        static_cast<unsigned long long>(no_new_frame_count_),
        static_cast<unsigned long long>(no_marker_count_),
        static_cast<unsigned long long>(wrong_id_count_),
        static_cast<unsigned long long>(below_area_count_),
        static_cast<unsigned long long>(pose_fail_count_),
        static_cast<unsigned long long>(missing_camera_info_count_ + invalid_camera_info_count_),
        static_cast<unsigned long long>(image_conversion_fail_count_),
        static_cast<unsigned long long>(nonmonotonic_stamp_count_));

    previous_camera_frame_count_ = camera_frame_count_.load();
    previous_camera_info_count_ = camera_info_count_;
    previous_detector_input_count_ = detector_input_count_;
    previous_detector_output_count_ = detector_output_count_;
    window_max_candidate_area_px_ = 0.0;
    window_min_edge_distance_px_ = std::numeric_limits<double>::infinity();
  }

  std::string input_source_;

  std::string image_topic_;
  std::string camera_info_topic_;
  std::string image_output_topic_;
  std::string camera_info_output_topic_;
  bool publish_image_stream_{true};
  bool preview_start_enabled_{false};
  double preview_rate_hz_{5.0};
  int preview_width_{320};
  int preview_height_{240};
  std::string preview_start_service_;
  std::string preview_stop_service_;
  std::atomic<bool> preview_enabled_{false};
  std::string camera_frame_id_;

  std::string video_device_;
  int device_width_{640};
  int device_height_{480};
  double device_fps_{30.0};
  double detect_rate_hz_{20.0};
  int capture_buffer_size_{1};

  double fx_{0.0};
  double fy_{0.0};
  double cx_{0.0};
  double cy_{0.0};
  int calibration_width_{640};
  int calibration_height_{480};
  double hfov_deg_{78.0};
  std::vector<double> dist_coeffs_vec_;

  std::string tag_pose_topic_;
  std::string tag_corners_topic_;
  std::string tag_metadata_topic_;
  std::string dictionary_name_;

  double tag_size_m_{0.16};
  int target_tag_id_{-1};
  double min_tag_area_px_{80.0};
  int reference_tag_id_{0};
  std::vector<int64_t> landing_tag_ids_;
  std::vector<double> landing_tag_sizes_m_;
  std::vector<double> landing_tag_offset_x_m_;
  std::vector<double> landing_tag_offset_y_m_;
  std::vector<double> landing_tag_offset_z_m_;
  std::vector<double> landing_tag_yaw_rad_;
  std::map<int, LandingTagConfig> landing_tags_;
  int switch_confirm_frames_{4};
  double switch_score_ratio_{1.10};
  double pose_filter_alpha_{0.45};
  double switch_filter_alpha_{0.20};
  double filter_reset_timeout_sec_{1.0};
  double max_filtered_step_m_{0.12};
  double uncertainty_weight_{0.35};
  double image_center_weight_{0.15};
  int active_tag_id_{-1};
  std::map<int, int> visible_streaks_;
  bool filter_initialized_{false};
  cv::Vec3d filtered_position_{0.0, 0.0, 0.0};
  cv::Vec4d filtered_orientation_{0.0, 0.0, 0.0, 1.0};
  rclcpp::Time last_filter_stamp_{0, 0, RCL_ROS_TIME};
  uint64_t tag_switch_count_{0};
  uint64_t filter_step_limit_count_{0};

  bool got_camera_info_{false};
  int last_frame_width_{0};
  int last_frame_height_{0};

  cv::Mat param_camera_matrix_;
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  cv::Ptr<cv::aruco::Dictionary> detector_dict_;
  cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
  cv::VideoCapture cap_;
  std::atomic<bool> capture_running_{false};
  std::thread capture_thread_;
  std::thread preview_thread_;
  std::mutex latest_frame_mutex_;
  cv::Mat latest_frame_;
  rclcpp::Time latest_frame_stamp_{0, 0, RCL_ROS_TIME};
  uint64_t latest_frame_sequence_{0};
  uint64_t processed_frame_sequence_{0};
  double longest_capture_gap_ms_{0.0};
  std::mutex preview_overlay_mutex_;
  std::vector<PreviewTagOverlay> preview_overlays_;
  std::optional<cv::Point2f> preview_landing_point_;
  int preview_active_tag_id_{-1};
  double preview_confidence_{0.0};
  int preview_source_width_{1};
  int preview_source_height_{1};

  std::atomic<uint64_t> camera_frame_count_{0};
  uint64_t camera_info_count_{0};
  uint64_t detector_input_count_{0};
  uint64_t detector_output_count_{0};
  uint64_t previous_camera_frame_count_{0};
  uint64_t previous_camera_info_count_{0};
  uint64_t previous_detector_input_count_{0};
  uint64_t previous_detector_output_count_{0};
  std::atomic<uint64_t> camera_not_open_count_{0};
  std::atomic<uint64_t> camera_read_fail_count_{0};
  uint64_t frames_replaced_before_detection_{0};
  uint64_t no_new_frame_count_{0};
  uint64_t invalid_camera_info_count_{0};
  uint64_t missing_camera_info_count_{0};
  uint64_t image_conversion_fail_count_{0};
  uint64_t no_marker_count_{0};
  uint64_t wrong_id_count_{0};
  uint64_t below_area_count_{0};
  uint64_t pose_fail_count_{0};
  uint64_t nonmonotonic_stamp_count_{0};
  double last_detection_area_px_{0.0};
  double window_max_candidate_area_px_{0.0};
  double last_edge_distance_px_{-1.0};
  double window_min_edge_distance_px_{std::numeric_limits<double>::infinity()};
  double longest_detection_gap_sec_{0.0};
  double last_processing_ms_{0.0};
  double max_processing_ms_{0.0};
  double processing_total_ms_{0.0};
  uint64_t processing_sample_count_{0};
  double last_image_age_start_ms_{0.0};
  double max_image_age_start_ms_{0.0};
  rclcpp::Time last_input_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_detection_stamp_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_camera_info_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_camera_info_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_tag_pose_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr pub_tag_corners_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_tag_metadata_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr preview_start_server_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr preview_stop_server_;
  rclcpp::TimerBase::SharedPtr capture_timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AprilTagCameraDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
