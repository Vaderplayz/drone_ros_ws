#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "cv_bridge/cv_bridge.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"

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
    camera_frame_id_ = declare_parameter<std::string>("camera_frame_id", "camera_optical_frame");

    video_device_ = declare_parameter<std::string>("video_device", "/dev/video0");
    device_width_ = declare_parameter<int>("device_width", 640);
    device_height_ = declare_parameter<int>("device_height", 480);
    device_fps_ = declare_parameter<double>("device_fps", 30.0);
    detect_rate_hz_ = declare_parameter<double>("detect_rate_hz", 20.0);

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
    tag_size_m_ = declare_parameter<double>("tag_size_m", std::numeric_limits<double>::quiet_NaN());
    target_tag_id_ = declare_parameter<int>("target_tag_id", -1);
    min_tag_area_px_ = declare_parameter<double>("min_tag_area_px", 80.0);
    dictionary_name_ = declare_parameter<std::string>("dictionary", "36h11");

    detector_dict_ = cv::aruco::getPredefinedDictionary(dictionaryFromString(dictionary_name_));
    const auto qos_sensor = rclcpp::SensorDataQoS();
    pub_image_ = create_publisher<sensor_msgs::msg::Image>(image_output_topic_, qos_sensor);
    pub_camera_info_ = create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_output_topic_, qos_sensor);
    pub_tag_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>(tag_pose_topic_, 10);

    if (!std::isfinite(tag_size_m_) || tag_size_m_ <= 0.0) {
      RCLCPP_FATAL(get_logger(), "tag_size_m must be provided and > 0.0");
      throw std::runtime_error("invalid tag_size_m");
    }

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
                "apriltag_camera_detector started source=%s dict=%s tag_size=%.3f target_id=%d min_area_px=%.1f out=%s use_sim_time=%s",
                input_source_.c_str(), dictionary_name_.c_str(), tag_size_m_, target_tag_id_,
                min_tag_area_px_, tag_pose_topic_.c_str(), useSimTime() ? "true" : "false");
  }

  ~AprilTagCameraDetectorNode() override {
    if (cap_.isOpened()) {
      cap_.release();
    }
  }

 private:
  void initRosTopicMode() {
    const auto qos_sensor = rclcpp::SensorDataQoS();
    sub_camera_info_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic_, qos_sensor,
        std::bind(&AprilTagCameraDetectorNode::cameraInfoCb, this, std::placeholders::_1));

    sub_image_ = create_subscription<sensor_msgs::msg::Image>(
        image_topic_, qos_sensor,
        std::bind(&AprilTagCameraDetectorNode::imageCb, this, std::placeholders::_1));

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

    const double actual_width = cap_.get(cv::CAP_PROP_FRAME_WIDTH);
    const double actual_height = cap_.get(cv::CAP_PROP_FRAME_HEIGHT);
    const double actual_fps = cap_.get(cv::CAP_PROP_FPS);

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, detect_rate_hz_));
    capture_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        std::bind(&AprilTagCameraDetectorNode::captureLoop, this));

    RCLCPP_INFO(get_logger(),
                "Device mode video=%s requested=%dx%d@%.1f negotiated=%.0fx%.0f@%.1f",
                video_device_.c_str(), device_width_, device_height_, device_fps_,
                actual_width, actual_height, actual_fps);
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

    if (publish_image_stream_ && image_output_topic_ != image_topic_) {
      auto out = *msg;
      if (out.header.frame_id.empty()) {
        out.header.frame_id = camera_frame_id_;
      }
      pub_image_->publish(out);
    }

    detectAndPublish(cv_ptr->image, rclcpp::Time(msg->header.stamp), msg->header.frame_id);
  }

  void captureLoop() {
    if (!cap_.isOpened()) {
      ++camera_not_open_count_;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Video device not open: %s", video_device_.c_str());
      return;
    }

    cv::Mat frame;
    if (!cap_.read(frame) || frame.empty()) {
      ++camera_read_fail_count_;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Failed to read frame from %s", video_device_.c_str());
      return;
    }

    ++camera_frame_count_;
    const auto stamp = now();
    if (publish_image_stream_) {
      publishDeviceStream(frame, stamp, camera_frame_id_);
    }
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

  void detectAndPublish(const cv::Mat &image, const rclcpp::Time &stamp, const std::string &frame_id) {
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
    cv::Ptr<cv::aruco::DetectorParameters> detector_params = cv::aruco::DetectorParameters::create();
    cv::aruco::detectMarkers(gray, detector_dict_, corners, ids, detector_params, rejected);

    if (ids.empty()) {
      ++no_marker_count_;
      return;
    }

    int best_idx = -1;
    double best_area_px = 0.0;
    bool target_id_seen = false;
    double target_max_area_px = 0.0;

    for (size_t i = 0; i < ids.size(); ++i) {
      if (target_tag_id_ >= 0 && ids[i] != target_tag_id_) {
        continue;
      }

      target_id_seen = true;
      const double area_px = quadArea(corners[i]);
      target_max_area_px = std::max(target_max_area_px, area_px);
      if (area_px < min_tag_area_px_) {
        continue;
      }

      if (best_idx < 0 || area_px > best_area_px) {
        best_idx = static_cast<int>(i);
        best_area_px = area_px;
      }
    }

    window_max_candidate_area_px_ = std::max(window_max_candidate_area_px_, target_max_area_px);

    if (best_idx < 0) {
      if (target_id_seen) {
        ++below_area_count_;
      } else {
        ++wrong_id_count_;
      }
      return;
    }

    std::vector<std::vector<cv::Point2f>> picked_corners{corners[best_idx]};
    std::vector<cv::Vec3d> rvecs, tvecs;
    cv::aruco::estimatePoseSingleMarkers(
        picked_corners,
        static_cast<float>(tag_size_m_),
        camera_matrix_,
        dist_coeffs_,
        rvecs,
        tvecs);

    if (rvecs.empty() || tvecs.empty()) {
      ++pose_fail_count_;
      return;
    }

    const cv::Vec3d rvec = rvecs[0];
    const cv::Vec3d tvec = tvecs[0];
    if (!std::isfinite(rvec[0]) || !std::isfinite(rvec[1]) || !std::isfinite(rvec[2]) ||
        !std::isfinite(tvec[0]) || !std::isfinite(tvec[1]) || !std::isfinite(tvec[2])) {
      ++pose_fail_count_;
      return;
    }

    const double angle = std::sqrt(rvec[0] * rvec[0] + rvec[1] * rvec[1] + rvec[2] * rvec[2]);
    double qx = 0.0;
    double qy = 0.0;
    double qz = 0.0;
    double qw = 1.0;
    if (angle > 1e-9) {
      const double ax = rvec[0] / angle;
      const double ay = rvec[1] / angle;
      const double az = rvec[2] / angle;
      const double s = std::sin(angle * 0.5);
      qx = ax * s;
      qy = ay * s;
      qz = az * s;
      qw = std::cos(angle * 0.5);
    }

    geometry_msgs::msg::PoseStamped out;
    out.header.stamp = stamp;
    out.header.frame_id = frame_id.empty() ? camera_frame_id_ : frame_id;

    out.pose.position.x = tvec[0];
    out.pose.position.y = tvec[1];
    out.pose.position.z = tvec[2];
    out.pose.orientation.x = qx;
    out.pose.orientation.y = qy;
    out.pose.orientation.z = qz;
    out.pose.orientation.w = qw;

    pub_tag_pose_->publish(out);
    ++detector_output_count_;
    if (last_detection_stamp_.nanoseconds() != 0 && stamp > last_detection_stamp_) {
      longest_detection_gap_sec_ = std::max(
          longest_detection_gap_sec_, (stamp - last_detection_stamp_).seconds());
    }
    last_detection_stamp_ = stamp;
    last_detection_area_px_ = best_area_px;

    const double fx = camera_matrix_.at<double>(0, 0);
    const double fy = camera_matrix_.at<double>(1, 1);
    const double z_abs = std::fabs(out.pose.position.z);
    const double area_m2 = pixelAreaToSquareMeters(best_area_px, z_abs, fx, fy);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                         "Tag detected id=%d pos_cam=[%.2f %.2f %.2f] area_m2=%.6f",
                         ids[best_idx], out.pose.position.x, out.pose.position.y,
                         out.pose.position.z, area_m2);
  }

  bool useSimTime() const {
    bool use_sim_time = false;
    get_parameter("use_sim_time", use_sim_time);
    return use_sim_time;
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

    RCLCPP_INFO(
        get_logger(),
        "DETECTOR_DIAG use_sim_time=%s camera_hz=%llu info_hz=%llu detect_hz=%llu output_hz=%llu "
        "expected_tag_detected=%s visibility_pct=%.1f visibility_total_pct=%.1f "
        "last_detection_age_s=%.3f longest_detection_gap_s=%.3f last_stamp_ns=%lld last_area_px=%.1f window_max_area_px=%.1f "
        "drop_total[read=%llu,no_marker=%llu,wrong_id=%llu,below_area=%llu,pose=%llu,no_info=%llu,convert=%llu,stamp=%llu]",
        useSimTime() ? "true" : "false",
        static_cast<unsigned long long>(camera_frame_count_ - previous_camera_frame_count_),
        static_cast<unsigned long long>(camera_info_count_ - previous_camera_info_count_),
        static_cast<unsigned long long>(input_delta),
        static_cast<unsigned long long>(output_delta),
        output_delta > 0 ? "true" : "false", visible_percent, total_visible_percent,
        last_age, longest_detection_gap_sec_,
        static_cast<long long>(last_detection_stamp_.nanoseconds()),
        last_detection_area_px_, window_max_candidate_area_px_,
        static_cast<unsigned long long>(camera_read_fail_count_ + camera_not_open_count_),
        static_cast<unsigned long long>(no_marker_count_),
        static_cast<unsigned long long>(wrong_id_count_),
        static_cast<unsigned long long>(below_area_count_),
        static_cast<unsigned long long>(pose_fail_count_),
        static_cast<unsigned long long>(missing_camera_info_count_ + invalid_camera_info_count_),
        static_cast<unsigned long long>(image_conversion_fail_count_),
        static_cast<unsigned long long>(nonmonotonic_stamp_count_));

    previous_camera_frame_count_ = camera_frame_count_;
    previous_camera_info_count_ = camera_info_count_;
    previous_detector_input_count_ = detector_input_count_;
    previous_detector_output_count_ = detector_output_count_;
    window_max_candidate_area_px_ = 0.0;
  }

  std::string input_source_;

  std::string image_topic_;
  std::string camera_info_topic_;
  std::string image_output_topic_;
  std::string camera_info_output_topic_;
  bool publish_image_stream_{true};
  std::string camera_frame_id_;

  std::string video_device_;
  int device_width_{640};
  int device_height_{480};
  double device_fps_{30.0};
  double detect_rate_hz_{20.0};

  double fx_{0.0};
  double fy_{0.0};
  double cx_{0.0};
  double cy_{0.0};
  int calibration_width_{640};
  int calibration_height_{480};
  double hfov_deg_{78.0};
  std::vector<double> dist_coeffs_vec_;

  std::string tag_pose_topic_;
  std::string dictionary_name_;

  double tag_size_m_{0.16};
  int target_tag_id_{-1};
  double min_tag_area_px_{80.0};

  bool got_camera_info_{false};
  int last_frame_width_{0};
  int last_frame_height_{0};

  cv::Mat param_camera_matrix_;
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  cv::Ptr<cv::aruco::Dictionary> detector_dict_;
  cv::VideoCapture cap_;

  uint64_t camera_frame_count_{0};
  uint64_t camera_info_count_{0};
  uint64_t detector_input_count_{0};
  uint64_t detector_output_count_{0};
  uint64_t previous_camera_frame_count_{0};
  uint64_t previous_camera_info_count_{0};
  uint64_t previous_detector_input_count_{0};
  uint64_t previous_detector_output_count_{0};
  uint64_t camera_not_open_count_{0};
  uint64_t camera_read_fail_count_{0};
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
  double longest_detection_gap_sec_{0.0};
  rclcpp::Time last_input_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_detection_stamp_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_camera_info_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_camera_info_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_tag_pose_;
  rclcpp::TimerBase::SharedPtr capture_timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AprilTagCameraDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
